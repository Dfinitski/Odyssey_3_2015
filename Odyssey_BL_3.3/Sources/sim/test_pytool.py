#!/usr/bin/env python3
"""Headless test of BootLoader_3.3.pyw protocol code against an emulated device
(FW 3.3 semantics and legacy 3.2 semantics) with packet loss."""
import sys, types, socket, threading, struct, random, time, importlib.util

# stub tkinter so the GUI module imports without a display
for name in ('tkinter', 'tkinter.ttk', 'tkinter.filedialog', 'tkinter.messagebox'):
    sys.modules[name] = types.ModuleType(name)
tkm = sys.modules['tkinter']
for sub in ('ttk', 'filedialog', 'messagebox'):
    setattr(tkm, sub, sys.modules['tkinter.' + sub])

spec = importlib.util.spec_from_file_location('bl', '../../PC_Tool/BootLoader_3.3.pyw')
from importlib.machinery import SourceFileLoader
bl = SourceFileLoader('bl', '../../PC_Tool/BootLoader_3.3.pyw').load_module()


class Device(threading.Thread):
    """FW 3.3 behaviour (legacy=False) or 3.2 behaviour (legacy=True)."""
    def __init__(self, port, legacy=False, loss=0.0):
        super().__init__(daemon=True)
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s.bind(('127.0.0.1', port))
        self.legacy, self.loss = legacy, loss
        self.flash = bytearray(b'\x00' * (4 * 8192 * 256))
        self.busy_until = 0; self.pending = None; self.last = None
        self.n_erase = self.n_prog = 0
        self.stop = False

    def send(self, data, addr):
        if random.random() >= self.loss:
            self.s.sendto(data, addr)

    def run(self):
        self.s.settimeout(0.01)
        while not self.stop:
            if self.pending and time.time() >= self.busy_until:
                rep, addr, key = self.pending; self.pending = None
                self.last = key
                self.send(rep, addr)
            try:
                d, addr = self.s.recvfrom(4096)
            except socket.timeout:
                continue
            if random.random() < self.loss:      # request lost
                continue
            d = bytearray(d); cmd = bytes(d[:3]); sn = d[3]; page = struct.unpack('!H', bytes(d[4:6]))[0]
            if cmd == b'MAC':
                self.last = None
                rep = b'MAC' + bytes([sn]) + (b'\0'*22 if self.legacy else b'3.3\x0f' + b'\0'*14 + bytes([127,0,0,1])) + bytes.fromhex('0004a3001234')
                self.send(rep, addr)
            elif cmd in (b'ERS', b'WPD', b'RPD'):
                key = (cmd, sn, page)
                if self.pending:
                    if self.legacy: print('LEGACY DEVICE GOT REQUEST WHILE BUSY -> would corrupt'); self.corrupt = True
                    continue
                if not self.legacy and key == self.last and cmd != b'RPD':
                    self.send(bytes(d[:32]), addr); continue
                if cmd == b'ERS':
                    base = page * 256
                    self.flash[base:base + 2*1024*1024] = b'\xff' * (2*1024*1024)
                    self.n_erase += 1; dly = 0.8
                    rep = bytes(d[:32])
                elif cmd == b'WPD':
                    a = page * 256
                    for k in range(256): self.flash[a+k] &= d[32+k]
                    self.n_prog += 1; dly = 0.001
                    rep = bytes(d[:32])
                else:
                    a = page * 256; dly = 0.0005
                    rep = bytes(d[:32]) + bytes(self.flash[a:a+256])
                self.pending = (rep, addr, key); self.busy_until = time.time() + dly
            elif cmd == b'WIP':
                self.send(bytes(d[:32]), addr)


def run(legacy, loss, port):
    random.seed(5)
    dev = Device(port, legacy, loss); dev.start()
    bl.UDP_PORT = port
    b = bl.Bootloader(log=lambda m: None)
    b.connect('127.0.0.1')
    print('connect: ver', b.version, 'caps', b.caps)
    img = bytes(random.getrandbits(8) for _ in range(300 * 256 + 77))
    img = img[:256*10] + b'\xff' * 256 * 5 + img[256*15:]       # some blank pages
    if len(img) % 256: img += b'\xff' * (256 - len(img) % 256)
    t = b.erase_slot(1)
    for i in range(len(img) // 256):
        b.write_page(8192 + i, img[i*256:(i+1)*256]) if img[i*256:(i+1)*256].count(b'\xff') != 256 else None
    ok = dev.flash[2*1024*1024:2*1024*1024 + len(img)] == img
    if b.caps & bl.CAP_RPD:
        for i in range(len(img) // 256):
            assert b.read_page(8192 + i) == img[i*256:(i+1)*256]
    print('legacy=%s loss=%.0f%%: flash content %s, erases=%d programs=%d'
          % (legacy, loss*100, 'OK' if ok else 'BAD', dev.n_erase, dev.n_prog))
    dev.stop = True
    return ok

r1 = run(False, 0.05, 50011)
r2 = run(True, 0.0, 50012)
print('PASS' if r1 and r2 else 'FAIL')
