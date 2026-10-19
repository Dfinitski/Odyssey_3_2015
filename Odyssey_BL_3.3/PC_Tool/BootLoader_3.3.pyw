#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  Odyssey-2 TRX - Bootloader PC tool 3.3
#  (successor of BootLoader 2.3 by N7DDC)
#
#  Runs on Python 2.7 and Python 3.x (Windows / Linux / macOS).
#
#  What changed against 2.3
#   * all network work in a background thread - the window never freezes
#   * one transaction routine: drains stale replies, matches cmd + SN (+ page),
#     ignores foreign packets, uses select() timeouts
#   * erase: the reply arrives as soon as the erase is finished (W25Q64JV,
#     typ ~5 s per slot); FW 3.3 -> request repeated every 2 s (the FPGA
#     ignores repeats while busy and re-acknowledges a finished erase),
#     error after 75 s (datasheet max 32 x 2 s); old FW -> one request
#   * page timeouts as in the Micron loader (1 s + late answer, 3 tries)
#   * whole erase + write + verify repeated once on failure
#   * page write retransmission only with FW 3.3 (safe: the FPGA answers a
#     duplicate without re-programming); old FW -> no retransmission (two
#     requests would toggle the request line twice and cancel each other)
#   * pages that are all 0xFF are skipped (the slot is erased already)
#   * read-back verification of every page (FW 3.3 'RPD' command)
#   * device discovery by broadcast (FW 3.3)
#   * slot 0 (bootloader itself) needs explicit confirmation
#   * settings (IP, last folder) in ~/.odyssey_bootloader.json instead of
#     rewriting the script itself
#

from __future__ import print_function, division

__version__ = "3.3"

import os, sys, time, socket, struct, threading, json, random, select, traceback

try:                                    # Python 2
    import Tkinter as tk
    import ttk
    import tkFileDialog as filedialog
    import tkMessageBox as messagebox
    import Queue as queue
except ImportError:                     # Python 3
    import tkinter as tk
    from tkinter import ttk, filedialog, messagebox
    import queue

PY2 = sys.version_info[0] == 2

UDP_PORT    = 50000
PAGE        = 256
SLOT_PAGES  = 8192                      # 2 MB per slot
SLOT_BASE   = {0: 0, 1: 8192, 2: 16384, 3: 24576}
CFG_FILE    = os.path.join(os.path.expanduser('~'), '.odyssey_bootloader.json')

CAP_RPD, CAP_SAFE_RETX, CAP_DISCOVERY, CAP_FCS, CAP_BCAST_WIP = 0x01, 0x02, 0x04, 0x08, 0x10

# Timings (flash on the board: Winbond W25Q64JV; values follow the proven
# Micron loader: 1 s per page request + 1 s for a late answer, 3 tries)
#   W25Q64JV: 64 KB block erase typ 0.15 s / max 2 s, page program typ 0.4 ms / max 3 ms
T_MAC, N_MAC   = 0.5, 5
T_WIP, N_WIP   = 0.5, 5
T_ERS_POLL     = 2.0
T_ERS_TOTAL    = 75.0           # failure limit only (32 x 2 s max + margin); typ ~5 s
T_WPD, N_WPD   = 1.0, 3         # FW 3.3: a late answer to an earlier try is accepted too
T_WPD_LEGACY   = 2.0            # FW <= 3.2: one request, 1 s + 1 s late answer
T_RPD, N_RPD   = 1.0, 3
WRITE_ATTEMPTS = 2              # whole procedure (erase + write + verify), like the Micron loader


def resource_path(relative_path):
    if hasattr(sys, '_MEIPASS'):
        return os.path.join(sys._MEIPASS, relative_path)
    return os.path.join(os.path.abspath("."), relative_path)


class DeviceError(Exception):
    pass


class Cancelled(Exception):
    pass


# ----------------------------------------------------------------------------
#   Protocol
# ----------------------------------------------------------------------------
class Bootloader(object):

    def __init__(self, log=print):
        self.log = log
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        try:
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        except socket.error:
            pass
        self.sock.bind(('', 0))
        self.ip = None
        self.caps = 0
        self.version = ''
        self.mac = ''
        self.sn = random.randint(0, 255)
        self.cancel = threading.Event()

    def close(self):
        try:
            self.sock.close()
        except Exception:
            pass

    # -- helpers -------------------------------------------------------------
    def next_sn(self):
        self.sn = (self.sn + 1) & 0xFF
        return self.sn

    @staticmethod
    def packet(cmd, sn, page=0, tail=b'', data=b''):
        b = bytearray(cmd.encode('ascii'))
        b.append(sn & 0xFF)
        b += struct.pack('!H', page & 0xFFFF)
        b += bytearray(32 - len(b) - len(tail))
        b += bytearray(tail)
        b += bytearray(data)
        return bytes(b)

    def drain(self):
        while True:
            r, _, _ = select.select([self.sock], [], [], 0)
            if not r:
                return
            try:
                self.sock.recvfrom(4096)
            except socket.error:
                return

    def wait_reply(self, cmd, sn, page, timeout, src_ip=None):
        """Wait for a matching reply; returns bytearray or None on timeout."""
        end = time.time() + timeout
        while True:
            if self.cancel.is_set():
                raise Cancelled()
            left = end - time.time()
            if left <= 0:
                return None
            r, _, _ = select.select([self.sock], [], [], min(left, 0.2))
            if not r:
                continue
            try:
                data, addr = self.sock.recvfrom(4096)
            except socket.error:
                continue
            data = bytearray(data)
            if len(data) < 32 or addr[1] != UDP_PORT:
                continue
            if src_ip is not None and addr[0] != src_ip:
                continue
            if bytes(data[0:3]) != cmd.encode('ascii') or data[3] != sn:
                continue
            if page is not None and struct.unpack('!H', bytes(data[4:6]))[0] != page:
                continue
            return data

    def transact(self, cmd, page=None, tail=b'', data=b'', timeout=0.3, tries=5, sn=None):
        if sn is None:
            sn = self.next_sn()
        pkt = self.packet(cmd, sn, page or 0, tail, data)
        self.drain()
        for n in range(tries):
            self.sock.sendto(pkt, (self.ip, UDP_PORT))
            rep = self.wait_reply(cmd, sn, page, timeout, self.ip)
            if rep is not None:
                return rep
            if n + 1 < tries:
                self.log('%s SN=%d page=%s: no reply, retry %d/%d' % (cmd, sn, page, n + 1, tries - 1))
        raise DeviceError('%s: device did not answer' % cmd)

    # -- commands ------------------------------------------------------------
    def connect(self, ip):
        self.ip = ip
        rep = self.transact('MAC', None, timeout=T_MAC, tries=N_MAC)
        self.mac = ':'.join('%02X' % x for x in rep[26:32])
        ver = bytes(rep[4:7])
        if all(0x20 <= c < 0x7F for c in bytearray(ver)):
            self.version = ver.decode('ascii')
            self.caps = rep[7]
        else:                           # BL <= 3.2 sends zeroes here
            self.version = '<= 3.2'
            self.caps = 0
        return self.mac, self.version, self.caps

    @staticmethod
    def bcast_targets():
        targets = ['255.255.255.255']
        try:                            # also the subnet broadcast of the main interface
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(('192.0.2.1', 9))
            my = s.getsockname()[0]
            s.close()
            targets.append('.'.join(my.split('.')[:3] + ['255']))
        except Exception:
            pass
        return targets

    def discover(self, wait=1.0):
        """Broadcast 'MAC'. The board answers directly to our MAC/IP, so it is
        found even when its static IP is in another subnet.
        Returns {ip: (mac_text, version_bytes, caps, mac_bytes)}."""
        sn = self.next_sn()
        pkt = self.packet('MAC', sn)
        self.drain()
        for t in self.bcast_targets():
            try:
                self.sock.sendto(pkt, (t, UDP_PORT))
            except socket.error:
                pass
        found = {}
        end = time.time() + wait
        while time.time() < end:
            r, _, _ = select.select([self.sock], [], [], 0.1)
            if not r:
                continue
            data, addr = self.sock.recvfrom(4096)
            data = bytearray(data)
            if len(data) >= 32 and bytes(data[0:3]) == b'MAC' and data[3] == sn:
                found[addr[0]] = (':'.join('%02X' % x for x in data[26:32]), bytes(data[4:7]),
                                  data[7], bytes(data[26:32]))
        return found

    def write_ip_bcast(self, mac, new_ip):
        """FW 3.3: change the static IP of the board with this MAC from any
        subnet ('WIP' to the Ethernet broadcast, MAC in bytes 22..27)."""
        ipb = socket.inet_aton(new_ip)
        sn = self.next_sn()
        pkt = self.packet('WIP', sn, 0, tail=bytes(bytearray(mac)) + ipb)
        self.drain()
        for n in range(N_WIP):
            for t in self.bcast_targets():
                try:
                    self.sock.sendto(pkt, (t, UDP_PORT))
                except socket.error:
                    pass
            end = time.time() + T_WIP
            while time.time() < end:
                rep = self.wait_reply('WIP', sn, None, end - time.time(), None)
                if rep is not None and bytes(rep[28:32]) == ipb:
                    return
        raise DeviceError('WIP: device did not answer')

    def write_ip(self, new_ip):
        tail = socket.inet_aton(new_ip)
        self.transact('WIP', None, tail=tail, timeout=T_WIP, tries=N_WIP)

    def reset(self):
        self.drain()
        self.sock.sendto(self.packet('RES', 0), (self.ip, UDP_PORT))

    @property
    def safe_retx(self):
        return bool(self.caps & CAP_SAFE_RETX)

    def erase_slot(self, slot):
        page = SLOT_BASE[slot]
        sn = self.next_sn()
        pkt = self.packet('ERS', sn, page)
        self.drain()
        t0 = time.time()
        if self.safe_retx:
            while time.time() - t0 < T_ERS_TOTAL:
                self.sock.sendto(pkt, (self.ip, UDP_PORT))
                if self.wait_reply('ERS', sn, page, T_ERS_POLL, self.ip) is not None:
                    return time.time() - t0
                self.log('erase: %.0f s ...' % (time.time() - t0))
        else:
            self.sock.sendto(pkt, (self.ip, UDP_PORT))
            if self.wait_reply('ERS', sn, page, T_ERS_TOTAL, self.ip) is not None:
                return time.time() - t0
        raise DeviceError('erase: device did not answer within %d s' % T_ERS_TOTAL)

    def write_page(self, page, data):
        if self.safe_retx:
            self.transact('WPD', page, data=data, timeout=T_WPD, tries=N_WPD)
        else:
            self.transact('WPD', page, data=data, timeout=T_WPD_LEGACY, tries=1)

    def read_page(self, page):
        rep = self.transact('RPD', page, timeout=T_RPD, tries=N_RPD)
        if len(rep) < 32 + PAGE:
            raise DeviceError('RPD: short reply')
        return bytes(rep[32:32 + PAGE])


# ----------------------------------------------------------------------------
#   File checks
# ----------------------------------------------------------------------------
def load_rbf(path, slot):
    name = os.path.basename(path)
    is_bl = name.lower().startswith('bootloader')
    if slot == 0 and not is_bl:
        raise ValueError('Slot 0 holds the bootloader.\nPlease select a Bootloader*.rbf file.')
    if slot != 0 and is_bl:
        raise ValueError('Slots 1-3 hold radio firmware.\nA Bootloader file cannot be written there.')
    with open(path, 'rb') as f:
        rbf = bytearray(f.read())
    if len(rbf) < 64 * 1024:
        raise ValueError('The file is too small (%d bytes) for an EP4CE55 configuration.' % len(rbf))
    if len(rbf) > SLOT_PAGES * PAGE:
        raise ValueError('The file is too big for a 2 MB slot (%d bytes).' % len(rbf))
    if len(rbf) % PAGE:
        rbf += bytearray(b'\xff' * (PAGE - len(rbf) % PAGE))
    return bytes(rbf)


# ----------------------------------------------------------------------------
#   GUI
# ----------------------------------------------------------------------------
class App(object):

    def __init__(self, root):
        self.root = root
        self.q = queue.Queue()
        self.worker = None
        self.cfg = self.load_cfg()
        self.bl = Bootloader(log=self.log_threadsafe)
        self.connected = False
        self.discovered = {}

        root.title('Odyssey Bootloader %s' % __version__)
        try:
            root.iconbitmap(resource_path('myicon.ico'))
        except Exception:
            pass
        root.protocol('WM_DELETE_WINDOW', self.quit)
        root.resizable(False, False)

        pad = dict(padx=6, pady=3)
        top = ttk.LabelFrame(root, text=' Device ')
        top.grid(row=0, column=0, sticky='ew', **pad)
        ttk.Label(top, text='IP address:').grid(row=0, column=0, sticky='w', **pad)
        self.ip_var = tk.StringVar(value=self.cfg.get('ip', '192.168.2.160'))
        ttk.Entry(top, textvariable=self.ip_var, width=16).grid(row=0, column=1, **pad)
        self.btn_test = ttk.Button(top, text='Connect', command=self.on_connect)
        self.btn_test.grid(row=0, column=2, **pad)
        self.btn_disc = ttk.Button(top, text='Discover', command=self.on_discover)
        self.btn_disc.grid(row=0, column=3, **pad)
        self.info_var = tk.StringVar(value='Put the radio in bootloader mode, then Connect.')
        ttk.Label(top, textvariable=self.info_var, width=62).grid(row=1, column=0, columnspan=4, sticky='w', **pad)

        ipf = ttk.LabelFrame(root, text=' Change IP ')
        ipf.grid(row=1, column=0, sticky='ew', **pad)
        ttk.Label(ipf, text='New IP:').grid(row=0, column=0, **pad)
        self.newip_var = tk.StringVar(value=self.ip_var.get())
        ttk.Entry(ipf, textvariable=self.newip_var, width=16).grid(row=0, column=1, **pad)
        self.btn_wip = ttk.Button(ipf, text='Write IP', command=self.on_write_ip)
        self.btn_wip.grid(row=0, column=2, **pad)

        fw = ttk.LabelFrame(root, text=' Firmware ')
        fw.grid(row=2, column=0, sticky='ew', **pad)
        self.slot_var = tk.IntVar(value=1)
        for s in range(4):
            ttk.Radiobutton(fw, text=('Slot 0 (bootloader)' if s == 0 else 'Slot %d' % s),
                            variable=self.slot_var, value=s).grid(row=0, column=s, **pad)
        self.file_var = tk.StringVar(value='')
        ttk.Entry(fw, textvariable=self.file_var, width=58).grid(row=1, column=0, columnspan=3, **pad)
        ttk.Button(fw, text='Browse', command=self.on_browse).grid(row=1, column=3, **pad)
        self.verify_var = tk.IntVar(value=1)
        self.chk_verify = ttk.Checkbutton(fw, text='Verify after writing', variable=self.verify_var)
        self.chk_verify.grid(row=2, column=0, sticky='w', **pad)
        self.btn_write = ttk.Button(fw, text='Write FW', command=self.on_write)
        self.btn_write.grid(row=2, column=2, **pad)
        self.btn_verify = ttk.Button(fw, text='Verify only', command=self.on_verify)
        self.btn_verify.grid(row=2, column=3, **pad)

        st = ttk.Frame(root)
        st.grid(row=3, column=0, sticky='ew', **pad)
        self.prog = ttk.Progressbar(st, length=470, mode='determinate')
        self.prog.grid(row=0, column=0, columnspan=3, **pad)
        self.status_var = tk.StringVar(value='')
        ttk.Label(st, textvariable=self.status_var, width=62).grid(row=1, column=0, columnspan=3, sticky='w', **pad)
        self.btn_cancel = ttk.Button(st, text='Cancel', command=self.on_cancel, state='disabled')
        self.btn_cancel.grid(row=2, column=0, **pad)
        self.btn_reset = ttk.Button(st, text='Reset device', command=self.on_reset)
        self.btn_reset.grid(row=2, column=1, **pad)
        ttk.Button(st, text='Quit', command=self.quit).grid(row=2, column=2, **pad)

        self.logbox = tk.Text(root, width=72, height=8, state='disabled')
        self.logbox.grid(row=4, column=0, **pad)

        self.set_connected(False)
        self.root.after(50, self.poll)

    # -- settings --------------------------------------------------------------
    def load_cfg(self):
        try:
            with open(CFG_FILE) as f:
                return json.load(f)
        except Exception:
            return {}

    def save_cfg(self):
        try:
            with open(CFG_FILE, 'w') as f:
                json.dump(self.cfg, f)
        except Exception:
            pass

    # -- thread plumbing -------------------------------------------------------
    def log_threadsafe(self, msg):
        self.q.put(('log', msg))

    def poll(self):
        try:
            while True:
                kind, arg = self.q.get_nowait()
                if kind == 'log':
                    self.log(arg)
                elif kind == 'status':
                    self.status_var.set(arg)
                elif kind == 'progress':
                    self.prog['value'] = arg
                elif kind == 'done':
                    self.job_done(*arg)
        except queue.Empty:
            pass
        self.root.after(50, self.poll)

    def log(self, msg):
        self.logbox['state'] = 'normal'
        self.logbox.insert('end', time.strftime('%H:%M:%S ') + msg + '\n')
        self.logbox.see('end')
        self.logbox['state'] = 'disabled'

    def busy(self):
        return self.worker is not None and self.worker.is_alive()

    def run_job(self, name, fn):
        if self.busy():
            return
        self.bl.cancel.clear()
        self.prog['value'] = 0
        for b in (self.btn_test, self.btn_disc, self.btn_wip, self.btn_write, self.btn_verify, self.btn_reset):
            b['state'] = 'disabled'
        self.btn_cancel['state'] = 'normal'

        def body():
            try:
                res = fn()
                self.q.put(('done', (name, True, res)))
            except Cancelled:
                self.q.put(('done', (name, False, 'cancelled')))
            except (DeviceError, ValueError, socket.error, IOError, OSError) as e:
                self.q.put(('done', (name, False, str(e))))
            except Exception as e:
                self.q.put(('log', traceback.format_exc()))
                self.q.put(('done', (name, False, repr(e))))
        self.worker = threading.Thread(target=body)
        self.worker.daemon = True
        self.worker.start()

    def job_done(self, name, ok, res):
        self.btn_cancel['state'] = 'disabled'
        self.btn_test['state'] = 'normal'
        self.btn_disc['state'] = 'normal'
        if name == 'connect':
            self.set_connected(ok)
            if ok:
                mac, ver, caps = res
                feat = []
                if caps & CAP_RPD: feat.append('verify')
                if caps & CAP_SAFE_RETX: feat.append('safe retry')
                if not caps: feat.append('legacy mode - consider updating the bootloader')
                self.info_var.set('Found: MAC %s, bootloader %s (%s)' % (mac, ver, ', '.join(feat)))
                self.cfg['ip'] = self.ip_var.get().strip(); self.save_cfg()
                self.chk_verify['state'] = 'normal' if caps & CAP_RPD else 'disabled'
                if not caps & CAP_RPD:
                    self.verify_var.set(0)
            else:
                self.info_var.set('Device not found: %s' % res)
                messagebox.showerror('Connect', 'Device was not found.\n\nCheck the IP address, the cable and that the radio\n'
                                     'is in bootloader mode. "Discover" can find it.\n\n%s' % res)
            return
        self.set_connected(self.connected)
        if name == 'discover':
            if ok and res:
                self.discovered = res
                lines = ['%s   MAC %s   ver %s' % (ip, m, v.decode('ascii', 'replace') if v.strip(b'\0') else '<=3.2')
                         for ip, (m, v, c, mb) in sorted(res.items())]
                self.log('Discovered:\n  ' + '\n  '.join(lines))
                self.ip_var.set(sorted(res.keys())[0])
                if any(c & CAP_BCAST_WIP for (m, v, c, mb) in res.values()):
                    self.btn_wip['state'] = 'normal'
                self.status_var.set('Found %d device(s). Connect (same subnet) or Write IP (any subnet).' % len(res))
            else:
                self.status_var.set('Nothing found (needs bootloader 3.3; check the PC firewall).')
            return
        if ok:
            self.status_var.set(res if isinstance(res, str) else 'Done.')
            if name == 'write_ip':
                self.discovered = {}
                self.ip_var.set(self.newip_var.get().strip())
                self.cfg['ip'] = self.ip_var.get(); self.save_cfg()
                self.set_connected(False)
                messagebox.showinfo('Write IP', 'The new IP address was written.\n'
                                    'Adjust the PC Ethernet settings if needed, then Connect again.')
            if name == 'write':
                if messagebox.askyesno('Write FW', res + '\n\nReset the device now?'):
                    self.bl.reset()
        else:
            self.status_var.set('ERROR: %s' % res)
            if res != 'cancelled':
                messagebox.showerror('Error', res)

    def set_connected(self, ok):
        self.connected = ok
        st = 'normal' if ok else 'disabled'
        for b in (self.btn_wip, self.btn_write, self.btn_verify, self.btn_reset):
            b['state'] = st
        if ok and not (self.bl.caps & CAP_RPD):
            self.btn_verify['state'] = 'disabled'

    # -- validation ------------------------------------------------------------
    @staticmethod
    def valid_ip(s):
        p = s.strip().split('.')
        return len(p) == 4 and all(x.isdigit() and 0 <= int(x) <= 255 for x in p)

    # -- actions ---------------------------------------------------------------
    def on_connect(self):
        ip = self.ip_var.get().strip()
        if not self.valid_ip(ip):
            messagebox.showerror('IP', 'Enter an address like 192.168.2.160')
            return
        self.status_var.set('Connecting to %s ...' % ip)
        self.run_job('connect', lambda: self.bl.connect(ip))

    def on_discover(self):
        self.status_var.set('Searching ...')
        self.run_job('discover', lambda: self.bl.discover(1.5))

    def on_write_ip(self):
        new = self.newip_var.get().strip()
        if not self.valid_ip(new) or new in ('0.0.0.0', '255.255.255.255'):
            messagebox.showerror('IP', 'Enter an address like 192.168.2.160')
            return
        if self.connected:
            self.run_job('write_ip', lambda: self.bl.write_ip(new) or 'IP written.')
            return
        d = self.discovered.get(self.ip_var.get().strip())
        if d is None or not (d[2] & CAP_BCAST_WIP):
            messagebox.showerror('Write IP', 'Connect to the device, or find it with Discover first.')
            return
        mac_text, mac = d[0], d[3]
        if not messagebox.askyesno('Write IP', 'Change the IP of the board\nMAC %s  (%s)\nto %s ?'
                                   % (mac_text, self.ip_var.get().strip(), new)):
            return
        self.run_job('write_ip', lambda: self.bl.write_ip_bcast(mac, new) or 'IP written.')

    def on_browse(self):
        path = filedialog.askopenfilename(title='Select file', initialdir=self.cfg.get('dir', os.curdir),
                                          filetypes=[('rbf files', '*.rbf'), ('all files', '*.*')])
        if path:
            self.file_var.set(path)
            self.cfg['dir'] = os.path.dirname(path); self.save_cfg()

    def on_cancel(self):
        self.bl.cancel.set()

    def on_reset(self):
        self.bl.reset()
        self.status_var.set('Reset command sent.')
        self.set_connected(False)

    def get_image(self):
        slot = self.slot_var.get()
        try:
            rbf = load_rbf(self.file_var.get(), slot)
        except (ValueError, IOError, OSError) as e:
            messagebox.showerror('File', str(e))
            return None, None
        return slot, rbf

    def on_write(self):
        slot, rbf = self.get_image()
        if rbf is None:
            return
        if slot == 0 and not messagebox.askyesno(
                'Slot 0', 'You are going to overwrite the BOOTLOADER itself.\n\n'
                'If this is interrupted (power, cable) the radio can only be\n'
                'recovered with a JTAG programmer.\n\nContinue?', icon='warning'):
            return
        verify = bool(self.verify_var.get()) and bool(self.bl.caps & CAP_RPD)
        self.run_job('write', lambda: self.job_write(slot, rbf, verify))

    def on_verify(self):
        slot, rbf = self.get_image()
        if rbf is None:
            return
        self.run_job('verify', lambda: self.job_verify(slot, rbf, 0, 100))

    # -- worker jobs (background thread) ---------------------------------------
    def job_write(self, slot, rbf, verify):
        for attempt in range(WRITE_ATTEMPTS):
            try:
                return self.job_write_once(slot, rbf, verify)
            except DeviceError as e:
                if attempt == WRITE_ATTEMPTS - 1:
                    raise
                self.q.put(('log', '%s -> retrying from the erase' % e))
                time.sleep(1.0)

    def job_write_once(self, slot, rbf, verify):
        q = self.q
        pages = len(rbf) // PAGE
        base = SLOT_BASE[slot]
        q.put(('status', 'Erasing slot %d ...' % slot))
        q.put(('progress', 0))
        t = self.bl.erase_slot(slot)
        q.put(('log', 'slot %d erased in %.1f s' % (slot, t)))
        w_end = 70 if verify else 100
        t0 = time.time(); written = 0
        for i in range(pages):
            chunk = rbf[i * PAGE:(i + 1) * PAGE]
            if chunk.count(b'\xff') != PAGE:          # all-0xFF page: already erased
                self.bl.write_page(base + i, chunk)
                written += 1
            if i % 32 == 0 or i == pages - 1:
                q.put(('progress', 2 + (w_end - 2) * (i + 1) / pages))
                q.put(('status', 'Writing slot %d: page %d / %d' % (slot, i + 1, pages)))
        q.put(('log', '%d pages written (%d skipped as blank) in %.1f s'
               % (written, pages - written, time.time() - t0)))
        if verify:
            self.job_verify(slot, rbf, w_end, 100)
            return 'Firmware written to slot %d and verified.' % slot
        return 'Firmware written to slot %d.' % slot

    def job_verify(self, slot, rbf, p0, p1):
        q = self.q
        pages = len(rbf) // PAGE
        base = SLOT_BASE[slot]
        t0 = time.time()
        for i in range(pages):
            got = self.bl.read_page(base + i)
            want = rbf[i * PAGE:(i + 1) * PAGE]
            if got != want:
                pos = [k for k in range(PAGE) if bytearray(got)[k] != bytearray(want)[k]]
                raise DeviceError('VERIFY FAILED at page %d (slot offset 0x%06X), %d byte(s) differ.\n'
                                  'Write the slot again.' % (base + i, i * PAGE + pos[0], len(pos)))
            if i % 32 == 0 or i == pages - 1:
                q.put(('progress', p0 + (p1 - p0) * (i + 1) / pages))
                q.put(('status', 'Verifying slot %d: page %d / %d' % (slot, i + 1, pages)))
        q.put(('log', 'slot %d verified, %d pages, %.1f s' % (slot, pages, time.time() - t0)))
        return 'Slot %d matches the file.' % slot

    def quit(self):
        if self.busy():
            if not messagebox.askyesno('Quit', 'An operation is running. Abort it and quit?'):
                return
            self.bl.cancel.set()
            self.worker.join(2.0)
        self.bl.close()
        self.root.destroy()


def main():
    root = tk.Tk()
    App(root)
    root.mainloop()


if __name__ == '__main__':
    main()
