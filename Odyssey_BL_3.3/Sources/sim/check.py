#!/usr/bin/env python3
import json, struct, zlib, sys

E = json.load(open('exp.json'))
exp = E['exp']; pc_mac = bytes.fromhex(E['pc_mac']); dev_ip = bytes(E['dev_ip']); pc_ip = bytes(E['pc_ip'])
DEV_MAC = bytes.fromhex('0004a300ffff')
errs = 0
def bad(msg):
    global errs; errs += 1; print('FAIL:', msg)

def csum_ok(b):
    if len(b) % 2: b += b'\0'
    s = sum(struct.unpack('!%dH' % (len(b)//2), b))
    while s >> 16: s = (s & 0xffff) + (s >> 16)
    return s == 0xffff

frames = []
for ln in open('tx_frames.txt'):
    p = ln.split()
    if p[0] == 'M': continue
    frames.append(bytes.fromhex(p[-1]))

print('tx frames: %d, expected %d' % (len(frames), len(exp)))
if len(frames) != len(exp): bad('frame count')

for i, (f, e) in enumerate(zip(frames, exp)):
    tag = '#%d %s' % (i, e[0] if e[0] != 'udp' else 'udp ' + e[1])
    if f[:8] != b'\x55'*7 + b'\xd5': bad(tag + ' preamble'); continue
    fr = f[8:]
    body, rx_fcs = fr[:-4], fr[-4:]
    if struct.pack('<I', zlib.crc32(body) & 0xffffffff) != rx_fcs: bad(tag + ' FCS'); continue
    if len(fr) < 64: bad(tag + ' runt %d' % len(fr))
    dst, src, typ = body[0:6], body[6:12], struct.unpack('!H', body[12:14])[0]
    if src != DEV_MAC: bad(tag + ' src mac %s' % src.hex())
    if e[0] in ('garp', 'arp'):
        if typ != 0x0806: bad(tag + ' type'); continue
        h, pr, hs, ps, op, smac, sip, tmac, tip = struct.unpack('!HHBBH6s4s6s4s', body[14:42])
        if e[0] == 'garp':
            ok = dst == b'\xff'*6 and op == 1 and sip == dev_ip and tip == dev_ip and smac == DEV_MAC
        else:
            ok = dst == pc_mac and op == 2 and sip == dev_ip and smac == DEV_MAC and tmac == pc_mac and tip == pc_ip
        if not ok: bad(tag + ' content ' + body[:42].hex())
        else: print('ok ', tag)
        continue
    if typ != 0x0800: bad(tag + ' type'); continue
    iph = body[14:34]
    if not csum_ok(iph): bad(tag + ' IP checksum')
    tot = struct.unpack('!H', iph[2:4])[0]
    if iph[9] == 1:
        icmp = body[34:14+tot]
        if not csum_ok(icmp): bad(tag + ' ICMP checksum')
        if icmp[0] != 0 or icmp[8:].hex() != e[1]: bad(tag + ' ICMP content')
        else: print('ok ', tag, 'id/seq', icmp[4:8].hex())
        continue
    u = body[34:14+tot]
    sport, dport, ulen, uc = struct.unpack('!HHHH', u[:8])
    data = u[8:]
    src_exp = bytes(e[3]) if (e[0] == 'udp' and e[1] == 'WIP' and len(e) > 3) else dev_ip
    dst_exp = bytes(e[4]) if (e[0] == 'udp' and e[1] == 'WIP' and len(e) > 4) else pc_ip
    if e[0] == 'udp' and e[1] == 'WIP' and len(e) > 5 and list(data[28:32]) != e[5]: bad(tag + ' new IP')
    if dst != pc_mac or iph[16:20] != dst_exp or iph[12:16] != src_exp or sport != 50000 or dport != 40000:
        bad(tag + ' addressing')
    c = data[:3].decode(errors='replace'); sn = data[3]
    if e[0] == 'rpd':
        if c != 'RPD' or sn != e[1] or struct.unpack('!H', data[4:6])[0] != e[2]: bad(tag + ' header')
        elif data[32:].hex() != e[3]: bad(tag + ' DATA MISMATCH')
        else: print('ok ', tag, 'page', e[2], '256 bytes verified')
        continue
    if c != e[1] or sn != e[2]: bad(tag + ' cmd/sn %s %d' % (c, sn)); continue
    if c == 'MAC':
        print('ok ', tag, 'ver', data[4:7], 'caps %02x' % data[7], 'ip', list(data[22:26]), 'mac', data[26:32].hex())
        if data[26:32] != DEV_MAC or data[22:26] != dev_ip: bad(tag + ' MAC/IP payload')
    elif c in ('ERS', 'WPD'):
        if struct.unpack('!H', data[4:6])[0] != e[3]: bad(tag + ' addr')
        else: print('ok ', tag, 'addr', e[3])
    else:
        print('ok ', tag, list(data[28:32]))

# flash content (bit-reversed RBF bytes)
def rev(b): return int('{:08b}'.format(b)[::-1], 2)
words = []
for ln in open('flash_slot1.hex'):
    ln = ln.split('//')[0].strip()
    if ln and not ln.startswith('@'): words += [int(x, 16) for x in ln.split()]
for k, v in E['pages'].items():
    off = (int(k) - 8192) * 256
    want = [rev(b) for b in bytes.fromhex(v)]
    if words[off:off+256] != want: bad('flash page %s content' % k)
    else: print('ok  flash page', k, 'content (bit reversed) correct')
if words[4*256:5*256] != [0xff]*256: bad('page 8196 not erased')

print('RESULT:', 'PASS' if errs == 0 else 'FAIL (%d)' % errs)
sys.exit(1 if errs else 0)
