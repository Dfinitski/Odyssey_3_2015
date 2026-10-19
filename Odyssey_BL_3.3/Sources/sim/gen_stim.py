#!/usr/bin/env python3
"""Build stim.txt for tb.v and the expected data for check.py."""
import struct, zlib, random, json

random.seed(1)
PC_MAC  = bytes.fromhex('a0b1c2d3e4f5')
PC_IP   = bytes([192,168,2,10])
DEV_IP  = bytes([192,168,2,160])
BCAST   = b'\xff'*6
OTHER   = bytes.fromhex('020000000001')

def csum(b):
    if len(b) % 2: b += b'\0'
    s = sum(struct.unpack('!%dH' % (len(b)//2), b))
    while s >> 16: s = (s & 0xffff) + (s >> 16)
    return (~s) & 0xffff

def eth(dst, src, typ, payload, pad=True):
    f = dst + src + struct.pack('!H', typ) + payload
    if pad and len(f) < 60: f += b'\0'*(60-len(f))
    return f

def fcs(f):
    return f + struct.pack('<I', zlib.crc32(f) & 0xffffffff)

def ipv4(src, dst, proto, payload, ident=0x1234, flags=0x4000):
    h = struct.pack('!BBHHHBBH4s4s', 0x45, 0, 20+len(payload), ident, flags, 64, proto, 0, src, dst)
    h = h[:10] + struct.pack('!H', csum(h)) + h[12:]
    return h + payload

def udp(sport, dport, data):
    return struct.pack('!HHHH', sport, dport, 8+len(data), 0) + data

def cmd(c, sn, addr=0, tail=b''):
    b = c.encode() + bytes([sn]) + struct.pack('!H', addr)
    b += b'\0' * (32 - len(b) - len(tail)) + tail
    return b

lines = []
exp = []          # list of expectations, in order
txn = 0           # tx frames expected so far

def send(frame, preamble=7, corrupt=False):
    f = fcs(frame)
    if corrupt:
        f = bytearray(f); f[100] ^= 0x01; f = bytes(f)
    w = b'\x55'*preamble + b'\xd5' + f
    lines.append('F %d %s' % (len(w), ' '.join('%02x' % x for x in w)))

def wait(n): lines.append('W %d' % n)
def delay(ns): lines.append('D %d' % ns)
def mark(n): lines.append('M %d' % n)

def udp_to_dev(payload, dst_mac=BCAST, dst_ip=DEV_IP, sport=40000):
    return eth(dst_mac, PC_MAC, 0x0800, ipv4(PC_IP, dst_ip, 17, udp(sport, 50000, payload)))

# 0. gratuitous ARP comes by itself
txn += 2; exp.append(('garp',)); exp.append(('garp',)); wait(txn); mark(0)

# 1. ARP request for another IP -> must be ignored; then for our IP -> reply
arp_other = eth(BCAST, PC_MAC, 0x0806, struct.pack('!HHBBH6s4s6s4s', 1, 0x0800, 6, 4, 1, PC_MAC, PC_IP, b'\0'*6, bytes([192,168,2,99])))
send(arp_other); delay(3000)
arp_me = eth(BCAST, PC_MAC, 0x0806, struct.pack('!HHBBH6s4s6s4s', 1, 0x0800, 6, 4, 1, PC_MAC, PC_IP, b'\0'*6, DEV_IP))
send(arp_me); txn += 1; exp.append(('arp',)); wait(txn); mark(1)

# 2. ping, 32 bytes payload (Windows default), shortened preamble (3 bytes)
icmp_data = bytes(range(0x61, 0x61+32))
icmp = struct.pack('!BBHHH', 8, 0, 0, 0x0001, 0x0042) + icmp_data
icmp = icmp[:2] + struct.pack('!H', csum(icmp)) + icmp[4:]
send(eth(BCAST, PC_MAC, 0x0800, ipv4(PC_IP, DEV_IP, 1, icmp)), preamble=3)
txn += 1; exp.append(('icmp', icmp_data.hex())); wait(txn); mark(2)

# 3. MAC request (unicast IP, stale MAC) and broadcast discovery
send(udp_to_dev(cmd('MAC', 0), dst_mac=OTHER)); txn += 1; exp.append(('udp', 'MAC', 0)); wait(txn)
send(udp_to_dev(cmd('MAC', 7), dst_ip=bytes([255,255,255,255]))); txn += 1; exp.append(('udp', 'MAC', 7)); wait(txn); mark(3)

# 4. ERS slot 1, duplicate ERS while busy must be ignored
send(udp_to_dev(cmd('ERS', 0, 8192))); delay(20000)
send(udp_to_dev(cmd('ERS', 0, 8192)))
txn += 1; exp.append(('udp', 'ERS', 0, 8192)); wait(txn); mark(4)
# illegal erase address -> ignored
send(udp_to_dev(cmd('ERS', 1, 8193))); delay(50000); mark(41)

# 5. pages
pages = {}
sn = 1
for p in range(3):
    data = bytes(random.getrandbits(8) for _ in range(256))
    pages[8192+p] = data
    if p == 1:
        # first a corrupted copy (bad FCS) -> no reply, nothing written
        send(udp_to_dev(cmd('WPD', sn, 8192+p) + bytes(256)), corrupt=True); delay(30000)
    send(udp_to_dev(cmd('WPD', sn, 8192+p) + data))
    txn += 1; exp.append(('udp', 'WPD', sn, 8192+p)); wait(txn)
    sn += 1
mark(5)
# retransmission of the last WPD (reply lost) -> re-ack, flash untouched
send(udp_to_dev(cmd('WPD', sn-1, 8192+2) + pages[8192+2]))
txn += 1; exp.append(('udp', 'WPD', sn-1, 8192+2)); wait(txn); mark(6)

# 5b. ARP immediately followed by a WPD (WPD must not be dropped), and a WPD to an
#     aliased page (>= 32768 -> would hit slot 0) which must be ignored
data = bytes(random.getrandbits(8) for _ in range(256)); pages[8195] = data
send(arp_me)
send(udp_to_dev(cmd('WPD', sn, 8195) + data))
txn += 2; exp.append(('arp',)); exp.append(('udp', 'WPD', sn, 8195)); wait(txn); sn += 1
send(udp_to_dev(cmd('WPD', sn, 32768 + 5) + data)); delay(30000); sn += 1
mark(61)

# 6. read back
for p in range(4):
    send(udp_to_dev(cmd('RPD', sn, 8192+p)))
    txn += 1; exp.append(('rpd', sn, 8192+p, pages[8192+p].hex())); wait(txn); sn += 1
mark(7)

# 7. WIP
send(udp_to_dev(cmd('WIP', sn, 0, bytes([192,168,2,161]))))
txn += 1; exp.append(('udp', 'WIP', sn)); wait(txn); mark(8)
delay(2_000_000)
sn += 1
# 8. broadcast WIP (from another subnet, PC 10.1.1.10): wrong MAC -> ignored,
#    IP 0.0.0.0 -> ignored, our MAC -> accepted; reply comes from the old IP .161
DEVMAC = bytes.fromhex('0004a300ffff')
def bwip(mac, ip, sn):
    p = cmd('WIP', sn, 0, mac + bytes(ip))
    return eth(BCAST, PC_MAC, 0x0800, ipv4(bytes([10,1,1,10]), bytes([255,255,255,255]), 17, udp(40000, 50000, p)))
send(bwip(bytes.fromhex('0004a3001111'), [10,1,1,20], sn)); delay(50000)
send(bwip(DEVMAC, [0,0,0,0], sn)); delay(50000)
send(bwip(DEVMAC, [10,1,1,20], sn))
txn += 1; exp.append(('udp', 'WIP', sn, [192,168,2,161], [10,1,1,10], [10,1,1,20])); wait(txn); mark(9)
delay(2_000_000)

# 8. frame with RX_ER?  (not generated) ; long jumbo ignored
lines.append('Q')
open('stim.txt', 'w').write('\n'.join(lines) + '\n')
json.dump({'exp': exp, 'pages': {str(k): v.hex() for k, v in pages.items()},
           'pc_mac': PC_MAC.hex(), 'pc_ip': list(PC_IP), 'dev_ip': list(DEV_IP)}, open('exp.json', 'w'))
print('frames expected', txn)
