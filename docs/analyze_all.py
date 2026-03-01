#!/usr/bin/env python3
"""Quick analysis of decoded SDIO traces - extracts key patterns."""

import csv
import sys
import os
from collections import defaultdict, Counter

# Same column/decode helpers from decode_sdio.py
COL_SAMPLE = 0; COL_DAT0 = 1; COL_CLK = 2; COL_CMD = 3; COL_DAT3 = 4; COL_DAT2 = 5; COL_DAT1 = 6

ATA_CMDS = {
    0x20: "READ_SECTORS", 0x30: "WRITE_SECTORS", 0xEC: "IDENTIFY_DEVICE",
    0xEF: "SET_FEATURES", 0xE7: "FLUSH_CACHE", 0xE1: "IDLE_IMMEDIATE",
    0x91: "INITIALIZE_DEVICE_PARAMS", 0xC8: "READ_DMA", 0xCA: "WRITE_DMA",
    0xB0: "SMART", 0xC6: "SET_MULTIPLE_MODE", 0xC4: "READ_MULTIPLE",
    0xC5: "WRITE_MULTIPLE", 0xC2: "VENDOR_0xC2", 0xE0: "STANDBY_IMMEDIATE",
    0x00: "NOP", 0x25: "READ_DMA_EXT", 0x35: "WRITE_DMA_EXT",
}

def crc7(bits):
    crc = 0
    for bit in bits:
        crc_bit = (crc >> 6) & 1
        crc = ((crc << 1) | bit) & 0x7F
        if crc_bit: crc ^= 0x09
    return crc

def decode_cmd52_arg(arg):
    return {
        'rw': (arg >> 31) & 1, 'fn': (arg >> 28) & 0x7,
        'raw': (arg >> 27) & 1, 'reg_addr': (arg >> 9) & 0x1FFFF,
        'write_data': arg & 0xFF,
    }

def decode_cmd53_arg(arg):
    rw = (arg >> 31) & 1; fn = (arg >> 28) & 0x7
    block_mode = (arg >> 27) & 1; op_code = (arg >> 26) & 1
    reg_addr = (arg >> 9) & 0x1FFFF; byte_count = arg & 0x1FF
    if byte_count == 0: byte_count = 512 if not block_mode else 0
    return {'rw': rw, 'fn': fn, 'block_mode': block_mode, 'op_code': op_code,
            'reg_addr': reg_addr, 'byte_count': byte_count}

def decode_cmd52_resp(arg):
    flags = (arg >> 8) & 0xFF; data = arg & 0xFF
    state = (flags >> 4) & 0x3
    return {'flags': flags, 'data': data, 'state': state}

def fast_decode(csv_path, max_rows=None):
    """Decode a trace, return structured data."""
    print(f"Decoding {os.path.basename(csv_path)}...")
    
    clk = 0; cmd_phase = 'IDLE'; cmd_bits = []
    transactions = []
    pending_cmd = None
    cmd_count = defaultdict(int)
    rising_edges = 0; row_count = 0
    
    with open(csv_path, 'r') as f:
        reader = csv.reader(f)
        next(reader)  # skip header
        for row in reader:
            row_count += 1
            if max_rows and row_count > max_rows: break
            if row_count % 5000000 == 0:
                print(f"  {row_count/1e6:.0f}M rows...")
            
            sample = int(row[0].strip())
            new_clk = int(row[2].strip())
            new_cmd = int(row[3].strip())
            
            if new_clk == 1 and clk == 0:
                rising_edges += 1
                
                if cmd_phase == 'IDLE':
                    if new_cmd == 0:
                        cmd_phase = 'COLLECTING'
                        cmd_bits = [0]
                        cmd_start = sample
                elif cmd_phase == 'COLLECTING':
                    cmd_bits.append(new_cmd)
                    if len(cmd_bits) == 48:
                        direction = cmd_bits[1]
                        cmd_index = 0
                        for i in range(2, 8): cmd_index = (cmd_index << 1) | cmd_bits[i]
                        argument = 0
                        for i in range(8, 40): argument = (argument << 1) | cmd_bits[i]
                        
                        if direction == 1:  # Host
                            pending_cmd = {'sample': cmd_start, 'cmd': cmd_index, 'arg': argument}
                            cmd_count[cmd_index] += 1
                        else:  # Card response
                            if pending_cmd:
                                transactions.append({
                                    'sample': pending_cmd['sample'],
                                    'cmd': pending_cmd['cmd'],
                                    'arg': pending_cmd['arg'],
                                    'resp': argument,
                                })
                                pending_cmd = None
                        cmd_phase = 'IDLE'
            
            clk = new_clk
    
    print(f"  {row_count} rows, {rising_edges} edges, {len(transactions)} txns")
    print(f"  Cmds: {dict(cmd_count)}")
    return transactions, cmd_count

def analyze_trace(name, transactions, cmd_count):
    """Analyze a decoded trace and return findings."""
    findings = []
    findings.append(f"\n{'='*60}")
    findings.append(f"TRACE: {name}")
    findings.append(f"{'='*60}")
    findings.append(f"Total transactions: {len(transactions)}")
    findings.append(f"Command counts: {dict(cmd_count)}")
    
    # Count re-inits (CMD5 sequences)
    cmd5_count = cmd_count.get(5, 0)
    cmd3_count = cmd_count.get(3, 0)
    findings.append(f"SDIO re-inits: {cmd3_count} (CMD3 count)")
    
    # ATA commands
    ata_cmds = []
    ata_cmd_counter = Counter()
    for t in transactions:
        if t['cmd'] == 52:
            d = decode_cmd52_arg(t['arg'])
            if d['fn'] == 1 and d['reg_addr'] == 0x07 and d['rw']:
                ata_cmd = d['write_data']
                ata_name = ATA_CMDS.get(ata_cmd, f"UNKNOWN_0x{ata_cmd:02X}")
                ata_cmds.append((t['sample'], ata_cmd, ata_name))
                ata_cmd_counter[ata_name] += 1
    
    findings.append(f"\nATA commands issued: {len(ata_cmds)}")
    for name_k, count in ata_cmd_counter.most_common():
        findings.append(f"  {name_k}: {count}")
    
    # CMD53 analysis
    cmd53_reads = []; cmd53_writes = []
    for t in transactions:
        if t['cmd'] == 53:
            d = decode_cmd53_arg(t['arg'])
            if d['rw']:
                cmd53_writes.append(d)
            else:
                cmd53_reads.append(d)
    
    findings.append(f"\nCMD53 transfers: {len(cmd53_reads)} reads, {len(cmd53_writes)} writes")
    
    if cmd53_reads:
        read_counts = Counter(d['byte_count'] for d in cmd53_reads)
        read_block = Counter(d['block_mode'] for d in cmd53_reads)
        findings.append(f"  Read byte_count distribution: {dict(read_counts.most_common(10))}")
        findings.append(f"  Read block_mode: {dict(read_block)}")
    
    if cmd53_writes:
        write_counts = Counter(d['byte_count'] for d in cmd53_writes)
        write_block = Counter(d['block_mode'] for d in cmd53_writes)
        findings.append(f"  Write byte_count distribution: {dict(write_counts.most_common(10))}")
        findings.append(f"  Write block_mode: {dict(write_block)}")
    
    # Vendor cmd 0xC2 analysis
    vendor_cmds = []
    for i, t in enumerate(transactions):
        if t['cmd'] == 52:
            d = decode_cmd52_arg(t['arg'])
            if d['fn'] == 1 and d['reg_addr'] == 0x07 and d['rw'] and d['write_data'] == 0xC2:
                # Look backwards for FEAT and SECCOUNT writes
                feat = None; seccount = None
                for j in range(i-1, max(i-10, 0), -1):
                    if transactions[j]['cmd'] == 52:
                        d2 = decode_cmd52_arg(transactions[j]['arg'])
                        if d2['fn'] == 1 and d2['rw']:
                            if d2['reg_addr'] == 0x01: feat = d2['write_data']
                            elif d2['reg_addr'] == 0x02: seccount = d2['write_data']
                # Look forward for STATUS read and SECCOUNT read result
                status = None; result_seccount = None
                for j in range(i+1, min(i+20, len(transactions))):
                    if transactions[j]['cmd'] == 52:
                        d2 = decode_cmd52_arg(transactions[j]['arg'])
                        r = decode_cmd52_resp(transactions[j]['resp'])
                        if d2['fn'] == 1 and not d2['rw']:
                            if d2['reg_addr'] == 0x07 and status is None: status = r['data']
                            if d2['reg_addr'] == 0x02 and result_seccount is None: result_seccount = r['data']
                vendor_cmds.append({'sample': t['sample'], 'feat': feat, 'seccount': seccount,
                                    'status': status, 'result_seccount': result_seccount})
    
    if vendor_cmds:
        findings.append(f"\nVendor CMD 0xC2 calls: {len(vendor_cmds)}")
        for vc in vendor_cmds[:20]:
            findings.append(f"  @{vc['sample']}: FEAT=0x{vc['feat']:02X} SECCOUNT=0x{vc['seccount']:02X} → STATUS=0x{vc['status']:02X} SECCOUNT_RESULT=0x{vc['result_seccount']:02X}" if all(v is not None for v in vc.values()) else f"  @{vc['sample']}: {vc}")
    
    # ATA command sequences (group into operations)
    if ata_cmds:
        findings.append(f"\nATA command sequence (first 50):")
        for sample, cmd, name_str in ata_cmds[:50]:
            # Find preceding LBA setup
            lba_info = ""
            for t in transactions:
                if t['sample'] >= sample: break
                if t['cmd'] == 52 and t['sample'] > sample - 500000:
                    d = decode_cmd52_arg(t['arg'])
                    if d['fn'] == 1 and d['rw'] and d['reg_addr'] in (0x02, 0x03, 0x04, 0x05, 0x06):
                        pass  # could extract but expensive
            findings.append(f"  @{sample}: {name_str}")
    
    # Standby/re-init pattern
    standby_count = sum(1 for _, c, _ in ata_cmds if c == 0xE0)
    if standby_count:
        findings.append(f"\nSTANDBY_IMMEDIATE count: {standby_count} (each triggers full SDIO re-init)")
    
    # Look for new/unknown ATA commands
    unknown = [(s, c, n) for s, c, n in ata_cmds if 'UNKNOWN' in n]
    if unknown:
        findings.append(f"\nUnknown ATA commands:")
        for s, c, n in unknown[:20]:
            findings.append(f"  @{s}: 0x{c:02X}")
    
    # LBA range analysis - extract actual read/write LBAs
    lba_ops = extract_lba_ops(transactions)
    if lba_ops:
        findings.append(f"\nLBA operations: {len(lba_ops)}")
        read_ops = [op for op in lba_ops if op['ata_cmd'] in (0x20, 0xC4)]
        write_ops = [op for op in lba_ops if op['ata_cmd'] in (0x30, 0xC5)]
        if read_ops:
            lbas = [op['lba'] for op in read_ops]
            findings.append(f"  Reads: {len(read_ops)}, LBA range: {min(lbas)}-{max(lbas)}")
            # Show first 20
            for op in read_ops[:20]:
                findings.append(f"    LBA={op['lba']} SEC={op['seccount']} ({op['ata_name']})")
        if write_ops:
            lbas = [op['lba'] for op in write_ops]
            findings.append(f"  Writes: {len(write_ops)}, LBA range: {min(lbas)}-{max(lbas)}")
            for op in write_ops[:20]:
                findings.append(f"    LBA={op['lba']} SEC={op['seccount']} ({op['ata_name']})")
    
    return '\n'.join(findings)

def extract_lba_ops(transactions):
    """Extract LBA/seccount from ATA read/write commands."""
    ops = []
    regs = {}  # track last written values to fn1 regs
    
    for t in transactions:
        if t['cmd'] == 52:
            d = decode_cmd52_arg(t['arg'])
            if d['fn'] == 1 and d['rw']:
                regs[d['reg_addr']] = d['write_data']
                if d['reg_addr'] == 0x07:  # ATA command
                    ata_cmd = d['write_data']
                    if ata_cmd in (0x20, 0x30, 0xC4, 0xC5):  # read/write variants
                        lba_lo = regs.get(0x03, 0)
                        lba_mid = regs.get(0x04, 0)
                        lba_hi = regs.get(0x05, 0)
                        dev_head = regs.get(0x06, 0)
                        seccount = regs.get(0x02, 0)
                        lba = lba_lo | (lba_mid << 8) | (lba_hi << 16) | ((dev_head & 0x0F) << 24)
                        ops.append({
                            'sample': t['sample'],
                            'ata_cmd': ata_cmd,
                            'ata_name': ATA_CMDS.get(ata_cmd, f"0x{ata_cmd:02X}"),
                            'lba': lba,
                            'seccount': seccount if seccount else 256,
                            'dev_head': dev_head,
                        })
    return ops

def main():
    traces = [
        ("Idle", "/home/pi/logictrace/N91_USBDrive_Idle_Fast6ch.csv", None),
        ("USBDrive", "/home/pi/logictrace/N91_USBDrive_Fast6ch.csv", None),
        ("FastFormat", "/home/pi/logictrace/N91_FastFormat_INFO_success_Fast6ch.csv", None),
        ("SlowFormat", "/home/pi/logictrace/N91_SlowFormat_success_Fast6ch.csv", 50000000),  # first 50M rows only
    ]
    
    all_findings = []
    for name, path, max_rows in traces:
        if not os.path.exists(path):
            print(f"Skipping {name}: not found")
            continue
        txns, cmd_count = fast_decode(path, max_rows)
        findings = analyze_trace(name, txns, cmd_count)
        all_findings.append(findings)
        print(findings)
    
    # Write combined report
    report_path = "/home/pi/logictrace/N91_TRACE_ANALYSIS_ALL.md"
    with open(report_path, 'w') as f:
        f.write("# N91 SDIO Trace Analysis - All Traces\n\n")
        f.write("Generated by analyze_all.py\n\n")
        for findings in all_findings:
            f.write(findings + "\n\n")
    print(f"\nReport written to {report_path}")

if __name__ == '__main__':
    main()
