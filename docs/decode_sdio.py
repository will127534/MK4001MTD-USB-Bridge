#!/usr/bin/env python3
"""
SDIO Protocol Decoder for Toshiba MK4001MTD Microdrive
Parses Saleae Logic CSV exports (transition-based).

CSV format: Sample, DAT0, CLK, CMD, CD/DAT3, DAT2, DAT1
"""

import csv
import sys
import os
from collections import defaultdict

# Column indices in CSV
COL_SAMPLE = 0
COL_DAT0 = 1
COL_CLK = 2
COL_CMD = 3
COL_DAT3 = 4  # CD/DAT3
COL_DAT2 = 5
COL_DAT1 = 6

# SDIO command indices
CMD5 = 5    # IO_SEND_OP_COND
CMD3 = 3    # SEND_RELATIVE_ADDR
CMD7 = 7    # SELECT/DESELECT_CARD
CMD15 = 15  # GO_INACTIVE_STATE
CMD52 = 52  # IO_RW_DIRECT
CMD53 = 53  # IO_RW_EXTENDED

# ATA commands
ATA_CMDS = {
    0x20: "READ_SECTORS",
    0x30: "WRITE_SECTORS",
    0xEC: "IDENTIFY_DEVICE",
    0xEF: "SET_FEATURES",
    0xE7: "FLUSH_CACHE",
    0xE1: "IDLE_IMMEDIATE",
    0x91: "INITIALIZE_DEVICE_PARAMS",
    0xC8: "READ_DMA",
    0xCA: "WRITE_DMA",
    0x25: "READ_DMA_EXT",
    0x35: "WRITE_DMA_EXT",
    0xB0: "SMART",
    0xA1: "IDENTIFY_PACKET_DEVICE",
    0x00: "NOP",
    0xC6: "SET_MULTIPLE_MODE",
    0xC4: "READ_MULTIPLE",
    0xC5: "WRITE_MULTIPLE",
}

# CCCR registers (function 0)
CCCR_REGS = {
    0x00: "CCCR_SDIO_REVISION",
    0x01: "SD_SPEC_REVISION",
    0x02: "IO_ENABLE",
    0x03: "IO_READY",
    0x04: "INT_ENABLE",
    0x05: "INT_PENDING",
    0x06: "IO_ABORT",
    0x07: "BUS_INTERFACE_CONTROL",
    0x08: "CARD_CAPABILITY",
    0x09: "COMMON_CIS_PTR_0",
    0x0A: "COMMON_CIS_PTR_1",
    0x0B: "COMMON_CIS_PTR_2",
    0x0C: "BUS_SUSPEND",
    0x0D: "FUNCTION_SELECT",
    0x0E: "EXEC_FLAGS",
    0x0F: "READY_FLAGS",
    0x10: "FN0_BLOCK_SIZE_0",
    0x11: "FN0_BLOCK_SIZE_1",
    0x12: "POWER_CONTROL",
    0x13: "HIGH_SPEED",
}

# FBR base addresses
def fbr_base(fn):
    return fn * 0x100

FBR_REGS = {
    0x00: "FBR_STD_FUNC_IF_CODE",
    0x01: "FBR_EXT_FUNC_IF_CODE",
    0x02: "FBR_POWER_SELECTION",
    0x09: "FBR_CIS_PTR_0",
    0x0A: "FBR_CIS_PTR_1",
    0x0B: "FBR_CIS_PTR_2",
    0x0C: "FBR_CSA_PTR_0",
    0x0D: "FBR_CSA_PTR_1",
    0x0E: "FBR_CSA_PTR_2",
    0x0F: "FBR_CSA_DATA",
    0x10: "FBR_BLOCK_SIZE_0",
    0x11: "FBR_BLOCK_SIZE_1",
}

# ATA register map (function 1)
ATA_REGS = {
    0x00: "DATA",
    0x01: "ERR/FEAT",
    0x02: "SECCOUNT",
    0x03: "LBA_LO",
    0x04: "LBA_MID",
    0x05: "LBA_HI",
    0x06: "DEV/HEAD",
    0x07: "CMD/STATUS",
}

def crc7(bits):
    """Calculate CRC7 for a bit sequence."""
    crc = 0
    for bit in bits:
        crc_bit = (crc >> 6) & 1
        crc = ((crc << 1) | bit) & 0x7F
        if crc_bit:
            crc ^= 0x09  # polynomial x^7 + x^3 + 1
    return crc

def crc16(bits):
    """Calculate CRC16-CCITT for a bit sequence."""
    crc = 0
    for bit in bits:
        crc_bit = (crc >> 15) & 1
        crc = ((crc << 1) | bit) & 0xFFFF
        if crc_bit:
            crc ^= 0x1021
    return crc

def decode_cmd52_arg(arg):
    """Decode CMD52 argument bits."""
    rw = (arg >> 31) & 1
    fn = (arg >> 28) & 0x7
    raw = (arg >> 27) & 1
    reg_addr = (arg >> 9) & 0x1FFFF
    write_data = arg & 0xFF
    return {
        'rw': rw,
        'fn': fn,
        'raw': raw,
        'reg_addr': reg_addr,
        'write_data': write_data,
    }

def decode_cmd52_resp(arg):
    """Decode CMD52 R5 response."""
    flags = (arg >> 8) & 0xFF
    data = arg & 0xFF
    
    flag_names = []
    if flags & 0x80: flag_names.append("OUT_OF_RANGE")
    if flags & 0x40: flag_names.append("FUNC_NUM_ERR")
    if flags & 0x20: flag_names.append("ERR")
    # bits 4:3 = IO_CURRENT_STATE
    state = (flags >> 4) & 0x3  
    state_names = {0: "DIS", 1: "CMD", 2: "TRN", 3: "RFU"}
    flag_names.append(f"STATE={state_names.get(state, '?')}")
    if flags & 0x02: flag_names.append("ILLEGAL_CMD")
    if flags & 0x01: flag_names.append("CRC_ERR")
    
    return {
        'flags': flags,
        'flag_str': '|'.join(flag_names),
        'data': data,
    }

def decode_cmd53_arg(arg):
    """Decode CMD53 argument bits."""
    rw = (arg >> 31) & 1
    fn = (arg >> 28) & 0x7
    block_mode = (arg >> 27) & 1
    op_code = (arg >> 26) & 1  # 1=incrementing addr, 0=fixed addr
    reg_addr = (arg >> 9) & 0x1FFFF
    byte_count = arg & 0x1FF
    if byte_count == 0:
        byte_count = 512 if not block_mode else 0  # 0 means 512 in byte mode
    return {
        'rw': rw,
        'fn': fn,
        'block_mode': block_mode,
        'op_code': op_code,
        'reg_addr': reg_addr,
        'byte_count': byte_count,
    }

def reg_name(fn, addr):
    """Get human-readable register name."""
    if fn == 0:
        if addr in CCCR_REGS:
            return CCCR_REGS[addr]
        # Check FBR regions
        fbr_fn = addr >> 8
        fbr_off = addr & 0xFF
        if 1 <= fbr_fn <= 7 and fbr_off in FBR_REGS:
            return f"FN{fbr_fn}_{FBR_REGS[fbr_off]}"
    elif fn == 1:
        if addr in ATA_REGS:
            return ATA_REGS[addr]
    return f"REG_0x{addr:04X}"


class SDIODecoder:
    def __init__(self, csv_path, sample_rate=None):
        self.csv_path = csv_path
        self.sample_rate = sample_rate  # Hz, if known
        self.transactions = []
        self.cmd_bits = []
        self.dat_bits = []  # list of (dat3, dat2, dat1, dat0) tuples
        self.state = 'IDLE'
        self.prev_clk = 0
        self.prev_cmd = 1
        self.cmd_bit_count = 0
        self.current_cmd_bits = []
        self.current_dat_bits = []  # list of (d3,d2,d1,d0)
        self.in_cmd = False
        self.in_response = False
        self.in_data = False
        self.expecting_response = False
        self.expecting_data = False
        self.last_cmd_index = -1
        self.last_cmd53_info = None
        self.data_bit_count = 0
        self.data_expected_bits = 0
        self.first_sample = None
        self.rca = 0
        
        # Stats
        self.cmd_count = defaultdict(int)
        self.total_rising_edges = 0
    
    def process_csv(self):
        """Read CSV and process all transitions."""
        print(f"Reading {self.csv_path}...")
        
        # Current signal states
        clk = 0
        cmd = 1  # CMD line idles high
        dat0 = 1
        dat1 = 1
        dat2 = 1
        dat3 = 1
        
        row_count = 0
        rising_edges = 0
        
        # We need to track state and detect rising CLK edges
        # At each rising edge, sample CMD and DAT lines
        
        # Bit accumulators
        cmd_bits = []
        dat_nibbles = []  # each entry is (d3,d2,d1,d0) sampled at rising edge
        
        # State machine
        cmd_phase = 'IDLE'  # IDLE, CMD, RESP, GAP
        data_phase = 'IDLE'  # IDLE, DATA
        
        cmd_start_sample = 0
        data_start_sample = 0
        gap_clocks = 0
        
        pending_cmd = None  # parsed command waiting for response
        pending_cmd53 = None  # CMD53 info waiting for data
        
        with open(self.csv_path, 'r') as f:
            reader = csv.reader(f)
            header = next(reader)  # skip header
            
            for row in reader:
                row_count += 1
                if row_count % 1000000 == 0:
                    print(f"  Processed {row_count/1e6:.1f}M rows, {rising_edges} CLK edges, {len(self.transactions)} transactions...")
                
                sample = int(row[COL_SAMPLE].strip())
                new_dat0 = int(row[COL_DAT0].strip())
                new_clk = int(row[COL_CLK].strip())
                new_cmd = int(row[COL_CMD].strip())
                new_dat3 = int(row[COL_DAT3].strip())
                new_dat2 = int(row[COL_DAT2].strip())
                new_dat1 = int(row[COL_DAT1].strip())
                
                if self.first_sample is None:
                    self.first_sample = sample
                
                # Detect CLK rising edge
                if new_clk == 1 and clk == 0:
                    rising_edges += 1
                    
                    # Sample CMD line
                    cmd_val = new_cmd if new_cmd != cmd else cmd  # use new value if changed this sample
                    # Actually for transition-based: the new values in this row are the NEW states
                    # So at this rising edge, the signal values are the NEW ones
                    cmd_val = new_cmd
                    d0 = new_dat0
                    d1 = new_dat1
                    d2 = new_dat2
                    d3 = new_dat3
                    
                    # CMD line state machine
                    if cmd_phase == 'IDLE':
                        if cmd_val == 0:  # Start bit
                            cmd_phase = 'COLLECTING'
                            cmd_bits = [0]
                            cmd_start_sample = sample
                    elif cmd_phase == 'COLLECTING':
                        cmd_bits.append(cmd_val)
                        if len(cmd_bits) == 48:
                            # Parse the 48-bit frame
                            direction = cmd_bits[1]  # 1=host, 0=card
                            cmd_index = 0
                            for i in range(2, 8):
                                cmd_index = (cmd_index << 1) | cmd_bits[i]
                            argument = 0
                            for i in range(8, 40):
                                argument = (argument << 1) | cmd_bits[i]
                            crc_val = 0
                            for i in range(40, 47):
                                crc_val = (crc_val << 1) | cmd_bits[i]
                            stop = cmd_bits[47]
                            
                            # Verify CRC7
                            expected_crc = crc7(cmd_bits[0:40])
                            crc_ok = (crc_val == expected_crc)
                            
                            if direction == 1:  # Host command
                                txn = {
                                    'type': 'CMD',
                                    'sample': cmd_start_sample,
                                    'cmd_index': cmd_index,
                                    'argument': argument,
                                    'crc': crc_val,
                                    'crc_ok': crc_ok,
                                    'raw_bits': cmd_bits[:],
                                }
                                pending_cmd = txn
                                self.cmd_count[cmd_index] += 1
                                cmd_phase = 'IDLE'
                                
                            else:  # Card response
                                txn = {
                                    'type': 'RESP',
                                    'sample': cmd_start_sample,
                                    'cmd_index': cmd_index,
                                    'argument': argument,
                                    'crc': crc_val,
                                    'crc_ok': crc_ok,
                                }
                                
                                # Pair with pending command
                                if pending_cmd is not None:
                                    combined = {
                                        'type': 'CMD_RESP',
                                        'sample': pending_cmd['sample'],
                                        'resp_sample': cmd_start_sample,
                                        'cmd_index': pending_cmd['cmd_index'],
                                        'cmd_arg': pending_cmd['argument'],
                                        'cmd_crc_ok': pending_cmd['crc_ok'],
                                        'resp_index': cmd_index,
                                        'resp_arg': argument,
                                        'resp_crc_ok': crc_ok,
                                    }
                                    self.transactions.append(combined)
                                    
                                    # If CMD53, prepare for data phase
                                    if pending_cmd['cmd_index'] == CMD53:
                                        pending_cmd53 = decode_cmd53_arg(pending_cmd['argument'])
                                        pending_cmd53['sample'] = pending_cmd['sample']
                                        data_phase = 'WAIT_START'
                                        dat_nibbles = []
                                    
                                    pending_cmd = None
                                else:
                                    # Orphan response
                                    self.transactions.append({
                                        'type': 'RESP_ONLY',
                                        'sample': cmd_start_sample,
                                        'cmd_index': cmd_index,
                                        'argument': argument,
                                        'crc_ok': crc_ok,
                                    })
                                
                                cmd_phase = 'IDLE'
                    
                    # Data line state machine (for CMD53)
                    if data_phase == 'WAIT_START':
                        # Wait for start bit (all DAT lines go low)
                        if d0 == 0:  # Start bit on DAT0
                            data_phase = 'COLLECTING'
                            dat_nibbles = []
                            data_start_sample = sample
                    elif data_phase == 'COLLECTING':
                        dat_nibbles.append((d3, d2, d1, d0))
                        
                        if pending_cmd53 is not None:
                            expected = pending_cmd53['byte_count']
                            # Each byte = 2 nibbles on 4-bit bus
                            expected_nibbles = expected * 2
                            # Plus 16 bits CRC per line = 16 nibble-clocks for CRC
                            # Plus 1 stop bit
                            total_expected = expected_nibbles + 16 + 1
                            
                            if len(dat_nibbles) >= total_expected:
                                # Extract data
                                data_nibs = dat_nibbles[:expected_nibbles]
                                data_bytes = []
                                for i in range(0, len(data_nibs), 2):
                                    hi = data_nibs[i]
                                    lo = data_nibs[i+1] if i+1 < len(data_nibs) else (0,0,0,0)
                                    byte_val = (hi[0] << 7) | (hi[1] << 6) | (hi[2] << 5) | (hi[3] << 4) | \
                                               (lo[0] << 3) | (lo[1] << 2) | (lo[2] << 1) | lo[3]
                                    data_bytes.append(byte_val)
                                
                                # Add data transaction
                                self.transactions.append({
                                    'type': 'DATA',
                                    'sample': data_start_sample,
                                    'rw': pending_cmd53['rw'],
                                    'fn': pending_cmd53['fn'],
                                    'reg_addr': pending_cmd53['reg_addr'],
                                    'block_mode': pending_cmd53['block_mode'],
                                    'op_code': pending_cmd53['op_code'],
                                    'byte_count': expected,
                                    'data': data_bytes,
                                    'cmd53_sample': pending_cmd53['sample'],
                                })
                                
                                data_phase = 'IDLE'
                                pending_cmd53 = None
                
                # Update current signal states
                clk = new_clk
                cmd = new_cmd
                dat0 = new_dat0
                dat1 = new_dat1
                dat2 = new_dat2
                dat3 = new_dat3
        
        self.total_rising_edges = rising_edges
        print(f"Done. {row_count} rows, {rising_edges} CLK rising edges, {len(self.transactions)} transactions")
        print(f"Command counts: {dict(self.cmd_count)}")
    
    def format_transaction(self, txn):
        """Format a transaction for human-readable output."""
        lines = []
        
        if txn['type'] == 'CMD_RESP':
            ci = txn['cmd_index']
            arg = txn['cmd_arg']
            rarg = txn['resp_arg']
            sample = txn['sample']
            
            cmd_name = f"CMD{ci}"
            detail = ""
            
            if ci == CMD5:
                ocr = arg & 0x00FFFFFF
                r_ocr = rarg & 0x00FFFFFF
                r_ready = (rarg >> 31) & 1
                r_nfn = (rarg >> 28) & 0x7
                r_mem = (rarg >> 27) & 1
                detail = f"  OCR=0x{ocr:06X} → Ready={r_ready} NumFn={r_nfn} Mem={r_mem} OCR=0x{r_ocr:06X}"
                
            elif ci == CMD3:
                r_rca = (rarg >> 16) & 0xFFFF
                self.rca = r_rca
                detail = f"  → RCA=0x{r_rca:04X}"
                
            elif ci == CMD7:
                rca = (arg >> 16) & 0xFFFF
                detail = f"  RCA=0x{rca:04X}"
                
            elif ci == CMD52:
                d = decode_cmd52_arg(arg)
                r = decode_cmd52_resp(rarg)
                rw = "WR" if d['rw'] else "RD"
                rname = reg_name(d['fn'], d['reg_addr'])
                detail = f"  {rw} fn{d['fn']} {rname}(0x{d['reg_addr']:04X})"
                if d['rw']:
                    detail += f" ← 0x{d['write_data']:02X}"
                detail += f" → 0x{r['data']:02X} [{r['flag_str']}]"
                
                # ATA command detection
                if d['fn'] == 1 and d['reg_addr'] == 0x07 and d['rw']:
                    ata_cmd = d['write_data']
                    ata_name = ATA_CMDS.get(ata_cmd, f"UNKNOWN_0x{ata_cmd:02X}")
                    detail += f" *** ATA CMD: {ata_name} ***"
                
            elif ci == CMD53:
                d = decode_cmd53_arg(arg)
                rw = "WR" if d['rw'] else "RD"
                mode = "BLK" if d['block_mode'] else "BYTE"
                inc = "INC" if d['op_code'] else "FIX"
                rname = reg_name(d['fn'], d['reg_addr'])
                detail = f"  {rw} fn{d['fn']} {rname}(0x{d['reg_addr']:04X}) {mode} {inc} count={d['byte_count']}"
            
            crc_warn = "" if txn['cmd_crc_ok'] else " [CMD CRC FAIL]"
            resp_crc_warn = "" if txn['resp_crc_ok'] else " [RESP CRC FAIL]"
            
            lines.append(f"@{sample}: {cmd_name} arg=0x{arg:08X} resp=0x{rarg:08X}{crc_warn}{resp_crc_warn}")
            if detail:
                lines.append(detail)
                
        elif txn['type'] == 'DATA':
            rw = "WR→" if txn['rw'] else "←RD"
            data_hex = ' '.join(f"{b:02X}" for b in txn['data'][:64])
            if len(txn['data']) > 64:
                data_hex += f" ... ({len(txn['data'])} bytes total)"
            rname = reg_name(txn['fn'], txn['reg_addr'])
            lines.append(f"  DATA {rw} fn{txn['fn']} {rname}(0x{txn['reg_addr']:04X}): {data_hex}")
            
        elif txn['type'] == 'RESP_ONLY':
            lines.append(f"@{txn['sample']}: ORPHAN RESP CMD{txn['cmd_index']} arg=0x{txn['argument']:08X}")
        
        return '\n'.join(lines)
    
    def write_log(self, path):
        """Write decoded transactions to file."""
        print(f"Writing {len(self.transactions)} transactions to {path}...")
        with open(path, 'w') as f:
            f.write(f"# SDIO Decoded Transactions\n")
            f.write(f"# Source: {self.csv_path}\n")
            f.write(f"# Total CLK rising edges: {self.total_rising_edges}\n")
            f.write(f"# Command counts: {dict(self.cmd_count)}\n\n")
            
            for txn in self.transactions:
                text = self.format_transaction(txn)
                if text:
                    f.write(text + '\n')
        print(f"Done writing {path}")
    
    def write_analysis(self, path):
        """Write analysis.md."""
        print(f"Writing analysis to {path}...")
        
        # Categorize transactions
        init_cmds = []  # CMD5, CMD3, CMD7
        cccr_accesses = []  # CMD52 fn0
        ata_reg_accesses = []  # CMD52 fn1
        ata_commands = []  # CMD52 fn1 reg 0x07 writes
        data_transfers = []  # CMD53
        
        for txn in self.transactions:
            if txn['type'] == 'CMD_RESP':
                ci = txn['cmd_index']
                if ci in (CMD5, CMD3, CMD7):
                    init_cmds.append(txn)
                elif ci == CMD52:
                    d = decode_cmd52_arg(txn['cmd_arg'])
                    if d['fn'] == 0:
                        cccr_accesses.append((txn, d))
                    elif d['fn'] == 1:
                        ata_reg_accesses.append((txn, d))
                        if d['reg_addr'] == 0x07 and d['rw']:
                            ata_commands.append((txn, d))
                elif ci == CMD53:
                    data_transfers.append(txn)
            elif txn['type'] == 'DATA':
                # attach to nearest CMD53
                pass
        
        with open(path, 'w') as f:
            f.write("# Toshiba MK4001MTD SDIO Protocol Analysis\n\n")
            f.write("Analysis of Nokia N91 boot SDIO communication with MK4001MTD microdrive.\n\n")
            f.write(f"**Source trace:** N91Boot_success_Fast6ch.csv\n")
            f.write(f"**Total transactions decoded:** {len(self.transactions)}\n")
            f.write(f"**Total CLK rising edges:** {self.total_rising_edges}\n\n")
            
            # Init sequence
            f.write("## 1. SDIO Initialization Sequence\n\n")
            for txn in init_cmds:
                f.write(f"- {self.format_transaction(txn)}\n")
            f.write("\n")
            
            # CCCR accesses
            f.write("## 2. CCCR Register Accesses (fn0)\n\n")
            f.write("| Sample | R/W | Register | Addr | Write | Read | Flags |\n")
            f.write("|--------|-----|----------|------|-------|------|-------|\n")
            for txn, d in cccr_accesses:
                r = decode_cmd52_resp(txn['resp_arg'])
                rw = "WR" if d['rw'] else "RD"
                rname = reg_name(0, d['reg_addr'])
                wd = f"0x{d['write_data']:02X}" if d['rw'] else "-"
                f.write(f"| {txn['sample']} | {rw} | {rname} | 0x{d['reg_addr']:04X} | {wd} | 0x{r['data']:02X} | {r['flag_str']} |\n")
            f.write("\n")
            
            # ATA register accesses
            f.write("## 3. ATA Register Accesses (fn1 via CMD52)\n\n")
            f.write("| Sample | R/W | Register | Addr | Write | Read | Flags |\n")
            f.write("|--------|-----|----------|------|-------|------|-------|\n")
            for txn, d in ata_reg_accesses[:200]:  # limit output
                r = decode_cmd52_resp(txn['resp_arg'])
                rw = "WR" if d['rw'] else "RD"
                rname = reg_name(1, d['reg_addr'])
                wd = f"0x{d['write_data']:02X}" if d['rw'] else "-"
                f.write(f"| {txn['sample']} | {rw} | {rname} | 0x{d['reg_addr']:04X} | {wd} | 0x{r['data']:02X} | {r['flag_str']} |\n")
            if len(ata_reg_accesses) > 200:
                f.write(f"\n*... {len(ata_reg_accesses) - 200} more ATA register accesses ...*\n")
            f.write("\n")
            
            # ATA commands
            f.write("## 4. ATA Commands Issued\n\n")
            for txn, d in ata_commands:
                ata_cmd = d['write_data']
                ata_name = ATA_CMDS.get(ata_cmd, f"UNKNOWN_0x{ata_cmd:02X}")
                f.write(f"- @{txn['sample']}: **{ata_name}** (0x{ata_cmd:02X})\n")
            f.write("\n")
            
            # Data transfers
            f.write("## 5. CMD53 Data Transfers\n\n")
            f.write(f"Total CMD53 transactions: {len(data_transfers)}\n\n")
            for txn in data_transfers[:50]:
                d53 = decode_cmd53_arg(txn['cmd_arg'])
                rw = "WR" if d53['rw'] else "RD"
                mode = "BLK" if d53['block_mode'] else "BYTE"
                rname = reg_name(d53['fn'], d53['reg_addr'])
                f.write(f"- @{txn['sample']}: {rw} fn{d53['fn']} {rname}(0x{d53['reg_addr']:04X}) {mode} count={d53['byte_count']}\n")
            if len(data_transfers) > 50:
                f.write(f"\n*... {len(data_transfers) - 50} more CMD53 transfers ...*\n")
            f.write("\n")
            
            # ATA register map
            f.write("## 6. Confirmed ATA-over-SDIO Register Map\n\n")
            f.write("| Fn1 Addr | ATA Register | Notes |\n")
            f.write("|----------|-------------|-------|\n")
            f.write("| 0x00 | DATA | CMD53 target for sector data |\n")
            f.write("| 0x01 | ERR/FEAT | Error (read) / Features (write) |\n")
            f.write("| 0x02 | SECCOUNT | Sector count |\n")
            f.write("| 0x03 | LBA_LO | LBA bits 0-7 |\n")
            f.write("| 0x04 | LBA_MID | LBA bits 8-15 |\n")
            f.write("| 0x05 | LBA_HI | LBA bits 16-23 |\n")
            f.write("| 0x06 | DEV/HEAD | Device/Head + LBA bits 24-27 |\n")
            f.write("| 0x07 | CMD/STATUS | Command (write) / Status (read) |\n")
            f.write("\n")
            
            # Summary
            f.write("## 7. Summary & Drive Quirks\n\n")
            f.write(f"- Total SDIO commands decoded: {sum(self.cmd_count.values())}\n")
            for ci, count in sorted(self.cmd_count.items()):
                f.write(f"  - CMD{ci}: {count}\n")
            f.write(f"- RCA assigned: 0x{self.rca:04X}\n")
            f.write(f"- ATA commands issued: {len(ata_commands)}\n")
            f.write(f"- CMD53 data transfers: {len(data_transfers)}\n")
            f.write("\n### Known Quirks\n\n")
            f.write("- Drive SDIO controller doesn't respond to CMD5 until ~9 seconds after power-on (spin-up)\n")
            f.write("- After warm reset, drive may already be in TRN state from prior session\n")
            f.write("- INT_PENDING at CCCR addr 0x05, bit1 = fn1 interrupt\n")
        
        print(f"Done writing {path}")


def main():
    csv_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'N91Boot_success_Fast6ch.csv')
    if len(sys.argv) > 1:
        csv_path = sys.argv[1]
    
    decoder = SDIODecoder(csv_path)
    decoder.process_csv()
    
    log_path = os.path.join(os.path.dirname(csv_path), 'decoded_transactions.log')
    decoder.write_log(log_path)
    
    analysis_path = os.path.join(os.path.dirname(csv_path), 'analysis.md')
    decoder.write_analysis(analysis_path)
    
    print("\nDone! Files written:")
    print(f"  {log_path}")
    print(f"  {analysis_path}")


if __name__ == '__main__':
    main()
