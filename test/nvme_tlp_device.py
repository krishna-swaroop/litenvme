#
# This file is part of LiteNVMe.
#
# Copyright (c) 2026 Krishna Swaroop <swaroopdhulipalla99@gmail.com>
# SPDX-License-Identifier: BSD-2-Clause

"""TLP-level NVMe SSD model behind SimPCIePHY, for whole-core simulations.

Extends NVMeSSDModel (test/ssd_model.py) — command decode, PRP walking, flash store, error and
reorder knobs — with what a real device presents on the link:

  - Type-0 config space: IDs, Command/Status, 64-bit BAR0 with sizing, PCIe capability with
    DevCap/DevCtl (the MPS/MRRS the host programs are honored for the device's own TLPs),
  - BAR0 NVMe registers: CAP/VS/CC/CSTS/AQA/ASQ/ACQ and SQ-tail / CQ-head doorbells,
  - admin commands: Identify Controller/Namespace, Set Features (Number of Queues), Create IO
    CQ/SQ; IO Read/Write on the created queue pair,
  - DMA as raw MemRd/MemWr TLPs, completions matched by tag.

Everything is exchanged as LitePCIe raw TLPs (little-endian dwords, 3DW/4DW headers as the
LitePCIe (de)packetizer produce them; 4DW addresses in wire order, high dword first). Protocol violations seen by the device are collected in
`violations` (e.g. a completion larger than MPS, a MemRd left unanswered) for the test to check.
"""

import os

from migen import *
from migen.sim import passive

TRACE = bool(os.environ.get("LITENVME_SIM_TRACE"))

from ssd_model import NVMeSSDModel, SC_SUCCESS, SC_INVALID_OPCODE, SC_INVALID_FIELD

# Raw TLP helpers (LitePCIe header layouts) --------------------------------------------------------

FMT_TYPE_MRD32, FMT_TYPE_MRD64 = 0b00_00000, 0b01_00000
FMT_TYPE_MWR32, FMT_TYPE_MWR64 = 0b10_00000, 0b11_00000
FMT_TYPE_CPL,   FMT_TYPE_CPLD  = 0b00_01010, 0b10_01010
FMT_TYPE_CFGRD0, FMT_TYPE_CFGWR0 = 0b00_00100, 0b10_00100

def dw0(fmt_type, length):
    return ((fmt_type & 0x7f) << 24) | (length & 0x3ff)

def dwords_to_beats(dws, data_width):
    n = data_width // 32
    beats = []
    for i in range(0, len(dws), n):
        chunk = dws[i:i + n]
        dat = sum((d & 0xffffffff) << (32 * j) for j, d in enumerate(chunk))
        be  = sum(0xf << (4 * j) for j in range(len(chunk)))
        beats.append((dat, be))
    return beats

def beats_to_dwords(beats, data_width):
    dws = []
    for dat, be in beats:
        for j in range(data_width // 32):
            if (be >> (4 * j)) & 0xf == 0xf:
                dws.append((dat >> (32 * j)) & 0xffffffff)
    return dws

def send_tlp(ep, dws, data_width):
    beats = dwords_to_beats(dws, data_width)
    for i, (dat, be) in enumerate(beats):
        yield ep.dat.eq(dat)
        yield ep.be.eq(be)
        yield ep.first.eq(i == 0)
        yield ep.last.eq(i == len(beats) - 1)
        yield ep.valid.eq(1)
        yield
        while not (yield ep.ready):
            yield
    yield ep.valid.eq(0)

# NVMe TLP device ----------------------------------------------------------------------------------

class NVMeTLPDevice(NVMeSSDModel):
    def __init__(self, phy, data_width, bar0_size=0x4000, mps_supported=2, rdy_delay=40,
                 vid=0x1d1d, did=0x5a17, model="LiteNVMe Sim SSD", serial="SIM0001", fw="1.0",
                 mdts=5, nsze=1 << 21, mqes=1023, dstrd=0, **kwargs):
        NVMeSSDModel.__init__(self, windows=[], data_width=data_width, sq_base=0, cq_base=0,
                              qsize=2, sq_db_adr=None, cq_db_adr=None, nsze=nsze, **kwargs)
        self.phy        = phy
        self.bar0_size  = bar0_size
        self.rdy_delay  = rdy_delay
        self.model, self.serial, self.fw, self.mdts = model, serial, fw, mdts
        self.vid, self.did = vid, did
        self.dstrd      = dstrd
        self.cap = (mqes & 0xffff) | (1 << 16) | (20 << 24) | (dstrd << 32) | (1 << 37) | (4 << 52)

        # Config space (dword index -> value) and BAR0 state.
        self.cfg = {0x00: (did << 16) | vid, 0x01: 0x0010_0000, 0x02: 0x0108_0200,
                    0x04: 0x4, 0x05: 0, 0x0d: 0x40,
                    0x10: 0x0002_0010, 0x11: mps_supported & 7, 0x12: 0x0000_2000}
        self.bar0 = 0
        self.cfg_bdfs = set()

        # Controller registers.
        self.cc = self.csts = self.aqa = self.asq = self.acq = 0
        self.doorbells = {}
        self.en_pending = None

        # Queues: qid -> dict(base, size, head/tail, phase, cqid).
        self.sqs, self.cqs = {}, {}
        self.num_queues_set = False

        self.dma_tag     = 0
        self.violations  = []
        self.admin_log   = []
        self.mmio_log    = []
        self.max_cpld_dw = 0

    # Config / MPS ---------------------------------------------------------------------------------
    @property
    def mem_enabled(self):
        return bool(self.cfg[0x01] & 0x2)

    @property
    def mps_bytes(self):
        return 128 << ((self.cfg[0x12] >> 5) & 7)

    @property
    def mrrs_bytes(self):
        return 128 << ((self.cfg[0x12] >> 12) & 7)

    def cfg_read(self, reg):
        if reg == 0x04:
            return (self.bar0 & 0xffffffff & ~(self.bar0_size - 1)) | 0x4
        if reg == 0x05:
            return (self.bar0 >> 32) & 0xffffffff
        return self.cfg.get(reg, 0)

    def cfg_write(self, reg, v):
        if reg == 0x04:
            self.bar0 = (self.bar0 & ~0xffffffff) | (v & 0xfffffff0)
        elif reg == 0x05:
            self.bar0 = (self.bar0 & 0xffffffff) | (v << 32)
        elif reg == 0x01:
            self.cfg[0x01] = (self.cfg[0x01] & 0xffff0000) | (v & 0x0000_0406)
        elif reg == 0x12:
            self.cfg[0x12] = (self.cfg[0x12] & 0xffff0000) | (v & 0x7fff)
            if ((v >> 5) & 7) > self.cfg[0x11] & 7:
                self.violations.append(f"DevCtl.MPS {(v >> 5) & 7} > supported {self.cfg[0x11] & 7}")

    def bar0_hit(self, adr):
        base = self.bar0 & ~(self.bar0_size - 1) & ~0xf
        return self.mem_enabled and base <= adr < base + self.bar0_size

    # BAR0 registers ------------------------------------------------------------------------------
    def reg_read(self, off):
        regs = {0x00: self.cap & 0xffffffff, 0x04: self.cap >> 32, 0x08: 0x0001_0400,
                0x14: self.cc, 0x1c: self.csts, 0x24: self.aqa,
                0x28: self.asq & 0xffffffff, 0x2c: self.asq >> 32,
                0x30: self.acq & 0xffffffff, 0x34: self.acq >> 32}
        return regs.get(off, 0)

    def reg_write(self, off, v):
        self.mmio_log.append((off, v))
        if off == 0x14:
            old, self.cc = self.cc, v
            if (v & 1) and not (old & 1):
                self.en_pending = self.rdy_delay
            elif not (v & 1) and (old & 1):
                self.controller_reset()
        elif off == 0x24: self.aqa = v
        elif off == 0x28: self.asq = (self.asq & ~0xffffffff) | v
        elif off == 0x2c: self.asq = (self.asq & 0xffffffff) | (v << 32)
        elif off == 0x30: self.acq = (self.acq & ~0xffffffff) | v
        elif off == 0x34: self.acq = (self.acq & 0xffffffff) | (v << 32)
        elif off >= 0x1000:
            stride = 4 << self.dstrd
            idx = (off - 0x1000) // stride
            self.doorbells[(idx // 2, idx % 2)] = v & 0xffff

    def controller_reset(self):
        self.csts = 0
        self.sqs, self.cqs, self.doorbells = {}, {}, {}
        self.en_pending = None

    def controller_enable(self):
        if ((self.cc >> 7) & 0xf) != 0 or ((self.cc >> 16) & 0xf) != 6 or ((self.cc >> 20) & 0xf) != 4:
            self.violations.append(f"unexpected CC 0x{self.cc:08x}")
        asz = (self.aqa & 0xfff) + 1
        acz = ((self.aqa >> 16) & 0xfff) + 1
        self.sqs[0] = dict(base=self.asq, size=asz, head=0, cqid=0)
        self.cqs[0] = dict(base=self.acq, size=acz, tail=0, phase=1)
        self.csts = 1

    # Host requests (CFG / MMIO) -> completions ---------------------------------------------------
    def host_process(self):
        phy, dw = self.phy, self.data_width
        @passive
        def gen():
            beats = []
            yield phy.req_sink.ready.eq(1)
            while True:
                yield
                if self.en_pending is not None:
                    self.en_pending -= 1
                    if self.en_pending <= 0:
                        self.en_pending = None
                        self.controller_enable()
                if not ((yield phy.req_sink.valid) and (yield phy.req_sink.ready)):
                    continue
                beats.append(((yield phy.req_sink.dat), (yield phy.req_sink.be)))
                if not (yield phy.req_sink.last):
                    continue
                dws, beats = beats_to_dwords(beats, dw), []
                yield phy.req_sink.ready.eq(0)
                cpl = self.handle_host_tlp(dws)
                if TRACE:
                    print(f"[dev] host TLP {[hex(d) for d in dws[:5]]} -> cpl {[hex(d) for d in (cpl or [])[:4]]}", flush=True)
                if cpl is not None:
                    yield from send_tlp(phy.cmp_source, cpl, dw)
                yield phy.req_sink.ready.eq(1)
        return gen()

    def _cpl(self, req_id, tag, data=None, status=0, lower=0):
        n = len(data) if data else 0
        fmt = FMT_TYPE_CPLD if data else FMT_TYPE_CPL
        # Byte Count: remaining bytes for a read; 4 for completions without data (PCIe spec).
        h1 = (0x0100 << 16) | ((status & 7) << 13) | (((4 * n) if data else 4) & 0xfff)
        h2 = ((req_id & 0xffff) << 16) | ((tag & 0xff) << 8) | (lower & 0x7f)
        return [dw0(fmt, n), h1, h2] + (data or [])

    def handle_host_tlp(self, dws):
        fmt_type = (dws[0] >> 24) & 0x7f
        length   = (dws[0] & 0x3ff) or 1024
        req_id, tag = dws[1] >> 16, (dws[1] >> 8) & 0xff
        if fmt_type in (FMT_TYPE_CFGRD0, FMT_TYPE_CFGWR0):
            bdf = (dws[2] >> 24, (dws[2] >> 19) & 0x1f, (dws[2] >> 16) & 0x7)
            self.cfg_bdfs.add(bdf)
            reg = ((dws[2] >> 2) & 0x3f) | (((dws[2] >> 8) & 0x7) << 6)
            if fmt_type == FMT_TYPE_CFGRD0:
                return self._cpl(req_id, tag, [self.cfg_read(reg)])
            self.cfg_write(reg, dws[3])
            return self._cpl(req_id, tag)
        is4 = bool(fmt_type & 0b0100000)
        hdr = 4 if is4 else 3
        # 4DW headers carry the address in wire order: DW2 = addr[63:32], DW3 = addr[31:0].
        adr = ((dws[2] << 32) | dws[3]) if is4 else dws[2]
        if fmt_type in (FMT_TYPE_MRD32, FMT_TYPE_MRD64):
            if not self.bar0_hit(adr):
                self.violations.append(f"MemRd outside BAR0 0x{adr:x}")
                return self._cpl(req_id, tag, status=0b001)  # UR
            off = adr - (self.bar0 & ~(self.bar0_size - 1) & ~0xf)
            data = [self.reg_read(off + 4 * i) for i in range(length)]
            return self._cpl(req_id, tag, data, lower=adr & 0x7f)
        if fmt_type in (FMT_TYPE_MWR32, FMT_TYPE_MWR64):
            if not self.bar0_hit(adr):
                self.violations.append(f"MemWr outside BAR0 0x{adr:x}")
                return None
            off = adr - (self.bar0 & ~(self.bar0_size - 1) & ~0xf)
            for i, v in enumerate(dws[hdr:hdr + length]):
                self.reg_write(off + 4 * i, v)
            return None
        self.violations.append(f"unexpected TLP fmt/type 0x{fmt_type:02x}")
        return None

    # Device DMA (raw TLP MemRd/MemWr) ------------------------------------------------------------
    def read_dwords(self, adr, n):
        phy, dw = self.phy, self.data_width
        out = []
        while n:
            chunk = min(n, self.mrrs_bytes // 4, (0x1000 - (adr & 0xfff)) // 4)
            tag = self.dma_tag = (self.dma_tag + 1) & 0xff
            yield from send_tlp(phy.req_source,
                [dw0(FMT_TYPE_MRD32, chunk), (0x0100 << 16) | (tag << 8) | 0xff, adr & 0xffffffff], dw)
            got, beats, cyc = [], [], 0
            yield phy.cmp_sink.ready.eq(1)
            while len(got) < chunk:
                yield
                cyc += 1
                if cyc > 20000:
                    self.violations.append(f"MemRd 0x{adr:x}+{4 * chunk} not completed")
                    raise RuntimeError(self.violations[-1])
                if not (yield phy.cmp_sink.valid):
                    continue
                beats.append(((yield phy.cmp_sink.dat), (yield phy.cmp_sink.be)))
                if not (yield phy.cmp_sink.last):
                    continue
                dws, beats = beats_to_dwords(beats, dw), []
                ctag, clen = (dws[2] >> 8) & 0xff, dws[0] & 0x3ff
                status = (dws[1] >> 13) & 7
                if ((dws[0] >> 24) & 0x7f) != FMT_TYPE_CPLD or ctag != tag or status != 0:
                    self.violations.append(f"bad completion for MemRd 0x{adr:x}: {[hex(d) for d in dws[:3]]}")
                if 4 * clen > self.mps_bytes:
                    self.violations.append(f"CplD of {4 * clen} B exceeds MPS {self.mps_bytes} B")
                self.max_cpld_dw = max(self.max_cpld_dw, clen)
                got += dws[3:3 + clen]
            yield phy.cmp_sink.ready.eq(0)
            out += got[:chunk]
            adr += 4 * chunk
            n   -= chunk
        return out

    def write_dwords(self, adr, dws):
        phy, dw = self.phy, self.data_width
        while dws:
            chunk = min(len(dws), self.mps_bytes // 4, (0x1000 - (adr & 0xfff)) // 4)
            yield from send_tlp(phy.req_source,
                [dw0(FMT_TYPE_MWR32, chunk), (0x0100 << 16) | 0xff, adr & 0xffffffff] + dws[:chunk], dw)
            adr += 4 * chunk
            dws  = dws[chunk:]

    # Command processing ---------------------------------------------------------------------------
    def identify_controller(self):
        b = bytearray(4096)
        b[0:2] = self.vid.to_bytes(2, "little")
        b[4:24]  = self.serial.ljust(20).encode()
        b[24:64] = self.model.ljust(40).encode()
        b[64:72] = self.fw.ljust(8).encode()
        b[77]    = self.mdts
        b[80:84] = (0x0001_0400).to_bytes(4, "little")
        b[512]   = 0x66  # SQES
        b[513]   = 0x44  # CQES
        b[516:520] = (1).to_bytes(4, "little")  # NN
        return b

    def identify_namespace(self):
        b = bytearray(4096)
        b[0:8]   = self.nsze.to_bytes(8, "little")
        b[8:16]  = self.nsze.to_bytes(8, "little")
        b[16:24] = self.nsze.to_bytes(8, "little")
        b[25]    = 0      # NLBAF (1 format)
        b[26]    = 0      # FLBAS -> format 0
        b[128:132] = (9 << 16).to_bytes(4, "little")  # LBAF0: 512 B, no metadata.
        return b

    def admin(self, cmd):
        op, cid = cmd[0] & 0xff, cmd[0] >> 16
        prp1 = cmd[6] | (cmd[7] << 32)
        self.admin_log.append(op)
        if op == 0x06:
            cns = cmd[10] & 0xff
            if cns == 1:
                data = self.identify_controller()
            elif cns == 0 and cmd[1] == self.nsid:
                data = self.identify_namespace()
            else:
                return SC_INVALID_FIELD, 0
            dws = [int.from_bytes(data[i:i + 4], "little") for i in range(0, 4096, 4)]
            yield from self.write_dwords(prp1, dws)
            return SC_SUCCESS, 0
        if op == 0x09 and (cmd[10] & 0xff) == 0x07:
            self.num_queues_set = True
            return SC_SUCCESS, 0  # 1 SQ + 1 CQ allocated.
        if op == 0x05:
            qid, qsz = cmd[10] & 0xffff, (cmd[10] >> 16) + 1
            if not (cmd[11] & 1):
                return SC_INVALID_FIELD, 0
            self.cqs[qid] = dict(base=prp1, size=qsz, tail=0, phase=1)
            return SC_SUCCESS, 0
        if op == 0x01:
            qid, qsz, cqid = cmd[10] & 0xffff, (cmd[10] >> 16) + 1, cmd[11] >> 16
            if cqid not in self.cqs or not (cmd[11] & 1):
                return SC_INVALID_FIELD, 0
            self.sqs[qid] = dict(base=prp1, size=qsz, head=0, cqid=cqid)
            return SC_SUCCESS, 0
        return SC_INVALID_OPCODE, 0

    def post_cqe(self, cqid, sqid, cid, sc, dw0_val=0):
        cq = self.cqs[cqid]
        while ((cq["tail"] - self.doorbells.get((cqid, 1), 0)) % cq["size"]) >= cq["size"] - 1:
            yield  # CQ full: wait for the host's CQ head doorbell.
        sqhd = self.sqs[sqid]["head"] if sqid in self.sqs else 0
        dw3 = (cid & 0xffff) | (cq["phase"] << 16) | ((sc & 0xff) << 17)
        yield from self.write_dwords(cq["base"] + 16 * cq["tail"], [dw0_val, 0, (sqid << 16) | sqhd, dw3])
        cq["tail"] = (cq["tail"] + 1) % cq["size"]
        if cq["tail"] == 0:
            cq["phase"] ^= 1

    def device_process(self):
        @passive
        def gen():
            k, pending = 0, []
            while True:
                if not (self.csts & 1):
                    pending = []
                    yield
                    continue
                work = False
                for qid in sorted(self.sqs):
                    sq = self.sqs.get(qid)
                    if sq is None or self.doorbells.get((qid, 0), 0) == sq["head"]:
                        continue
                    work = True
                    sqe = yield from self.read_dwords(sq["base"] + 64 * sq["head"], 16)
                    sq["head"] = (sq["head"] + 1) % sq["size"]
                    cid = sqe[0] >> 16
                    if qid == 0:
                        sc, res = yield from self.admin(sqe)
                        yield from self.post_cqe(0, 0, cid, sc, res)
                    else:
                        cmd = dict(op=sqe[0] & 0xff, cid=cid, nsid=sqe[1],
                                   prp1=sqe[6] | (sqe[7] << 32), prp2=sqe[8] | (sqe[9] << 32),
                                   slba=sqe[10] | (sqe[11] << 32), nlb=(sqe[12] & 0xffff) + 1)
                        self.commands.append(cmd)
                        for _ in range(self.latency):
                            yield
                        sc = yield from self.execute(k, cmd)
                        cmd["sc"] = sc
                        k += 1
                        pending.append((qid, cid, sc))
                        if len(pending) >= self.reorder:
                            q, c, s = pending.pop(self.rng.randrange(len(pending)))
                            yield from self.post_cqe(sq["cqid"], q, c, s)
                    break
                if not work:
                    if pending:
                        q, c, s = pending.pop(self.rng.randrange(len(pending)))
                        yield from self.post_cqe(self.sqs[q]["cqid"], q, c, s)
                    else:
                        self.idle_cycles += 1
                        yield
        return gen()
