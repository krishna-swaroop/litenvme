#
# This file is part of LiteNVMe.
#
# Copyright (c) 2026 Krishna Swaroop <swaroopdhulipalla99@gmail.com>
# SPDX-License-Identifier: BSD-2-Clause

"""Independent NVMe SSD model for Migen simulations of the full host-memory data path.

The model talks to the design the way a real SSD does: through the PCIe completer ports of
the host-memory responders (MemRd requests answered with CplD beats, MemWr requests). It does
not share any command/PRP logic with the DUT:

  - on SQ doorbell, fetch the 64 B SQE from host memory with a MemRd,
  - decode opcode/NSID/SLBA/NLB and PRP1/PRP2 (PRP lists read from host memory),
  - WRITE: read the payload with MRRS-sized MemRds and commit it to its own flash dict,
  - READ : write the payload from flash with MPS-sized MemWrs,
  - post a CQE (CID, SQHD, phase, status) with a MemWr, honoring the CQ head doorbell.

Knobs: per-command latency, an out-of-order completion window, error injection, and a
default fill for never-written LBAs. `idle_cycles` counts cycles with no command to execute
and no completion to post, which lets a test check the host keeps the device fed.
"""

import random

from migen import *
from migen.sim import passive

from litex.soc.interconnect import stream

# PCIe completer port model (layouts of the LitePCIe slave port fields the responder uses) --------

def req_layout(data_width):
    return [
        ("adr",    64),
        ("we",     1),
        ("len",    10),  # dwords.
        ("tag",    8),
        ("req_id", 16),
        ("dat",    data_width),
    ]

def cmp_layout(data_width):
    return [
        ("dat",    data_width),
        ("end",    1),
        ("len",    10),
        ("tag",    8),
        ("req_id", 16),
        ("cmp_id", 16),
        ("err",    1),
    ]

class PCIeCompleterPort(Module):
    """Stand-in for a LitePCIe crossbar slave port (no byte-enables, like the USP CQ path)."""
    def __init__(self, data_width):
        self.sink   = stream.Endpoint(req_layout(data_width))
        self.source = stream.Endpoint(cmp_layout(data_width))

# NVMe SSD model -----------------------------------------------------------------------------------

SC_SUCCESS          = 0x00
SC_INVALID_OPCODE   = 0x01
SC_INVALID_FIELD    = 0x02
SC_INVALID_NS       = 0x0b
SC_LBA_OUT_OF_RANGE = 0x80

class NVMeSSDModel:
    def __init__(self, windows, data_width, sq_base, cq_base, qsize, sq_db_adr, cq_db_adr,
                 nsid=1, nsze=1 << 20, lba_bytes=512, page_bytes=4096, mps=512, mrrs=512,
                 latency=0, reorder=1, status_for=None, fill=None, seed=0):
        # windows: list of (base, size, PCIeCompleterPort) routing host addresses to ports.
        self.windows     = windows
        self.data_width  = data_width
        self.beat_dwords = data_width // 32
        self.sq_base, self.cq_base, self.qsize = sq_base, cq_base, qsize
        self.sq_db_adr, self.cq_db_adr = sq_db_adr, cq_db_adr
        self.nsid, self.nsze, self.lba_bytes = nsid, nsze, lba_bytes
        self.page_bytes, self.mps, self.mrrs = page_bytes, mps, mrrs
        self.latency    = latency
        self.reorder    = reorder
        self.status_for = status_for  # f(cmd_index, cmd) -> status code (SC) or None.
        self.fill       = fill or (lambda lba: bytes([(lba * 7 + i) & 0xff for i in range(lba_bytes)]))
        self.rng        = random.Random(seed)

        self.flash      = {}   # lba -> bytes
        self.sq_tail_db = 0
        self.cq_head_db = 0
        self.commands   = []   # decoded commands, in fetch order.
        self.idle_cycles = 0
        self.dma_errors  = []  # host addresses outside every window.

    # Host-memory access through the responder ports ----------------------------------------------
    def _port(self, adr, nbytes):
        for base, size, port in self.windows:
            if base <= adr and adr + nbytes <= base + size:
                return port
        self.dma_errors.append((adr, nbytes))
        raise RuntimeError(f"SSD DMA outside host memory windows: 0x{adr:x}+{nbytes}")

    def read_dwords(self, adr, n):
        assert adr % (self.data_width // 8) == 0, "model issues beat-aligned MemRd only"
        port = self._port(adr, 4 * n)
        s, c = port.sink, port.source
        yield s.adr.eq(adr); yield s.we.eq(0); yield s.len.eq(n & 0x3ff)
        yield s.first.eq(1); yield s.last.eq(1); yield s.valid.eq(1)
        yield
        while not (yield s.ready):
            yield
        yield s.valid.eq(0)
        yield c.ready.eq(1)
        dws, cyc = [], 0
        while True:
            yield
            if (yield c.valid):
                beat = (yield c.dat)
                dws += [(beat >> (32 * i)) & 0xffffffff for i in range(self.beat_dwords)]
                if (yield c.last):
                    break
            cyc += 1
            if cyc > 10000:
                raise RuntimeError("SSD MemRd completion timeout")
        yield c.ready.eq(0)
        return dws[:n]

    def write_dwords(self, adr, dws):
        n   = len(dws)
        off = (adr // 4) % self.beat_dwords
        assert off == 0 or off + n <= self.beat_dwords, "unaligned MemWr must fit one beat"
        port = self._port(adr, 4 * n)
        s = port.sink
        beats = [dws[i:i + self.beat_dwords] for i in range(0, n, self.beat_dwords)]
        for i, b in enumerate(beats):
            yield s.adr.eq(adr); yield s.we.eq(1); yield s.len.eq(n & 0x3ff)
            yield s.dat.eq(sum((d & 0xffffffff) << (32 * j) for j, d in enumerate(b)))
            yield s.first.eq(i == 0); yield s.last.eq(i == len(beats) - 1)
            yield s.valid.eq(1)
            yield
            while not (yield s.ready):
                yield
        yield s.valid.eq(0)
        yield

    # PRP decoding ---------------------------------------------------------------------------------
    def prp_segments(self, prp1, prp2, nbytes):
        page = self.page_bytes
        first = min(nbytes, page - (prp1 % page))
        segs, rem = [(prp1, first)], nbytes - first
        if rem == 0:
            return segs
        if rem <= page:
            assert prp2 % page == 0, "PRP2 must be page aligned"
            return segs + [(prp2, rem)]
        npages = (rem + page - 1) // page
        assert npages <= page // 8, "chained PRP lists not modeled"
        entries = yield from self.read_dwords(prp2, 2 * npages)
        for i in range(npages):
            a = entries[2 * i] | (entries[2 * i + 1] << 32)
            assert a % page == 0, "PRP list entry must be page aligned"
            segs.append((a, min(page, rem)))
            rem -= page
        return segs

    # Command execution ----------------------------------------------------------------------------
    def _status(self, k, cmd):
        if cmd["op"] not in (0x01, 0x02):
            return SC_INVALID_OPCODE
        if cmd["nsid"] != self.nsid:
            return SC_INVALID_NS
        if cmd["slba"] + cmd["nlb"] > self.nsze:
            return SC_LBA_OUT_OF_RANGE
        if self.status_for is not None:
            sc = self.status_for(k, cmd)
            if sc is not None:
                return sc
        return SC_SUCCESS

    def execute(self, k, cmd):
        sc = self._status(k, cmd)
        if sc != SC_SUCCESS:
            return sc
        nbytes = cmd["nlb"] * self.lba_bytes
        segs   = yield from self.prp_segments(cmd["prp1"], cmd["prp2"], nbytes)
        if cmd["op"] == 0x01:  # Write: pull payload from host memory.
            data = bytearray()
            for adr, ln in segs:
                for o in range(0, ln, self.mrrs):
                    dws = yield from self.read_dwords(adr + o, min(self.mrrs, ln - o) // 4)
                    for d in dws:
                        data += d.to_bytes(4, "little")
            for i in range(cmd["nlb"]):
                self.flash[cmd["slba"] + i] = bytes(data[i * self.lba_bytes:(i + 1) * self.lba_bytes])
        else:                  # Read: push payload into host memory.
            data = b"".join(self.flash.get(cmd["slba"] + i, self.fill(cmd["slba"] + i))
                            for i in range(cmd["nlb"]))
            pos = 0
            for adr, ln in segs:
                for o in range(0, ln, self.mps):
                    chunk = data[pos:pos + min(self.mps, ln - o)]
                    pos  += len(chunk)
                    dws = [int.from_bytes(chunk[j:j + 4], "little") for j in range(0, len(chunk), 4)]
                    yield from self.write_dwords(adr + o, dws)
        return sc

    # Processes ------------------------------------------------------------------------------------
    def mmio_process(self, eng):
        """Doorbell MMIO model for the engine's accessor port."""
        @passive
        def gen():
            yield eng.mmio_done.eq(0)
            while True:
                if (yield eng.mmio_start):
                    adr = (yield eng.mmio_adr)
                    val = (yield eng.mmio_wdata)
                    if adr == self.sq_db_adr:
                        self.sq_tail_db = val
                    elif adr == self.cq_db_adr:
                        self.cq_head_db = val
                    yield eng.mmio_done.eq(1)
                    yield
                    yield eng.mmio_done.eq(0)
                yield
        return gen()

    def process(self):
        @passive
        def gen():
            sq_head, cq_tail, phase = 0, 0, 1
            pending = []  # (cid, sc) executed, CQE not yet posted.
            k = 0

            def post(i):
                nonlocal cq_tail, phase
                cid, sc = pending.pop(i)
                while ((cq_tail - self.cq_head_db) % self.qsize) >= self.qsize - 1:
                    yield  # CQ full: wait for the host to advance the CQ head doorbell.
                dw3 = (cid & 0xffff) | (phase << 16) | ((sc & 0xff) << 17)
                yield from self.write_dwords(self.cq_base + 16 * cq_tail, [0, 0, sq_head & 0xffff, dw3])
                cq_tail = (cq_tail + 1) % self.qsize
                if cq_tail == 0:
                    phase ^= 1

            while True:
                if sq_head != self.sq_tail_db:
                    sqe = yield from self.read_dwords(self.sq_base + 64 * sq_head, 16)
                    sq_head = (sq_head + 1) % self.qsize
                    cmd = dict(op=sqe[0] & 0xff, cid=sqe[0] >> 16, nsid=sqe[1],
                               prp1=sqe[6] | (sqe[7] << 32), prp2=sqe[8] | (sqe[9] << 32),
                               slba=sqe[10] | (sqe[11] << 32), nlb=(sqe[12] & 0xffff) + 1)
                    self.commands.append(cmd)
                    for _ in range(self.latency):
                                    yield
                    sc = yield from self.execute(k, cmd)
                    cmd["sc"] = sc
                    k += 1
                    pending.append((cmd["cid"], sc))
                    if len(pending) >= self.reorder:
                        yield from post(self.rng.randrange(len(pending)))
                elif pending:
                    yield from post(self.rng.randrange(len(pending)))
                else:
                    self.idle_cycles += 1
                    yield
        return gen()
