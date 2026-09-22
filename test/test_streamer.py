# This file is part of LiteNVMe.
#
# Copyright (c) 2026 Krishna Swaroop <swaroopdhulipalla99@gmail.com>
# SPDX-License-Identifier: BSD-2-Clause
#
# Simulation of the stream ring front-end over the real I/O path:
#
#   wr_sink / rd_source
#        |
#   LiteNVMeStreamer --cmd--> LiteNVMeIOEngineAXI --SQE/CQE--> ctrl LiteNVMeHostMemResponder
#        | .dma.axi                                                 ^ PCIe completer port
#        v                                                          |
#   data LiteNVMeHostMemResponder <--- PCIe completer port --- NVMeSSDModel (test/ssd_model.py)
#
# The SSD model fetches SQEs, decodes PRPs, moves payload and posts CQEs through the
# responders' PCIe ports, exactly like a device, and keeps its own flash contents. Checks
# are made against that independent state, never against the DUT's staging memory:
#
#   - write: every LBA in the SSD model holds the streamed bytes,
#   - read : rd_source emits the SSD model's bytes for a range the DUT never wrote,
#   - ordering under out-of-order completions, error status reporting, overlap efficiency.

import random
import unittest

from migen import *
from migen.sim import run_simulation, passive

from litenvme.io_engine import LiteNVMeIOEngineAXI
from litenvme.hostmem   import LiteNVMeHostMemAXIRAM, LiteNVMeHostMemResponder
from litenvme.streamer  import LiteNVMeStreamDMA, LiteNVMeStreamer
from litenvme.pattern   import LiteNVMeStreamPatternGen, LiteNVMeStreamPatternCheck

from ssd_model import PCIeCompleterPort, NVMeSSDModel, SC_INVALID_FIELD

from migen.fhdl import verilog
from litex.gen import LiteXModule
from litex.soc.interconnect import stream
from litepcie.common import phy_layout
from litepcie.core.rootport import LitePCIeRootPort
from litenvme.core import LiteNVMe

# Layout -------------------------------------------------------------------------------------------

CTRL_BASE = 0x1000_0000
CTRL_SIZE = 0x400          # IO SQ (8 x 64 B) + IO CQ (8 x 16 B).
SQ_BASE   = CTRL_BASE + 0x000
CQ_BASE   = CTRL_BASE + 0x200
DATA_BASE = 0x2000_0000    # Slot ring (its own responder / crossbar slave port).
SQ_DB_ADR = 0xe000_1008
CQ_DB_ADR = 0xe000_100c
LBA_BYTES = 512

# Helpers ------------------------------------------------------------------------------------------

def beats_from_bytes(data, beat_bytes):
    return [int.from_bytes(data[i:i + beat_bytes], "little") for i in range(0, len(data), beat_bytes)]

def bytes_from_beats(beats, beat_bytes):
    return b"".join(b.to_bytes(beat_bytes, "little") for b in beats)

def pattern_bytes(lba, count, seed):
    # Independent Python model of litenvme.pattern: word(D) = D ^ seed, D = absolute dword index.
    out = bytearray()
    for d in range(lba * LBA_BYTES // 4, (lba + count) * LBA_BYTES // 4):
        out += (((d & 0xffffffff) ^ seed) & 0xffffffff).to_bytes(4, "little")
    return bytes(out)

# DUT ----------------------------------------------------------------------------------------------

class _DUT(Module):
    def __init__(self, data_width=256, qsize=8, qd=7, nslots=4, slot_bytes=0x400, with_pattern=False):
        self.ctrl_port = PCIeCompleterPort(data_width)
        self.data_port = PCIeCompleterPort(data_width)
        self.submodules.eng = eng = LiteNVMeIOEngineAXI(
            qid=1, qsize=qsize, qd=qd, data_width=data_width, with_csr=False,
            hostmem_base=CTRL_BASE)
        self.submodules.st = st = LiteNVMeStreamer(
            data_width=data_width, buf_base=DATA_BASE, nslots=nslots, slot_bytes=slot_bytes,
            with_csr=False)
        self.submodules.ctrl = LiteNVMeHostMemResponder(
            port=self.ctrl_port, base=CTRL_BASE, size=CTRL_SIZE, data_width=data_width,
            with_csr=False, extra_axi_masters=[eng.axi])
        self.submodules.data = LiteNVMeHostMemResponder(
            port=self.data_port, base=DATA_BASE, size=st.size, data_width=data_width,
            with_csr=False, extra_axi_masters=[st.dma.axi])
        self.comb += [
            st.source.connect(eng.sink),
            eng.source.connect(st.sink),
        ]
        if with_pattern:
            self.submodules.gen = gen = LiteNVMeStreamPatternGen(data_width=data_width, with_csr=False)
            self.submodules.chk = chk = LiteNVMeStreamPatternCheck(data_width=data_width, with_csr=False)
            self.comb += [
                gen.source.connect(st.wr_sink),
                st.rd_source.connect(chk.sink),
            ]

# Test harness -------------------------------------------------------------------------------------

class _Harness:
    def __init__(self, dut, ssd, data_width, seed=0, src_gap=0.0, sink_gap=0.0):
        self.dut, self.ssd = dut, ssd
        self.beat_bytes = data_width // 8
        self.rng = random.Random(seed)
        self.src_gap, self.sink_gap = src_gap, sink_gap
        self.results = {}

    def setup(self):
        eng = self.dut.eng
        yield eng.sq_base.eq(SQ_BASE)
        yield eng.cq_base.eq(CQ_BASE)
        yield eng.sq_db_adr.eq(SQ_DB_ADR)
        yield eng.cq_db_adr.eq(CQ_DB_ADR)
        yield eng.prp_list_base.eq(CTRL_BASE + CTRL_SIZE)  # unused: slots span <= 2 pages.
        yield eng.enable.eq(1)
        yield

    def transfer(self, write, lba, count, timeout=200000):
        st = self.dut.st
        yield st.write.eq(write)
        yield st.lba.eq(lba)
        yield st.count.eq(count)
        yield st.start.eq(1)
        yield
        yield st.start.eq(0)
        yield
        cyc = 0
        while not (yield st.done):
            yield
            cyc += 1
            if cyc > timeout:
                raise RuntimeError(f"transfer timeout (write={write} lba={lba} count={count})")
        return dict(
            cycles    = (yield st.cycles),
            beats     = (yield st.data_beats),
            stalls    = (yield st.stall_cycles),
            submitted = (yield st.submitted),
            completed = (yield st.completed),
            err_count = (yield st.err_count),
            err_stat  = (yield st.err_status),
            stale     = (yield st.stale_count),
            hwm       = (yield st.slots_hwm),
            error     = (yield st.error),
        )

    def source(self, beats):
        """Drive wr_sink with `beats`, with random valid gaps."""
        s = self.dut.st.wr_sink
        for b in beats:
            while self.rng.random() < self.src_gap:
                yield s.valid.eq(0)
                yield
            yield s.data.eq(b)
            yield s.valid.eq(1)
            yield
            while not (yield s.ready):
                yield
        yield s.valid.eq(0)
        yield

    def sink(self, nbeats, key):
        """Collect `nbeats` from rd_source with random ready gaps; record `last` positions."""
        s = self.dut.st.rd_source
        got, lasts = [], []
        while len(got) < nbeats:
            rdy = self.rng.random() >= self.sink_gap
            yield s.ready.eq(rdy)
            yield
            if rdy and (yield s.valid):
                got.append((yield s.data))
                if (yield s.last):
                    lasts.append(len(got) - 1)
        yield s.ready.eq(0)
        yield
        self.results[key] = (got, lasts)

# Tests --------------------------------------------------------------------------------------------

class TestStreamDMA(unittest.TestCase):
    def test_fill_drain_rate(self):
        dw, n = 256, 64
        class DUT(Module):
            def __init__(self):
                self.submodules.dma = LiteNVMeStreamDMA(data_width=dw)
                self.submodules.ram = LiteNVMeHostMemAXIRAM(size=n * dw // 8, data_width=dw, with_dbg=False)
                self.comb += self.dma.axi.connect(self.ram.axi)
        dut  = DUT()
        d    = dut.dma
        rng  = random.Random(1)
        data = [rng.getrandbits(dw) for _ in range(n)]
        res  = {}

        def command(we):
            yield d.we.eq(we); yield d.adr.eq(0); yield d.beats.eq(n); yield d.last_en.eq(1)
            yield d.start.eq(1); yield
            yield d.start.eq(0)

        def tb():
            # Fill from an always-valid source; count cycles until the last B is received.
            yield from command(1)
            yield
            c = 1
            while (yield d.busy):
                yield; c += 1
                self.assertLess(c, 2000)
            res["fill"] = c
            # Drain into an always-ready sink.
            yield d.source.ready.eq(1)
            yield from command(0)
            got, last_at, c = [], None, 0
            while len(got) < n:
                yield
                c += 1
                if (yield d.source.valid):
                    got.append((yield d.source.data))
                    if (yield d.source.last):
                        last_at = len(got) - 1
                self.assertLess(c, 2000)
            res["drain"], res["got"], res["last"] = c, got, last_at

        @passive
        def src():
            i = 0
            while True:
                yield d.sink.valid.eq(i < n)
                yield d.sink.data.eq(data[i] if i < n else 0)
                yield
                if i < n and (yield d.sink.ready):
                    i += 1

        run_simulation(dut, [tb(), src()])
        print(f"\nstream DMA: fill {n} beats in {res['fill']} cycles, drain in {res['drain']} cycles")
        self.assertEqual(res["got"], data)
        self.assertEqual(res["last"], n - 1)
        # ~1 beat/clk: a few cycles of pipeline fill/response latency on top of `n`.
        self.assertLessEqual(res["fill"],  n + 8)
        self.assertLessEqual(res["drain"], n + 8)


class TestStreamer(unittest.TestCase):
    def _make(self, data_width=256, nslots=4, slot_bytes=0x400, latency=0, reorder=1,
              status_for=None, seed=0, with_pattern=False):
        dut = _DUT(data_width=data_width, nslots=nslots, slot_bytes=slot_bytes,
                   with_pattern=with_pattern)
        ssd = NVMeSSDModel(
            windows    = [(CTRL_BASE, CTRL_SIZE, dut.ctrl_port),
                          (DATA_BASE, dut.st.size, dut.data_port)],
            data_width = data_width, sq_base=SQ_BASE, cq_base=CQ_BASE, qsize=8,
            sq_db_adr  = SQ_DB_ADR, cq_db_adr=CQ_DB_ADR, lba_bytes=LBA_BYTES,
            latency    = latency, reorder=reorder, status_for=status_for, seed=seed)
        return dut, ssd

    def _roundtrip(self, data_width=256, nslots=4, slot_bytes=0x400, count=19, latency=0,
                   reorder=1, src_gap=0.0, sink_gap=0.0, seed=0):
        dut, ssd = self._make(data_width, nslots, slot_bytes, latency, reorder, seed=seed)
        h = _Harness(dut, ssd, data_width, seed=seed, src_gap=src_gap, sink_gap=sink_gap)
        rng = random.Random(seed)
        bb  = data_width // 8
        wr_lba, rd_lba = 100, 5000
        wr_data = bytes(rng.getrandbits(8) for _ in range(count * LBA_BYTES))
        # Range the DUT never writes: its content comes only from the SSD model.
        for i in range(count):
            ssd.flash[rd_lba + i] = bytes(rng.getrandbits(8) for _ in range(LBA_BYTES))
        rd_expect = b"".join(ssd.flash[rd_lba + i] for i in range(count))

        stats = {}
        def ctrl():
            yield from h.setup()
            stats["wr"] = yield from h.transfer(1, wr_lba, count)
            stats["rd"] = yield from h.transfer(0, rd_lba, count)
            stats["rd2"] = yield from h.transfer(0, wr_lba, count)  # read back what we wrote.
        def data():
            yield from h.source(beats_from_bytes(wr_data, bb))
            yield from h.sink(count * LBA_BYTES // bb, "rd")
            yield from h.sink(count * LBA_BYTES // bb, "rd2")
        run_simulation(dut, [ctrl(), data(), ssd.process(), ssd.mmio_process(dut.eng)])

        nbeats = count * LBA_BYTES // bb
        flash  = b"".join(ssd.flash.get(wr_lba + i, b"") for i in range(count))
        self.assertEqual(flash, wr_data, "SSD flash does not hold the streamed write data")
        got, lasts = h.results["rd"]
        self.assertEqual(bytes_from_beats(got, bb), rd_expect, "read data != SSD contents")
        self.assertEqual(lasts, [nbeats - 1])
        got, lasts = h.results["rd2"]
        self.assertEqual(bytes_from_beats(got, bb), wr_data, "read-back != written data")
        self.assertEqual(lasts, [nbeats - 1])
        nchunks = -(-count * LBA_BYTES // slot_bytes)
        for k in ("wr", "rd", "rd2"):
            s = stats[k]
            self.assertEqual(s["error"], 0, f"{k}: {s}")
            self.assertEqual(s["submitted"], nchunks)
            self.assertEqual(s["completed"], nchunks)
            self.assertEqual(s["beats"], nbeats)
            self.assertLessEqual(s["hwm"], nslots)
        self.assertEqual(ssd.dma_errors, [])
        return dut, ssd, stats

    # Functional -----------------------------------------------------------------------------------

    def test_roundtrip_partial_last_chunk(self):
        # 19 sectors over 1 KiB slots: 9 full chunks + a 1-sector tail, ring wraps twice.
        self._roundtrip(count=19)

    def test_roundtrip_backpressure(self):
        # Random gaps on the user stream in both directions and SSD latency.
        self._roundtrip(count=16, latency=40, src_gap=0.3, sink_gap=0.3, seed=3)

    def test_roundtrip_out_of_order_completions(self):
        # The model completes commands in a random order within a window of 3.
        _, ssd, _ = self._roundtrip(count=24, reorder=3, latency=10, seed=7)
        cids = [c["cid"] for c in ssd.commands]
        self.assertEqual(len(cids), 3 * 12)

    def test_roundtrip_two_page_slots(self):
        # 8 KiB slots: each command uses PRP1 + PRP2.
        _, ssd, _ = self._roundtrip(nslots=2, slot_bytes=0x2000, count=40)
        self.assertTrue(any(c["prp2"] != 0 for c in ssd.commands))

    def test_error_status(self):
        # Third write command fails: the transfer still completes, reports it, frees all slots.
        dut, ssd = self._make(status_for=lambda k, cmd: SC_INVALID_FIELD if k == 2 else None)
        h = _Harness(dut, ssd, 256)
        stats = {}
        def ctrl():
            yield from h.setup()
            stats["wr"] = yield from h.transfer(1, 0, 12)
            stats["wr2"] = yield from h.transfer(1, 64, 4)  # next transfer starts clean.
        def data():
            yield from h.source([i for i in range(12 * 16)])
            yield from h.source([i for i in range(4 * 16)])
        run_simulation(dut, [ctrl(), data(), ssd.process(), ssd.mmio_process(dut.eng)])
        s = stats["wr"]
        self.assertEqual(s["error"], 1)
        self.assertEqual(s["err_count"], 1)
        self.assertEqual((s["err_stat"] >> 1) & 0xff, SC_INVALID_FIELD)
        self.assertEqual(s["completed"], 6)
        self.assertEqual(stats["wr2"]["error"], 0)

    def test_pattern_roundtrip(self):
        # PL pattern generator -> streamer -> SSD -> streamer -> PL checker.
        dut, ssd = self._make(with_pattern=True)
        h = _Harness(dut, ssd, 256)
        lba, count, seed = 300, 14, 0x5eed1234
        res = {}
        def ctrl():
            gen, chk = dut.gen, dut.chk
            yield from h.setup()
            for m in (gen, chk):
                yield m.lba.eq(lba); yield m.count.eq(count); yield m.seed.eq(seed)
            yield gen.start.eq(1); yield
            yield gen.start.eq(0)
            res["wr"] = yield from h.transfer(1, lba, count)
            yield chk.start.eq(1); yield
            yield chk.start.eq(0)
            res["rd"] = yield from h.transfer(0, lba, count)
            yield
            res["beats"], res["errors"] = (yield chk.beats), (yield chk.errors)
            res["last_error"] = (yield chk.last_error)
            # Corrupt one SSD byte and read again: the checker must see exactly one bad beat.
            b = bytearray(ssd.flash[lba + 5]); b[100] ^= 0x40; ssd.flash[lba + 5] = bytes(b)
            yield chk.start.eq(1); yield
            yield chk.start.eq(0)
            yield from h.transfer(0, lba, count)
            yield
            res["errors2"], res["first2"] = (yield chk.errors), (yield chk.first_err)
        run_simulation(dut, [ctrl(), ssd.process(), ssd.mmio_process(dut.eng)])
        flash = b"".join(ssd.flash[lba + i] for i in range(count))
        flash_ok = bytearray(flash); flash_ok[5 * LBA_BYTES + 100] ^= 0x40
        self.assertEqual(bytes(flash_ok), pattern_bytes(lba, count, seed))
        self.assertEqual(res["beats"], count * 16)
        self.assertEqual(res["errors"], 0)
        self.assertEqual(res["last_error"], 0)
        self.assertEqual(res["errors2"], 1)
        self.assertEqual(res["first2"], (5 * LBA_BYTES + 100) // 32)

    # Performance ----------------------------------------------------------------------------------

    def _stream_both_ways(self, latency, count=32, nslots=4, slot_bytes=0x400):
        dut, ssd = self._make(nslots=nslots, slot_bytes=slot_bytes, latency=latency)
        h = _Harness(dut, ssd, 256)
        stats = {}
        def ctrl():
            yield from h.setup()
            for k, w in (("wr", 1), ("rd", 0)):
                i0 = ssd.idle_cycles
                stats[k] = yield from h.transfer(w, 0, count)
                stats[k]["ssd_busy"] = 1 - (ssd.idle_cycles - i0) / stats[k]["cycles"]
        def data():
            yield from h.source(list(range(count * LBA_BYTES // 32)))
            yield from h.sink(count * LBA_BYTES // 32, "rd")
        run_simulation(dut, [ctrl(), data(), ssd.process(), ssd.mmio_process(dut.eng)])
        for k in ("wr", "rd"):
            s = stats[k]
            s["cyc_per_cmd"] = s["cycles"] / s["submitted"]
            print(f"\nlatency {latency} {k}: {s['beats']} beats in {s['cycles']} cycles "
                  f"({s['beats'] / s['cycles']:.2f} beat/clk), {s['cyc_per_cmd']:.0f} cycles/command, "
                  f"SSD busy {s['ssd_busy']:.0%}, user stalls {s['stalls']}, slot hwm {s['hwm']}")
        return stats

    def test_overlap_keeps_ssd_busy(self):
        # When the device is the bottleneck, the ring must keep it fed: the (sequential) SSD
        # model should be busy for nearly the whole transfer in both directions. A serialized
        # fill -> command -> wait front-end would leave it idle during every fill/drain.
        stats = self._stream_both_ways(latency=200)
        for k in ("wr", "rd"):
            self.assertGreater(stats[k]["ssd_busy"], 0.85, f"{k}: SSD model starved")
            self.assertEqual(stats[k]["hwm"], 4, f"{k}: ring not fully used")

    def test_engine_command_rate_floor(self):
        # With a fast device the per-command cost of the I/O engine (16 dword SQE writes, CQE
        # reap, doorbells) sets the floor: ~107 cycles/command with 256-bit beats. Pin it so a
        # regression is visible; this is what bounds throughput for small slots (at 125 MHz,
        # ~1.2 M commands/s -> ~37 GB/s at 32 KiB but ~4.7 GB/s at 4 KiB per command).
        stats = self._stream_both_ways(latency=20)
        for k in ("wr", "rd"):
            self.assertLess(stats[k]["cyc_per_cmd"], 125, f"{k}: engine command rate regressed")


class _StubPHY(LiteXModule):
    """Separate-channel PHY stub (USP-style) exposing only what LitePCIeEndpoint needs."""
    endianness = "little"
    def __init__(self, data_width):
        self.data_width = data_width
        self.req_sink   = stream.Endpoint(phy_layout(data_width))
        self.cmp_sink   = stream.Endpoint(phy_layout(data_width))
        self.req_source = stream.Endpoint(phy_layout(data_width))
        self.cmp_source = stream.Endpoint(phy_layout(data_width))


class TestStreamerCore(unittest.TestCase):
    def test_core_elaborates_with_streamer(self):
        # LiteNVMe(with_streamer=True) on a real LitePCIe RootPort: the slot ring gets its own
        # crossbar slave port + responder, the streamer owns the engine, and Verilog converts.
        class Top(LiteXModule):
            def __init__(self):
                self.phy  = _StubPHY(256)
                self.ep   = LitePCIeRootPort(self.phy, endianness="little", address_width=64,
                                             with_configuration=True)
                self.nvme = LiteNVMe(self.ep, hostmem_size=0x8000, qsize=8, qd=7,
                                     with_streamer=True, streamer_nslots=4,
                                     streamer_slot_bytes=0x1000)
        top = Top()
        verilog.convert(top)
        self.assertEqual(len(top.ep.crossbar.user_slaves), 2)
        self.assertIsNone(top.nvme.request)
        self.assertEqual(top.nvme.streamer_mem.size, 4 * 0x1000)

    def test_overlapping_regions_rejected(self):
        with self.assertRaisesRegex(AssertionError, "overlaps"):
            LiteNVMe(LitePCIeRootPort(_StubPHY(256), endianness="little", address_width=64,
                                      with_configuration=True),
                     hostmem_base=0x1000_0000, hostmem_size=0x8000, with_streamer=True,
                     streamer_base=0x1000_4000, streamer_nslots=2, streamer_slot_bytes=0x1000)


if __name__ == "__main__":
    unittest.main()
