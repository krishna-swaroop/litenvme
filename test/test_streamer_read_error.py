# This file is part of LiteNVMe.
#
# Copyright (c) 2026 Krishna Swaroop <swaroopdhulipalla99@gmail.com>
# SPDX-License-Identifier: BSD-2-Clause

"""Fail-stop read checks against the independent SSD/PCIe model."""

import unittest

from migen.sim import run_simulation, passive
from migen import Module

from ssd_model import NVMeSSDModel, SC_INVALID_FIELD
from litenvme.streamer import LiteNVMeStreamer
from test_streamer import (
    CTRL_BASE, SQ_BASE, CQ_BASE, DATA_BASE, SQ_DB_ADR, CQ_DB_ADR, LBA_BYTES,
    _DUT, _Harness, bytes_from_beats,
)


def _beats(data, beat_bytes):
    return [int.from_bytes(data[i:i + beat_bytes], "little")
            for i in range(0, len(data), beat_bytes)]


def _lba_data(lba, salt=0):
    return bytes(((salt + 31 * lba + 13 * i) & 0xff) for i in range(LBA_BYTES))


def _bounded_source(sink, beats, timeout=200000):
    """Send a fixed seed-write payload with a hard bound on missing ready."""
    cycles = 0
    for beat in beats:
        yield sink.data.eq(beat)
        yield sink.valid.eq(1)
        yield
        cycles += 1
        while not (yield sink.ready):
            if cycles >= timeout:
                raise RuntimeError("seed write source timed out waiting for ready")
            yield
            cycles += 1
        if cycles >= timeout:
            raise RuntimeError("seed write source timed out")
    yield sink.valid.eq(0)
    yield


class TestStreamerReadError(unittest.TestCase):
    seed_lba = 100
    read_lba = 1000
    clean_lba = 2000
    chunk_sectors = 2  # 1 KiB slots.

    def _make(self, fail_chunk, count=12, nslots=4, reorder=1, latency=0, seed=0):
        def status_for(_index, cmd):
            if (cmd["op"] == 0x02 and fail_chunk is not None and
                    cmd["slba"] == self.read_lba + fail_chunk * self.chunk_sectors):
                return SC_INVALID_FIELD
            return None

        dut = _DUT(data_width=256, nslots=nslots, slot_bytes=0x400)
        ssd = NVMeSSDModel(
            windows=[(CTRL_BASE, 0x400, dut.ctrl_port),
                     (DATA_BASE, dut.st.size, dut.data_port)],
            data_width=256, sq_base=SQ_BASE, cq_base=CQ_BASE, qsize=8,
            sq_db_adr=SQ_DB_ADR, cq_db_adr=CQ_DB_ADR, lba_bytes=LBA_BYTES,
            latency=latency, reorder=reorder, status_for=status_for, seed=seed)
        h = _Harness(dut, ssd, data_width=256)

        # Put recognizable old bytes in every slot through a successful write transfer.
        seed_data = bytes(((0xd3 + 7 * i) & 0xff) for i in range(count * LBA_BYTES))
        seed_beats = _beats(seed_data, 32)
        for i in range(count):
            ssd.flash[self.read_lba + i] = _lba_data(self.read_lba + i, salt=0x29)
            ssd.flash[self.clean_lba + i] = _lba_data(self.clean_lba + i, salt=0x71)
        return dut, ssd, h, seed_beats

    def _read_and_collect(self, dut, lba, count, stall_cycles=0, timeout=200000):
        st = dut.st
        source = st.rd_source
        yield st.write.eq(0)
        yield st.lba.eq(lba)
        yield st.count.eq(count)
        yield st.start.eq(1)
        yield
        yield st.start.eq(0)
        yield

        beats, lasts = [], []
        held = None
        held_cycles = 0
        saw_held = False
        cycles = 0
        while True:
            ready = stall_cycles == 0 or (held is not None and held_cycles >= stall_cycles)
            yield source.ready.eq(ready)
            yield
            valid = (yield source.valid)
            if not ready:
                if held is not None:
                    self.assertTrue(valid, "rd_source.valid dropped while ready was low")
                if valid:
                    current = ((yield source.data), (yield source.last))
                    if held is None:
                        held = current
                    else:
                        self.assertEqual(current, held,
                                         "rd_source changed while ready was low")
                    held_cycles += 1
                    saw_held = True
            elif ready and valid:
                beats.append((yield source.data))
                if (yield source.last):
                    lasts.append(len(beats) - 1)

            cycles += 1
            if (yield st.done):
                break
            if cycles >= timeout:
                raise RuntimeError("read transfer timed out")

        yield source.ready.eq(0)
        yield
        return dict(
            beats=beats, lasts=lasts, saw_held=saw_held,
            cycles=(yield st.cycles), submitted=(yield st.submitted),
            completed=(yield st.completed), error=(yield st.error),
            err_count=(yield st.err_count), err_status=(yield st.err_status),
        )

    def _run(self, fail_chunk, count=12, nslots=4, reorder=1, latency=0, seed=0,
             stall_cycles=0, clean_read=False):
        dut, ssd, h, seed_beats = self._make(
            fail_chunk, count=count, nslots=nslots, reorder=reorder, latency=latency, seed=seed)
        result, completions = {}, []
        monitor_scope = {"name": None}
        clean_count = min(count, 4)

        def control():
            yield from h.setup()
            result["seed"] = yield from h.transfer(1, self.seed_lba, count)
            monitor_scope["name"] = "failed_read"
            result["read"] = yield from self._read_and_collect(
                dut, self.read_lba, count, stall_cycles=stall_cycles)
            monitor_scope["name"] = None
            if clean_read:
                result["clean"] = yield from self._read_and_collect(
                    dut, self.clean_lba, clean_count)

        def data():
            yield from _bounded_source(dut.st.wr_sink, seed_beats)

        @passive
        def completion_monitor():
            while True:
                yield
                if monitor_scope["name"] is not None and (yield dut.st.sink.valid):
                    completions.append((monitor_scope["name"], (yield dut.st.sink.tag),
                                        (yield dut.st.sink.status)))

        run_simulation(dut, [control(), data(), ssd.process(),
                             ssd.mmio_process(dut.eng), completion_monitor()])
        return result, ssd, completions

    def _assert_prefix(self, result, ssd, count, fail_chunk):
        read = result["read"]
        valid_sectors = min(count, fail_chunk * self.chunk_sectors)
        expected = b"".join(ssd.flash[self.read_lba + i] for i in range(valid_sectors))
        got = bytes_from_beats(read["beats"], 32)
        self.assertEqual(got, expected, "read output must be exactly the successful LBA prefix")
        self.assertEqual(read["lasts"], [], "failed reads must not synthesize a success TLAST")
        self.assertEqual(read["error"], 1)
        self.assertGreaterEqual(read["err_count"], 1)
        self.assertEqual((read["err_status"] >> 1) & 0xff, SC_INVALID_FIELD)

    def test_failed_first_read_hides_seeded_slot_and_next_transfer_is_clean(self):
        count = 12
        result, ssd, _ = self._run(
            fail_chunk=0, count=count, nslots=2, clean_read=True)
        self._assert_prefix(result, ssd, count, fail_chunk=0)
        self.assertEqual(result["read"]["beats"], [])
        self.assertLess(result["read"]["submitted"], count // self.chunk_sectors,
                        "the failed completion must stop issuing unsubmitted reads")

        clean = result["clean"]
        clean_expected = b"".join(ssd.flash[self.clean_lba + i] for i in range(4))
        self.assertEqual(bytes_from_beats(clean["beats"], 32), clean_expected)
        self.assertEqual(clean["lasts"], [len(clean["beats"]) - 1])
        self.assertEqual(clean["error"], 0, "a new transfer must clear the prior error")

    def test_failed_middle_read_emits_exact_prefix(self):
        result, ssd, _ = self._run(fail_chunk=2, count=12, nslots=4)
        self._assert_prefix(result, ssd, count=12, fail_chunk=2)
        self.assertEqual(len(result["read"]["beats"]), 4 * 16)

    def test_failed_last_read_emits_prefix_without_tlast(self):
        count = 8
        result, ssd, _ = self._run(fail_chunk=3, count=count, nslots=4)
        self._assert_prefix(result, ssd, count=count, fail_chunk=3)
        self.assertEqual(len(result["read"]["beats"]), 6 * 16)

    def test_out_of_order_error_completion_preserves_lba_prefix(self):
        result, ssd, completions = self._run(
            fail_chunk=2, count=12, nslots=4, reorder=3, latency=5, seed=5)
        self._assert_prefix(result, ssd, count=12, fail_chunk=2)
        read_completions = [(tag, status) for scope, tag, status in completions
                            if scope == "failed_read"]
        error_pos = next(i for i, (_tag, status) in enumerate(read_completions)
                         if ((status >> 1) & 0x7ff) != 0)
        prefix_positions = [i for i, (tag, status) in enumerate(read_completions)
                            if tag in (0, 1) and ((status >> 1) & 0x7ff) == 0]
        self.assertTrue(prefix_positions)
        self.assertEqual(read_completions[error_pos][0], 2)
        self.assertLess(error_pos, min(prefix_positions),
                        "the error completion should arrive before the first LBA slot")

    def test_backpressure_keeps_pending_read_beat_stable(self):
        result, ssd, _ = self._run(
            fail_chunk=2, count=12, nslots=4, latency=5, stall_cycles=400)
        self._assert_prefix(result, ssd, count=12, fail_chunk=2)
        self.assertTrue(result["read"]["saw_held"],
                        "test did not observe a valid beat held by output backpressure")

    def test_sct_error_with_zero_sc_marks_read_failed(self):
        class DUT(Module):
            def __init__(self):
                self.submodules.st = LiteNVMeStreamer(
                    data_width=256, nslots=2, slot_bytes=0x400, with_csr=False)

        dut = DUT()
        st = dut.st
        result = {}

        def tb():
            yield st.source.ready.eq(1)
            yield st.lba.eq(self.read_lba)
            yield st.count.eq(self.chunk_sectors)
            yield st.start.eq(1)
            yield
            yield st.start.eq(0)
            yield

            cycles = 0
            while (yield st.submitted) == 0:
                yield
                cycles += 1
                self.assertLess(cycles, 100)

            # NVMe status bits [11:9] carry SCT. SC remains zero here.
            yield st.sink.tag.eq(0)
            yield st.sink.status.eq(1 << 9)
            yield st.sink.valid.eq(1)
            yield
            yield st.sink.valid.eq(0)

            cycles = 0
            while not (yield st.done):
                yield
                cycles += 1
                self.assertLess(cycles, 100)
            result["error"] = (yield st.error)
            result["err_count"] = (yield st.err_count)
            result["err_status"] = (yield st.err_status)
            result["submitted"] = (yield st.submitted)
            result["completed"] = (yield st.completed)

        run_simulation(dut, tb())
        self.assertEqual(result["error"], 1)
        self.assertEqual(result["err_count"], 1)
        self.assertEqual(result["err_status"], 1 << 9)
        self.assertEqual(result["submitted"], 1)
        self.assertEqual(result["completed"], 1)


if __name__ == "__main__":
    unittest.main()
