# This file is part of LiteNVMe.
#
# Copyright (c) 2026 Krishna Swaroop <swaroopdhulipalla99@gmail.com>
# SPDX-License-Identifier: BSD-2-Clause

import unittest

from migen import *
from migen.sim import run_simulation

from litex.gen import LiteXModule
from litex.soc.interconnect import stream

from litenvme.gen import LiteNVMeCore
from litenvme.pattern import LiteNVMeStreamPatternCheck


def _pattern_beat(lba, beat, seed):
    value = 0
    for lane in range(8):
        dword = (lba * 128 + beat * 8 + lane) & 0xffffffff
        value |= ((dword ^ seed) & 0xffffffff) << (32 * lane)
    return value


class _Pads:
    def __init__(self, data_width):
        self.tvalid = Signal()
        self.tready = Signal()
        self.tlast  = Signal()
        self.tdata  = Signal(data_width)


class _Platform:
    def __init__(self, data_width):
        self.data_width = data_width

    def add_extension(self, _extension):
        pass

    def request(self, _name):
        return _Pads(self.data_width)


class _StreamStub(LiteXModule):
    def __init__(self, data_width):
        self.wr_sink   = stream.Endpoint([("data", data_width)])
        self.rd_source = stream.Endpoint([("data", data_width)])
        self.start     = Signal()
        self.start_en  = Signal()
        self.write     = Signal()
        self.lba       = Signal(64)
        self.count     = Signal(32)
        self.nsid      = Signal(32)
        self.busy      = Signal()
        self.error     = Signal()


class TestPatternAbort(unittest.TestCase):
    def test_checker_aborts_prefix_then_accepts_clean_transfer(self):
        dut = LiteNVMeStreamPatternCheck(data_width=256, with_csr=False)
        result = {}
        prefix_lba, prefix_seed = 7, 0x1298abef
        next_lba, next_seed = 19, 0x5eed1234

        def bench():
            yield dut.lba.eq(prefix_lba)
            yield dut.count.eq(2)
            yield dut.seed.eq(prefix_seed)
            yield dut.start.eq(1)
            yield
            yield dut.start.eq(0)

            for beat in range(5):
                yield dut.sink.valid.eq(1)
                data = _pattern_beat(prefix_lba, beat, prefix_seed)
                yield dut.sink.data.eq(data ^ (1 if beat == 1 else 0))
                yield dut.sink.last.eq(0)
                yield

            # A beat arriving with abort asserted must remain unacknowledged.
            yield dut.sink.valid.eq(1)
            yield dut.sink.data.eq(_pattern_beat(prefix_lba, 5, prefix_seed))
            yield dut.sink.last.eq(0)
            yield dut.abort.eq(1)
            yield
            result["abort_ready"] = (yield dut.sink.ready)
            result["abort_handshake"] = (yield dut.sink.valid) & (yield dut.sink.ready)
            yield  # The counter and checker consume abort on the following edge.
            yield dut.sink.valid.eq(0)
            result["aborted"] = {
                "busy": (yield dut.busy),
                "done": (yield dut.done),
                "ready": (yield dut.sink.ready),
                "last_error": (yield dut.last_error),
                "beats": (yield dut.beats),
                "errors": (yield dut.errors),
                "first_err": (yield dut.first_err),
            }
            yield dut.abort.eq(0)

            yield dut.lba.eq(next_lba)
            yield dut.count.eq(1)
            yield dut.seed.eq(next_seed)
            yield dut.start.eq(1)
            yield
            yield dut.start.eq(0)
            for beat in range(16):
                yield dut.sink.valid.eq(1)
                yield dut.sink.data.eq(_pattern_beat(next_lba, beat, next_seed))
                yield dut.sink.last.eq(beat == 15)
                yield
            yield dut.sink.valid.eq(0)
            yield dut.sink.last.eq(0)
            yield
            result["complete"] = {
                "busy": (yield dut.busy),
                "done": (yield dut.done),
                "last_error": (yield dut.last_error),
                "beats": (yield dut.beats),
                "errors": (yield dut.errors),
                "first_err": (yield dut.first_err),
            }

        run_simulation(dut, bench())
        self.assertEqual(result["aborted"], {
            "busy": 0, "done": 1, "ready": 0, "last_error": 1,
            "beats": 5, "errors": 1, "first_err": 1,
        })
        self.assertEqual(result["abort_ready"], 0)
        self.assertEqual(result["abort_handshake"], 0)
        self.assertEqual(result["complete"], {
            "busy": 0, "done": 1, "last_error": 0,
            "beats": 16, "errors": 0, "first_err": 0,
        })

    def test_stream_ring_pulses_abort_once_and_allows_next_check(self):
        width = 256
        dut = LiteXModule()
        streamer = _StreamStub(width)
        LiteNVMeCore.add_stream_ring(
            dut, _Platform(width), streamer, width, Signal(reset=1), "csr", with_pattern=True,
        )
        checker = dut.nvme_pattern_check
        mux = dut.nvme_stream_mux
        result = {}
        seed = 0x10293847

        def pulse_start(lba, count, pattern_seed):
            yield checker.ctrl._lba_lo.storage.eq(lba & 0xffffffff)
            yield checker.ctrl._lba_hi.storage.eq((lba >> 32) & 0xffffffff)
            yield checker.ctrl._count.storage.eq(count)
            yield checker.ctrl._seed.storage.eq(pattern_seed)
            yield checker.ctrl._ctrl.fields.start.eq(1)
            yield
            yield checker.ctrl._ctrl.fields.start.eq(0)
            yield

        def bench():
            yield mux._sel.fields.pattern.eq(1)
            yield streamer.error.eq(0)
            yield streamer.busy.eq(1)
            yield  # Arm the previous-busy sampler before starting the checker.
            yield from pulse_start(0, 1, seed)

            for beat in range(3):
                yield streamer.rd_source.valid.eq(1)
                yield streamer.rd_source.data.eq(_pattern_beat(0, beat, seed))
                yield streamer.rd_source.last.eq(0)
                yield

            yield streamer.rd_source.valid.eq(0)
            yield streamer.error.eq(1)
            yield streamer.busy.eq(0)
            yield
            result["abort_pulse_first"] = (yield checker.abort)
            yield
            result["abort_pulse_second"] = (yield checker.abort)
            result["aborted"] = {
                "busy": (yield checker.busy),
                "done": (yield checker.done),
                "last_error": (yield checker.last_error),
                "beats": (yield checker.beats),
                "errors": (yield checker.errors),
            }

            # st.error stays high. Its retirement pulse is gone, so a fresh check starts.
            yield from pulse_start(0, 1, seed)
            for beat in range(16):
                yield streamer.rd_source.valid.eq(1)
                yield streamer.rd_source.data.eq(_pattern_beat(0, beat, seed))
                yield streamer.rd_source.last.eq(beat == 15)
                yield
            yield streamer.rd_source.valid.eq(0)
            yield streamer.rd_source.last.eq(0)
            yield
            result["complete"] = {
                "busy": (yield checker.busy),
                "done": (yield checker.done),
                "last_error": (yield checker.last_error),
                "beats": (yield checker.beats),
                "errors": (yield checker.errors),
            }

        run_simulation(dut, bench())
        self.assertEqual(result["abort_pulse_first"], 1, result)
        self.assertEqual(result["abort_pulse_second"], 0)
        self.assertEqual(result["aborted"], {
            "busy": 0, "done": 1, "last_error": 1, "beats": 3, "errors": 0,
        })
        self.assertEqual(result["complete"], {
            "busy": 0, "done": 1, "last_error": 0, "beats": 16, "errors": 0,
        })

    def test_stream_ring_does_not_abort_idle_checker_on_write_error(self):
        width = 256
        dut = LiteXModule()
        streamer = _StreamStub(width)
        LiteNVMeCore.add_stream_ring(
            dut, _Platform(width), streamer, width, Signal(reset=1), "csr", with_pattern=True,
        )
        checker = dut.nvme_pattern_check
        mux = dut.nvme_stream_mux
        result = {}

        def bench():
            yield mux._sel.fields.pattern.eq(1)
            yield streamer.write.eq(1)
            yield streamer.error.eq(0)
            yield streamer.busy.eq(1)
            yield  # Arm the previous-busy sampler for the errored write.
            yield streamer.busy.eq(0)
            yield streamer.error.eq(1)
            yield
            result["abort"] = (yield checker.abort)
            yield
            result["status"] = {
                "busy": (yield checker.busy),
                "done": (yield checker.done),
                "last_error": (yield checker.last_error),
                "beats": (yield checker.beats),
                "errors": (yield checker.errors),
                "first_err": (yield checker.first_err),
            }

        run_simulation(dut, bench())
        self.assertEqual(result["abort"], 0)
        self.assertEqual(result["status"], {
            "busy": 0, "done": 1, "last_error": 0,
            "beats": 0, "errors": 0, "first_err": 0,
        })


if __name__ == "__main__":
    unittest.main()
