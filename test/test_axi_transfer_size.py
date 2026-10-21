# This file is part of LiteNVMe.
#
# Copyright (c) 2026 Krishna Swaroop <swaroopdhulipalla99@gmail.com>
# SPDX-License-Identifier: BSD-2-Clause
#
# AXI AxSIZE is log2(bytes per transfer). Keep this oracle independent of LiteX's
# AXSIZE lookup table, which is incorrect for 32- and 64-byte beats in the pinned
# dependency.

import unittest

from migen import *
from migen.sim import run_simulation
from migen.fhdl import verilog
from litex.soc.interconnect import stream

from litenvme.block import LiteNVMeHostMemStreamDMA
from litenvme.hostmem import LiteNVMeHostMemCSR, LiteNVMeHostMemDMA
from litenvme.io_engine import LiteNVMeMemPort, LiteNVMeMemPortToAXI


# AXI AxSIZE encodes log2(bytes per beat): 8, 16, 32 and 64 bytes respectively.
EXPECTED_SIZE = ((64, 3), (128, 4), (256, 5), (512, 6))


def _hostmem_port(data_width):
    beat_bytes = data_width // 8
    req_layout = [
        ("adr",    64),
        ("we",     1),
        ("len",    10),
        ("tag",    8),
        ("req_id", 16),
        ("dat",    data_width),
        ("be",     beat_bytes),
    ]
    cmp_layout = [
        ("dat",    data_width),
        ("end",    1),
        ("len",    10),
        ("tag",    8),
        ("req_id", 16),
        ("cmp_id", 16),
        ("err",    1),
    ]

    class Port(Module):
        def __init__(self):
            self.sink = stream.Endpoint(req_layout)
            self.source = stream.Endpoint(cmp_layout)

    return Port()


class TestAXITransferSize(unittest.TestCase):
    def _convert_ports(self, dut, size_signals):
        # Convert the actual component with its AxSIZE fields exported. Functional
        # values are checked by simulation below; conversion also catches port-width
        # and Verilog lowering regressions in these Migen modules.
        converted = verilog.convert(dut, ios=set(size_signals))
        self.assertTrue(converted.main_source)

    def _assert_sizes(self, case_fn):
        actuals = {}
        for data_width, expected in EXPECTED_SIZE:
            with self.subTest(data_width=data_width):
                actuals[data_width] = case_fn(data_width)
        self.assertEqual(actuals, dict(EXPECTED_SIZE),
            "AXI AxSIZE must be log2(bytes per beat) at every supported width")

    def _stream_dma_case(self, data_width, write):
        dut = LiteNVMeHostMemStreamDMA(data_width=data_width)
        axi = dut.axi
        size = axi.aw.size if write else axi.ar.size
        valid = axi.aw.valid if write else axi.ar.valid
        actual = {}

        def tb():
            yield axi.aw.ready.eq(0)
            yield axi.w.ready.eq(0)
            yield axi.ar.ready.eq(0)
            yield dut.start.eq(0)
            yield dut.we.eq(int(write))
            yield dut.adr.eq(0)
            yield dut.beats.eq(1)
            yield
            yield
            yield dut.start.eq(1)
            yield
            yield dut.start.eq(0)
            for _ in range(12):
                if (yield valid):
                    break
                yield
            actual["valid"] = (yield valid)
            actual["size"] = (yield size)

        run_simulation(dut, tb())
        self.assertEqual(actual.get("valid"), 1, "expected an active AXI address transfer")
        return actual["size"]

    def test_block_stream_dma_write_and_read_ports(self):
        for write in (True, False):
            with self.subTest(direction="write" if write else "read"):
                self._assert_sizes(lambda width: self._stream_dma_case(width, write))

    def _hostmem_csr_write_case(self, data_width):
        beat_dwords = data_width // 32
        dut = LiteNVMeHostMemCSR(
            data_width=data_width,
            beat_dwords=beat_dwords,
            beat_dwords_shift=log2_int(beat_dwords),
        )
        actual = {}

        def tb():
            yield dut.axi.aw.ready.eq(0)
            yield dut.axi.w.ready.eq(0)
            yield dut.axi.b.valid.eq(0)
            yield dut._csr_adr.storage.eq(0)
            yield dut._csr_wdata.storage.eq(0x12345678)
            yield dut._csr_we.storage.eq(0)
            yield
            yield dut._csr_we.storage.eq(1)
            yield
            yield dut._csr_we.storage.eq(0)
            for _ in range(12):
                if (yield dut.axi.aw.valid):
                    break
                yield
            actual["valid"] = (yield dut.axi.aw.valid)
            actual["size"] = (yield dut.axi.aw.size)

        run_simulation(dut, tb())
        self.assertEqual(actual.get("valid"), 1, "expected an active CSR AXI write")
        return actual["size"]

    def test_hostmem_csr_write_port(self):
        self._assert_sizes(self._hostmem_csr_write_case)

    def _hostmem_dma_case(self, data_width, write):
        port = _hostmem_port(data_width)
        beat_bytes = data_width // 8
        beat_dwords = data_width // 32
        dut = LiteNVMeHostMemDMA(
            port=port,
            data_width=data_width,
            base=0x1000_0000,
            size=0x4000,
            beat_bytes_shift=log2_int(beat_bytes),
            beat_dwords=beat_dwords,
            beat_dwords_shift=log2_int(beat_dwords),
        )
        axi = dut.axi
        size = axi.aw.size if write else axi.ar.size
        valid = axi.aw.valid if write else axi.ar.valid
        actual = {}

        def tb():
            yield axi.aw.ready.eq(0)
            yield axi.w.ready.eq(0)
            yield axi.ar.ready.eq(0)
            yield axi.r.valid.eq(0)
            yield port.source.ready.eq(0)
            req = port.sink
            yield req.valid.eq(0)
            yield req.first.eq(0)
            yield req.last.eq(0)
            yield req.adr.eq(0x1000_0100)
            yield req.we.eq(int(write))
            yield req.len.eq(beat_dwords)
            yield req.tag.eq(1)
            yield req.req_id.eq(0x1234)
            yield req.dat.eq(0x1234)
            yield req.be.eq((1 << beat_bytes) - 1)
            yield
            yield req.first.eq(1)
            yield req.last.eq(1)
            yield req.valid.eq(1)
            for _ in range(20):
                if (yield req.ready):
                    break
                yield
            self.assertEqual((yield req.ready), 1, "host-memory request was not accepted")
            yield
            yield req.valid.eq(0)
            yield req.first.eq(0)
            yield req.last.eq(0)
            for _ in range(20):
                if (yield valid):
                    break
                yield
            actual["valid"] = (yield valid)
            actual["size"] = (yield size)

        run_simulation(dut, tb())
        self.assertEqual(actual.get("valid"), 1, "expected an active host-memory AXI request")
        return actual["size"]

    def test_hostmem_responder_write_and_read_ports(self):
        for write in (True, False):
            with self.subTest(direction="write" if write else "read"):
                self._assert_sizes(lambda width: self._hostmem_dma_case(width, write))

    def _engine_bridge_case(self, data_width, write):
        mem = LiteNVMeMemPort(adr_width=32)
        dut = LiteNVMeMemPortToAXI(mem, data_width=data_width)
        axi = dut.axi
        size = axi.aw.size if write else axi.ar.size
        valid = axi.aw.valid if write else axi.ar.valid
        actual = {}

        def tb():
            yield axi.aw.ready.eq(0)
            yield axi.w.ready.eq(0)
            yield axi.ar.ready.eq(0)
            yield mem.stb.eq(0)
            yield mem.adr.eq(0)
            yield mem.dat_w.eq(0x12345678)
            yield mem.we.eq(int(write))
            yield
            yield
            yield mem.stb.eq(1)
            for _ in range(12):
                if (yield valid):
                    break
                yield
            actual["valid"] = (yield valid)
            actual["size"] = (yield size)

        run_simulation(dut, tb())
        self.assertEqual(actual.get("valid"), 1, "expected an active engine AXI request")
        return actual["size"]

    def test_engine_memory_bridge_write_and_read_ports(self):
        for write in (True, False):
            with self.subTest(direction="write" if write else "read"):
                self._assert_sizes(lambda width: self._engine_bridge_case(width, write))

    def test_verilog_port_conversion(self):
        for data_width, _expected in EXPECTED_SIZE:
            with self.subTest(data_width=data_width):
                block = LiteNVMeHostMemStreamDMA(data_width=data_width)
                self._convert_ports(block, (block.axi.aw.size, block.axi.ar.size))

                beat_dwords = data_width // 32
                csr = LiteNVMeHostMemCSR(
                    data_width=data_width,
                    beat_dwords=beat_dwords,
                    beat_dwords_shift=log2_int(beat_dwords),
                )
                self._convert_ports(csr, (csr.axi.aw.size,))

                port = _hostmem_port(data_width)
                beat_bytes = data_width // 8
                responder = LiteNVMeHostMemDMA(
                    port=port,
                    data_width=data_width,
                    base=0x1000_0000,
                    size=0x4000,
                    beat_bytes_shift=log2_int(beat_bytes),
                    beat_dwords=beat_dwords,
                    beat_dwords_shift=log2_int(beat_dwords),
                )
                self._convert_ports(responder, (responder.axi.aw.size, responder.axi.ar.size))

                mem = LiteNVMeMemPort(adr_width=32)
                bridge = LiteNVMeMemPortToAXI(mem, data_width=data_width)
                self._convert_ports(bridge, (bridge.axi.aw.size, bridge.axi.ar.size))


if __name__ == "__main__":
    unittest.main()
