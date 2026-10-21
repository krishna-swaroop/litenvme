# SPDX-License-Identifier: BSD-2-Clause
"""A late DDR read fault must poison the PCIe header before any CplD escapes."""
from collections import deque
import random

import pytest
from migen import Module
from migen.sim import passive, run_simulation
from litex.soc.interconnect import axi
from litenvme.hostmem import LiteNVMeHostMemResponder
from ssd_model import PCIeCompleterPort

BASE = 0x10000000


class ExternalRAM(Module):
    def __init__(self, width):
        self.axi = axi.AXIInterface(data_width=width, address_width=32, id_width=1)


def exercise(width, offset_dw, length_dw, fail_index, response=2, *, packetized=False, follow_healthy=False):
    backend = ExternalRAM(width)
    port = PCIeCompleterPort(width)
    dut = LiteNVMeHostMemResponder(port, base=BASE, size=16384,
                                   data_width=width, with_csr=False, backend=backend)
    top = dut
    if packetized:
        from litepcie.tlp.packetizer import LitePCIeTLPPacketizer
        top = Module()
        top.submodules.responder = dut
        top.submodules.packetizer = packetizer = LitePCIeTLPPacketizer(
            data_width=width, endianness="little", capabilities=["COMPLETION"])
        top.comb += port.source.connect(packetizer.cmp_sink)
    beat_dw = width // 32
    count = (offset_dw % beat_dw + length_dw + beat_dw - 1) // beat_dw
    rng = random.Random(width * 65537 + offset_dw + length_dw)
    words = [rng.getrandbits(32) for _ in range(4096)]
    stats = {"received": 0, "issued": 0}
    output = []
    healthy = []

    @passive
    def ram():
        bus = backend.axi
        pending = deque()
        yield bus.ar.ready.eq(1)
        yield bus.r.valid.eq(0)
        while True:
            if (yield bus.r.valid) and (yield bus.r.ready):
                pending.popleft()
                stats["received"] += 1
            if (yield bus.ar.valid) and (yield bus.ar.ready):
                address = (yield bus.ar.addr)
                assert (yield bus.ar.len) == 0
                assert (yield bus.ar.size) == (width // 8).bit_length() - 1
                index = stats["issued"]
                stats["issued"] += 1
                data = sum(words[address // 4 + lane] << (32 * lane) for lane in range(beat_dw))
                pending.append((data, response if index == fail_index else 0))
            if pending:
                yield bus.r.valid.eq(1)
                yield bus.r.data.eq(pending[0][0])
                yield bus.r.resp.eq(pending[0][1])
                yield bus.r.last.eq(1)
                yield bus.r.id.eq(0)
            else:
                yield bus.r.valid.eq(0)
            yield

    def stimulus():
        s = port.sink
        c = packetizer.source if packetized else port.source
        yield c.ready.eq(0)
        yield s.we.eq(0)
        yield s.adr.eq(BASE + 4 * offset_dw)
        yield s.len.eq(length_dw)
        yield s.tag.eq(73)
        yield s.req_id.eq(0x1234)
        yield s.first.eq(1)
        yield s.last.eq(1)
        yield s.valid.eq(1)
        for _ in range(100):
            yield
            if (yield s.ready):
                break
        else:
            pytest.fail("MemRd was never accepted")
        yield s.valid.eq(0)
        stalled = None
        for cycle in range(10000):
            ready = rng.randrange(4) != 0
            yield c.ready.eq(ready)
            yield
            if (yield c.valid):
                if packetized:
                    current = ((yield c.dat), (yield c.be), (yield c.first), (yield c.last))
                else:
                    current = ((yield c.dat), (yield c.err), (yield c.first),
                               (yield c.last), (yield c.end), (yield c.tag),
                               (yield c.req_id), (yield c.len))
                # The final backend response can change the packet status. No
                # successful first header can escape before it is known.
                assert stats["received"] == count, "completion published before all DDR responses"
                if stalled is not None:
                    assert current == stalled, "completion changed while stalled"
                stalled = current if not ready else None
                if ready:
                    output.append(current)
                    if current[3]:
                        break
        else:
            pytest.fail("completion deadlock (including requests larger than the old 64-beat FIFO)")
        if follow_healthy:
            assert not packetized
            yield c.ready.eq(0)
            for _ in range(4):
                yield
            yield s.tag.eq(74)
            yield s.valid.eq(1)
            for _ in range(100):
                yield
                if (yield s.ready):
                    break
            else:
                pytest.fail("healthy request after error was not accepted")
            yield s.valid.eq(0)
            yield c.ready.eq(1)
            for _ in range(10000):
                yield
                if (yield c.valid):
                    assert (yield c.err) == 0
                    assert (yield c.tag) == 74
                    healthy.append((yield c.dat))
                    if (yield c.last):
                        break
            else:
                pytest.fail("healthy completion after error deadlocked")

    run_simulation(top, [ram(), stimulus()])
    if packetized:
        dwords = []
        for data, be, first, last in output:
            for lane in range(beat_dw):
                nibble = (be >> (4 * lane)) & 15
                assert nibble in (0, 15)
                if nibble:
                    dwords.append((data >> (32 * lane)) & 0xffffffff)
        assert len(dwords) == 3, "error Cpl must contain exactly its three-DWORD header"
        assert len(output) == (3 + beat_dw - 1) // beat_dw
        assert [b[2] for b in output] == [1] + [0] * (len(output) - 1)
        assert [b[3] for b in output] == [0] * (len(output) - 1) + [1]
        assert (dwords[0] >> 24) & 0x7f == 0x0a  # Cpl without data.
        assert dwords[0] & 0x3ff == 0
        assert (dwords[1] >> 13) & 7 == 1  # Existing packetizer maps err to UR.
        assert dwords[1] & 0xfff == 0
        assert (dwords[2] >> 16) == 0x1234
        assert (dwords[2] >> 8) & 255 == 73
        return
    poisoned = fail_index is not None
    assert len(output) == (1 if poisoned else (length_dw + beat_dw - 1) // beat_dw)
    assert [beat[1] for beat in output] == [int(poisoned)] * len(output)
    assert [beat[2] for beat in output] == [1] + [0] * (len(output) - 1)
    assert [beat[3] for beat in output] == [0] * (len(output) - 1) + [1]
    assert [beat[4] for beat in output] == [0] * (len(output) - 1) + [1]
    assert all(beat[5:] == (73, 0x1234, 0 if poisoned else length_dw) for beat in output)
    payload = [(beat[0] >> (32 * lane)) & 0xffffffff for beat in output for lane in range(beat_dw)]
    if poisoned:
        assert all(word == 0 for word in payload), "failed memory payload leaked into completion"
    else:
        assert payload[:length_dw] == words[offset_dw:offset_dw + length_dw]
    if follow_healthy:
        assert len(healthy) == (length_dw + beat_dw - 1) // beat_dw
        actual = [(value >> (32 * lane)) & 0xffffffff for value in healthy for lane in range(beat_dw)]
        assert actual[:length_dw] == words[offset_dw:offset_dw + length_dw]


@pytest.mark.parametrize("width", [64, 128, 256])
@pytest.mark.parametrize("case", ["first", "late", "unaligned_flush", "maximum", "healthy"])
def test_ddr_error_is_known_before_pcie_header(width, case):
    lanes = width // 32
    if case == "first":
        exercise(width, 0, lanes * 3, 0)
    elif case == "late":
        exercise(width, 0, lanes * 3, 2, response=3)
    elif case == "unaligned_flush":
        # Last backend beat participates in both a combined beat and a flush.
        exercise(width, 1, lanes * 2, 2)
    elif case == "maximum":
        length = 1023
        reads = (1 + length + lanes - 1) // lanes
        exercise(width, 1, length, reads - 1)
    else:
        exercise(width, 1, lanes * 3 + 1, None)


@pytest.mark.parametrize("width", [64, 128, 256])
def test_maximum_healthy_read_completes(width):
    exercise(width, 1, 1023, None)


@pytest.mark.parametrize("width", [64, 128, 256])
def test_late_ddr_error_packetizes_as_one_header_only_completion(width):
    lanes = width // 32
    exercise(width, 1, lanes * 3, 3, packetized=True)


@pytest.mark.parametrize("width", [64, 128, 256])
def test_failed_request_then_healthy_request_has_no_stale_poison_or_payload(width):
    lanes = width // 32
    exercise(width, 1, lanes * 2, 2, follow_healthy=True)
