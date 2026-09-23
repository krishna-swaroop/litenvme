# This file is part of LiteNVMe.
#
# Copyright (c) 2026 Krishna Swaroop <swaroopdhulipalla99@gmail.com>
# SPDX-License-Identifier: BSD-2-Clause
#
# Host-memory responder: a device MemRd whose address is not beat-aligned must complete with the
# requested dwords only, first dword first (CplD payload is dword-aligned), in ceil(len/beat)
# beats. SSDs issue such reads e.g. for PRP-list entries (8 B apart) or small SQ/CQ slices.

import random
import unittest

from migen import *
from migen.sim import run_simulation

from litenvme.hostmem import LiteNVMeHostMemResponder

from ssd_model import PCIeCompleterPort

BASE = 0x1000_0000
SIZE = 0x400


class TestHostMemUnalignedRead(unittest.TestCase):
    def _run(self, data_width, cases):
        bd   = data_width // 32
        port = PCIeCompleterPort(data_width)
        dut  = LiteNVMeHostMemResponder(port=port, base=BASE, size=SIZE, data_width=data_width,
                                        with_csr=False)
        rng  = random.Random(data_width)
        mem  = [rng.getrandbits(32) for _ in range(SIZE // 4)]
        got  = {}

        def tb():
            s, c = port.sink, port.source
            # Preload with full-beat MemWr.
            nbeats = SIZE // (4 * bd)
            for i in range(nbeats):
                yield s.adr.eq(BASE); yield s.we.eq(1); yield s.len.eq((SIZE // 4) & 0x3ff)
                yield s.dat.eq(sum(mem[i * bd + j] << (32 * j) for j in range(bd)))
                yield s.first.eq(i == 0); yield s.last.eq(i == nbeats - 1); yield s.valid.eq(1)
                yield
                while not (yield s.ready):
                    yield
            yield s.valid.eq(0)
            for _ in range(8):
                yield
            # Unaligned MemRds.
            for off, n in cases:
                yield s.adr.eq(BASE + 4 * off); yield s.we.eq(0); yield s.len.eq(n)
                yield s.first.eq(1); yield s.last.eq(1); yield s.valid.eq(1)
                yield
                while not (yield s.ready):
                    yield
                yield s.valid.eq(0)
                yield c.ready.eq(1)
                beats, cyc = [], 0
                while True:
                    yield
                    if (yield c.valid):
                        beats.append(((yield c.dat), (yield c.last)))
                        if beats[-1][1]:
                            break
                    cyc += 1
                    self.assertLess(cyc, 500, f"no completion for off={off} len={n}")
                yield c.ready.eq(0)
                got[(off, n)] = beats

        run_simulation(dut, tb())
        for off, n in cases:
            beats = got[(off, n)]
            dws = [(b >> (32 * j)) & 0xffffffff for b, _ in beats for j in range(bd)]
            self.assertEqual(len(beats), -(-n // bd), f"beat count off={off} len={n}")
            self.assertEqual([l for _, l in beats], [0] * (len(beats) - 1) + [1])
            self.assertEqual(dws[:n], mem[off:off + n], f"payload off={off} len={n}")

    def test_unaligned_256b(self):
        self._run(256, [(o, n) for o in range(8) for n in (1, 2, 5, 8, 9, 16, 17)])

    def test_unaligned_128b(self):
        self._run(128, [(o, n) for o in range(4) for n in (1, 2, 3, 4, 5, 9)])


if __name__ == "__main__":
    unittest.main()
