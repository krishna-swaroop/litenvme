#
# This file is part of LiteNVMe.
#
# Copyright (c) 2026 Krishna Swaroop <swaroopdhulipalla99@gmail.com>
# SPDX-License-Identifier: BSD-2-Clause

"""LiteNVMe stream pattern generator / checker — position-encoded test data for SSD round trips.

Every 32-bit word of the pattern encodes its absolute position on the drive:

    word(D) = D[31:0] ^ seed,    D = lba * (lba_bytes / 4) + dword offset inside the transfer

so a read-back compare detects both corrupted data and data landing at the wrong LBA/offset
(e.g. a slot or PRP mix-up), not just a stuck bus. Changing `seed` between runs makes stale
drive contents from an earlier run fail the check.

Both blocks take {start, lba, count (logical blocks), seed} and run `count * lba_bytes` bytes.
The generator asserts `last` on the final beat; the checker counts mismatching beats and
records the index of the first one.
"""

from migen import *

from litex.gen import *

from litex.soc.interconnect import stream
from litex.soc.interconnect.csr import *

# Helpers ------------------------------------------------------------------------------------------

def _pattern_beat(dw_idx, seed, beat_dwords):
    return Cat(*[((dw_idx + i)[:32] ^ seed) for i in range(beat_dwords)])

class _PatternCounter(LiteXModule):
    """Shared control: latches the command and tracks the dword index / beats left."""
    def __init__(self, data_width, lba_bytes, with_csr):
        beat_dwords = data_width // 32
        lba_dwords  = lba_bytes // 4
        self.start  = Signal()
        self.lba    = Signal(64)
        self.count  = Signal(32)
        self.seed   = Signal(32)
        self.abort  = Signal(reset=0)  # Abort a truncated transfer while preserving checker statistics.
        self.busy   = Signal()
        self.done   = Signal()

        self.dw_idx     = Signal(32)
        self.beats_left = Signal(32)
        self.seed_r     = Signal(32)
        self.advance    = Signal()  # One beat produced/consumed this cycle.

        # # #

        beats_per_lba_shift = log2_int(lba_bytes // (data_width // 8))
        done_r = Signal(reset=1)
        self.comb += [
            self.busy.eq(self.beats_left != 0),
            self.done.eq(done_r),
        ]
        self.sync += [
            If(self.abort,
                self.beats_left.eq(0),
                done_r.eq(1),
            ).Elif(self.start & ~self.busy,
                self.dw_idx.eq(self.lba * lba_dwords),
                self.beats_left.eq(self.count << beats_per_lba_shift),
                self.seed_r.eq(self.seed),
                done_r.eq(self.count == 0),
            ).Elif(self.advance,
                self.dw_idx.eq(self.dw_idx + beat_dwords),
                self.beats_left.eq(self.beats_left - 1),
                If(self.beats_left == 1, done_r.eq(1)),
            )
        ]

    def add_csr(self):
        self._lba_lo = CSRStorage(32, description="Start LBA low.")
        self._lba_hi = CSRStorage(32, description="Start LBA high.")
        self._count  = CSRStorage(32, description="Number of logical blocks.")
        self._seed   = CSRStorage(32, description="Pattern seed.")
        self._ctrl   = CSRStorage(fields=[
            CSRField("start", size=1, offset=0, pulse=True, description="Start."),
        ])
        self.comb += [
            self.lba.eq(Cat(self._lba_lo.storage, self._lba_hi.storage)),
            self.count.eq(self._count.storage),
            self.seed.eq(self._seed.storage),
            self.start.eq(self._ctrl.fields.start),
        ]

# Pattern Generator --------------------------------------------------------------------------------

class LiteNVMeStreamPatternGen(LiteXModule):
    def __init__(self, data_width=256, lba_bytes=512, with_csr=True):
        self.source = stream.Endpoint([("data", data_width)])

        # # #

        self.ctrl = ctrl = _PatternCounter(data_width, lba_bytes, with_csr)
        self.start, self.lba, self.count, self.seed = ctrl.start, ctrl.lba, ctrl.count, ctrl.seed
        self.busy,  self.done = ctrl.busy, ctrl.done

        self.comb += [
            self.source.valid.eq(ctrl.busy),
            self.source.data.eq(_pattern_beat(ctrl.dw_idx, ctrl.seed_r, data_width // 32)),
            self.source.last.eq(ctrl.beats_left == 1),
            ctrl.advance.eq(self.source.valid & self.source.ready),
        ]

        if with_csr:
            ctrl.add_csr()
            self._status = CSRStatus(fields=[
                CSRField("done", size=1, offset=0),
                CSRField("busy", size=1, offset=1),
            ])
            self.comb += [
                self._status.fields.done.eq(self.done),
                self._status.fields.busy.eq(self.busy),
            ]

# Pattern Checker ----------------------------------------------------------------------------------

class LiteNVMeStreamPatternCheck(LiteXModule):
    def __init__(self, data_width=256, lba_bytes=512, with_csr=True):
        self.sink = stream.Endpoint([("data", data_width)])

        self.beats      = Signal(32)  # Beats checked.
        self.errors     = Signal(32)  # Mismatching beats.
        self.first_err  = Signal(32)  # Beat index of the first mismatch.
        self.last_error = Signal()    # Bad `last`, or a checker aborted on a truncated prefix.

        # # #

        self.ctrl = ctrl = _PatternCounter(data_width, lba_bytes, with_csr)
        self.start, self.lba, self.count, self.seed = ctrl.start, ctrl.lba, ctrl.count, ctrl.seed
        self.busy,  self.done = ctrl.busy, ctrl.done
        self.abort = ctrl.abort

        expected = Signal(data_width)
        fire     = Signal()
        self.comb += [
            self.sink.ready.eq(ctrl.busy & ~ctrl.abort),
            expected.eq(_pattern_beat(ctrl.dw_idx, ctrl.seed_r, data_width // 32)),
            fire.eq(self.sink.valid & self.sink.ready),
            ctrl.advance.eq(fire),
        ]
        self.sync += [
            # A failed stream read can end after a valid prefix; retain its measurements.
            If(ctrl.abort,
                self.last_error.eq(1),
            ).Elif(ctrl.start & ~ctrl.busy,
                self.beats.eq(0),
                self.errors.eq(0),
                self.first_err.eq(0),
                self.last_error.eq(0),
            ).Elif(fire,
                self.beats.eq(self.beats + 1),
                If(self.sink.data != expected,
                    self.errors.eq(self.errors + 1),
                    If(self.errors == 0, self.first_err.eq(self.beats)),
                ),
                If(self.sink.last != (ctrl.beats_left == 1),
                    self.last_error.eq(1),
                ),
            )
        ]

        if with_csr:
            ctrl.add_csr()
            self._status = CSRStatus(fields=[
                CSRField("done",       size=1, offset=0),
                CSRField("busy",       size=1, offset=1),
                CSRField("last_error", size=1, offset=2),
            ])
            self._beats     = CSRStatus(32, description="Beats checked.")
            self._errors    = CSRStatus(32, description="Mismatching beats.")
            self._first_err = CSRStatus(32, description="Beat index of the first mismatch.")
            self.comb += [
                self._status.fields.done.eq(self.done),
                self._status.fields.busy.eq(self.busy),
                self._status.fields.last_error.eq(self.last_error),
                self._beats.status.eq(self.beats),
                self._errors.status.eq(self.errors),
                self._first_err.status.eq(self.first_err),
            ]
