#
# This file is part of LiteNVMe.
#
# Copyright (c) 2026 Krishna Swaroop <swaroopdhulipalla99@gmail.com>
# SPDX-License-Identifier: BSD-2-Clause

"""LiteNVMe stream ring — sustained AXI-Stream write/read front-end over the I/O engine.

The block streamer (litenvme/block.py) runs one command at a time: fill the staging buffer,
issue one engine command, wait for its completion, then accept more data. That serializes
the user stream with the SSD latency. This front-end instead cuts the transfer into chunks
and cycles them through a ring of `nslots` buffer slots, so filling slot N+1, the SSD moving
slot N and the completion of slot N-1 all overlap:

  - write : wr_sink fills slot[fill] (pipelined, ~1 beat/clk); once its data is committed to
            host memory (all AXI write responses received), a WRITE command is queued for the
            slot and the next free slot is filled. A slot returns to FREE on its completion.
  - read  : READ commands are queued for every FREE slot (up to `nslots` outstanding). Slots
            are drained to rd_source strictly in LBA order as their completions arrive (the
            device may complete in any order), then returned to FREE.

Slot states: FREE -> BUSY (submitted, SSD owns the buffer) -> FREE (write) or
             FREE -> BUSY -> READY (completed, data valid) -> FREE (read, after drain).

A slot is never refilled or re-submitted while the device may still access it, and read data
is only exposed after its completion. Completions are matched to slots with the engine request
`tag` (the slot index), so the front-end never predicts the engine CID allocation.

Host memory: the slot ring is meant to live in its own host-memory region, served by a second
LiteNVMeHostMemResponder on its own crossbar slave port (see LiteNVMe(with_streamer=True)).
The streamer DMA (`dma.axi`) is then the only local master on that region's backend, so the
user stream never competes with the engine's SQE/CQE accesses, and the SSD's data DMA runs on
the opposite AXI channel (write mode: SSD reads while we write; read mode: the reverse).

Interface (control as plain signals — a CSR wrapper is added with `with_csr=True`):

    start, write, lba(64), count(32 sectors), nsid(32)      # transfer command
    done (registered, 1 at idle/complete), busy, error (sticky per transfer)
    wr_sink   : stream Endpoint [data]  — write payload in (count * lba_bytes bytes)
    rd_source : stream Endpoint [data]  — read payload out (last on the final beat)
    source/sink : engine-facing request/completion streams (connect to engine.sink/source).

The transfer is cut into chunks of `slot_bytes` (the final chunk may be shorter). `count` is
in logical blocks of `lba_bytes`. A `count` of 0 completes immediately.
"""

from migen import *

from litex.gen import *

from litex.soc.interconnect import stream
from litex.soc.interconnect import axi
from litex.soc.interconnect.csr import *

from litenvme.io_engine import request_layout, completion_layout, TAG_WIDTH

# Constants ----------------------------------------------------------------------------------------

SLOT_FREE  = 0
SLOT_BUSY  = 1  # Submitted: the SSD owns the buffer.
SLOT_READY = 2  # Read completed: data valid, waiting to be drained.

# Stream DMA ---------------------------------------------------------------------------------------

class LiteNVMeStreamDMA(LiteXModule):
    """Pipelined beat-wide AXI master moving a data stream into / out of a flat backend.

    One command at a time: drive `we` (1=fill from `sink`, 0=drain to `source`), `adr`
    (backend byte address, beat-aligned), `beats`, `last_en` (assert `source.last` on the
    final drained beat), then pulse `start`; `busy` stays high until the command is complete.

    Unlike LiteNVMeHostMemStreamDMA this does not wait for each response before issuing the
    next beat: fills issue one AW+W per cycle and count B responses, drains keep up to
    `max_reads` single-beat reads outstanding. A fill only completes once every B response has
    been received, i.e. once the whole payload is committed to the backend.

    `max_reads` must stay below the backend read-response buffering (64 for
    LiteNVMeHostMemAXIRAM), since a stalled `source` stops draining R.
    """
    def __init__(self, data_width=256, address_width=32, max_reads=16):
        self.axi    = axi.AXIInterface(data_width=data_width, address_width=address_width,
                                       id_width=1, mode="rw")
        self.sink   = stream.Endpoint([("data", data_width)])  # fill data in.
        self.source = stream.Endpoint([("data", data_width)])  # drain data out.

        # Command.
        self.start   = Signal()
        self.we      = Signal()
        self.adr     = Signal(address_width)
        self.beats   = Signal(32)
        self.last_en = Signal()
        self.busy    = Signal()

        # # #

        beat_bytes = data_width // 8
        axsize     = axi.AXSIZE[beat_bytes]

        adr        = Signal(address_width)  # Next AW/AR address.
        issue_left = Signal(32)             # Beats still to issue (AW+W or AR).
        resp_left  = Signal(32)             # Responses still to receive (B or R).
        last_r     = Signal()
        aw_done    = Signal()
        w_done     = Signal()
        rd_pending = Signal(max=max_reads + 1)

        self.comb += [
            self.axi.aw.addr.eq(adr),
            self.axi.aw.len.eq(0),
            self.axi.aw.size.eq(axsize),
            self.axi.aw.burst.eq(axi.BURST_INCR),
            self.axi.aw.id.eq(0),
            self.axi.w.data.eq(self.sink.data),
            self.axi.w.strb.eq(2**beat_bytes - 1),
            self.axi.w.last.eq(1),
            self.axi.b.ready.eq(1),
            self.axi.ar.addr.eq(adr),
            self.axi.ar.len.eq(0),
            self.axi.ar.size.eq(axsize),
            self.axi.ar.burst.eq(axi.BURST_INCR),
            self.axi.ar.id.eq(0),
            self.source.data.eq(self.axi.r.data),
        ]

        self.fsm = fsm = FSM(reset_state="IDLE")
        self.comb += self.busy.eq(~fsm.ongoing("IDLE"))

        fsm.act("IDLE",
            If(self.start & (self.beats != 0),
                NextValue(adr,        self.adr),
                NextValue(issue_left, self.beats),
                NextValue(resp_left,  self.beats),
                NextValue(last_r,     self.last_en),
                NextValue(aw_done,    0),
                NextValue(w_done,     0),
                If(self.we,
                    NextState("FILL"),
                ).Else(
                    NextState("DRAIN"),
                )
            )
        )

        # Fill: AW and W are independent channels; present both for the beat at the head of
        # `sink`, drop each once accepted, and consume the beat when both have been accepted.
        beat_v    = Signal()
        aw_ok     = Signal()
        w_ok      = Signal()
        beat_fire = Signal()
        self.comb += [
            beat_v.eq(fsm.ongoing("FILL") & (issue_left != 0) & self.sink.valid),
            self.axi.aw.valid.eq(beat_v & ~aw_done),
            self.axi.w.valid.eq(beat_v & ~w_done),
            aw_ok.eq(aw_done | (self.axi.aw.valid & self.axi.aw.ready)),
            w_ok.eq(w_done   | (self.axi.w.valid  & self.axi.w.ready)),
            beat_fire.eq(beat_v & aw_ok & w_ok),
            self.sink.ready.eq(beat_fire),
        ]
        fsm.act("FILL",
            If(beat_fire,
                NextValue(aw_done,    0),
                NextValue(w_done,     0),
                NextValue(adr,        adr + beat_bytes),
                NextValue(issue_left, issue_left - 1),
            ).Else(
                If(self.axi.aw.valid & self.axi.aw.ready, NextValue(aw_done, 1)),
                If(self.axi.w.valid  & self.axi.w.ready,  NextValue(w_done,  1)),
            ),
            If(self.axi.b.valid,
                NextValue(resp_left, resp_left - 1),
                If(resp_left == 1,
                    NextState("IDLE"),
                )
            )
        )

        # Drain: keep up to `max_reads` reads in flight; R flows to `source` under backpressure.
        ar_fire = Signal()
        r_fire  = Signal()
        self.comb += [
            self.axi.ar.valid.eq(fsm.ongoing("DRAIN") & (issue_left != 0) &
                                 (rd_pending < max_reads)),
            ar_fire.eq(self.axi.ar.valid & self.axi.ar.ready),
            self.source.valid.eq(fsm.ongoing("DRAIN") & self.axi.r.valid),
            self.source.last.eq(last_r & (resp_left == 1)),
            self.axi.r.ready.eq(fsm.ongoing("DRAIN") & self.source.ready),
            r_fire.eq(self.axi.r.valid & self.axi.r.ready),
        ]
        self.sync += [
            If(ar_fire & ~r_fire,
                rd_pending.eq(rd_pending + 1),
            ).Elif(r_fire & ~ar_fire,
                rd_pending.eq(rd_pending - 1),
            )
        ]
        fsm.act("DRAIN",
            If(ar_fire,
                NextValue(adr,        adr + beat_bytes),
                NextValue(issue_left, issue_left - 1),
            ),
            If(r_fire,
                NextValue(resp_left, resp_left - 1),
                If(resp_left == 1,
                    NextState("IDLE"),
                )
            )
        )

# LiteNVMe Streamer --------------------------------------------------------------------------------

class LiteNVMeStreamer(LiteXModule):
    """Chunked, overlapped AXI-Stream write/read front-end (see module docstring).

    Parameters
    ----------
    data_width : int
        Stream / host-memory beat width.
    buf_base : int
        Host (PCIe) byte address of slot 0; slot i is at buf_base + i*slot_bytes. This is the
        PRP address given to the SSD and must be page-aligned.
    nslots, slot_bytes : int
        Ring geometry. slot_bytes bounds the per-command transfer size and must be a multiple
        of lba_bytes. Slots either span whole pages (slot_bytes multiple of page_bytes) or pack
        inside one page (page_bytes multiple of slot_bytes), so a slot never straddles a page
        boundary at an offset (the engine builds PRPs for page-aligned buffers).
    """
    def __init__(self, data_width=256, buf_base=0x2000_0000, nslots=8, slot_bytes=0x8000,
                 lba_bytes=512, page_bytes=4096, max_reads=16, with_csr=True):
        beat_bytes = data_width // 8
        assert lba_bytes >= beat_bytes and (lba_bytes % beat_bytes) == 0
        assert (slot_bytes % lba_bytes) == 0
        assert (slot_bytes % page_bytes == 0) or (page_bytes % slot_bytes == 0)
        assert (buf_base % page_bytes) == 0
        assert 2 <= nslots <= 2**TAG_WIDTH
        assert (slot_bytes // lba_bytes) < 2**16  # engine nlb field.
        self.data_width = data_width
        self.buf_base   = buf_base
        self.nslots     = nslots
        self.slot_bytes = slot_bytes
        self.size       = nslots * slot_bytes

        chunk_sectors = slot_bytes // lba_bytes
        sec_to_beats  = log2_int(lba_bytes // beat_bytes)

        # Engine-facing streams (named like the other front-ends for a symmetric mux).
        self.source = stream.Endpoint(request_layout())     # -> engine.sink
        self.sink   = stream.Endpoint(completion_layout())  # <- engine.source

        # User data streams.
        self.wr_sink   = stream.Endpoint([("data", data_width)])
        self.rd_source = stream.Endpoint([("data", data_width)])

        # Control / status.
        self.start    = Signal()
        self.start_en = Signal(reset=1)  # qualifies `start` (e.g. tie to the core's init_done).
        self.write    = Signal()
        self.lba   = Signal(64)
        self.count = Signal(32)
        self.nsid  = Signal(32, reset=1)
        self.done  = Signal()
        self.busy  = Signal()
        self.error = Signal()

        # Per-transfer statistics.
        self.cycles        = Signal(64)  # start -> done.
        self.data_beats    = Signal(32)  # beats accepted on wr_sink / delivered on rd_source.
        self.stall_cycles  = Signal(64)  # cycles the user side was held off by the streamer.
        self.submitted     = Signal(32)  # commands queued to the engine.
        self.completed     = Signal(32)  # completions matched to a slot.
        self.err_count     = Signal(16)  # completions with a non-success status.
        self.err_status    = Signal(16)  # status of the first failing completion.
        self.stale_count   = Signal(16)  # completions whose tag did not match a BUSY slot.
        self.slots_hwm     = Signal(max=nslots + 1)  # high-water mark of busy slots.

        # # #

        # Stream DMA (joins the slot region's host-memory backend via .axi).
        self.dma = dma = LiteNVMeStreamDMA(data_width=data_width, address_width=32,
                                           max_reads=max_reads)
        self.comb += [
            self.wr_sink.connect(dma.sink),
            dma.source.connect(self.rd_source),
        ]

        # Command queue to the engine: the engine only accepts a request after writing its SQE,
        # so commands are queued and the data path moves on to the next slot immediately.
        self.cmd_fifo = cmd_fifo = stream.SyncFIFO(request_layout(), depth=nslots, buffered=False)
        self.comb += cmd_fifo.source.connect(self.source)

        # Latched transfer.
        w_write = Signal()
        w_nsid  = Signal(32)

        # Data-path (fill / drain) side: pointer, sectors left, LBA of the next write command.
        d_ptr  = Signal(max=nslots)
        d_left = Signal(32)
        d_nlb  = Signal(32)
        # Read issue side: pointer, sectors left to issue.
        i_ptr  = Signal(max=nslots)
        i_left = Signal(32)
        i_nlb  = Signal(32)
        # Next command LBA (write: advanced by the fill side; read: by the issue side).
        c_lba  = Signal(64)

        self.comb += [
            d_nlb.eq(Mux(d_left < chunk_sectors, d_left, chunk_sectors)),
            i_nlb.eq(Mux(i_left < chunk_sectors, i_left, chunk_sectors)),
        ]

        def ptr_inc(ptr):
            return If(ptr == (nslots - 1), NextValue(ptr, 0)).Else(NextValue(ptr, ptr + 1))

        # Slot state.
        slot_state = [Signal(2, name=f"slot{i}_state") for i in range(nslots)]
        slot_arr   = Array(slot_state)
        busy_count = Signal(max=nslots + 1)

        # Registered done (request_gen/block idiom: 1 at idle, cleared by start).
        done_r = Signal(reset=1)
        self.comb += self.done.eq(done_r)

        self.fsm = fsm = FSM(reset_state="IDLE")
        self.comb += self.busy.eq(~fsm.ongoing("IDLE"))

        # Command push (write: from the fill FSM; read: from the issue logic).
        wr_cmd_valid = Signal()
        rd_cmd_valid = Signal()
        push_ptr     = Signal(max=nslots)
        push         = Signal()
        self.comb += [
            rd_cmd_valid.eq((fsm.ongoing("RD-WAIT") | fsm.ongoing("RD-DRAIN")) &
                            (i_left != 0) & (slot_arr[i_ptr] == SLOT_FREE)),
            push_ptr.eq(Mux(w_write, d_ptr, i_ptr)),
            cmd_fifo.sink.valid.eq(wr_cmd_valid | rd_cmd_valid),
            cmd_fifo.sink.op.eq(w_write),
            cmd_fifo.sink.nsid.eq(w_nsid),
            cmd_fifo.sink.lba.eq(c_lba),
            cmd_fifo.sink.nlb.eq(Mux(w_write, d_nlb, i_nlb)),
            cmd_fifo.sink.buf.eq(buf_base + push_ptr * slot_bytes),
            cmd_fifo.sink.tag.eq(push_ptr),
            push.eq(cmd_fifo.sink.valid & cmd_fifo.sink.ready),
        ]

        # Read issue side advances on its own, concurrently with the drain FSM.
        self.sync += [
            If(push & ~w_write,
                c_lba.eq(c_lba + i_nlb),
                i_left.eq(i_left - i_nlb),
                If(i_ptr == (nslots - 1), i_ptr.eq(0)).Else(i_ptr.eq(i_ptr + 1)),
            )
        ]

        # Completion side: always accepted; matched to its slot by tag.
        cpl_sc     = Signal(8)
        cpl_sct    = Signal(3)
        cpl_tag_ok = Signal()
        cpl_ok     = Signal()
        cpl_err    = Signal()
        self.comb += [
            self.sink.ready.eq(1),
            cpl_sc.eq(self.sink.status[1:9]),
            cpl_sct.eq(self.sink.status[9:12]),
            cpl_tag_ok.eq((self.sink.tag < nslots) & (slot_arr[self.sink.tag] == SLOT_BUSY)),
            cpl_ok.eq(self.sink.valid & cpl_tag_ok),
            cpl_err.eq((cpl_sc != 0) | (cpl_sct != 0)),
        ]

        # Drain completion (read slot handed back).
        drain_done = Signal()

        # Per-slot state updates. In any cycle these events target distinct slots (a completion
        # targets a BUSY slot, a push a FREE one, a drain a READY one), so priority is moot.
        clear_slots = Signal()
        for i in range(nslots):
            self.sync += [
                If(clear_slots,
                    slot_state[i].eq(SLOT_FREE),
                ).Elif(cpl_ok & (self.sink.tag == i),
                    slot_state[i].eq(Mux(w_write, SLOT_FREE, SLOT_READY)),
                ).Elif(push & (push_ptr == i),
                    slot_state[i].eq(SLOT_BUSY),
                ).Elif(drain_done & (d_ptr == i),
                    slot_state[i].eq(SLOT_FREE),
                )
            ]

        # Busy-slot count / high-water (BUSY or READY: not yet reusable).
        self.sync += [
            If(clear_slots,
                busy_count.eq(0),
            ).Elif(push & ~(w_write & cpl_ok) & ~drain_done,
                busy_count.eq(busy_count + 1),
            ).Elif(~push & ((w_write & cpl_ok) | drain_done),
                busy_count.eq(busy_count - 1),
            ),
        ]

        # Statistics.
        running = Signal()
        self.comb += running.eq(~fsm.ongoing("IDLE"))
        self.sync += [
            If(clear_slots,
                self.cycles.eq(0),
                self.data_beats.eq(0),
                self.stall_cycles.eq(0),
                self.submitted.eq(0),
                self.completed.eq(0),
                self.err_count.eq(0),
                self.err_status.eq(0),
                self.stale_count.eq(0),
                self.slots_hwm.eq(0),
            ).Else(
                If(running, self.cycles.eq(self.cycles + 1)),
                If((self.wr_sink.valid & self.wr_sink.ready) |
                   (self.rd_source.valid & self.rd_source.ready),
                    self.data_beats.eq(self.data_beats + 1),
                ),
                If(running & w_write & self.wr_sink.valid & ~self.wr_sink.ready,
                    self.stall_cycles.eq(self.stall_cycles + 1),
                ),
                If(running & ~w_write & self.rd_source.ready & ~self.rd_source.valid,
                    self.stall_cycles.eq(self.stall_cycles + 1),
                ),
                If(busy_count > self.slots_hwm, self.slots_hwm.eq(busy_count)),
                If(push, self.submitted.eq(self.submitted + 1)),
                If(cpl_ok, self.completed.eq(self.completed + 1)),
                If(self.sink.valid & ~cpl_tag_ok, self.stale_count.eq(self.stale_count + 1)),
                If(cpl_ok & cpl_err,
                    self.err_count.eq(self.err_count + 1),
                    If(self.err_count == 0, self.err_status.eq(self.sink.status)),
                ),
            ),
        ]
        self.comb += self.error.eq((self.err_count != 0) | (self.stale_count != 0))

        # Outstanding commands (queued or at the device).
        outstanding = Signal(max=nslots + 1)
        self.sync += [
            If(push & ~cpl_ok,
                outstanding.eq(outstanding + 1),
            ).Elif(cpl_ok & ~push,
                outstanding.eq(outstanding - 1),
            )
        ]

        fsm.act("IDLE",
            If(self.start & self.start_en,
                clear_slots.eq(1),
                NextValue(w_write, self.write),
                NextValue(w_nsid,  self.nsid),
                NextValue(d_ptr,   0),
                NextValue(i_ptr,   0),
                NextValue(d_left,  self.count),
                NextValue(i_left,  Mux(self.write, 0, self.count)),
                NextValue(c_lba,   self.lba),
                NextValue(done_r,  0),
                If(self.write,
                    NextState("WR-WAIT"),
                ).Else(
                    NextState("RD-WAIT"),
                )
            )
        )

        # Write: fill the next slot as soon as it is FREE, queue its command once committed.
        fsm.act("WR-WAIT",
            If(d_left == 0,
                NextState("FINISH"),
            ).Elif(slot_arr[d_ptr] == SLOT_FREE,
                dma.start.eq(1),
                dma.we.eq(1),
                dma.adr.eq(d_ptr * slot_bytes),
                dma.beats.eq(d_nlb << sec_to_beats),
                NextState("WR-FILL"),
            )
        )
        fsm.act("WR-FILL",
            If(~dma.busy, NextState("WR-CMD")),
        )
        fsm.act("WR-CMD",
            wr_cmd_valid.eq(1),
            If(cmd_fifo.sink.ready,
                NextValue(c_lba,  c_lba + d_nlb),
                NextValue(d_left, d_left - d_nlb),
                ptr_inc(d_ptr),
                NextState("WR-WAIT"),
            )
        )

        # Read: drain slots in order as they complete (commands are issued concurrently).
        fsm.act("RD-WAIT",
            If(d_left == 0,
                NextState("FINISH"),
            ).Elif(slot_arr[d_ptr] == SLOT_READY,
                dma.start.eq(1),
                dma.we.eq(0),
                dma.adr.eq(d_ptr * slot_bytes),
                dma.beats.eq(d_nlb << sec_to_beats),
                dma.last_en.eq(d_left == d_nlb),
                NextState("RD-DRAIN"),
            )
        )
        fsm.act("RD-DRAIN",
            If(~dma.busy,
                drain_done.eq(1),
                NextValue(d_left, d_left - d_nlb),
                ptr_inc(d_ptr),
                NextState("RD-WAIT"),
            )
        )

        # Wait for every queued command to complete.
        fsm.act("FINISH",
            If(outstanding == 0,
                NextValue(done_r, 1),
                NextState("IDLE"),
            )
        )

        if with_csr:
            self.add_csr()

    # CSRs ---------------------------------------------------------------------------------------
    def add_csr(self):
        self._lba_lo = CSRStorage(32, description="Start LBA low.")
        self._lba_hi = CSRStorage(32, description="Start LBA high.")
        self._count  = CSRStorage(32, reset=1, description="Number of logical blocks to transfer.")
        self._nsid   = CSRStorage(32, reset=1, description="Namespace ID.")
        self._ctrl = CSRStorage(fields=[
            CSRField("start", size=1, offset=0, pulse=True, description="Start the transfer."),
            CSRField("write", size=1, offset=1,             description="1=write (stream->SSD), 0=read."),
        ])
        self._status = CSRStatus(fields=[
            CSRField("done",  size=1, offset=0, description="Transfer complete / idle."),
            CSRField("busy",  size=1, offset=1, description="Transfer in progress."),
            CSRField("error", size=1, offset=2, description="Error completion or unmatched tag seen."),
        ])
        self._cycles       = CSRStatus(64, description="Cycles from start to done.")
        self._data_beats   = CSRStatus(32, description="Data beats moved on the user stream.")
        self._stall_cycles = CSRStatus(64, description="Cycles the user stream was held off.")
        self._submitted    = CSRStatus(32, description="Commands queued to the engine.")
        self._completed    = CSRStatus(32, description="Completions matched to a slot.")
        self._errors       = CSRStatus(fields=[
            CSRField("count",  size=16, offset=0,  description="Completions with error status."),
            CSRField("status", size=16, offset=16, description="Status of the first error."),
        ])
        self._stale     = CSRStatus(16, description="Completions not matching a busy slot.")
        self._slots_hwm = CSRStatus(16, description="High-water mark of busy slots.")

        self.comb += [
            self.lba.eq(Cat(self._lba_lo.storage, self._lba_hi.storage)),
            self.count.eq(self._count.storage),
            self.nsid.eq(self._nsid.storage),
            self.start.eq(self._ctrl.fields.start),
            self.write.eq(self._ctrl.fields.write),

            self._status.fields.done.eq(self.done),
            self._status.fields.busy.eq(self.busy),
            self._status.fields.error.eq(self.error),
            self._cycles.status.eq(self.cycles),
            self._data_beats.status.eq(self.data_beats),
            self._stall_cycles.status.eq(self.stall_cycles),
            self._submitted.status.eq(self.submitted),
            self._completed.status.eq(self.completed),
            self._errors.fields.count.eq(self.err_count),
            self._errors.fields.status.eq(self.err_status),
            self._stale.status.eq(self.stale_count),
            self._slots_hwm.status.eq(self.slots_hwm),
        ]
