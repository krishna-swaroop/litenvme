# LiteNVMe Stream Ring Front-End

`litenvme/streamer.py` (`LiteNVMeStreamer`) is a sustained AXI-Stream write/read front-end
for the I/O engine. It is the throughput-oriented counterpart of the block streamer
(`litenvme/block.py`): same `{start, write, lba, count}` control and `wr_sink`/`rd_source`
data streams, but transfers are cut into chunks that cycle through a ring of buffer slots,
so the user stream, the SSD DMA and command completion overlap.

Status: simulation only (Migen). Not yet validated on hardware.

## Why

The block streamer fills its staging buffer, issues one command, waits for its completion,
and only then accepts more data. Its staging DMA also waits for each AXI response, which
limits it to one 256-bit beat every 3 cycles (1.33 GB/s at 125 MHz, measured in simulation)
before any SSD time is added. The stream ring fixes both:

| | Block streamer | Stream ring |
|---|---|---|
| Staging DMA | 3 cycles/beat | ~1 beat/clk (64 beats in 67 fill / 66 drain cycles) |
| Commands in flight | 1 | up to `nslots` |
| Transfer size | ≤ staging window | unbounded (chunked) |
| Completion order | in order | any order (matched by tag) |

## Operation

```
write:  wr_sink ──fill──▶ slot[k] ──WRITE cmd──▶ SSD reads slot[k] ──CQE──▶ slot[k] FREE
read :  READ cmd for every FREE slot ──▶ SSD writes slot[k] ──CQE──▶ READY ──drain──▶ rd_source
```

- Slot states: `FREE → BUSY → FREE` (write), `FREE → BUSY → READY → FREE` (read).
- A write command is queued only after every AXI write response for its slot has been
  received (payload committed). A slot is never refilled or resubmitted while BUSY.
- Read data is exposed only after its completion; slots drain strictly in LBA order.
- Completions are matched to slots with the engine request/completion `tag` field (the slot
  index), so the device may complete commands in any order.
- The final chunk may be shorter than a slot. `rd_source.last` marks the final beat.
- A completion with non-success status sets `error`, increments `err_count` and records the
  first status. The transfer still runs to completion and frees every slot. A completion
  whose tag does not match a BUSY slot is counted in `stale_count` and also sets `error`.

## Integration

```python
nvme = LiteNVMe(pcie_endpoint,
    with_streamer       = True,
    streamer_base       = 0x2000_0000,  # host (PCIe) address of the slot ring.
    streamer_nslots     = 8,
    streamer_slot_bytes = 0x8000,       # 32 KiB per command.
)
```

The slot ring gets its own `LiteNVMeHostMemResponder` on a second crossbar slave port,
separate from the queue/PRP window (`hostmem_base`). The streamer DMA is the only local
master on that backend, so user data never competes with the engine's SQE/CQE accesses. The
SSD's data DMA uses the opposite AXI channel (write mode: SSD reads while we write; read mode:
the reverse). The two regions must not overlap.

`streamer_backend` accepts an external backend with a matching `.axi` slave, as
`hostmem_backend` does.

### Standalone core (litenvme_gen)

`with_streamer: True` exposes the stream ring in the generated core (`stream_wr_axis` /
`stream_rd_axis` data pins; commands via CSRs, or pins with `streamer_ctrl: "pins"`).
`with_stream_pattern: True` adds the pattern generator/checker, selected by
`nvme_stream_mux_sel`. Pair it with `init_mode: "external"`: the core has no soft-CPU and
exposes its CSR bus as an AXI-Lite slave (`csr_axil_*`, 64 KiB aperture, CSRs at offset 0), so
a Zynq PS or MicroBlaze runs bring-up with `software/litenvme` while the PL moves the data.
`examples/zu17eg_ssd0.yml` is a complete example. Transfers only start once `init_done` is set.

Geometry rules (asserted at elaboration):

- `slot_bytes` is a multiple of the LBA size.
- `buf_base` is page-aligned, and each slot is either whole pages or packs inside one page.
  The engine builds PRPs for page-aligned buffers.
- `slot_bytes / lba_bytes < 65536` (engine NLB field) and `nslots ≤ 256` (tag width).
- Keep the engine `qd ≥ nslots`, so every slot can be in flight.

## Test data: pattern generator / checker

`litenvme/pattern.py` provides `LiteNVMeStreamPatternGen` and `LiteNVMeStreamPatternCheck`.
Every 32-bit word is `D ^ seed`, where `D` is the absolute dword index on the drive
(`lba * lba_bytes/4 + offset`). A read-back compare therefore catches data at the wrong LBA or
offset, not only bit errors. Use a new seed per run so stale drive contents fail. The checker
reports mismatching beats, the first bad beat index, and `last` misplacement.

## Verification

`test/test_streamer.py` runs the real engine, two real host-memory responders and the
streamer against `test/ssd_model.py`. That independent SSD model fetches SQEs, decodes PRPs,
moves data in 512 B MemRd/MemWr through the responders' PCIe ports, keeps its own flash
contents and posts CQEs. Write data is checked against the model's flash. Read data is
checked on a range the DUT never wrote, so stale slot contents cannot pass.

Covered: partial final chunk and ring wrap, random stream gaps in both directions, SSD
latency, out-of-order completions, PRP1+PRP2 slots, error status, the pattern gen/check round
trip with injected corruption, core elaboration with the second slave port, and region
overlap rejection. Three deliberate ownership bugs were each confirmed to fail the suite:
queue before fill commit, drain before completion, refill while BUSY.

Performance (simulation, 256-bit, 1 KiB slots, sequential SSD model):

- With 200-cycle SSD latency the SSD model is busy 92% of each transfer and all slots are used.
- With a fast device the floor is the engine at ~107 cycles/command (16 single-dword SQE
  writes, CQE reap, doorbells): about 1.2 M commands/s at 125 MHz. That is ~37 GB/s with
  32 KiB slots but only ~4.7 GB/s with 4 KiB commands, so use slots of 16 KiB or more.

## Known limits / follow-ups

- No command timeout: a missing completion stalls the transfer. Recovery and abort are not
  implemented.
- A failed read chunk is still drained (flagged by `error`).
- The backend is on-FPGA RAM. A DDR-backed ring needs burst AXI in the streamer DMA and
  responder.
- Engine SQE build is dword-granular. Writing the SQE as full beats would lower the
  per-command floor.
- The responder serves one SSD MemRd at a time. Pipelining it would raise write throughput.
- The generated core's stream data pins are in the core clock domain. Add an async FIFO to
  cross from a source's own clock.
