#
# This file is part of LiteNVMe.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""LiteNVMe top-level core — drop-in NVMe block access for a LiteX/LitePCIe SoC.

This packages the validated building blocks into one integratable core:

    LiteNVMe(pcie_endpoint, ...)
      ├── cfg       : LiteNVMePCIeCfgAccessor   (CSR)  — PCIe config space (BAR0, MEM/BME)
      ├── mmio      : LiteNVMePCIeMmioAccessor   (CSR)  — BAR0 MMIO for controller setup
      ├── mmio_db   : LiteNVMePCIeMmioAccessor   (—)    — dedicated doorbell path for the engine
      ├── hostmem   : LiteNVMeHostMemResponder          — host-memory window (SSD DMA target)
      └── io_engine : LiteNVMeIOEngineAXI        (CSR)  — hardware I/O command engine (QD)
                         .axi joins the hostmem AXI backend via the responder's arbiter
                         .connect_mmio(mmio_db) drives SQ/CQ doorbells

Interfaces (the requested "standardized interfaces"):

- Config / control: CSR (auto AXI-Lite in LiteX). The cfg/mmio accessors and the engine
  config (sq_base/cq_base/doorbell addresses, enable) and status (busy/inflight/
  submitted/completed) are CSR-exposed.
- Request / completion: stream Endpoints on the engine (`request` sink:
  {op,nsid,lba,nlb,buf}; `completion` source: {cid,status,sqhd}). User logic (or a CSR
  shim, or a DMA front-end) drives requests and consumes completions.
- Bulk data: addressed host memory (the hostmem window). NVMe is the DMA master, so the
  data path is *addressed* (PRP buffers), not a free-running stream — see
  doc/THROUGHPUT_DESIGN.md for the AXI-MM vs AXI-Stream rationale. The hostmem window is
  the canonical data interface; an optional AXI-Stream adapter can be layered on top for
  sequential streaming use-cases.

Bring-up model: a host/firmware does one-time admin setup (enable controller, create the
IO SQ/CQ) via the cfg/mmio CSRs, programs the engine's queue/doorbell addresses, then
sets `io_engine.enable`. Steady-state I/O is then hardware-driven by the engine with up
to `qd` commands outstanding — no per-command CPU cost.

Validated in simulation end to end (test/test_io_engine_integration.py exercises the
engine + bridge + arbiter + real backend). SoC wiring example: bench/alibaba_xcku3p.py.
"""

from migen import *

from litex.gen import *

from litex.soc.interconnect.csr import *

from litenvme.cfg       import LiteNVMePCIeCfgAccessor
from litenvme.mem       import LiteNVMePCIeMmioAccessor
from litenvme.hostmem   import LiteNVMeHostMemResponder
from litenvme.io_engine import LiteNVMeIOEngineAXI

# LiteNVMe Core Control ----------------------------------------------------------------------------

class LiteNVMeCoreControl(LiteXModule):
    """init_done / init_error: bring-up firmware sets these after NVMe initialization.

    A standalone core wires them to top-level status pins; a test SoC reads them over CSR/
    Etherbone. User/data logic gates on `init_done & ~init_error` (like LiteDRAM's core)."""
    def __init__(self):
        self.init_done  = CSRStorage()
        self.init_error = CSRStorage()

# LiteNVMe Core ------------------------------------------------------------------------------------

class LiteNVMe(LiteXModule):
    """Drop-in NVMe core for a LiteX/LitePCIe RootPort SoC.

    Parameters
    ----------
    pcie_endpoint : LitePCIeRootPort
        The PCIe RootPort endpoint; its crossbar provides the master/slave ports.
    hostmem_base, hostmem_size : int
        Host-memory window the SSD DMAs to/from (queues + PRP data buffers).
    data_width : int
        PCIe / backend data width (64/128/256). Defaults to the PHY's data width.
    qid, qsize, qd : int
        IO queue id, ring size, and max commands outstanding for the engine.
    requester_id : int
        PCIe requester id for the accessors.

    Notable attributes
    ------------------
    request    : stream Endpoint sink   — submit {op,nsid,lba,nlb,buf} descriptors.
    completion : stream Endpoint source — receive {cid,status,sqhd} descriptors.
    cfg, mmio, mmio_db, hostmem, io_engine : the sub-blocks (for CSR access / wiring).
    """
    def __init__(self, pcie_endpoint, hostmem_base=0x1000_0000, hostmem_size=0x8_0000,
                 data_width=None, qid=1, qsize=64, qd=32, requester_id=0x0000,
                 hostmem_backend=None, with_request_gen=False,
                 with_block_streamer=False, block_streamer_csr=True, with_hostmem_csr=True,
                 staging_base=0x4_0000, staging_size=0x4_0000, with_csr=True,
                 with_streamer=False, streamer_csr=True, streamer_base=0x2000_0000,
                 streamer_nslots=8, streamer_slot_bytes=0x8000, streamer_backend=None,
                 prp_list_shift=None,
                 with_accessor_csr=None, with_engine_csr=None, with_request_gen_csr=None,
                 extra_axi_masters=None):
        # hostmem_backend: optional pre-built host-memory backend exposing a matching
        # `.axi` slave. Default (None) instantiates on-FPGA BRAM. Pass a LiteDRAM AXI
        # port wrapper here for a DDR-backed host-memory window with large buffers
        # (sustained high-QD streaming). See doc/THROUGHPUT_DESIGN.md section 5.
        #
        # with_request_gen: instantiate a hardware request generator (LiteNVMeRequestGen)
        # bound to the engine's request/completion streams, so the core can self-drive at
        # full rate for benchmarking (no CPU in the steady-state loop). The user request
        # stream is then owned by the generator (see self.request_gen); leave False to
        # drive `self.request`/`self.completion` from your own logic.
        self.pcie_endpoint = pcie_endpoint
        endpoint = pcie_endpoint
        # The endpoint (and its PHY) belong to the SoC: don't re-export their CSRs under ours.
        self.autocsr_exclude = {"pcie_endpoint"}

        if data_width is None:
            data_width = endpoint.phy.data_width
        if with_accessor_csr is None:
            with_accessor_csr = with_csr
        if with_engine_csr is None:
            with_engine_csr = with_csr
        if with_request_gen_csr is None:
            with_request_gen_csr = with_csr
        self.with_accessor_csr    = with_accessor_csr
        self.with_engine_csr      = with_engine_csr
        self.with_request_gen_csr = with_request_gen_csr

        # Master ports off the PCIe crossbar.
        cfg_port     = endpoint.crossbar.get_master_port()
        mmio_port    = endpoint.crossbar.get_master_port()
        mmio_db_port = endpoint.crossbar.get_master_port()

        # PCIe config-space accessor (BAR0 discovery, Command.MEM/BME) — CSR-driven setup.
        self.cfg = LiteNVMePCIeCfgAccessor(
            port         = cfg_port,
            requester_id = requester_id,
            tag          = 0x42,
            with_csr     = with_accessor_csr,
        )

        # BAR0 MMIO accessor for controller setup (CAP/CC/CSTS/AQA/queue create) — CSR.
        self.mmio = LiteNVMePCIeMmioAccessor(
            port     = mmio_port,
            tag      = 0x44,
            with_csr = with_accessor_csr,
        )

        # Dedicated MMIO accessor the engine uses for SQ/CQ doorbells (no CSR; engine-owned).
        self.mmio_db = LiteNVMePCIeMmioAccessor(
            port     = mmio_db_port,
            tag      = 0x45,
            with_csr = False,
        )

        # Hardware I/O command engine (engine + dword->AXI bridge).
        self.io_engine = io_engine = LiteNVMeIOEngineAXI(
            qid=qid, qsize=qsize, qd=qd, data_width=data_width, with_csr=with_engine_csr,
            hostmem_base=hostmem_base, prp_list_shift=prp_list_shift)

        # Engine front-ends share one AXI backend; build them before the responder so their
        # masters join the arbiter. Front-ends (request_gen, block streamer, external logic)
        # are mutually exclusive owners of the engine sink/source -- a generated core builds
        # exactly one; a test SoC may build several and mux them externally.
        hostmem_axi_masters = [io_engine.axi]
        if extra_axi_masters is not None:
            hostmem_axi_masters += list(extra_axi_masters)

        # Optional LiteSATA-style block-streaming front-end (sector read/write data streams).
        if with_block_streamer:
            from litenvme.block import LiteNVMeBlockStreamer
            self.block_streamer = block_streamer = LiteNVMeBlockStreamer(
                data_width   = data_width,
                hostmem_base = hostmem_base,
                staging_base = staging_base,
                staging_size = staging_size,
                with_csr     = block_streamer_csr,
            )
            hostmem_axi_masters.append(block_streamer.dma.axi)

        # Optional stream-ring front-end (sustained, overlapped AXI-Stream write/read). Its slot
        # ring lives in a separate host-memory region with its own responder on a second
        # crossbar slave port, so user data never shares the backend with SQE/CQE traffic.
        if with_streamer:
            from litenvme.streamer import LiteNVMeStreamer
            self.streamer = streamer = LiteNVMeStreamer(
                data_width = data_width,
                buf_base   = streamer_base,
                nslots     = streamer_nslots,
                slot_bytes = streamer_slot_bytes,
                with_csr   = streamer_csr,
            )
            streamer_size = streamer.size
            assert (streamer_base + streamer_size <= hostmem_base) or \
                   (hostmem_base + hostmem_size <= streamer_base), "streamer region overlaps hostmem"
            def streamer_decoder(a):
                return (a >= streamer_base) & (a < (streamer_base + streamer_size))
            self.streamer_port = endpoint.crossbar.get_slave_port(address_decoder=streamer_decoder)
            self.streamer_mem  = LiteNVMeHostMemResponder(
                port              = self.streamer_port,
                base              = streamer_base,
                size              = streamer_size,
                data_width        = data_width,
                with_csr          = False,
                extra_axi_masters = [streamer.dma.axi],
                backend           = streamer_backend,
            )

        # Host-memory responder; the engine + front-ends join the AXI backend as masters.
        def hostmem_decoder(a):
            return (a >= hostmem_base) & (a < (hostmem_base + hostmem_size))
        self.hostmem_port = endpoint.crossbar.get_slave_port(address_decoder=hostmem_decoder)
        self.hostmem = LiteNVMeHostMemResponder(
            port              = self.hostmem_port,
            base              = hostmem_base,
            size              = hostmem_size,
            data_width        = data_width,
            with_csr          = with_hostmem_csr,
            extra_axi_masters = hostmem_axi_masters,
            backend           = hostmem_backend,
        )

        # Drive the engine's doorbells through the dedicated MMIO accessor.
        self.comb += io_engine.connect_mmio(self.mmio_db)

        # Optional hardware request generator (benchmark self-driver).
        if with_request_gen:
            from litenvme.request_gen import LiteNVMeRequestGen
            self.request_gen = request_gen = LiteNVMeRequestGen(with_csr=with_request_gen_csr)

        # Front-end -> engine connection. When a single front-end is present it owns the
        # engine streams; when several are present (test SoC), the SoC wires the mux and the
        # raw engine streams are re-exported here for it.
        front_ends = []
        if with_block_streamer: front_ends.append(block_streamer)
        if with_request_gen:    front_ends.append(request_gen)
        if with_streamer:       front_ends.append(streamer)
        if len(front_ends) == 1:
            self.comb += [
                front_ends[0].source.connect(io_engine.sink),
                io_engine.source.connect(front_ends[0].sink),
            ]
            self.request    = None
            self.completion = None
        else:
            # No front-end, or several (muxed by the SoC): re-export the engine streams.
            self.request    = io_engine.sink
            self.completion = io_engine.source

    def add_csrs(self, soc, name="nvme"):
        """Register the core's sub-blocks as CSR modules on `soc` with a common prefix.

        Convenience for SoC integration. Call after instantiating the core, e.g.:
            self.nvme = LiteNVMe(self.pcie_endpoint, ...)
            self.nvme.add_csrs(self)

        The registered sub-blocks are then excluded from the core's own CSR gathering, so each
        CSR lives in exactly one bank (LiteX gathers CSRs from every reachable attribute; a
        CSR reachable through two banks gets two bus decoders driving the same strobes).
        """
        self.autocsr_exclude = set(self.autocsr_exclude) | {
            "cfg", "mmio", "mmio_db", "io_engine", "hostmem", "request_gen", "block_streamer",
            "streamer", "streamer_mem",
        }
        if self.with_accessor_csr:
            soc.add_module(name=f"{name}_cfg",    module=self.cfg)
            soc.add_module(name=f"{name}_mmio",   module=self.mmio)
        if self.with_engine_csr:
            soc.add_module(name=f"{name}_engine", module=self.io_engine.engine)
        if hasattr(self.hostmem, "csr"):
            soc.add_module(name=f"{name}_hostmem", module=self.hostmem.csr)
        if hasattr(self, "request_gen") and self.with_request_gen_csr:
            soc.add_module(name=f"{name}_gen",   module=self.request_gen)
        if hasattr(self, "block_streamer") and hasattr(self.block_streamer, "_ctrl"):
            soc.add_module(name=f"{name}_block", module=self.block_streamer)
        if hasattr(self, "streamer") and hasattr(self.streamer, "_ctrl"):
            soc.add_module(name=f"{name}_streamer", module=self.streamer)
