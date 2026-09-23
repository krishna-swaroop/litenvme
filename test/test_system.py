# This file is part of LiteNVMe.
#
# Copyright (c) 2026 Krishna Swaroop <swaroopdhulipalla99@gmail.com>
# SPDX-License-Identifier: BSD-2-Clause
#
# Whole-core simulation of the generated standalone core (init_mode="external", stream ring,
# pattern generator/checker), driven by the UNMODIFIED C host library over AXI-Lite:
#
#   software/litenvme (C, in a thread) --ctypes callbacks--> AXI-Lite master (csr_axil pads)
#       |                                                         |
#       |                           LiteNVMeCore (litenvme.gen) with SimPCIePHY (test/sim_phy.py)
#       |                                                         |  raw TLPs
#       +--------- checks ------------------------------> NVMeTLPDevice (test/nvme_tlp_device.py)
#
# The C code is built against csr.h/soc.h exported from the simulated SoC, whose CSR map is
# checked to be identical to the hardware (USPPCIEPHY) build of the same configuration.

import ctypes
import os
import queue
import shutil
import subprocess
import tempfile
import threading
import unittest

from migen import *
from migen.sim import run_simulation, passive

from litex.build.xilinx import XilinxPlatform
from litex.soc.integration import export

import litenvme.gen as gen

from sim_phy import SimPCIePHY, SimCRG
from nvme_tlp_device import NVMeTLPDevice

ROOT     = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
SW_DIR   = os.path.join(ROOT, "software", "litenvme")
DEVICE   = "xczu17eg-ffvc1760-2-i"
CLK_FREQ = 125e6

SIM_CONFIG = {
    "phy"                 : "USPPCIEPHY",
    "phy_device"          : DEVICE,
    "phy_lanes"           : 4,
    "phy_speed"           : "gen3",
    "data_width"          : 256,
    "clk_freq"            : CLK_FREQ,
    "clk_external"        : True,
    "pcie_rst_resets_sys" : False,
    "pcie_perst_delay"    : 1e-6,
    "init_mode"           : "external",
    "qid"                 : 1,
    "qsize"               : 8,
    "qd"                  : 7,
    "hostmem_base"        : 0x1000_0000,
    "hostmem_size"        : 0x5400,
    "bar0_base"           : 0xe000_0000,
    "with_streamer"       : True,
    "streamer_base"       : 0x2000_0000,
    "streamer_nslots"     : 2,
    "streamer_slot_bytes" : 0x1000,
    "with_stream_pattern" : True,
}

# Indices of lnv_sim_main()'s result array (software/litenvme/sim/sim_main.c).
(RC_INIT, RC_WRITE, RC_VERIFY, RC_VERIFY_BADSEED, RC_REINIT, RC_VERIFY_AFTER_REINIT, MPS, MRRS,
 NSZE, LBA_BYTES, WR_CYCLES, WR_SUBMITTED, WR_COMPLETED, WR_HWM, RD_CYCLES, RD_SUBMITTED,
 RD_COMPLETED, CHECKED, BAD, BAD_BADSEED, RC_RANGE, N_RESULTS) = range(22)
LNV_ERANGE, LNV_EVERIFY = -7, -9


def build_core(phy_cls, config=SIM_CONFIG):
    cfg = gen._normalize_config(dict(config))
    gen.validate_config(cfg)
    cfg["phy"] = phy_cls
    platform = XilinxPlatform(DEVICE, io=[], toolchain="vivado")
    crg = gen.LiteNVMeCRG
    if phy_cls is SimPCIePHY:
        gen.LiteNVMeCRG = SimCRG
    try:
        soc = gen.LiteNVMeCore(platform, cfg)
    finally:
        gen.LiteNVMeCRG = crg
    soc.finalize()
    return soc


def csr_registers(soc):
    csv = export.get_csr_csv(csr_regions=soc.csr_regions, constants=soc.constants,
                             mem_regions=soc.mem_regions)
    return [l for l in csv.splitlines() if l.startswith("csr_register")]


def pattern_bytes(lba, count, seed):
    out = bytearray()
    for d in range(lba * 128, (lba + count) * 128):
        out += (((d & 0xffffffff) ^ seed) & 0xffffffff).to_bytes(4, "little")
    return bytes(out)


def build_sim_library(soc, workdir):
    inc = os.path.join(workdir, "include", "generated")
    os.makedirs(inc)
    with open(os.path.join(inc, "soc.h"), "w") as f:
        f.write(export.get_soc_header(soc.constants))
    with open(os.path.join(inc, "csr.h"), "w") as f:
        f.write(export.get_csr_header(regions=soc.csr_regions, constants=soc.constants,
                                      csr_ordering=soc.csr.ordering,
                                      csr_base=soc.mem_regions["csr"].origin,
                                      with_access_functions=True))
    lib = os.path.join(workdir, "liblitenvme_sim.so")
    cc  = os.environ.get("CC", shutil.which("cc") or "gcc")
    subprocess.check_call([cc, "-shared", "-fPIC", "-O1", "-std=gnu11", "-Wall", "-Wextra", "-Werror",
        "-DLNV_HAL_EXTERN_ACCESSORS",
        "-I", SW_DIR, "-I", os.path.join(SW_DIR, "include"), "-I", os.path.join(workdir, "include"),
        "-o", lib,
        os.path.join(SW_DIR, "litenvme.c"),
        os.path.join(SW_DIR, "sim", "hal_sim.c"),
        os.path.join(SW_DIR, "sim", "sim_main.c")])
    return ctypes.CDLL(lib)


class _Driver:
    """Runs lnv_sim_main() in a thread; its CSR accesses become AXI-Lite transactions."""
    RD_FN    = ctypes.CFUNCTYPE(ctypes.c_uint32, ctypes.c_uint32)
    WR_FN    = ctypes.CFUNCTYPE(None, ctypes.c_uint32, ctypes.c_uint32)
    TIME_FN  = ctypes.CFUNCTYPE(ctypes.c_uint64)
    DELAY_FN = ctypes.CFUNCTYPE(None, ctypes.c_uint32)
    LOG_FN   = ctypes.CFUNCTYPE(None, ctypes.c_char_p)

    def __init__(self, lib, lba, count, seed):
        self.lib, self.requests = lib, queue.Queue()
        self.cycle, self.virtual_us, self.log = 0, 0, []
        self.out = (ctypes.c_uint64 * N_RESULTS)()
        self.sn, self.mn = ctypes.create_string_buffer(21), ctypes.create_string_buffer(41)
        self.rc = None
        self.accesses = 0
        self.verbose = bool(os.environ.get("LITENVME_SIM_VERBOSE"))
        self._cbs = (self.RD_FN(self._rd), self.WR_FN(self._wr), self.TIME_FN(self._time),
                     self.DELAY_FN(self._delay), self.LOG_FN(self._log))
        lib.lnv_sim_set_callbacks(*self._cbs)
        lib.lnv_sim_main.argtypes = [ctypes.c_uint64, ctypes.c_uint32, ctypes.c_uint32,
                                     ctypes.POINTER(ctypes.c_uint64), ctypes.c_char_p, ctypes.c_char_p]
        self.thread = threading.Thread(target=self._run, args=(lba, count, seed), daemon=True)

    def _call(self, *req):
        done, box = threading.Event(), []
        self.requests.put((req, box, done))
        done.wait()
        return box[0] if box else None

    def _rd(self, off):      self.accesses += 1; return self._call("r", off)
    def _wr(self, off, v):   self.accesses += 1; self._call("w", off, v)
    def _time(self):         return self.cycle // int(CLK_FREQ / 1e6) + self.virtual_us
    def _delay(self, us):    self.virtual_us += us; self._call("d", 50)

    def _log(self, line):
        line = line.decode()
        self.log.append(line)
        if self.verbose:
            print(f"[{self.cycle:8d}] {line}", end="", flush=True)

    def _run(self, lba, count, seed):
        try:
            self.rc = self.lib.lnv_sim_main(lba, count, seed, self.out, self.sn, self.mn)
        finally:
            self.requests.put((("end",), [], threading.Event()))

    # Simulation side ------------------------------------------------------------------------------
    def process(self, pads, max_cycles, probes=()):
        @passive
        def clock():
            while True:
                self.cycle += 1
                if self.verbose and self.cycle % 20000 == 0:
                    print(f"[{self.cycle:8d}] ... {self.accesses} CSR accesses", flush=True)
                yield

        def axil():
            for s in (pads.awvalid, pads.wvalid, pads.bready, pads.arvalid, pads.rready):
                yield s.eq(0)
            self.thread.start()
            while True:
                if self.cycle > max_cycles:
                    raise RuntimeError(f"simulation cycle cap reached; last log: {self.log[-5:]}")
                # Block (GIL released) until the driver issues its next access: simulated time
                # only advances for accesses and explicit delays, i.e. driver compute time is 0.
                req, box, done = self.requests.get()
                if req[0] == "end":
                    break
                if os.environ.get("LITENVME_SIM_TRACE"):
                    print(f"[{self.cycle:8d}] csr {req}", flush=True)
                if req[0] == "w":
                    yield pads.awaddr.eq(req[1]); yield pads.awvalid.eq(1)
                    yield pads.wdata.eq(req[2]);  yield pads.wstrb.eq(0xf); yield pads.wvalid.eq(1)
                    aw = w = False
                    while not (aw and w):
                        yield
                        if not aw and (yield pads.awready): aw = True; yield pads.awvalid.eq(0)
                        if not w  and (yield pads.wready):  w  = True; yield pads.wvalid.eq(0)
                    yield pads.bready.eq(1)
                    while True:
                        yield
                        if (yield pads.bvalid): break
                    yield pads.bready.eq(0)
                elif req[0] == "r":
                    yield pads.araddr.eq(req[1]); yield pads.arvalid.eq(1)
                    while True:
                        yield
                        if (yield pads.arready): break
                    yield pads.arvalid.eq(0); yield pads.rready.eq(1)
                    while True:
                        yield
                        if (yield pads.rvalid):
                            box.append((yield pads.rdata))
                            break
                    yield pads.rready.eq(0)
                else:
                    for _ in range(req[1]):
                        yield
                done.set()
            yield
            self.probes = []
            for p in probes:
                self.probes.append((yield p))
        return [clock(), axil()]


class TestSystem(unittest.TestCase):
    def test_sim_csr_map_matches_hardware_build(self):
        # The driver built against the simulated SoC's headers is the one that runs on hardware.
        from litepcie.phy.usppciephy import USPPCIEPHY
        self.assertEqual(csr_registers(build_core(SimPCIePHY)), csr_registers(build_core(USPPCIEPHY)))

    def test_bringup_pattern_roundtrip(self):
        soc = build_core(SimPCIePHY)
        dev = NVMeTLPDevice(soc.pcie_phy, data_width=256, latency=10, reorder=2, seed=5)
        workdir = tempfile.mkdtemp(prefix="litenvme_sim_")
        try:
            lib = build_sim_library(soc, workdir)
            lba, count, seed = 0x1234, 24, 0x5eedf00d   # 3 slot-sized chunks over a 2-slot ring.
            drv = _Driver(lib, lba, count, seed)
            run_simulation(soc,
                drv.process(soc.csr_axil_pads, max_cycles=1_500_000,
                            probes=[soc.pcie_phy.root_cfg[0x12], soc.pcie_phy.root_cfg[0x08]]) +
                [dev.host_process(), dev.device_process()],
                clocks={"sys": 8, "pcie": 8})
        finally:
            shutil.rmtree(workdir, ignore_errors=True)

        out = list(drv.out)
        log = "".join(drv.log)
        print(f"\n{log}driver: {drv.accesses} CSR accesses, {drv.cycle} cycles; results {out}")
        signed = lambda v: v - (1 << 64) if v >= (1 << 63) else v

        # Driver view (the same calls litenvme_test makes on hardware).
        self.assertEqual(drv.rc, 0, log)
        self.assertEqual(signed(out[RC_INIT]), 0)
        self.assertEqual(drv.sn.value.decode(), "SIM0001")
        self.assertEqual(drv.mn.value.decode(), "LiteNVMe Sim SSD")
        self.assertEqual((out[MPS], out[MRRS]), (512, 512))  # min(root 1024, SSD 512); MRRS capped.
        self.assertEqual((out[NSZE], out[LBA_BYTES]), (1 << 21, 512))
        self.assertEqual(signed(out[RC_WRITE]), 0)
        self.assertEqual((out[WR_SUBMITTED], out[WR_COMPLETED], out[WR_HWM]), (3, 3, 2))
        self.assertEqual(signed(out[RC_VERIFY]), 0)
        self.assertEqual((out[RD_SUBMITTED], out[RD_COMPLETED]), (3, 3))
        self.assertEqual((out[CHECKED], out[BAD]), (count * 16, 0))
        self.assertEqual(signed(out[RC_VERIFY_BADSEED]), LNV_EVERIFY)
        self.assertEqual(out[BAD_BADSEED], count * 16)
        self.assertEqual(signed(out[RC_RANGE]), LNV_ERANGE)
        self.assertEqual(signed(out[RC_REINIT]), 0)
        self.assertEqual(signed(out[RC_VERIFY_AFTER_REINIT]), 0)

        # Device view (independent of the core and the driver).
        self.assertEqual(dev.violations, [])
        self.assertEqual(dev.cfg_bdfs, {(0, 1, 0)})
        self.assertEqual(dev.admin_log, [0x06, 0x06, 0x09, 0x05, 0x01] * 2)
        self.assertEqual(((dev.cfg[0x12] >> 5) & 7, (dev.cfg[0x12] >> 12) & 7), (2, 2))
        self.assertTrue(dev.cfg[0x01] & 0x6 == 0x6)  # MEM + BME.
        self.assertLessEqual(dev.max_cpld_dw * 4, 512)
        flash = b"".join(dev.flash[lba + i] for i in range(count))
        self.assertEqual(flash, pattern_bytes(lba, count, seed))
        self.assertEqual([c["nlb"] for c in dev.commands if c["op"] == 0x01], [8, 8, 8])
        # Root Port (sampled at the end): DevCtl.MPS raised to 512 B, bridge window over BAR0.
        root_devctl, root_window = drv.probes
        self.assertEqual((root_devctl >> 5) & 7, 2)
        self.assertEqual(root_window, 0xe000_e000)


if __name__ == "__main__":
    unittest.main()
