#
# This file is part of LiteNVMe.
#
# Copyright (c) 2026 Krishna Swaroop <swaroopdhulipalla99@gmail.com>
# SPDX-License-Identifier: BSD-2-Clause

"""Simulation stand-ins for the generated core's vendor pieces.

SimPCIePHY replaces USPPCIEPHY (constructor-compatible) for Migen simulation of the whole
generated core: it exposes the same separate request/completion raw-TLP streams (little-endian,
not qword-aligned, like the USP adapters produce/consume), the same CSR names (so csr.h matches
the hardware build), a "pcie" clock domain, the cfg_mgmt port backed by a small Root Port config
space, and a link that comes up once PERST# is released. SimCRG replaces the generator's CRG.
"""

from migen import *

from litex.gen import *

from litex.soc.interconnect import stream
from litex.soc.interconnect.csr import *

from litepcie.common import phy_layout, msi_layout

# Root Port config space (dword index -> reset value).
ROOT_CFG_RESET = {
    0x00: 0x9034_10ee,               # VID/DID (Xilinx-like).
    0x01: 0x0010_0000,               # Status.CapList.
    0x02: 0x0604_0000,               # Class: PCI-PCI bridge.
    0x08: 0x0000_0000,               # MemBase/MemLimit.
    0x0d: 0x0000_0040,               # Capabilities pointer.
    0x10: 0x0042_0010,               # PCIe cap: ID 0x10, next 0, v2, Root Port.
    0x11: 0x0000_0003,               # DevCap: MPS supported 1024 B.
    0x12: 0x0000_2000,               # DevCtl: MPS 128 B, MRRS 512 B.
}


class SimCRG(LiteXModule):
    def __init__(self, platform, clk_external, pcie_rst_resets_sys=True):
        self.cd_sys = ClockDomain()


class SimPCIePHY(LiteXModule):
    endianness    = "little"
    qword_aligned = False

    def __init__(self, platform, pads, speed="gen3", data_width=256, cd="sys",
                 ip_name="pcie4_uscale_plus", pcie_data_width=None, bar0_size=0x100000,
                 mode="RootPort", with_cfg_mgmt=True, with_rq_buffer=False, link_delay=64):
        self.data_width = data_width
        self.req_sink   = stream.Endpoint(phy_layout(data_width))  # endpoint -> link (our requests).
        self.cmp_sink   = stream.Endpoint(phy_layout(data_width))  # endpoint -> link (our completions).
        self.req_source = stream.Endpoint(phy_layout(data_width))  # link -> endpoint (device requests).
        self.cmp_source = stream.Endpoint(phy_layout(data_width))  # link -> endpoint (device completions).
        self.msi        = stream.Endpoint(msi_layout())

        self.id               = Signal(16)
        self.max_request_size = Signal(16, reset=512)
        self.max_payload_size = Signal(16, reset=512)
        self.bar0_mask        = 0

        self.cd_pcie = ClockDomain()

        # CSRs (same names/fields as USPPCIEPHY).
        self._link_status = CSRStatus(fields=[
            CSRField("status",     size=1),
            CSRField("phy_down",   size=1),
            CSRField("phy_status", size=2),
            CSRField("rate",       size=2),
            CSRField("width",      size=3),
            CSRField("ltssm",      size=6),
        ])
        self._msi_enable        = CSRStatus()
        self._bus_master_enable = CSRStatus()
        self._max_request_size  = CSRStatus(16)
        self._max_payload_size  = CSRStatus(16)

        # # #

        # Link: up `link_delay` cycles after PERST# is released, down while it is asserted.
        self.perst_n = perst_n = pads.rst_n
        self.link_up = link_up = Signal()
        count = Signal(max=link_delay + 1)
        self.sync += [
            If(~perst_n,
                count.eq(0),
                link_up.eq(0),
            ).Elif(count != link_delay,
                count.eq(count + 1),
            ).Else(
                link_up.eq(1),
            )
        ]
        self.comb += [
            self._link_status.fields.status.eq(link_up),
            self._link_status.fields.phy_down.eq(~link_up),
            self._link_status.fields.phy_status.eq(Mux(link_up, 3, 0)),
            self._link_status.fields.rate.eq(2),   # 8.0 GT/s.
            self._link_status.fields.width.eq(2),  # x4.
            self._link_status.fields.ltssm.eq(Mux(link_up, 0x10, 0)),
            self._max_request_size.status.eq(self.max_request_size),
            self._max_payload_size.status.eq(self.max_payload_size),
        ]

        # cfg_mgmt: Root Port local config space ("pcie" domain), one done pulse per access.
        self.with_cfg_mgmt = with_cfg_mgmt
        if with_cfg_mgmt:
            self.cfg_mgmt_addr            = Signal(10)
            self.cfg_mgmt_function_number = Signal(8)
            self.cfg_mgmt_write           = Signal()
            self.cfg_mgmt_write_data      = Signal(32)
            self.cfg_mgmt_byte_enable     = Signal(4, reset=0b1111)
            self.cfg_mgmt_read            = Signal()
            self.cfg_mgmt_read_data       = Signal(32)
            self.cfg_mgmt_read_write_done = Signal()

            self.root_cfg = root_cfg = [Signal(32, reset=ROOT_CFG_RESET.get(i, 0), name=f"root_cfg{i}")
                                        for i in range(64)]
            regs = Array(root_cfg)
            req  = Signal()
            self.comb += req.eq(self.cfg_mgmt_read | self.cfg_mgmt_write)
            sync_pcie = []
            for i in range(64):
                for b in range(4):
                    sync_pcie.append(
                        If(self.cfg_mgmt_write & ~self.cfg_mgmt_read_write_done &
                           (self.cfg_mgmt_addr == i) & self.cfg_mgmt_byte_enable[b],
                            root_cfg[i][8*b:8*b+8].eq(self.cfg_mgmt_write_data[8*b:8*b+8]),
                        )
                    )
            sync_pcie += [
                self.cfg_mgmt_read_write_done.eq(req & ~self.cfg_mgmt_read_write_done),
                self.cfg_mgmt_read_data.eq(Mux(self.cfg_mgmt_addr < 64, regs[self.cfg_mgmt_addr[:6]], 0)),
            ]
            self.sync.pcie += sync_pcie

    def update_config(self, config):
        pass
