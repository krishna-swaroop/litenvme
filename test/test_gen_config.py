# This file is part of LiteNVMe.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import os
import tempfile
import unittest

from litenvme.gen import _normalize_config, load_config, validate_config


BASE_CONFIG = {
    "phy"        : "USPPCIEPHY",
    "phy_device" : "xcku3p-ffvb676-2-e",
}


class TestGeneratorConfig(unittest.TestCase):
    def test_example_config_loads(self):
        cfg = load_config(os.path.join("examples", "alibaba_xcku3p.yml"))
        self.assertEqual(cfg["init_mode"], "firmware")
        self.assertEqual(cfg["firmware"], "none")
        self.assertFalse(cfg["with_request_gen"])
        self.assertTrue(cfg["with_block_streamer"])

    def test_unknown_key_rejected(self):
        cfg = dict(BASE_CONFIG, unknown_option=True)
        cfg = _normalize_config(cfg)
        with self.assertRaisesRegex(ValueError, "Unknown LiteNVMe config key"):
            validate_config(cfg)

    def test_bad_init_mode_rejected(self):
        cfg = dict(BASE_CONFIG, init_mode="software")
        cfg = _normalize_config(cfg)
        with self.assertRaisesRegex(ValueError, "init_mode"):
            validate_config(cfg)

    def test_rtl_init_defaults_to_cpuless(self):
        cfg = dict(BASE_CONFIG, init_mode="rtl", with_block_streamer=True)
        cfg = _normalize_config(cfg)
        validate_config(cfg)
        self.assertIsNone(cfg["cpu"])
        self.assertEqual(cfg["uart"], "stub")
        self.assertEqual(cfg["firmware"], "none")
        self.assertFalse(cfg["with_request_gen"])

    def test_firmware_mode_requires_cpu(self):
        cfg = dict(BASE_CONFIG, init_mode="firmware", cpu=None)
        cfg = _normalize_config(cfg)
        with self.assertRaisesRegex(ValueError, "requires a CPU"):
            validate_config(cfg)

    def test_load_config_uses_safe_loader(self):
        with tempfile.NamedTemporaryFile("w", suffix=".yml", delete=False) as f:
            f.write("{'phy': 'USPPCIEPHY', 'phy_device': 'xcku3p-ffvb676-2-e'}\n")
            filename = f.name
        try:
            cfg = load_config(filename)
        finally:
            os.unlink(filename)
        self.assertEqual(cfg["init_mode"], "firmware")

    def test_zu17eg_example_loads(self):
        cfg = load_config(os.path.join("examples", "zu17eg_ssd0.yml"))
        self.assertEqual(cfg["init_mode"], "external")
        self.assertIsNone(cfg["cpu"])
        self.assertTrue(cfg["with_streamer"])
        self.assertFalse(cfg["with_request_gen"])

    def test_external_rejects_cpu_and_firmware(self):
        for extra, msg in ((dict(cpu="vexriscv"), "no embedded CPU"),
                           (dict(firmware="auto"), "does not use embedded firmware")):
            cfg = _normalize_config(dict(BASE_CONFIG, init_mode="external", **extra))
            with self.assertRaisesRegex(ValueError, msg):
                validate_config(cfg)

    def test_single_front_end(self):
        cfg = _normalize_config(dict(BASE_CONFIG, with_streamer=True, with_block_streamer=True))
        with self.assertRaisesRegex(ValueError, "only one of"):
            validate_config(cfg)

    def test_streamer_needs_qd(self):
        cfg = _normalize_config(dict(BASE_CONFIG, init_mode="external", with_streamer=True,
                                     streamer_nslots=16, qd=8))
        with self.assertRaisesRegex(ValueError, "qd must be >= streamer_nslots"):
            validate_config(cfg)

    def test_streamer_hostmem_layout_checked(self):
        cfg = _normalize_config(dict(BASE_CONFIG, init_mode="external", with_streamer=True,
                                     qsize=16, qd=15, hostmem_size=0x4000))
        with self.assertRaisesRegex(ValueError, "hostmem_size too small"):
            validate_config(cfg)

    def test_rtl_streamer_needs_pins(self):
        cfg = _normalize_config(dict(BASE_CONFIG, init_mode="rtl", with_streamer=True))
        with self.assertRaisesRegex(ValueError, "streamer_ctrl='pins'"):
            validate_config(cfg)

    def test_prp_list_shift(self):
        from litenvme.gen import streamer_prp_list_shift
        self.assertEqual(streamer_prp_list_shift(0x1000), 3)   # 1 page: no list, minimum stride.
        self.assertEqual(streamer_prp_list_shift(0x8000), 6)   # 8 pages: 7 entries -> 64 B.
        self.assertEqual(streamer_prp_list_shift(0x20000), 8)  # 32 pages: 31 entries -> 256 B.


if __name__ == "__main__":
    unittest.main()
