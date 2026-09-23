# LiteNVMe host library (external control)

Portable C bring-up and control for a core generated with `init_mode: "external"` and
`with_streamer: True`. It runs on an external processor (Zynq PS under Linux or bare-metal,
MicroBlaze, ...) over the core's AXI-Lite port. The PL owns the steady-state data path.

| File | Role |
|---|---|
| `litenvme.h/.c` | Bring-up (BAR0, MPS/MRRS, controller, Identify, IO queues, engine) and stream / pattern transfers |
| `litenvme_hal.h` | Platform layer: CSR accessors for the generated `csr.h`, time/delay/log hooks |
| `hal_linux.c` | Linux userspace backend (`/dev/mem` or UIO) |
| `litenvme_test.c` | CLI: `status`, `perst`, `init`, `pattern`, `verify`, `bench` |
| `sim/` | Simulation backend used by `test/test_system.py` (same library, callbacks into the simulator) |

Build against the headers generated for your core:

```sh
make GEN_DIR=<litenvme_gen output>/software/include CROSS_COMPILE=aarch64-linux-gnu-
./litenvme_test -a <AXI-Lite base> -v init
./litenvme_test -a <AXI-Lite base> --serial <SSD serial> init pattern <scratch LBA> 1G
```

Writes are refused unless `--serial` matches Identify Controller, and only the given LBA range
is touched.

The bring-up follows the hardware-validated firmware sequence (`bench/firmware/main.c`),
including its lag-safe CFG/MMIO reads. It adds:

- The SSD's MRRS is capped at the negotiated MPS (the host-memory responder answers each MemRd
  with one CplD).
- Timeouts come from CAP.TO.
- CAP.MPSMIN, CSS and MQES are checked.
- Identify Namespace is required to report 512 B LBAs without metadata.
- MDTS must allow at least one full slot.
- The root-port memory window is sized to the discovered BAR.

For bare-metal, compile `litenvme.c` with a platform file that defines `lnv_csr_base`,
`lnv_time_us()`, `lnv_udelay()` and `lnv_log()`.
