# USB-C DP Checkpoint

## Milestone 2026-05-02 1928 — FULL CHAIN 1→19 (train + EDID + framer + video-on)

End-to-end stage walk through 19 stages succeeds on CC1 passive cable:

```
host_cap: AUX_SWAP=0x0 (flip=0)
DPCD rev=0x11 max_link_rate=0xa max_lane_count=0x82
set_link_rate: applied 270000 kHz
CR done after 1 vswing tries
EQ iter try=0 status=77 00 81 00 22 00     ← INTERLANE_ALIGN_DONE
EQ done after 0 tries
link trained: rate_code=0xa lanes=2
EDID OK: vendor=XYM
EDID DTD: 1920x1080 @ pixclk=148500 kHz htotal=2200 vtotal=1125
config_video: TU=32 VS=26 (rem=400) 1920x1080 @ 148500 kHz, 2-lane HBR
stage18: config-video done
stage19: video-on done (firmware framer enabled)
last_error: 0
```

New: stage 18 (`config-video`) parses EDID detailed-timing block 0 and
programs 14 Cadence framer/MSA registers via mailbox WRITE_REGISTER (TU/VS
calculated to match Linux's loop).  Stage 19 (`video-on`) sends
`DPTX_SET_VIDEO=1` (opcode 0x0c).  Firmware accepts both — framer is now
configured and enabled, waiting on VOP to drive real pixels.

Next: VOP→CDN-DP routing + scanout buffer to actually feed pixels.

---

## Milestone 2026-05-02 1830 — FULL CHAIN: train + EDID in one clean run on CC1 PIN_C

Stages 1→16 + EDID succeed in a single sysctl walk on CC1 PIN_C real-PD VDM:

```
host_cap: AUX_SWAP=0x0 (flip=0)            ← orientation-aware
DPCD rev=0x11 max_link_rate=0xa max_lane_count=0x82
set_link_rate: applied 270000 kHz
set_lane_count: lanes=2 DP_MODE_CTL=0xc110
CR done after 1 vswing tries
EQ iter try=0 status=77 00 81 00 22 00     ← INTERLANE_ALIGN_DONE both lanes
EQ done after 0 tries
link trained: rate_code=0xa (270000 kHz) lanes=2
EDID OK: vendor=XYM product=0x66150000
last_error: 0
```

**Key fix:** `rk_cdn_dp_set_host_cap()` writes `AUX_SWAP_INVERSION_CONTROL`
based on flip (CC1=0x0, CC2=0x3) instead of hardcoded 0x3 with a
`skip_aux_swap` escape hatch.  The old code crashed firmware on CC1 when
0x3 was written; we worked around with skip_aux_swap=1 which accepted
firmware default but left lane 1 EQ stalled.  With orientation-correct
values both CC1 and CC2 work without firmware crash.

Reproducible recipe (kernel #20):
```
hw.fusb302.role_pref=1
dev.rk_cdn_dp.0.skip_aux_swap=0
dev.rk_cdn_dp.0.hostcap_usb_ss=1
dev.rk_cdn_dp.0.hostcap_lanes=2
dev.rk_cdn_dp.0.allow_phys=1 allow_aux=1
for s in 1..14 16; do sysctl dev.rk_cdn_dp.0.stage=$s; done
sysctl dev.rk_cdn_dp.0.edid_now=1
```

Frontier: video framer + VOP pipeline to actually output pixels.  CC2
passive cable lane-1 issue still untested with the new AUX_SWAP split.

---

## Milestone 2026-05-02 deep-night — EDID via mailbox works on Armbian's display

```
EDID OK: vendor=XYM product=0x66150000
EDID first 32B: 00 ff ff ff ff ff ff 00 63 2d 66 15 00 00 00 00
                0c 23 01 04 b5 00 00 78 3f ee 91 a3 54 4c 99 26
```

Header bytes match Armbian's working display on the same hardware exactly,
read via `DPTX_GET_EDID` mailbox (opcode 0x02) right after stage 13 — no link
training required.  AUX channel works; the only remaining gap is main-link
data lane EQ on lane 1 specifically.

Operator path: `dev.rk_cdn_dp.0.edid_now=1` triggers an EDID block-0 read
directly without going through the stage walker (separate stage 17 was added
too for the linear-walk path).

---

## Milestone 2026-05-02 late-late — FULL DP LINK TRAINED (2-lane HBR)

Both phases of DP link training complete on FreeBSD RockPro64 over USB-C:
```
CR done after 1 vswing tries
EQ iter try=0 status=77 00 81 00 22 00   ← both lanes EQ'd + INTERLANE_ALIGN
EQ done after 0 tries
link trained: rate_code=0xa (270000 kHz) lanes=2
```
A 2-lane × 2.7 Gbps = 5.4 Gbps DP link is up.  Next step: EDID read + video
config + framer.

Two further fixes since the previous milestone:
1. AUX_SWAP_INVERSION_CONTROL now flip-aware (CC1=0x0, CC2=0x3) replacing
   the hardcoded 0x3 + skip_aux_swap escape-hatch.
2. PHY init tracks `init_flip` and forces a full teardown if `sc->flip`
   changes since the last init, so `conn_dir` GRF can re-latch on a CC1↔CC2
   cable swap.

---

## Milestone 2026-05-02 late — CR SUCCEEDS, EQ PARTIAL (lane 0 ✓, lane 1 stuck)

CR (clock recovery) phase of DP link training now completes on FreeBSD
RockPro64. EQ (channel equalization) lane 0 fully equalizes; lane 1 stuck at
CR_DONE only.  Most likely cause: hardcoded `PMA_LANE_CFG = PIN_ASSIGN_D_F`
needs flip-aware selection.

Three new functions in `rk_typec_phy.c`:
- `rk_typec_phy_dp_set_link_rate()` — full PLL re-program A3→…→A0 with
  RBR/HBR/HBR2 PLL config tables ported from Linux.
- `rk_typec_phy_dp_set_lane_count()` — RMW DP_MODE_CTL[15:12] lane disable.
- `rk_typec_phy_dp_set_signal_levels()` — per-lane swing/pre-emphasis.

`rk_cdn_dp.c` adds stage 16 (`link-train-full`) with full CR + EQ + stop
sequence ported from Linux's `cdn_dp_link_training_clock_recovery` and
`cdn_dp_link_training_channel_equalization`.

Live sequence (CC2 passive cable, kernel #11):
```
set_link_rate: applied 270000 kHz (ssc=0)
set_lane_count: lanes=2 DP_MODE_CTL=0xc110
CR iter v_tries=1 max_vs=1 status=11 00 80 00 22 00 train[0]=06
CR done after 1 vswing tries (max_vs=1)
EQ iter try=0 status=17 00 80 00 62 00 train[0]=06
EQ iter try=1..4 status=17 00 80 00 66 00 train[0]=2e (stuck)
EQ failed after 5 tries
```

Frontier: Add flip-aware PMA_LANE_CFG selector so lane 1 lands on a real DP
wire pair on CC2 orientation.

---

## Milestone 2026-05-02 evening — DPCD READS WORK, LINK TRAINING KICKED OFF

Live sequence on FreeBSD RockPro64 (kernel #8) reaches DPCD capability read
and starts link training pattern 1 successfully:

```
host_cap payload:  14 12 02 03 0f 00 e4 01     (lanes=2 flip=1)
host_cap:          AUX_SWAP=0x3
sink_count:        raw=0x41 count=1 (try 0)
DPCD rev=0x11 max_link_rate=0xa (270000 kHz) max_lane_count=0x82
                    enhanced_frame=1 downspread=1
stage13: dpcd-read done
stage14: link-plan done   (train_rate=0xa, train_lanes=2 — HBR 2.7Gbps × 2)
stage15: link-train-start done  (CR not done yet — expected, no CR/EQ loop yet)
```

The three fixes that unlocked this:

1. `rk_typec_phy.c` — when PHY is already in A2_READY at first phy_enable
   call, run only `dp_aux_set_flip` + `dp_aux_calibration` and return early.
   The previous code re-ran the full common 24M setup which knocked the PHY
   out of A2_READY into a state from which ENTER_A2 could never re-assert.
2. `rk_cdn_dp.c` — DPCD short-reply parser now calls
   `DPTX_GET_LAST_AUX_STATUS` (mailbox opcode 0x0e) for the actual wire
   status. The 5-byte short-reply body bytes do NOT carry the AUX wire
   status (zeros even when the wire actually NACK'd). Mirrors Linux's
   `cdn_dp_aux_transfer`.
3. `dev.rk_cdn_dp.0.skip_aux_swap=0` — write `AUX_SWAP_INVERSION_CONTROL`
   to 0x3 (RockPro64 TYPEC0) during set_host_cap. Earlier morning
   checkpoint claimed `=1` was needed; that was wrong.

Working with both:
- PIN_C real PD VDM (`pin=0x4 dp_status=0x8a` from CC1)
- PIN_E passive cable (`pin=0x10 dp_status=0x80` from CC2)

Frontier: implement CR/EQ training loops to actually bring up the link,
then connector + EDID + framebuffer.

---

## Earlier milestone reached on 2026-05-02 (morning, superseded):

- `fusb302` now completes the real PD + DP Alt Mode path on RockPro64.
- Confirmed live sequence:
  - `PD connected as DFP (5V)`
  - `VDM DP caps=0x00000405 pin_support=0x4`
  - `VDM DP config OK, pin_assignment=0x4`
  - `DP Alt Mode: dp_ready=1 pin=0x4 usb_ss=1 dp_status=0x8a`
- `rk_cdn_dp` stage 13 (`hostcap+dpcd-read`) now completes successfully on the
  Cadence mailbox path when run with the Linux-consistent 2-lane shape and with
  `skip_aux_swap=1`.
- Confirmed live stage-13 success sequence:
  - `host-cap using lanes=2`
  - `host_cap payload: 14 12 02 03 0f 00 1b 01`
  - `sink_count: raw=0x41 count=1`
  - `after dpcd-read 0x000 success`
  - `stage13: dpcd-read done`

What this milestone means:

- The Type-C/PD/VDM side is no longer the blocker.
- The first Cadence firmware mailbox `READ_DPCD` transaction is no longer the
  blocker.
- The remaining work has moved forward into the next Cadence bring-up steps
  after initial DPCD capability discovery, such as link-training/video-enable
  sequencing and any remaining provider attach hygiene.

Load-path cleanup after the milestone:

- `rk_cdn_dp` now refreshes the `fusb302` Type-C provider handle before staged
  bring-up instead of depending only on whatever provider state existed at
  attach time.
- Host-cap logging now prints the effective negotiated `usb_ss` value rather
  than only the override knob, so a normal successful 2-lane run no longer
  looks like `usb_ss=-1`.
- `kldxref /boot/kernel` is still not clean on the patched arm64 modules
  because of the existing `DT_RELA` linker-hints issue, so explicit
  `kldload /boot/kernel/<module>.ko` remains the safe operator path for now.

Important conditions for reproducing the success:

- `fusb302` must be the live Type-C provider.
- DP Alt Mode must be the negotiated live path, not the old passive fallback.
- The working live Alt Mode state is:
  - `pin_assignment=0x4`
  - `usb_ss=1`
  - `dp_status=0x8a`
- `rk_cdn_dp` must avoid the old bad RockPro64 `AUX_SWAP` path:
  - `dev.rk_cdn_dp.0.skip_aux_swap=1`
- The working host-cap shape is:
  - `lanes=2`
  - `flip=0` on the successful captured run

Housekeeping note:

- The old temporary stage/run scripts used during iterative bring-up are not
  part of the driver design and can be discarded after this checkpoint.

Current safe posture on the RockPro64 boards:

- Boot the known-good `RP64KERN_RKDRM` kernel only.
- Keep `rk_cdn_dp` out of the kernel image.
- Keep `ofwbus` and other core bus changes out of the live boot path.
- Keep panic handling enabled so failures land in DDB instead of forcing blind resets.
- Treat `fusb302` as the current stable Type-C base.
- Use the safe kernel plus module-only `rk_cdn_dp` iterations as the only live USB-C DP test path.

What was confirmed:

- The board boots reliably again from the restored safe SD card kernel.
- `sysctl debug.kdb.enter=1` reaches `db>`, so DDB is usable on the live kernel.
- `dumpon -l` reports the swap-backed dump device.
- `dp@fec00000` exists in the OFW tree but remains unbound when `rk_cdn_dp.ko` is loaded after boot.
- `rk_cdn_dp.ko` now builds as a safer staged scaffold with inert attach semantics and explicit `allow_phys` / `allow_aux` gates in source.
- A userspace `/dev/mem` probe showed the raw RK3399 PMU state for the HDCP/CDN-DP path is already open on the safe kernel:
  - `PWRDN_ST[21] == 0`
  - `BUS_IDLE_REQ[11] == 0`
  - `BUS_IDLE_ST[11] == 0`
  - `BUS_IDLE_ACK[11] == 0`
- The live blocker is no longer PMU state itself. It is now in the staged driver path after stage 5.

What was attempted and rolled back:

- A late-binding `ofwbus` rescan and `bus_driver_added` experiment was added to let post-boot `rk_cdn_dp.ko` bind the existing DP node.
- That kernel still hung during boot at the same late attach point, so the `ofwbus` patch is not part of the safe path.
- A narrower `ofwbus` `bus_driver_added()` hook that only cleared stale direct-child `unknown` devclasses was also built and tested, and that test kernel crashed as well.
- Building `rk_cdn_dp` into the kernel is also not part of the safe path until a lower-risk attach strategy is proven.

Current blocker:

- The current `rk_cdn_dp` module path now binds safely and advances through:
  - stage 1: power-domain
  - stage 2: handles
  - stage 3: clocks
  - stage 4: resets
  - stage 5: phys
  - stage 6: firmware get
  - stage 7: firmware prep
  - stage 8: firmware load
  - stage 9: firmware active
  - stage 10: Rockchip HPD selection
  - stage 11: mailbox HPD state
  - stage 12: mailbox host capabilities
- The new live blocker is stage 13 (`dpcd-read`).
- That stage no longer crashes immediately. It now fails in the Cadence
  firmware/mailbox path while trying to read DPCD capability bytes.

Cadence firmware milestone:

- The driver no longer uses the old raw AUX register path as the primary probe.
- `rk_cdn_dp` now uses the Cadence firmware/mailbox direction.
- The firmware file is loaded from:
  - `/boot/firmware/rockchip/dptx.bin`
- FreeBSD loads it with:
  - `firmware_get("rockchip/dptx.bin")`

What `dptx.bin` is supposed to do:

- `dptx.bin` is the Cadence DisplayPort transmitter firmware blob for the
  RK3399 CDN-DP block.
- It is not USB-PD firmware and not FUSB302 firmware.
- It is loaded into the Cadence DP controller's internal memory and brings up
  the controller's embedded microcontroller.
- After that, the driver is supposed to use the Cadence mailbox protocol for:
  - HPD state
  - DPCD reads and writes
  - EDID fetch
  - link training
  - later video/audio configuration

What is now proven:

- `dptx.bin` is present and loadable from the board firmware path.
- Firmware load and activation complete without the earlier raw-AUX crashes.
- Rockchip HPD routing through `GRF_SOC_CON26` works.
- Mailbox HPD query works and reports `hpd_status=1`.
- Mailbox host-cap stage works.
- First mailbox DPCD read is the remaining blocker.

Latest mailbox-side findings:

- The original mailbox DPCD read failed with:
  - `mailbox DPCD reply header failed (60)`
- Host-cap was then split and tested with explicit overrides:
  - `lanes=4 flip=1`
  - `lanes=2 flip=1`
- Neither override cleared the DPCD timeout.
- A `DP_SINK_COUNT` wait was added before the first DPCD caps read.
- That changed behavior:
  - the failure path no longer returned immediately
  - repeated mailbox retries eventually left the board wedged hard enough that
    serial `reset` produced no response

Important FreeBSD-native finding:

- FreeBSD already exports useful Type-C cable state from the FUSB302 driver via:
  - `fusb302_get_typec_status(device_t, struct fusb302_typec_status *)`
- That status includes:
  - `attached`
  - `role`
  - `orientation`
  - `togss_raw`
  - `vbusok`
- `rk_cdn_dp` has been updated locally to use that as the polarity fallback
  when the extcon handle path is missing.

Rule going forward:

- Risky USB-C/DP changes must survive a non-boot-critical test path before they are integrated into the kernel image.
- Keep the default `/boot/kernel` on the last known-good image.
- Treat `rk_cdn_dp` as module-first and stage-driven only.
- Advance one risky subsystem at a time: binding, then power, then handles, then clocks, then resets, then PHY, then AUX.
- Keep all future live iterations module-only unless a built-in kernel change is unavoidable.
- Preserve every failed test kernel as a backup and repair the SD card back to the known-good kernel before continuing.

New FreeBSD issue findings:

- arm64 standalone KMOD metadata loss
  Self-built arm64 modules produced by `make -C sys/modules/...` were loading
  into memory without registering full module metadata. The final `.ko` lacked
  the linker-set boundary symbols that working base modules carry:
  `__start_set_modmetadata_set`, `__stop_set_modmetadata_set`,
  `__start_set_sysctl_set`, `__stop_set_sysctl_set`,
  `__start_set_sysinit_set`, and `__stop_set_sysinit_set`.

  Observable symptoms:
  - `kldload rk_cdn_dp.ko` succeeded
  - no module event logs ran
  - no `hw.rk_cdn_dp.*` sysctls appeared
  - no single-child reprobe logic ran

  Local fix now under test:
  - added `sys/conf/ldscript.kmod.arm64`
  - rebuilt `rk_cdn_dp.ko`
  - verified the rebuilt module gained the missing linker-set symbols
  - verified module event logging now runs
  - verified the existing `dp@fec00000` node now binds as `rk_cdn_dp0`
  - verified staged per-device sysctls now appear as `dev.rk_cdn_dp.0.*`

- late OFW/FDT rebinding through core bus code is still unsafe
  Both broad and narrow `ofwbus` late-binding experiments destabilized test
  kernels, so they remain off the safe path even though the module-local
  single-child reprobe path now works once arm64 KMOD metadata is fixed.

Latest safe-kernel DP findings:

- A module-only stage-1 bypass now checks PMU readiness directly from `rk_cdn_dp`.
  If domain 21 is already on and `BUS_IDLE_ST[11]` / `BUS_IDLE_ACK[11]` are
  already clear, stage 1 skips the freezing `rk3399_power_enable_domain()`
  provider call entirely.
- With that bypass in place, the live safe-kernel module now reaches:
  - `dev.rk_cdn_dp.0.stage=1` cleanly
  - `dev.rk_cdn_dp.0.stage=2` cleanly
  - `dev.rk_cdn_dp.0.stage=3` cleanly
  - `dev.rk_cdn_dp.0.stage=4` cleanly
  - `dev.rk_cdn_dp.0.stage=5` cleanly
- Stage 6 is now explicitly `aux-probe` and requires:
  - `dev.rk_cdn_dp.0.allow_aux=1`
- After opening that gate, `dev.rk_cdn_dp.0.stage=6` drops to DDB with:
  - `external_abort()`
  - `generic_bs_r_4()`
- So the next target is not PMU or provider bring-up. It is the exact Cadence
  DP MMIO read performed first in AUX/HPD probe.

What not to do next:

- Do not keep retrying `kernel.test` built-in `rk3399_power` experiments. Even
  reduced provider-only test kernels still failed to complete boot.
- Do not go back to raw AUX register pokes as the main path. They were useful
  for narrowing the old crash frontier, but the real bring-up path is now the
  Cadence firmware/mailbox path.
- Do not assume `nphys` alone gives the correct USB-C lane/polarity state.
  FreeBSD now needs that information from `fusb302`/extcon before mailbox DPCD
  reads are expected to work.

Board-doc corrections from `/home/b1nc0d3x/RP64-DOCS`:

- RockPro64 USB-C video is fixed to `TYPEC0` / `USB3.0 PHY0`.
- `TYPEC1` / `USB3.0 PHY1` is wired out as the fixed USB 3.0 A-path on this
  board, not as an alternate USB-C DP connector.
- `CC2` therefore means "flipped orientation on TYPEC0", not "use port 1".
- `FUSB302B` is the external CC/Type-C controller on `I2C4`.
- `VBUS_TYPEC` is switched by `SY6280AAC` through `VCC5V0_TYPEC0_EN`.
- The SoC exposes explicit AUX polarity reversal pins:
  - `TYPEC0_AUXP_PD_PU`
  - `TYPEC0_AUXM_PU_PD`
  These confirm that orientation is supposed to be handled as AUX/lane flip on
  one port, not by switching DP controllers/PHYs.
- RK3399 has only one built-in DisplayPort controller shared by two Type-C
  PHYs. On RockPro64, the only board-valid DP target is `TYPEC0`.

Driver direction updated from those docs:

- `rk_cdn_dp` should default to `active_port=0` on RockPro64.
- FUSB302 orientation should feed `hostcap_flip`, not `active_port`.
- Any path that maps `CC2 -> active_port=1` on RockPro64 is wrong.

Latest stable `READ_DPCD` boundary result:

- With the corrected RockPro64 recipe:
  - `active_port=0`
  - `hostcap_flip=1`
  - `hpd_status=1`
  - `stage=12` passes cleanly
- The first real mailbox `READ_DPCD` command is now reached on the correct
  `TYPEC0` path.
- The current stable live discriminator is:
  - `mbox_last_send_written=0`
  - `mbox_last_empty=1`
  - `mbox_last_empty_after_send=1`
  - `mbox_last_header=0`
  - `mbox_last_keep_alive=43`
- Interpretation:
  - the Cadence firmware uCPU is still alive
  - no reply header is queued
  - `MAILBOX_EMPTY` remains asserted even immediately after the attempted send
  - the remaining blocker is still the first real `READ_DPCD` transaction
    boundary, now narrowed to the last missing precondition before a mailbox
    reply is produced

Boot-safety correction for `rk_typec_phy`:

- The first attempt to add the fuller RK3399 DP PHY behavior directly into the
  built-in `rk_typec_phy` boot path produced a non-booting kernel.
- That behavior is now being moved behind a boot-safe gate:
  - `hw.rk_typec_phy_dp_extras`
- Default must remain `0` so the system boots with the old safe PHY path.
- When explicitly enabled later, the gated extras apply:
  - FUSB302-based flip/orientation read
  - `typec_conn_dir`
  - external PSM select
  - `uphy_dp_sel`
  - DP A2 -> A0 transition
- This keeps the kernel boot-safe while preserving the missing Type-C PHY
  recipe needed for the next `READ_DPCD` test.

Late-load `fusb302` bridge result:

- A late-load helper module was added instead of putting more DP Alt Mode state
  into built-in `fusb302`:
  - `rk3399_fusb302_helper.ko`
- The helper exports the same getter name that `rk_cdn_dp` probes for:
  - `fusb302_get_dp_altmode_state(device_t,
    struct rk3399_typec_dp_altmode_status *)`
- The helper exposes post-boot sysctls:
  - `hw.rk3399_fusb302_helper.valid`
  - `hw.rk3399_fusb302_helper.dp_ready`
  - `hw.rk3399_fusb302_helper.usb_ss`
  - `hw.rk3399_fusb302_helper.pin_assignment`
  - `hw.rk3399_fusb302_helper.dp_status`
  - `hw.rk3399_fusb302_helper.get_count`
- On the stable `kernel.old` system, this module builds, loads, and is consumed
  by `rk_cdn_dp`.

What the helper path proved:

- With helper-fed legacy-good values:
  - `valid=1`
  - `dp_ready=1`
  - `usb_ss=0`
  - `pin_assignment=0x8`
  - `dp_status=0x9a`
- `rk_cdn_dp` consumes them successfully at `stage=12`:
  - `hw.rk3399_fusb302_helper.get_count=1`
  - `dev.rk_cdn_dp.0.dp_altmode_valid=1`
  - `dev.rk_cdn_dp.0.dp_altmode_ready=1`
  - `dev.rk_cdn_dp.0.dp_altmode_usb_ss=0`
  - `dev.rk_cdn_dp.0.dp_altmode_pin_assignment=8`
  - `dev.rk_cdn_dp.0.dp_altmode_status=154`
- `stage=12` still completes cleanly and preserves:
  - `hpd_status=1`
  - `last_error=0`

What `stage=13` still does with the helper:

- The helper is consumed again:
  - `hw.rk3399_fusb302_helper.get_count=2`
- But the first real `READ_DPCD` mailbox send still stalls at the same
  boundary:
  - `mbox_last_send_header=0x03010005`
  - `mbox_last_send_size=9`
  - `mbox_last_send_written=0`
  - `mbox_last_full=1`
  - `mbox_last_empty=1`
  - `mbox_last_empty_after_send=1`
  - `mbox_last_write_full_first=1`
  - `mbox_last_write_full_last=1`
  - `mbox_last_write_full_polls=1676`
  - `mbox_last_header=0`
  - `stage` falls back to `12`

Conclusion from that helper result:

- The remaining blocker is not:
  - built-in `fusb302` changes
  - helper-vs-built-in DP Alt Mode state plumbing
  - missing consumption of legacy-good `pin_assignment=0x8` /
    `dp_status=0x9a`
- The remaining blocker is still the deeper Cadence-side precondition behind
  the first real `READ_DPCD` mailbox send.

Eliminated FreeBSD boot theory:

- A dedicated A/B kernel test reverted only the built-in `fusb302` Alt Mode
  export/sysctl additions.
- That reverted kernel still hung during late boot after deep device attach.
- So the built-in `fusb302` export path is not the primary FreeBSD boot
  blocker.

Useful FUSB300C datasheet note:

- Public document:
  - `https://docs.rs-online.com/2c77/0900766b8145d60e.pdf`
  - `FUSB300C Programmable USB Type-C Controller`
- It confirms the FUSB300-class device is a thin client driven by host
  software over I2C and interrupts:
  - `WAKE`
  - `VBUSOK`
  - `BC_LVL`
  - `COMP`
- This supports the current diagnosis that real Type-C / Alt Mode behavior is
  event-driven, not just a set of final state values.
- It does not provide:
  - USB-PD Alt Mode details
  - Cadence mailbox details
  - RK3399 DP integration details

Post-boot helper plan:

- Do not continue modifying the early `rk_typec_phy` boot path.
- Use a loadable helper module instead:
  - `rk3399_tcphy_helper`
- The helper resolves `/phy@ff7c0000` and its `rockchip,grf` syscon after boot
  and exposes explicit sysctls to apply the RockPro64 TC-PHY GRF fields on
  demand:
  - `rockchip,typec-conn-dir`
  - `rockchip,external-psm`
  - `rockchip,uphy-dp-sel`
- This allows one-field-at-a-time testing before `stage=13` without changing
  the early driver dependency graph.

Native DTB result:

- The RockPro64 DTB was patched under `&tcphy0` to add:
  - `rockchip,typec-conn-dir`
  - `rockchip,external-psm`
  - `rockchip,uphy-dp-sel`
- After rebooting on that DTB, the old transmit-side blocker disappeared
  without the helper loaded:
  - `mbox_last_send_written=9`
  - `mbox_last_full=0`
- So those TC-PHY properties are a required native board precondition.

Remaining blocker after the DTB fix:

- The mailbox reply still never appears:
  - `mbox_last_empty=1`
  - `mbox_last_empty_after_send=1`
  - `mbox_last_header=0`
- That moved the frontier from "cannot send READ_DPCD" to
  "successful READ_DPCD send with no reply."

Later helper experiments that did not move the new frontier:

- `rk3399_tcphy_helper` was extended post-boot to test additional RK3399 PHY
  writes without touching the kernel:
  - `PMA_LANE_CFG = PIN_ASSIGN_C_E`
  - `PMA_LANE_CFG = PIN_ASSIGN_D_F`
  - `DP_MODE_CTL = DP_MODE_ENTER_A2`
  - `DP_MODE_CTL = DP_MODE_ENTER_A2` then `DP_MODE_ENTER_A0`
- All of those writes applied successfully and read back expected values, but
  none changed the live `stage=13` mailbox boundary:
  - `dev.rk_cdn_dp.0.stage=12`
  - `dev.rk_cdn_dp.0.mbox_last_send_written=0`
  - `dev.rk_cdn_dp.0.mbox_last_full=1`
  - `dev.rk_cdn_dp.0.mbox_last_empty=1`
- So the next missing ingredient is no longer basic TC-PHY field programming;
  it is higher-level Type-C / DP Alt Mode state or another PHY-side sequence
  beyond these direct register writes.

Detach/rebind crash note:

- A later crash reported:
  - `Fatal data abort`
  - `elr: rk_cdn_dp_mailbox_write + 0xac`
  - `far: 0x8`
  immediately after a `detached` message on serial.
- That points to a detach/rebind race in `rk_cdn_dp` where mailbox MMIO was
  touched after teardown began.
- Source-side mitigation applied:
  - disable the async module-load rebind task during staged bring-up
  - reject mailbox MMIO once a new `detached` flag is set in release/detach
