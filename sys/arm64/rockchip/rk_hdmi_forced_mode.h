/*
 * Single source of truth for the HDMI forced video mode.
 *
 * Mirrors rk_dp_forced_mode.h for the USB-C DP path.  The HDMI bring-up
 * path reads these constants when filling the default modeset mode and
 * when injecting the preferred mode into the HDMI-A-1 connector's
 * probed_modes list.  Edit values here, rebuild rk_drm.
 *
 * Polarity convention: 1 = positive sync, 0 = negative sync.
 *
 *   - 1080p60 with CEA-standard horizontal sync (hsync_start=2008,
 *     hsync_end=2052, htotal=2200) but vertical sync placement shifted
 *     2 lines earlier than CEA standard (vsync_start=1082, vsync_end=
 *     1087 — yields vact_start=43 instead of CEA's 41).  This is the
 *     empirically required timing for the XYM W156F1 panel's HDMI
 *     input acceptance check; standard CEA 1080p60 makes the panel
 *     show "No Support."
 *
 * To experiment, change just the values in this file and rebuild.
 */

#ifndef _RK_HDMI_FORCED_MODE_H_
#define _RK_HDMI_FORCED_MODE_H_

#define RK_HDMI_FORCED_CLOCK_KHZ      148500
#define RK_HDMI_FORCED_HDISPLAY       1920
#define RK_HDMI_FORCED_HSYNC_START    2008
#define RK_HDMI_FORCED_HSYNC_END      2052
#define RK_HDMI_FORCED_HTOTAL         2200
#define RK_HDMI_FORCED_VDISPLAY       1080
#define RK_HDMI_FORCED_VSYNC_START    1082
#define RK_HDMI_FORCED_VSYNC_END      1087
#define RK_HDMI_FORCED_VTOTAL         1125
#define RK_HDMI_FORCED_H_POLARITY     1	/* PHSYNC (positive) */
#define RK_HDMI_FORCED_V_POLARITY     1	/* PVSYNC (positive) */

/* Derived values — DO NOT hand-edit. */
#define RK_HDMI_FORCED_HBLANK \
	(RK_HDMI_FORCED_HTOTAL - RK_HDMI_FORCED_HDISPLAY)
#define RK_HDMI_FORCED_VBLANK \
	(RK_HDMI_FORCED_VTOTAL - RK_HDMI_FORCED_VDISPLAY)
#define RK_HDMI_FORCED_HSYNC_LEN \
	(RK_HDMI_FORCED_HSYNC_END - RK_HDMI_FORCED_HSYNC_START)
#define RK_HDMI_FORCED_VSYNC_LEN \
	(RK_HDMI_FORCED_VSYNC_END - RK_HDMI_FORCED_VSYNC_START)

#endif /* _RK_HDMI_FORCED_MODE_H_ */
