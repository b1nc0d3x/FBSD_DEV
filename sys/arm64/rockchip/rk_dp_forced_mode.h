/*
 * Single source of truth for the USB-C DP forced video mode.
 *
 * The rk_drm and rk_cdn_dp drivers must push the exact same timing
 * values into the VOP and into the Cadence framer respectively.  If
 * they disagree the sink rejects the stream (link trains but no video
 * reaches the panel).  Edit values here only, then rebuild both modules.
 *
 * Polarity convention: 1 = positive sync, 0 = negative sync.
 *
 *   - The XYM W156F1 portable monitor on this rig drives 1920x1080@60
 *     fine over HDMI but its USB-C DP path won't render 1080p timing
 *     yet (link trains, sink reports valid, panel stays dark).  The
 *     custom 1400x1050 timing below is the empirical working baseline.
 *   - We've also failed at 1280x720 CEA-standard timing — likely
 *     because the panel's USB-C DP scaler dislikes the narrow / positive
 *     CEA-style sync pulse.  Wide-NHSYNC like the values below works.
 *
 * To experiment, change just the values in this file and rebuild.
 */

#ifndef _RK_DP_FORCED_MODE_H_
#define _RK_DP_FORCED_MODE_H_

/*
 * 1920x1080@60 with DMT-style wide-NHSYNC layout, mimicking the
 * sync format the panel accepts at 1400x1050.  Custom (non-CEA-VIC)
 * timing — htotal/vtotal match CEA 1080p60 (so VPLL doesn't need a
 * new entry), but hsync placement & width and sync polarity differ
 * from VIC 16, so the resulting stream shouldn't be recognized as a
 * CEA VIC by the panel.
 *
 * Layout: hfront=88, hsync=144 (wide), hback=48 → htotal 2200.
 */
#define RK_DP_FORCED_CLOCK_KHZ      148500
#define RK_DP_FORCED_HDISPLAY       1920
#define RK_DP_FORCED_HSYNC_START    2008
#define RK_DP_FORCED_HSYNC_END      2152
#define RK_DP_FORCED_HTOTAL         2200
#define RK_DP_FORCED_VDISPLAY       1080
#define RK_DP_FORCED_VSYNC_START    1084
#define RK_DP_FORCED_VSYNC_END      1089
#define RK_DP_FORCED_VTOTAL         1125
#define RK_DP_FORCED_H_POLARITY     0	/* NHSYNC (negative) */
#define RK_DP_FORCED_V_POLARITY     1	/* PVSYNC (positive) */

/* Derived values — DO NOT hand-edit. */
#define RK_DP_FORCED_HBLANK \
	(RK_DP_FORCED_HTOTAL - RK_DP_FORCED_HDISPLAY)
#define RK_DP_FORCED_VBLANK \
	(RK_DP_FORCED_VTOTAL - RK_DP_FORCED_VDISPLAY)
#define RK_DP_FORCED_HSYNC_LEN \
	(RK_DP_FORCED_HSYNC_END - RK_DP_FORCED_HSYNC_START)
#define RK_DP_FORCED_VSYNC_LEN \
	(RK_DP_FORCED_VSYNC_END - RK_DP_FORCED_VSYNC_START)

/* Match window for picking this mode from an EDID-derived list. */
#define RK_DP_FORCED_CLOCK_LO	    (RK_DP_FORCED_CLOCK_KHZ - 250)
#define RK_DP_FORCED_CLOCK_HI	    (RK_DP_FORCED_CLOCK_KHZ + 250)

#endif /* _RK_DP_FORCED_MODE_H_ */
