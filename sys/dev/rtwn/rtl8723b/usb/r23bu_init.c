/*-
 * SPDX-License-Identifier: ISC
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

/*
 * Skeleton init for 8723BU.  These stubs return success without
 * touching hardware — enough to link and attach; will fail at run
 * time when the chip doesn't respond as an 8188E.  Each function is
 * ready to be filled in from the Linux vendor reference
 * (`drivers/staging/rtl8723bs/hal/rtl8723b_hal_init.c` for the power
 * sequence + init tables) or from Linux mainline `rtl8xxxu`.
 */

#include <sys/cdefs.h>
#include "opt_wlan.h"

#include <sys/param.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/mbuf.h>
#include <sys/kernel.h>
#include <sys/socket.h>
#include <sys/systm.h>
#include <sys/malloc.h>
#include <sys/queue.h>
#include <sys/taskqueue.h>
#include <sys/bus.h>
#include <sys/endian.h>
#include <sys/linker.h>

#include <net/if.h>
#include <net/ethernet.h>
#include <net/if_media.h>

#include <net80211/ieee80211_var.h>
#include <net80211/ieee80211_radiotap.h>

#include <dev/rtwn/if_rtwnreg.h>
#include <dev/rtwn/if_rtwnvar.h>

#include <dev/rtwn/usb/rtwn_usb_var.h>

#include <dev/rtwn/rtl8192c/r92c.h>
#include <dev/rtwn/rtl8192c/r92c_reg.h>

#include <dev/rtwn/rtl8723b/r23b.h>
#include <dev/rtwn/rtl8723b/usb/r23bu.h>

/*
 * Port of Linux staging/rtl8723bs's RTL8723B_TRANS_CARDEMU_TO_ACT
 * power sequence (drivers/staging/rtl8723bs/include/hal_pwr_seq.h),
 * USB-interface entries only.  Each block below implements one row
 * of the Linux power-seq table.
 *
 * Register offsets that don't have R92C_* macros yet are written as
 * raw hex — mapping:
 *   0x0049 = REG_HSIMR (interrupt trigger mode)
 *   0x0058 = REG_HSISR (interrupt status)
 *   0x005a = REG_HSISR_EXT (upper 16 bits)
 *   0x0062 = REG_GPIO_INTMASK   (input-mode select)
 *   0x0063 = REG_GPIO_INT_MODE  (mode select)
 *   0x0067 = REG_MULTI_FUNC_CTRL / BT_GPS_SEL
 *   0x0069 = REG_GPIO_PULL_CTRL
 */
int
r23bu_power_on(struct rtwn_softc *sc)
{
#define RTWN_CHK(res) do {	\
	if (res != 0)		\
		return (EIO);	\
} while (0)
	int ntries;

	/* 0x20[0] = 1: enable LDOA12 MACRO block. */
	RTWN_CHK(rtwn_setbits_1(sc, R92C_LDOA15_CTRL, 0, 0x01));

	/* 0x67[4] = 0: disable BT_GPS_SEL pins. */
	RTWN_CHK(rtwn_setbits_1(sc, 0x0067, 0x10, 0));

	/* Delay 1 ms. */
	rtwn_delay(sc, 1000);

	/* 0x00[5] = 0: release analog Ips to digital. */
	RTWN_CHK(rtwn_setbits_1(sc, R92C_SYS_ISO_CTRL, 0x20, 0));

	/* 0x05[4:2] = 0: disable SW LPS + WLSUS_EN. */
	RTWN_CHK(rtwn_setbits_1(sc, R92C_APS_FSMCO + 1, 0x1c, 0));

	/*
	 * 0x06[1] polling: wait until power ready
	 * (R92C_APS_FSMCO byte 2, bit 1).
	 */
	for (ntries = 0; ntries < 5000; ntries++) {
		if (rtwn_read_1(sc, R92C_APS_FSMCO + 2) & 0x02)
			break;
		rtwn_delay(sc, 10);
	}
	if (ntries == 5000) {
		device_printf(sc->sc_dev,
		    "r23bu_power_on: timeout waiting for power ready\n");
		return (ETIMEDOUT);
	}

	/* 0x06[0] = 1: release WLON reset. */
	RTWN_CHK(rtwn_setbits_1(sc, R92C_APS_FSMCO + 2, 0, 0x01));

	/* 0x05[7] = 0: disable HWPDN. */
	RTWN_CHK(rtwn_setbits_1(sc, R92C_APS_FSMCO + 1, 0x80, 0));

	/* 0x05[4:3] = 0: disable WL suspend. */
	RTWN_CHK(rtwn_setbits_1(sc, R92C_APS_FSMCO + 1, 0x18, 0));

	/* 0x05[0] = 1: APFM_ONMAC (turn on MAC by HW state machine). */
	RTWN_CHK(rtwn_setbits_1(sc, R92C_APS_FSMCO + 1, 0, 0x01));

	/* Poll 0x05[0] = 0: wait for HW state machine to finish. */
	for (ntries = 0; ntries < 5000; ntries++) {
		if (!(rtwn_read_1(sc, R92C_APS_FSMCO + 1) & 0x01))
			break;
		rtwn_delay(sc, 10);
	}
	if (ntries == 5000) {
		device_printf(sc->sc_dev,
		    "r23bu_power_on: timeout waiting for APFM_ONMAC\n");
		return (ETIMEDOUT);
	}

	/* 0x10[6] = 1: enable WL control XTAL setting. */
	RTWN_CHK(rtwn_setbits_1(sc, 0x0010, 0, 0x40));

	/* 0x49[1] = 1: enable falling edge interrupt trigger. */
	RTWN_CHK(rtwn_setbits_1(sc, 0x0049, 0, 0x02));

	/* 0x63[1] = 1: enable GPIO9 interrupt mode. */
	RTWN_CHK(rtwn_setbits_1(sc, 0x0063, 0, 0x02));

	/* 0x62[1] = 0: enable GPIO9 input mode. */
	RTWN_CHK(rtwn_setbits_1(sc, 0x0062, 0x02, 0));

	/* 0x58[0] = 1: enable HSISR GPIO[C:0] interrupt. */
	RTWN_CHK(rtwn_setbits_1(sc, 0x0058, 0, 0x01));

	/* 0x5A[1] = 1: enable HSISR GPIO9 interrupt. */
	RTWN_CHK(rtwn_setbits_1(sc, 0x005A, 0, 0x02));

	/* 0x69[6] = 1: GPIO9 internal pull high. */
	RTWN_CHK(rtwn_setbits_1(sc, 0x0069, 0, 0x40));

	/* Enable MAC DMA/WMAC/SCHEDULE/SEC blocks. */
	RTWN_CHK(rtwn_write_2(sc, R92C_CR, 0));
	{
		uint16_t reg = rtwn_read_2(sc, R92C_CR);
		reg |= R92C_CR_HCI_TXDMA_EN | R92C_CR_HCI_RXDMA_EN |
		    R92C_CR_TXDMA_EN | R92C_CR_RXDMA_EN |
		    R92C_CR_PROTOCOL_EN | R92C_CR_SCHEDULE_EN |
		    R92C_CR_MACTXEN | R92C_CR_MACRXEN |
		    R92C_CR_ENSEC | R92C_CR_CALTMR_EN;
		RTWN_CHK(rtwn_write_2(sc, R92C_CR, reg));
	}

	return (0);
#undef RTWN_CHK
}

void
r23bu_power_off(struct rtwn_softc *sc)
{
	device_printf(sc->sc_dev, "r23bu_power_off: STUB\n");
}

#ifndef RTWN_WITHOUT_UCODE
/*
 * 8723B firmware download enable — differs from 8188E in that it
 * must first enable the 8051 MCU (SYS_FUNC_EN bit 10 = FEN_CPU)
 * before flipping MCUFWDL_EN.  Ported from Linux staging/rtl8723bs
 * hal/rtl8723b_hal_init.c::_FWDownloadEnable.
 */
void
r23b_fw_download_enable(struct rtwn_softc *sc, int enable)
{
	int ntries;

	if (enable) {
		/* Enable 8051 CPU (REG_SYS_FUNC_EN+1 |= 0x04). */
		rtwn_setbits_1(sc, R92C_SYS_FUNC_EN + 1, 0, 0x04);

		/*
		 * Set MCUFWDL bit 0 with retry: Linux polls until the
		 * bit stays set, retrying up to 100 times.  On some
		 * chip cuts the first write is discarded.
		 */
		for (ntries = 0; ntries < 100; ntries++) {
			rtwn_setbits_1(sc, R92C_MCUFWDL, 0, 0x01);
			if (rtwn_read_1(sc, R92C_MCUFWDL) & 0x01)
				break;
			rtwn_delay(sc, 1000);
		}

		/* 8051 reset: clear MCUFWDL+2 bit 3. */
		rtwn_setbits_1(sc, R92C_MCUFWDL + 2, 0x08, 0);
	} else {
		/* MCU download disable. */
		rtwn_setbits_1(sc, R92C_MCUFWDL, 0x01, 0);
	}
}
#endif

/*
 * 8723B BB / RF glue enable.  Mirrors r88eu_init_bb: reset BB, enable
 * RF via RF_CTRL, bring BB back up via SYS_FUNC_EN, then let the
 * generic r92c walker program the PHY_REG and AGC tables from
 * rtl8723b_bb / rtl8723b_agc.  8723B does NOT need the DIO_RF bit
 * (that's a PCIe path) — this dongle is USB.
 */
void
r23bu_init_bb(struct rtwn_softc *sc)
{

	/* Reset BB: clear BBRSTB + BB_GLB_RST while touching SYS_FUNC_EN. */
	rtwn_setbits_1(sc, R92C_SYS_FUNC_EN,
	    R92C_SYS_FUNC_EN_BBRSTB | R92C_SYS_FUNC_EN_BB_GLB_RST, 0);

	/* Enable RF core. */
	rtwn_write_1(sc, R92C_RF_CTRL,
	    R92C_RF_CTRL_EN | R92C_RF_CTRL_RSTB | R92C_RF_CTRL_SDMRSTB);

	/* Bring BB glue back up (USB path — no DIO_RF). */
	rtwn_write_1(sc, R92C_SYS_FUNC_EN,
	    R92C_SYS_FUNC_EN_USBA | R92C_SYS_FUNC_EN_USBD |
	    R92C_SYS_FUNC_EN_BB_GLB_RST | R92C_SYS_FUNC_EN_BBRSTB);

	/* Walk PHY_REG + AGC tables (extracted from Linux staging). */
	r92c_init_bb_common(sc);
}

void
r23bu_init_intr(struct rtwn_softc *sc)
{
	/*
	 * 8723B interrupt masks — similar to 8188E but with additional
	 * BT-coex mask bits.  Placeholder writes 0 for now.
	 */
	rtwn_write_4(sc, R92C_HISR, 0xffffffff);
	rtwn_write_4(sc, R92C_HIMR, 0);
}

void
r23bu_init_rx_agg(struct rtwn_softc *sc)
{
	/* Placeholder — 8723B RX agg thresholds pending. */
}

void
r23bu_post_init(struct rtwn_softc *sc)
{
	/*
	 * Post-init hooks: BT coex enable, RA table setup, dynamic
	 * mechanism (DM) init.  Deferred until BT coex layer is ported.
	 */
}
