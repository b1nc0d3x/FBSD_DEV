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
 * rtwn(4) USB attach for the Realtek RTL8723BU.
 *
 * 8723BU is a 1T1R 802.11n USB dongle with an integrated Bluetooth
 * radio.  The BT interface (Interface 0/1 of the USB device) is
 * handled by ng_ubt(4); this driver claims only Interface 2 (WiFi).
 *
 * First-pass strategy: reuse 8188E (`r88e_*`) and 8192C (`r92c_*`)
 * callbacks wherever the hardware protocol matches.  8723B-specific
 * callbacks (power on/off, BT coex arbitration, PA/LNA init, EFUSE
 * map) are added incrementally as they surface on hardware in the
 * loop.  Any callback still pointing at the 8188E variant here is a
 * candidate for chip-specific replacement.
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
#include <dev/rtwn/if_rtwn_nop.h>

#include <dev/rtwn/usb/rtwn_usb_var.h>

#include <dev/rtwn/rtl8192c/r92c_reg.h>
#include <dev/rtwn/rtl8192c/r92c_var.h>
#include <dev/rtwn/rtl8192c/usb/r92cu.h>
#include <dev/rtwn/rtl8192c/usb/r92cu_tx_desc.h>

#include <dev/rtwn/rtl8188e/r88e.h>
#include <dev/rtwn/rtl8188e/r88e_priv.h>
#include <dev/rtwn/rtl8188e/r88e_rom_image.h>	/* for macaddr field */
#include <dev/rtwn/rtl8188e/usb/r88eu.h>

#include <dev/rtwn/rtl8723b/r23b.h>
#include <dev/rtwn/rtl8723b/r23b_priv.h>
#include <dev/rtwn/rtl8723b/usb/r23bu.h>

static struct rtwn_r88e_txpwr r23b_txpwr;	/* placeholder, real 8723B table pending */

void	r23bu_attach(struct rtwn_usb_softc *);

/*
 * 8723B EFUSE map has the MAC address at a different offset than 8188E.
 * Placeholder: reuse 8188E parser for now, will be replaced when the
 * 8723B EFUSE layout is ported.
 */
static void
r23bu_set_macaddr(struct rtwn_softc *sc, uint8_t *buf)
{
	struct r88e_rom *rom = (struct r88e_rom *)buf;

	IEEE80211_ADDR_COPY(sc->sc_ic.ic_macaddr, rom->diff_d0.usb.macaddr);
}

static void
r23b_postattach(struct rtwn_softc *sc)
{
	struct r92c_softc *rs = sc->sc_priv;
	struct ieee80211com *ic = &sc->sc_ic;

	rs->rs_scan_start = ic->ic_scan_start;
	ic->ic_scan_start = r92c_scan_start;
	rs->rs_scan_end = ic->ic_scan_end;
	ic->ic_scan_end = r92c_scan_end;
}

static void
r23bu_attach_private(struct rtwn_softc *sc)
{
	struct r92c_softc *rs;

	rs = malloc(sizeof(struct r92c_softc), M_RTWN_PRIV, M_WAITOK | M_ZERO);

	rs->rs_txpwr			= &r23b_txpwr;
	rs->rs_txagc			= NULL;

	/* 8188E-shaped callbacks — good enough for first attach. */
	rs->rs_set_bw20			= r88e_set_bw20;
	rs->rs_get_txpower		= r88e_get_txpower;
	rs->rs_set_gain			= r88e_set_gain;
	rs->rs_tx_enable_ampdu		= r88e_tx_enable_ampdu;
	rs->rs_tx_setup_hwseq		= r88e_tx_setup_hwseq;
	rs->rs_tx_setup_macid		= r88e_tx_setup_macid;
	rs->rs_set_rom_opts		= r23bu_set_macaddr;

	rs->rf_read_delay[0]		= 10;
	rs->rf_read_delay[1]		= 100;
	rs->rf_read_delay[2]		= 10;

	sc->sc_priv = rs;
}

static void
r23bu_adj_devcaps(struct rtwn_softc *sc)
{
	/* 2.4 GHz only — 5 GHz caps stay off. */
}

void
r23bu_attach(struct rtwn_usb_softc *uc)
{
	struct rtwn_softc *sc		= &uc->uc_sc;

	/* USB part. */
	uc->uc_align_rx			= r92cu_align_rx;
	uc->tx_agg_desc_num		= 6;

	/* Common part. */
	sc->sc_flags			= RTWN_FLAG_EXT_HDR;

	/*
	 * First-pass callback table: mostly r88e/r92c reuses.  Each entry
	 * marked TODO is a known chip-specific difference that will show
	 * up during hardware bring-up.
	 */
	sc->sc_set_chan			= r92c_set_chan;
	sc->sc_fill_tx_desc		= r92c_fill_tx_desc;
	sc->sc_fill_tx_desc_raw 	= r92c_fill_tx_desc_raw;
	sc->sc_fill_tx_desc_null	= r92c_fill_tx_desc_null;
	sc->sc_dump_tx_desc		= r92cu_dump_tx_desc;
	sc->sc_tx_radiotap_flags	= r92c_tx_radiotap_flags;
	sc->sc_rx_radiotap_flags	= r92c_rx_radiotap_flags;
	sc->sc_get_rx_stats		= r88e_get_rx_stats;
	sc->sc_get_rssi_cck		= r88e_get_rssi_cck;
	sc->sc_get_rssi_ofdm		= r88e_get_rssi_ofdm;
	sc->sc_classify_intr		= r88e_classify_intr;
	sc->sc_handle_tx_report		= r88e_ratectl_tx_complete;
	sc->sc_handle_tx_report2	= r88e_ratectl_tx_complete_periodic;
	sc->sc_handle_c2h_report	= r88e_handle_c2h_report;
	sc->sc_check_frame		= rtwn_nop_int_softc_mbuf;
	sc->sc_rf_read			= r92c_rf_read;
	sc->sc_rf_write			= r88e_rf_write;	/* TODO: 8723B RF write may differ */
	sc->sc_check_condition		= r92c_check_condition;
	sc->sc_efuse_postread		= rtwn_nop_softc;
	sc->sc_parse_rom		= r88e_parse_rom;	/* TODO: 8723B EFUSE map port */
	sc->sc_set_led			= r88e_set_led;
	sc->sc_power_on			= r23bu_power_on;	/* stub */
	sc->sc_power_off		= r23bu_power_off;	/* stub */
#ifndef RTWN_WITHOUT_UCODE
	sc->sc_fw_reset			= r88e_fw_reset;	/* TODO: 8723B reset via HMEBOX */
	sc->sc_fw_download_enable	= r23b_fw_download_enable;
#endif
	sc->sc_llt_init			= r92c_llt_init;
	sc->sc_set_page_size		= r92c_set_page_size;
	sc->sc_lc_calib			= r92c_lc_calib;
	sc->sc_iq_calib			= r88e_iq_calib;	/* TODO: 8723B has its own IQK */
	sc->sc_read_chipid_vendor	= rtwn_nop_softc_uint32;
	sc->sc_adj_devcaps		= r23bu_adj_devcaps;
	sc->sc_vap_preattach		= rtwn_nop_softc_vap;
	sc->sc_postattach		= r23b_postattach;
	sc->sc_detach_private		= r92c_detach_private;
	sc->sc_set_media_status		= r88e_set_media_status;
#ifndef RTWN_WITHOUT_UCODE
	sc->sc_set_rsvd_page		= r88e_set_rsvd_page;
	sc->sc_set_pwrmode		= r88e_set_pwrmode;
	sc->sc_set_rssi			= rtwn_nop_softc;
#endif
	sc->sc_beacon_init		= r92c_beacon_init;
	sc->sc_beacon_enable		= r88e_beacon_enable;
	sc->sc_sta_beacon_enable	= r88e_sta_beacon_enable;
	sc->sc_beacon_set_rate		= rtwn_nop_void_int;
	sc->sc_beacon_select		= rtwn_nop_softc_int;
	sc->sc_temp_measure		= r88e_temp_measure;
	sc->sc_temp_read		= r88e_temp_read;
	sc->sc_init_tx_agg		= r92cu_init_tx_agg;
	sc->sc_init_rx_agg		= r23bu_init_rx_agg;	/* stub */
	sc->sc_init_ampdu		= rtwn_nop_softc;
	sc->sc_init_intr		= r23bu_init_intr;	/* stub */
	sc->sc_init_edca		= r92c_init_edca;
	sc->sc_init_bb			= r23bu_init_bb;	/* stub */
	sc->sc_init_rf			= r92c_init_rf;	/* TODO: 8723B has RFE0+RFE1 */
	sc->sc_init_antsel		= rtwn_nop_softc;
	sc->sc_post_init		= r23bu_post_init;	/* stub */
	sc->sc_init_bcnq1_boundary	= rtwn_nop_int_softc;
	sc->sc_set_tx_power		= r92c_set_tx_power;

	sc->mac_prog			= &rtl8723b_mac[0];
	sc->mac_size			= nitems(rtl8723b_mac);
	sc->bb_prog			= &rtl8723b_bb[0];
	sc->bb_size			= nitems(rtl8723b_bb);
	sc->agc_prog			= &rtl8723b_agc[0];
	sc->agc_size			= nitems(rtl8723b_agc);
	sc->rf_prog			= &rtl8723b_rf[0];

	sc->name			= "RTL8723BU";
	sc->fwname			= "rtwn-rtl8723bufw";
	sc->fwsig			= R23B_FW_SIG;

	sc->page_count			= R23BU_TX_PAGE_COUNT;
	sc->pktbuf_count		= R23B_TXPKTBUF_COUNT;

	sc->ackto			= 0x40;
	sc->npubqpages			= R23BU_PUBQ_NPAGES;
	sc->page_size			= R92C_TX_PAGE_SIZE;

	sc->txdesc_len			= sizeof(struct r92cu_tx_desc);
	sc->efuse_maxlen		= R23B_EFUSE_MAX_LEN;
	sc->efuse_maplen		= R23B_EFUSE_MAP_LEN;
	sc->rx_dma_size			= R23B_RX_DMA_BUFFER_SIZE;

	sc->macid_limit			= R23B_MACID_MAX + 1;
	sc->macid_rpt2_max_num		= 2;
	sc->cam_entry_limit		= R92C_CAM_ENTRY_COUNT;
	sc->fwsize_limit		= R23B_MAX_FW_SIZE;
	sc->temp_delta			= R23B_CALIB_THRESHOLD;

	sc->bcn_status_reg[0]		= R92C_TDECTRL;
	sc->bcn_status_reg[1]		= R92C_TDECTRL;
	sc->rcr				= 0;

	sc->ntxchains			= 1;
	sc->nrxchains			= 1;

	r23bu_attach_private(sc);
}
