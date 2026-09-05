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

#ifndef RTL8723BU_H
#define RTL8723BU_H

#include <dev/rtwn/rtl8723b/r23b.h>

/*
 * USB-side page allocation for 8723B.  Numbers taken from Linux
 * rtl8xxxu_gen2_priv.rtl8xxxu_setup_ampdu() and rtl8723bu
 * page-count constants; refined during hardware bring-up.
 */
#define R23BU_PUBQ_NPAGES	186
#define R23BU_TX_PAGE_COUNT	247

/* USB attach entry point (called from rtwn_usb_attach.h dispatcher). */
void	r23bu_attach(struct rtwn_usb_softc *);

/* USB-side init (chip enable, MAC init, interrupt masks). */
int	r23bu_power_on(struct rtwn_softc *);
void	r23bu_power_off(struct rtwn_softc *);
void	r23bu_init_bb(struct rtwn_softc *);
void	r23bu_init_intr(struct rtwn_softc *);
void	r23bu_init_rx_agg(struct rtwn_softc *);
void	r23bu_post_init(struct rtwn_softc *);

#ifndef RTWN_WITHOUT_UCODE
void	r23b_fw_download_enable(struct rtwn_softc *, int);
#endif

#endif	/* RTL8723BU_H */
