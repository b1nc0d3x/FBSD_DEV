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

#ifndef RTL8723B_H
#define RTL8723B_H

#include <dev/rtwn/if_rtwn_ridx.h>

/*
 * RTL8723BU global definitions.
 *
 * 8723B is a 1T1R 2.4GHz-only 802.11n silicon with an integrated
 * Bluetooth core (Interface 0/1 of the USB device; the WiFi is
 * Interface 2).  This driver only touches the WiFi side.
 *
 * Register layout is close enough to 8188E that most callbacks can
 * be reused from rtl8188e/ during initial bring-up.  Chip-specific
 * differences will be layered in as they surface on hardware.
 */

#define R23B_TXPKTBUF_COUNT	255	/* larger PKTBUF than 8188e (~177) */

#define R23B_MACID_MAX		127	/* wider CAM than 8188e */
#define R23B_RX_DMA_BUFFER_SIZE	0x3E80

#define R23B_INTR_MSG_LEN	60

#define R23B_CALIB_THRESHOLD	4

#define R23B_EFUSE_MAX_LEN	512	/* 8723B has 512-byte EFUSE */
#define R23B_EFUSE_MAP_LEN	512

/*
 * 8723B firmware header magic.  The firmware image (rtl8723bu_nic.bin)
 * begins with signature word 0x5301.  rtwn's fw loader compares the
 * high 12 bits vs sc->fwsig, so the constant is `signature >> 4`.
 */
#define R23B_FW_SIG		0x530

/*
 * 8723B WCPU firmware is ~32 KB (rtl8723bu_nic.bin from linux-firmware
 * is 32108 bytes).  R92C_MAX_FW_SIZE (0x4000) is too small.  Cap at
 * 64 KB — matches the space carved in the chip's boot ROM.
 */
#define R23B_MAX_FW_SIZE	0x10000

#endif	/* RTL8723B_H */
