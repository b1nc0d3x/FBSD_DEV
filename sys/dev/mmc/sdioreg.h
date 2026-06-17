/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 *
 * SDIO vendor / device / function-class IDs.  Function-bus drivers
 * (bwfm_sdio, sdiobt, etc.) match against (manfid, prodid) pairs
 * via newbus probe methods on the sdio_func bus.  Centralised here
 * so multiple drivers can share the namespace without each rolling
 * its own table.
 *
 * Naming follows the FreeBSD pcireg.h convention (PCI_VENDOR_*,
 * PCI_DEVICE_*) since the matching model is the same.
 *
 * Sources:
 *   - SD Association SDIO Specification 3.0 (CIS tuple layout)
 *   - SDIO Card Manufacturer ID registry (incl. cypress.com PDF)
 *   - Linux drivers/net/wireless/broadcom/brcm80211/brcmfmac/sdio.h
 *   - Linux drivers/mmc/core/sdio_ids.h
 */

#ifndef _DEV_MMC_SDIOREG_H_
#define _DEV_MMC_SDIOREG_H_

/* ------------------------------------------------------------------
 * Manufacturer (vendor) IDs.  CISTPL_MANFID first word.
 * ------------------------------------------------------------------ */
#define	SDIO_VENDOR_BROADCOM		0x02d0	/* Broadcom / Cypress */
#define	SDIO_VENDOR_ATHEROS		0x0271	/* Atheros / QCA */
#define	SDIO_VENDOR_MARVELL		0x02df	/* Marvell */
#define	SDIO_VENDOR_TI			0x0097	/* Texas Instruments */
#define	SDIO_VENDOR_REALTEK		0x024c	/* Realtek */
#define	SDIO_VENDOR_INTEL		0x0089	/* Intel */
#define	SDIO_VENDOR_SIANO		0x039a	/* Siano (DTV demod) */
#define	SDIO_VENDOR_CYPRESS		0x04b4	/* Cypress (post-Broadcom) */

/* ------------------------------------------------------------------
 * Broadcom WiFi SDIO product IDs (CISTPL_MANFID second word).  These
 * match the chip family — actual firmware/NVRAM lookup uses the
 * (manfid, prodid) pair as a key.
 * ------------------------------------------------------------------ */
#define	SDIO_DEVICE_BCM4329		0x4329	/* BCM4329 */
#define	SDIO_DEVICE_BCM4330		0x4330	/* BCM4330 */
#define	SDIO_DEVICE_BCM4334		0x4334	/* BCM4334 */
#define	SDIO_DEVICE_BCM43340		0xa94c	/* BCM43340 */
#define	SDIO_DEVICE_BCM43341		0xa94d	/* BCM43341 */
#define	SDIO_DEVICE_BCM43362		0xa962	/* BCM43362 */
#define	SDIO_DEVICE_BCM43364		0xa9a4	/* BCM43364 */
#define	SDIO_DEVICE_BCM43430		0xa9a6	/* BCM43430 (Pi 3) */
#define	SDIO_DEVICE_BCM43455		0xa9bf	/* BCM43455 (Pi 3B+/4) */
#define	SDIO_DEVICE_BCM43456		0xa9c0	/* BCM43456 (Pi CM4) */
#define	SDIO_DEVICE_BCM4339		0x4339	/* BCM4339 */
#define	SDIO_DEVICE_BCM4345		0x4345	/* BCM4345 */
#define	SDIO_DEVICE_BCM4354		0x4354	/* BCM4354 */
#define	SDIO_DEVICE_BCM4356		0x4356	/* BCM4356 */
#define	SDIO_DEVICE_BCM4359		0x4359	/* BCM4359 */
#define	SDIO_DEVICE_BCM4373		0x4373	/* BCM4373 (Cypress CYW4373) */

/* ------------------------------------------------------------------
 * SDIO Function-class codes (CISTPL_FUNCID class byte).  These tell
 * the bus what KIND of function a card slot exposes, before the
 * per-vendor (manfid, prodid) lookup.  Most modern WiFi chips report
 * 0x0c (WLAN); some older ones use 0xff (vendor-specific).
 * ------------------------------------------------------------------ */
#define	SDIO_FUNCCLASS_NONE		0x00	/* No standard interface */
#define	SDIO_FUNCCLASS_UART		0x01	/* UART */
#define	SDIO_FUNCCLASS_BT_TYPEA		0x02	/* Bluetooth Type-A */
#define	SDIO_FUNCCLASS_BT_TYPEB		0x03	/* Bluetooth Type-B */
#define	SDIO_FUNCCLASS_GPS		0x04	/* GPS receiver */
#define	SDIO_FUNCCLASS_CAMERA		0x05	/* Camera */
#define	SDIO_FUNCCLASS_PHS		0x06	/* PHS (Personal Handy-phone) */
#define	SDIO_FUNCCLASS_WLAN		0x07	/* WLAN interface */
#define	SDIO_FUNCCLASS_ATA		0x08	/* Embedded SDIO-ATA standard */
#define	SDIO_FUNCCLASS_BT_TYPEAMP	0x09	/* Bluetooth AMP */
#define	SDIO_FUNCCLASS_VENDOR		0xff	/* Vendor-specific */

#endif /* _DEV_MMC_SDIOREG_H_ */
