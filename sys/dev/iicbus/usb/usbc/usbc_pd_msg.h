/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw
 * All rights reserved.
 */

#ifndef _USBC_PD_MSG_H_
#define _USBC_PD_MSG_H_

#include <dev/iicbus/usb/usbc/usbc_pd.h>

/*
 * Hardware-independent helpers for building and parsing PD messages.
 * The TCPC chip driver puts the resulting header + payload into its
 * TX FIFO using whatever framing the chip requires (BMC tokens for
 * FUSB302, or a raw packet for ITE TCPCI-class chips).
 */

struct usbc_pd_msg {
	uint16_t	hdr;
	uint8_t		ndo;		/* number of data objects, 0..7 */
	uint32_t	data[7];
};

/*
 * Build a PD message header (no data objects).
 *
 * type:	control or data message type (USBC_PD_CTRL_* / USBC_PD_DATA_*)
 * ndo:		number of 32-bit data objects (0 for control messages)
 * msg_id:	rolling 3-bit MessageID counter, caller-managed
 * power_role:	source/sink (USBC_PD_ROLE_*)
 * data_role:	UFP/DFP (USBC_PD_DATA_*)
 * rev:		PD revision (USBC_PD_REV_*)
 */
uint16_t	usbc_pd_build_header(uint8_t type, uint8_t ndo, uint8_t msg_id,
		    enum usbc_pd_power_role power_role,
		    enum usbc_pd_data_role data_role,
		    enum usbc_pd_rev rev);

/*
 * Build a Fixed Supply Source PDO at the given voltage and max current.
 * Caller selects flags from USBC_PD_PDO_FIXED_* (USB_COMMS, DUAL_ROLE,
 * UNCHUNKED_EXT, etc) and ORs them into the result.
 */
uint32_t	usbc_pd_build_fixed_pdo(uint16_t voltage_mv,
		    uint16_t max_current_ma, uint32_t flags);

/*
 * Build a Fixed Supply Sink Request data object (RDO).
 *
 * pdo_index:	1-based index of the source PDO being selected
 * op_current_ma:	operating current the sink will draw
 * max_current_ma:	maximum current the sink may draw
 * flags:	bitmap of additional RDO flags (giveback, capability mismatch,
 *		USB comms, no suspend, etc — see PD 6.4.2)
 */
#define	USBC_PD_RDO_GIVEBACK		(1u << 27)
#define	USBC_PD_RDO_CAP_MISMATCH	(1u << 26)
#define	USBC_PD_RDO_USB_COMMS_CAPABLE	(1u << 25)
#define	USBC_PD_RDO_NO_USB_SUSPEND	(1u << 24)
#define	USBC_PD_RDO_UNCHUNKED_EXT	(1u << 23)

uint32_t	usbc_pd_build_request(uint8_t pdo_index, uint16_t op_current_ma,
		    uint16_t max_current_ma, uint32_t flags);

/*
 * Convenience: build a complete control message (header only, no data).
 * Returns true on success.
 */
bool		usbc_pd_msg_ctrl(struct usbc_pd_msg *msg, uint8_t ctrl_type,
		    uint8_t msg_id, enum usbc_pd_power_role pr,
		    enum usbc_pd_data_role dr, enum usbc_pd_rev rev);

/*
 * Convenience: build a complete data message with the given PDO/RDO/etc
 * payload.  Returns true if the payload fits in 7 data objects.
 */
bool		usbc_pd_msg_data(struct usbc_pd_msg *msg, uint8_t data_type,
		    const uint32_t *data, uint8_t ndo, uint8_t msg_id,
		    enum usbc_pd_power_role pr, enum usbc_pd_data_role dr,
		    enum usbc_pd_rev rev);

#endif /* !_USBC_PD_MSG_H_ */
