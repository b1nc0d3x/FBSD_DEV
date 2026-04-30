/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/*
 * FUSB302 USB Type-C controller with USB-PD source policy and DP Alt Mode
 * VDM state machine.  Ported from Rockchip Linux 4.4 BSP (drivers/mfd/fusb302.c).
 *
 * Implements the DFP (source) path: CC detection → source caps negotiation →
 * Discover_Identity → Discover_SVIDs → Discover_Modes → Enter_Mode →
 * DP_Status → DP_Config.  On success the result is exported via
 * fusb302_get_dp_altmode_state() for rk_cdn_dp.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/callout.h>
#include <sys/kernel.h>
#include <sys/intr.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <sys/sysctl.h>
#include <sys/taskqueue.h>
#include <machine/atomic.h>

#include <dev/iicbus/iiconf.h>
#include <dev/iicbus/iicbus.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/ofw/openfirm.h>

#include <dev/regulator/regulator.h>

#include <arm64/rockchip/rk3399_typec_altmode_var.h>

#include "iicbus_if.h"
#include "fusb302_var.h"

/* -----------------------------------------------------------------------
 * Register map
 * ----------------------------------------------------------------------- */
#define	FUSB_REG_DEVICE_ID	0x01
#define	FUSB_REG_SWITCHES0	0x02
#define	FUSB_REG_SWITCHES1	0x03
#define	FUSB_REG_CONTROL0	0x06
#define	FUSB_REG_CONTROL1	0x07
#define	FUSB_REG_CONTROL2	0x08
#define	FUSB_REG_CONTROL3	0x09
#define	FUSB_REG_MASK1		0x0A
#define	FUSB_REG_POWER		0x0B
#define	FUSB_REG_RESET		0x0C
#define	FUSB_REG_MASKA		0x0E
#define	FUSB_REG_MASKB		0x0F
#define	FUSB_REG_CONTROL4	0x10
#define	FUSB_REG_STATUS0A	0x3C
#define	FUSB_REG_STATUS1A	0x3D
#define	FUSB_REG_INTERRUPTA	0x3E
#define	FUSB_REG_INTERRUPTB	0x3F
#define	FUSB_REG_STATUS0	0x40
#define	FUSB_REG_STATUS1	0x41
#define	FUSB_REG_INTERRUPT	0x42
#define	FUSB_REG_FIFO		0x43

/* SWITCHES0 bits */
#define	FUSB_SW0_PDWN1		0x01
#define	FUSB_SW0_PDWN2		0x02
#define	FUSB_SW0_MEAS_CC1	0x04
#define	FUSB_SW0_MEAS_CC2	0x08
#define	FUSB_SW0_VCONN_CC1	0x10
#define	FUSB_SW0_VCONN_CC2	0x20
#define	FUSB_SW0_PU_EN1		0x40
#define	FUSB_SW0_PU_EN2		0x80

/* SWITCHES1 bits */
#define	FUSB_SW1_TXCC1		0x01
#define	FUSB_SW1_TXCC2		0x02
#define	FUSB_SW1_AUTO_CRC	0x04
#define	FUSB_SW1_DATAROLE	0x10
#define	FUSB_SW1_SPECREV	0x60	/* bits 6:5 */
#define	FUSB_SW1_POWERROLE	0x80

/* CONTROL0 bits */
#define	FUSB_CTL0_HOST_CUR_DEF	0x04
#define	FUSB_CTL0_INT_MASK	0x20

/* CONTROL1 bits */
#define	FUSB_CTL1_RX_FLUSH	0x04

/* CONTROL2 bits */
#define	FUSB_CTL2_TOG_RD_ONLY	0x20
#define	FUSB_CTL2_MODE_DFP	0x06
#define	FUSB_CTL2_TOGGLE	0x01

/* CONTROL3 bits */
#define	FUSB_CTL3_AUTO_RETRY	0x01
#define	FUSB_CTL3_N_RETRIES	0x06	/* bits 2:1 — value 3 = 3 retries */
#define	FUSB_CTL3_SEND_HARDRST	0x40

/* CONTROL4 bits */
#define	FUSB_CTL4_TOG_USRC_EXIT	0x01

/* POWER bits */
#define	FUSB_POWER_ALL		0x0f

/* RESET bits */
#define	FUSB_RESET_PD_RESET	0x02
#define	FUSB_RESET_SW_RES	0x01

/* STATUS0 bits */
#define	FUSB_ST0_VBUSOK		0x80
#define	FUSB_ST0_BC_LVL_MASK	0x03

/* STATUS1 bits */
#define	FUSB_ST1_RX_EMPTY	0x20

/* STATUS1A togss field */
#define	FUSB_ST1A_TOGSS_MASK	0x38
#define	FUSB_ST1A_TOGSS_SHIFT	3

/* INTERRUPTA bits */
#define	FUSB_INTRA_HARDRST	0x01
#define	FUSB_INTRA_TXSENT	0x04
#define	FUSB_INTRA_HARDSENT	0x08
#define	FUSB_INTRA_RETRYFAIL	0x10
#define	FUSB_INTRA_TOGDONE	0x40

/* INTERRUPTB bits */
#define	FUSB_INTRB_GCRCSENT	0x01

/* Interrupt mask register initial values for PD operation */
#define	FUSB_MASK1_PD		0x75	/* unmask COLLISION, ALERT, VBUSOK */
#define	FUSB_MASKA_PD		0xa2	/* unmask TOGDONE,TXSENT,HARDSENT,RETRYFAIL,HARDRST */
#define	FUSB_MASKB_PD		0xfe	/* unmask GCRCSENT */

/* TX FIFO tokens */
#define	FUSB_TKN_TXON		0xa1
#define	FUSB_TKN_SYNC1		0x12
#define	FUSB_TKN_SYNC2		0x13
#define	FUSB_TKN_PACKSYM	0x80
#define	FUSB_TKN_JAMCRC		0xff
#define	FUSB_TKN_EOP		0x14
#define	FUSB_TKN_TXOFF		0xfe

/* PD control message types */
#define	PD_CMT_GOODCRC		1
#define	PD_CMT_ACCEPT		3
#define	PD_CMT_PS_RDY		6
#define	PD_CMT_GETSINKCAP	8
#define	PD_CMT_SOFTRESET	13

/* PD data message types */
#define	PD_DMT_SRCCAP		1
#define	PD_DMT_REQUEST		2
#define	PD_DMT_SINKCAP		4
#define	PD_DMT_VDM		15

/* VDM command IDs */
#define	VDM_DISC_ID		0x01
#define	VDM_DISC_SVIDS		0x02
#define	VDM_DISC_MODES		0x03
#define	VDM_ENTER_MODE		0x04
#define	VDM_DP_STATUS		0x10
#define	VDM_DP_CONFIG		0x11

/* VDM command types */
#define	VDM_TYPE_INIT		0
#define	VDM_TYPE_ACK		1
#define	VDM_TYPE_NACK		2

/* DP pin mode bits */
#define	DP_PIN_A		0x01
#define	DP_PIN_B		0x02
#define	DP_PIN_C		0x04
#define	DP_PIN_D		0x08
#define	DP_PIN_E		0x10
#define	DP_PIN_F		0x20
#define	DP_PIN_MF_MASK		(DP_PIN_B | DP_PIN_D | DP_PIN_F)
#define	DP_PIN_BR2_MASK		(DP_PIN_A | DP_PIN_B)
#define	DP_PIN_DP_MASK		(DP_PIN_C | DP_PIN_D | DP_PIN_E | DP_PIN_F)

/* PD header field extraction */
#define	PD_HDR_CNT(h)		(((h) >> 12) & 7)
#define	PD_HDR_TYPE(h)		((h) & 0xf)
#define	PD_HDR_ID(h)		(((h) >> 9) & 7)
#define	PD_IS_CTRL(h, t)	(PD_HDR_CNT(h) == 0 && PD_HDR_TYPE(h) == (t))
#define	PD_IS_DATA(h, t)	(PD_HDR_CNT(h) != 0 && PD_HDR_TYPE(h) == (t))

/* VDM header field extraction */
#define	VDM_GET_STRUCT(h)	(((h) >> 15) & 1)
#define	VDM_GET_CMD_TYPE(h)	(((h) >> 6) & 3)
#define	VDM_GET_CMD(h)		((h) & 0x1f)

/* DP status fields */
#define	DP_STATUS_HPD(s)	(((s) >> 7) & 1)
#define	DP_STATUS_MF_PREF(s)	(((s) >> 4) & 1)

/* DP capability pin field (receptacle vs plug) */
#define	PD_DP_PIN_CAPS(x)	((((x) >> 6) & 1) ? (((x) >> 16) & 0x3f) \
				 : (((x) >> 8) & 0x3f))
#define	PD_DP_SIGNAL_GEN2(x)	(((x) >> 3) & 1)

/* TX state values */
#define	FUSB_TX_IDLE	0
#define	FUSB_TX_BUSY	1
#define	FUSB_TX_FAILED	2
#define	FUSB_TX_SUCCESS	3

/* Event bits (stored in pending_events; match Linux event model) */
#define	FUSB_EVT_CC		0x01
#define	FUSB_EVT_RX		0x02
#define	FUSB_EVT_TX		0x04
#define	FUSB_EVT_REC_RESET	0x08
#define	FUSB_EVT_CONTINUE	0x20
#define	FUSB_EVT_TIMER_MUX	0x40
#define	FUSB_EVT_TIMER_STATE	0x80

/* Sender response timeout (ms) */
#define	T_SENDER_RESPONSE	30
#define	T_SRC_TRANSITION	30
#define	T_TYPEC_SEND_SRCCAP	100

/* Source capability PDO: 5V fixed, 900mA, USB comm capable */
#define	FUSB_SRC_PDO_5V900MA	((1u << 26) | (100u << 10) | 90u)

/* -----------------------------------------------------------------------
 * Connection-state enum (DFP / source path only)
 * ----------------------------------------------------------------------- */
enum fusb302_conn_state {
	FUSB_ST_DISABLED = 0,
	FUSB_ST_ERROR_RECOVERY,
	FUSB_ST_UNATTACHED,
	FUSB_ST_ATTACHED_SRC,
	FUSB_ST_SRC_STARTUP,
	FUSB_ST_SRC_SEND_CAPS,
	FUSB_ST_SRC_DISCOVERY,
	FUSB_ST_SRC_NEGOTIATE_CAP,
	FUSB_ST_SRC_CAP_RESPONSE,
	FUSB_ST_SRC_TRANSITION_SUPPLY,
	FUSB_ST_SRC_TRANSITION_DEFAULT,
	FUSB_ST_SRC_READY,
	FUSB_ST_SRC_GET_SINK_CAPS,
	FUSB_ST_SRC_SEND_HARDRST,
	FUSB_ST_SRC_SEND_SOFTRST,
	FUSB_ST_SRC_SOFTRST,
};

/* VDM state (signed: -1 = error) */
enum fusb302_vdm_state {
	VDM_DISC_ID_ST = 0,
	VDM_DISC_SVID_ST,
	VDM_DISC_MODES_ST,
	VDM_ENTER_MODE_ST,
	VDM_DP_STATUS_ST,
	VDM_DP_CONFIG_ST,
	VDM_NOTIFY_ST,
	VDM_READY_ST,
	VDM_ERR_ST = -1,
};

/* -----------------------------------------------------------------------
 * Software context
 * ----------------------------------------------------------------------- */
struct fusb302_softc {
	device_t		dev;
	struct mtx		mtx;
	uint8_t			addr;
	int			irq_rid;
	struct resource		*irq_res;
	void			*irq_cookie;
	struct task		irq_task;
	regulator_t		vbus_supply;
	bool			vbus_enabled;
	bool			initialized;

	/* CC detection legacy state */
	bool			state_valid;
	uint8_t			device_id;
	uint8_t			power;
	uint8_t			control2;
	uint8_t			status0;
	uint8_t			status1;
	uint8_t			status0a;
	uint8_t			status1a;
	enum fusb302_typec_orientation	orientation;
	enum fusb302_typec_role		role;
	bool			attached;

	/* PD / VDM state machine */
	enum fusb302_conn_state	conn_state;
	int			sub_state;
	uint32_t		work_continue;
	volatile uint32_t	pending_events;

	/* PD messaging */
	uint16_t		send_head;
	uint16_t		rec_head;
	uint32_t		send_load[7];
	uint32_t		rec_load[7];
	int			msg_id;
	int			tx_state;

	/* CC polarity: 0 = CC1, 1 = CC2 */
	int			cc_polarity;
	bool			vconn_enabled;
	bool			is_pd_support;

	/* VDM */
	enum fusb302_vdm_state	vdm_state;
	int			vdm_send_state;
	uint16_t		vdm_svid[12];
	int			vdm_svid_num;
	uint32_t		vdm_id;
	uint8_t			val_tmp;

	/* Notify / role */
	bool			notify_is_cc;
	bool			notify_is_pd;
	bool			notify_is_enter_mode;
	int			notify_power_role;	/* 1 = source */
	int			notify_data_role;	/* 1 = DFP */
	uint32_t		notify_dp_caps;
	uint32_t		notify_dp_status;
	int			notify_pin_support;
	int			notify_pin_def;

	/* Partner sink caps */
	uint32_t		partner_cap[7];

	/* Retry/retry counters */
	int			caps_counter;
	int			hardrst_count;
	int			pos_power;

	/* Timers */
	struct callout		timer_state;
	struct callout		timer_mux;

	/* DP Alt Mode result exported to rk_cdn_dp */
	struct rk3399_typec_dp_altmode_status	dp_altmode;
};

static struct ofw_compat_data compat_data[] = {
	{ "fcs,fusb302",	1 },
	{ NULL,			0 }
};
IICBUS_FDT_PNP_INFO(compat_data);

/* -----------------------------------------------------------------------
 * Forward declarations
 * ----------------------------------------------------------------------- */
static int	fusb302_probe(device_t dev);
static int	fusb302_attach(device_t dev);
static int	fusb302_detach(device_t dev);
static void	fusb302_irq_task(void *context, int pending);
static void	fusb302_intr(void *context);
static void	fusb302_timer_state_cb(void *arg);
static void	fusb302_timer_mux_cb(void *arg);
static int	fusb302_try_ofw_irq(struct fusb302_softc *sc);
static void	fusb302_add_sysctls(struct fusb302_softc *sc);

/* -----------------------------------------------------------------------
 * Low-level I2C helpers
 * ----------------------------------------------------------------------- */
static int
fusb302_read_reg(struct fusb302_softc *sc, uint8_t reg, uint8_t *val)
{
	return (iic2errno(iicdev_readfrom(sc->dev, reg, val, 1, IIC_WAIT)));
}

static int
fusb302_write_reg(struct fusb302_softc *sc, uint8_t reg, uint8_t val)
{
	return (iic2errno(iicdev_writeto(sc->dev, reg, &val, 1, IIC_WAIT)));
}

/* Read-modify-write a register: new = (old & ~mask) | (val & mask) */
static int
fusb302_update_reg(struct fusb302_softc *sc, uint8_t reg, uint8_t mask,
    uint8_t val)
{
	uint8_t cur;
	int error;

	error = fusb302_read_reg(sc, reg, &cur);
	if (error != 0)
		return (error);
	cur = (cur & ~mask) | (val & mask);
	return (fusb302_write_reg(sc, reg, cur));
}

/*
 * Read one PD message from the RX FIFO into sc->rec_head / sc->rec_load.
 * Automatically skips GoodCRC packets (hardware auto-reply artefacts).
 * Caller must hold sc->mtx.
 */
static int
fusb302_fifo_read_locked(struct fusb302_softc *sc)
{
	uint8_t buf[32];
	int len, tries, error;

	tries = 8;
	do {
		error = iicdev_readfrom(sc->dev, FUSB_REG_FIFO, buf, 3,
		    IIC_WAIT);
		if (error != 0)
			return (iic2errno(error));

		sc->rec_head = (uint16_t)buf[1] | ((uint16_t)buf[2] << 8);
		len = PD_HDR_CNT(sc->rec_head) << 2;

		error = iicdev_readfrom(sc->dev, FUSB_REG_FIFO, buf,
		    len + 4, IIC_WAIT);
		if (error != 0)
			return (iic2errno(error));

		if (!PD_IS_CTRL(sc->rec_head, PD_CMT_GOODCRC)) {
			memcpy(sc->rec_load, buf, len);
			return (0);
		}
	} while (--tries > 0);

	return (EIO);
}

/*
 * Write a PD message from sc->send_head / sc->send_load to the TX FIFO.
 * Caller must hold sc->mtx.
 */
static int
fusb302_fifo_write_locked(struct fusb302_softc *sc)
{
	uint8_t senddata[40];
	int pos, error;
	uint8_t len;

	pos = 0;
	senddata[pos++] = FUSB_TKN_SYNC1;
	senddata[pos++] = FUSB_TKN_SYNC1;
	senddata[pos++] = FUSB_TKN_SYNC1;
	senddata[pos++] = FUSB_TKN_SYNC2;

	len = (uint8_t)(PD_HDR_CNT(sc->send_head) << 2);
	senddata[pos++] = FUSB_TKN_PACKSYM | ((len + 2) & 0x1f);
	senddata[pos++] = (uint8_t)(sc->send_head & 0xff);
	senddata[pos++] = (uint8_t)(sc->send_head >> 8);

	memcpy(&senddata[pos], sc->send_load, len);
	pos += len;

	senddata[pos++] = FUSB_TKN_JAMCRC;
	senddata[pos++] = FUSB_TKN_EOP;
	senddata[pos++] = FUSB_TKN_TXOFF;
	senddata[pos++] = FUSB_TKN_TXON;

	error = iicdev_writeto(sc->dev, FUSB_REG_FIFO, senddata, pos,
	    IIC_WAIT);
	return (iic2errno(error));
}

static void
fusb302_flush_rx_fifo_locked(struct fusb302_softc *sc)
{
	(void)fusb302_write_reg(sc, FUSB_REG_CONTROL1, FUSB_CTL1_RX_FLUSH);
}

/* -----------------------------------------------------------------------
 * Hardware configuration
 * ----------------------------------------------------------------------- */

/*
 * Configure SWITCHES0/1 for PD communication on the active CC pin.
 * cc_polarity: 0 = CC1, 1 = CC2.  Always called as DFP (source role).
 */
static void
fusb302_set_polarity_locked(struct fusb302_softc *sc, int polarity)
{
	uint8_t sw0, sw1;

	sc->cc_polarity = polarity;

	sw0 = 0;
	if (sc->vconn_enabled) {
		/* VCONN on the opposite CC pin */
		sw0 |= polarity ? FUSB_SW0_VCONN_CC1 : FUSB_SW0_VCONN_CC2;
	}
	/* DFP: measure + pull-up on active CC */
	if (polarity == 0)
		sw0 |= FUSB_SW0_MEAS_CC1 | FUSB_SW0_PU_EN1;
	else
		sw0 |= FUSB_SW0_MEAS_CC2 | FUSB_SW0_PU_EN2;

	(void)fusb302_update_reg(sc, FUSB_REG_SWITCHES0,
	    FUSB_SW0_VCONN_CC1 | FUSB_SW0_VCONN_CC2 |
	    FUSB_SW0_MEAS_CC1 | FUSB_SW0_MEAS_CC2 |
	    FUSB_SW0_PU_EN1 | FUSB_SW0_PU_EN2, sw0);

	sw1 = polarity ? FUSB_SW1_TXCC2 : FUSB_SW1_TXCC1;
	(void)fusb302_update_reg(sc, FUSB_REG_SWITCHES1,
	    FUSB_SW1_TXCC1 | FUSB_SW1_TXCC2, sw1);
}

/* Enable or disable PD reception on the active CC pin. */
static void
fusb302_enable_rx_locked(struct fusb302_softc *sc, bool enable)
{
	uint8_t mask, val;

	mask = FUSB_SW0_MEAS_CC1 | FUSB_SW0_MEAS_CC2;
	val = 0;
	if (enable) {
		val = sc->cc_polarity ? FUSB_SW0_MEAS_CC2 : FUSB_SW0_MEAS_CC1;
		fusb302_flush_rx_fifo_locked(sc);
	}
	(void)fusb302_update_reg(sc, FUSB_REG_SWITCHES0, mask, val);
	(void)fusb302_update_reg(sc, FUSB_REG_SWITCHES1, FUSB_SW1_AUTO_CRC,
	    enable ? FUSB_SW1_AUTO_CRC : 0);
}

/* Write power/data role into SWITCHES1 so the firmware stamps them in TX. */
static void
fusb302_set_msg_header_locked(struct fusb302_softc *sc)
{
	uint8_t val;

	val = (uint8_t)((sc->notify_power_role << 7) |
	    (sc->notify_data_role << 4));
	(void)fusb302_update_reg(sc, FUSB_REG_SWITCHES1,
	    FUSB_SW1_POWERROLE | FUSB_SW1_DATAROLE, val);
	/* PD spec rev 1 */
	(void)fusb302_update_reg(sc, FUSB_REG_SWITCHES1,
	    FUSB_SW1_SPECREV, 2u << 5);
}

/* Issue a PD layer reset (does not affect CC toggle). */
static void
fusb302_pd_reset_locked(struct fusb302_softc *sc)
{
	(void)fusb302_write_reg(sc, FUSB_REG_RESET, FUSB_RESET_PD_RESET);
}

/* -----------------------------------------------------------------------
 * State machine helpers
 * ----------------------------------------------------------------------- */
static void
fusb302_set_state_locked(struct fusb302_softc *sc,
    enum fusb302_conn_state state)
{
	sc->conn_state = state;
	sc->sub_state = 0;
	sc->val_tmp = 0;
	sc->work_continue |= FUSB_EVT_CONTINUE;
}

static void
fusb302_reset_pd_params_locked(struct fusb302_softc *sc)
{
	sc->caps_counter = 0;
	sc->msg_id = 0;
	sc->vdm_state = VDM_DISC_ID_ST;
	sc->vdm_send_state = 0;
	sc->vdm_svid_num = 0;
	sc->vdm_id = 0;
	sc->val_tmp = 0;
	sc->pos_power = 0;
	memset(sc->vdm_svid, 0, sizeof(sc->vdm_svid));
}

/* Start or restart a callout timer; ms = 0 stops it. */
static void
fusb302_start_state_timer(struct fusb302_softc *sc, int ms)
{
	callout_reset_sbt(&sc->timer_state, SBT_1MS * ms, 0,
	    fusb302_timer_state_cb, sc, 0);
}

static void
fusb302_start_mux_timer(struct fusb302_softc *sc, int ms)
{
	callout_reset_sbt(&sc->timer_mux, SBT_1MS * ms, 0,
	    fusb302_timer_mux_cb, sc, 0);
}

/* -----------------------------------------------------------------------
 * PD message building
 * ----------------------------------------------------------------------- */

/*
 * Build a control or data message in sc->send_head / sc->send_load.
 * data_cnt: number of 32-bit data objects.
 */
static void
fusb302_build_header_locked(struct fusb302_softc *sc, int type, int data_cnt)
{
	sc->send_head = (uint16_t)(
	    ((sc->msg_id & 0x7) << 9) |
	    ((sc->notify_power_role & 0x1) << 8) |
	    (1 << 6) |				/* PD spec rev 1 */
	    ((sc->notify_data_role & 0x1) << 5) |
	    ((data_cnt & 0x7) << 12) |
	    (type & 0xf));
}

/* Build a source-capabilities data message (single 5V PDO). */
static void
fusb302_set_mesg_srccap_locked(struct fusb302_softc *sc)
{
	fusb302_build_header_locked(sc, PD_DMT_SRCCAP, 1);
	sc->send_load[0] = FUSB_SRC_PDO_5V900MA;
}

/* Build a control message (no data objects). */
static void
fusb302_set_mesg_ctrl_locked(struct fusb302_softc *sc, int cmd)
{
	fusb302_build_header_locked(sc, cmd, 0);
}

/* Build a VDM data message. */
static void
fusb302_set_vdm_mesg_locked(struct fusb302_softc *sc, int cmd, int type,
    int mode)
{
	fusb302_build_header_locked(sc, PD_DMT_VDM, 0);
	/* header: data count filled per-command below */

	sc->send_load[0] = (1u << 15) |	/* structured VDM */
	    (0u << 13) |			/* SVID reserved bits */
	    ((type & 3) << 6) |
	    (cmd & 0x1f);

	switch (cmd) {
	case VDM_DISC_ID:
	case VDM_DISC_SVIDS:
		sc->send_load[0] |= (0xff00u << 16);
		fusb302_build_header_locked(sc, PD_DMT_VDM, 1);
		break;
	case VDM_DISC_MODES:
		sc->send_load[0] |=
		    ((uint32_t)sc->vdm_svid[sc->val_tmp >> 1] << 16);
		fusb302_build_header_locked(sc, PD_DMT_VDM, 1);
		break;
	case VDM_ENTER_MODE:
		fusb302_build_header_locked(sc, PD_DMT_VDM, 1);
		sc->send_load[0] |= ((mode & 7) << 8) | (0xff01u << 16);
		break;
	case VDM_DP_STATUS:
		fusb302_build_header_locked(sc, PD_DMT_VDM, 2);
		sc->send_load[0] |= (1u << 8) | (0xff01u << 16);
		sc->send_load[1] = 5;		/* DFP connected, HPD high */
		break;
	case VDM_DP_CONFIG:
		fusb302_build_header_locked(sc, PD_DMT_VDM, 2);
		sc->send_load[0] |= (1u << 8) | (0xff01u << 16);
		/* pin_assignment_def set by process_vdm_msg when modes arrived */
		sc->send_load[1] =
		    ((uint32_t)sc->notify_pin_def << 8) | (1 << 2) | 2;
		device_printf(sc->dev, "VDM DP config: send_load[1]=0x%08x "
		    "pin_def=0x%x\n", sc->send_load[1], sc->notify_pin_def);
		break;
	default:
		break;
	}
}

/* Select pin assignment from DP caps + status (mirrors Linux algorithm). */
static int
fusb302_dp_pin_assignment(uint32_t caps, uint32_t status)
{
	uint32_t pin_caps;

	pin_caps = PD_DP_PIN_CAPS(caps);
	if (!DP_STATUS_MF_PREF(status))
		pin_caps &= ~DP_PIN_MF_MASK;
	if (PD_DP_SIGNAL_GEN2(caps))
		pin_caps &= ~DP_PIN_DP_MASK;
	else
		pin_caps &= ~DP_PIN_BR2_MASK;
	if (pin_caps & (DP_PIN_C | DP_PIN_D))
		pin_caps &= ~(DP_PIN_E | DP_PIN_F);
	if (pin_caps == 0)
		return (0);
	return (1 << (31 - __builtin_clz(pin_caps)));
}

/* -----------------------------------------------------------------------
 * TX state machine (mirrors Linux policy_send_data)
 * ----------------------------------------------------------------------- */
static int
fusb302_policy_send_data_locked(struct fusb302_softc *sc)
{
	int error;

	switch (sc->tx_state) {
	case FUSB_TX_IDLE:
		error = fusb302_fifo_write_locked(sc);
		if (error != 0) {
			device_printf(sc->dev, "FIFO write error: %d\n", error);
			sc->tx_state = FUSB_TX_FAILED;
		} else {
			sc->tx_state = FUSB_TX_BUSY;
		}
		break;
	default:
		/* wait for hardware TXSENT or RETRYFAIL alert */
		break;
	}
	return (sc->tx_state);
}

/* -----------------------------------------------------------------------
 * VDM receive processing
 * ----------------------------------------------------------------------- */
static void
fusb302_process_vdm_msg_locked(struct fusb302_softc *sc)
{
	uint32_t hdr;
	int i;
	uint32_t tmp;

	hdr = sc->rec_load[0];
	if (!VDM_GET_STRUCT(hdr)) {
		device_printf(sc->dev, "VDM: unstructured, ignored\n");
		return;
	}

	switch (VDM_GET_CMD_TYPE(hdr)) {
	case VDM_TYPE_INIT:
		if (VDM_GET_CMD(hdr) == 0x06 /* ATTENTION */) {
			sc->notify_dp_status = sc->rec_load[1] & 0xff;
			device_printf(sc->dev, "VDM attention dp_status=0x%x\n",
			    sc->notify_dp_status);
		}
		break;

	case VDM_TYPE_ACK:
		switch (VDM_GET_CMD(hdr)) {
		case VDM_DISC_ID:
			sc->vdm_id = sc->rec_load[1];
			break;
		case VDM_DISC_SVIDS:
			for (i = 0; i < 6; i++) {
				tmp = (sc->rec_load[i + 1] >> 16) & 0xffff;
				if (tmp)
					sc->vdm_svid[sc->vdm_svid_num++] = tmp;
				else
					break;
				tmp = sc->rec_load[i + 1] & 0xffff;
				if (tmp)
					sc->vdm_svid[sc->vdm_svid_num++] = tmp;
				else
					break;
			}
			break;
		case VDM_DISC_MODES:
			if (PD_HDR_CNT(sc->rec_head) > 1) {
				tmp = sc->rec_load[1];
				if (((tmp >> 8) & 0x3f) || ((tmp >> 16) & 0x3f)) {
					sc->notify_dp_caps = tmp;
					sc->notify_pin_def = 0;
					sc->notify_pin_support = PD_DP_PIN_CAPS(tmp);
					device_printf(sc->dev,
					    "VDM DP caps=0x%08x pin_support=0x%x\n",
					    tmp, sc->notify_pin_support);
				}
				sc->val_tmp |= 1;
			}
			break;
		case VDM_ENTER_MODE:
			sc->val_tmp = 1;
			break;
		case VDM_DP_STATUS:
			sc->notify_dp_status = sc->rec_load[1] & 0xff;
			device_printf(sc->dev, "VDM DP status=0x%08x\n",
			    sc->rec_load[1]);
			sc->val_tmp = 1;
			break;
		case VDM_DP_CONFIG:
			sc->val_tmp = 1;
			sc->notify_is_enter_mode = true;
			device_printf(sc->dev,
			    "VDM DP config OK, pin_assignment=0x%x\n",
			    sc->notify_pin_def);
			break;
		default:
			break;
		}
		break;

	case VDM_TYPE_NACK:
		device_printf(sc->dev, "VDM NACK for cmd=0x%x\n",
		    VDM_GET_CMD(hdr));
		sc->vdm_state = VDM_ERR_ST;
		break;
	}
}

/* -----------------------------------------------------------------------
 * VDM send sub-functions (mirror Linux vdm_send_* functions)
 * Each returns: 0 = success, -EINPROGRESS = still waiting, <0 = error
 * ----------------------------------------------------------------------- */

/* Macro mirrors Linux AUTO_VDM_HANDLE */
#define	VDM_HANDLE(fn, sc, evt, cond)			\
do {							\
	(cond) = (fn)((sc), (evt));			\
	if ((cond) == 0) {				\
		(sc)->vdm_state++;			\
		(sc)->work_continue |= FUSB_EVT_CONTINUE;\
	} else if ((cond) != -EINPROGRESS) {		\
		(sc)->vdm_state = VDM_ERR_ST;		\
	}						\
} while (0)

static int
fusb302_vdm_send_discid_locked(struct fusb302_softc *sc, uint32_t evt)
{
	int tmp;

	switch (sc->vdm_send_state) {
	case 0:
		fusb302_set_vdm_mesg_locked(sc, VDM_DISC_ID, VDM_TYPE_INIT, 0);
		sc->vdm_id = 0;
		sc->tx_state = FUSB_TX_IDLE;
		sc->vdm_send_state++;
		/* FALLTHROUGH */
	case 1:
		tmp = fusb302_policy_send_data_locked(sc);
		if (tmp == FUSB_TX_SUCCESS) {
			sc->vdm_send_state++;
			fusb302_start_state_timer(sc, T_SENDER_RESPONSE);
		} else if (tmp == FUSB_TX_FAILED) {
			device_printf(sc->dev, "VDM DISC_ID TX failed\n");
			return (-EIO);
		}
		if (sc->vdm_send_state != 2)
			break;
		/* FALLTHROUGH */
	default:
		if (sc->vdm_id) {
			sc->vdm_send_state = 0;
			return (0);
		} else if (evt & FUSB_EVT_TIMER_STATE) {
			device_printf(sc->dev, "VDM DISC_ID timeout\n");
			sc->work_continue |= FUSB_EVT_CONTINUE;
			return (-ETIMEDOUT);
		}
		break;
	}
	return (-EINPROGRESS);
}

static int
fusb302_vdm_send_discsvid_locked(struct fusb302_softc *sc, uint32_t evt)
{
	int tmp;

	switch (sc->vdm_send_state) {
	case 0:
		fusb302_set_vdm_mesg_locked(sc, VDM_DISC_SVIDS, VDM_TYPE_INIT,
		    0);
		sc->vdm_svid_num = 0;
		memset(sc->vdm_svid, 0, sizeof(sc->vdm_svid));
		sc->tx_state = FUSB_TX_IDLE;
		sc->vdm_send_state++;
		/* FALLTHROUGH */
	case 1:
		tmp = fusb302_policy_send_data_locked(sc);
		if (tmp == FUSB_TX_SUCCESS) {
			sc->vdm_send_state++;
			fusb302_start_state_timer(sc, T_SENDER_RESPONSE);
		} else if (tmp == FUSB_TX_FAILED) {
			device_printf(sc->dev, "VDM DISC_SVIDS TX failed\n");
			return (-EIO);
		}
		if (sc->vdm_send_state != 2)
			break;
		/* FALLTHROUGH */
	default:
		if (sc->vdm_svid_num) {
			sc->vdm_send_state = 0;
			return (0);
		} else if (evt & FUSB_EVT_TIMER_STATE) {
			device_printf(sc->dev, "VDM DISC_SVIDS timeout\n");
			sc->work_continue |= FUSB_EVT_CONTINUE;
			return (-ETIMEDOUT);
		}
		break;
	}
	return (-EINPROGRESS);
}

static int
fusb302_vdm_send_discmodes_locked(struct fusb302_softc *sc, uint32_t evt)
{
	int tmp;

	if ((sc->val_tmp >> 1) != sc->vdm_svid_num) {
		switch (sc->vdm_send_state) {
		case 0:
			fusb302_set_vdm_mesg_locked(sc, VDM_DISC_MODES,
			    VDM_TYPE_INIT, 0);
			sc->tx_state = FUSB_TX_IDLE;
			sc->vdm_send_state++;
			/* FALLTHROUGH */
		case 1:
			tmp = fusb302_policy_send_data_locked(sc);
			if (tmp == FUSB_TX_SUCCESS) {
				sc->vdm_send_state++;
				fusb302_start_state_timer(sc, T_SENDER_RESPONSE);
			} else if (tmp == FUSB_TX_FAILED) {
				device_printf(sc->dev,
				    "VDM DISC_MODES TX failed\n");
				return (-EIO);
			}
			if (sc->vdm_send_state != 2)
				break;
			/* FALLTHROUGH */
		default:
			if (sc->val_tmp & 1) {
				sc->val_tmp &= 0xfe;
				sc->val_tmp += 2;
				sc->vdm_send_state = 0;
				sc->work_continue |= FUSB_EVT_CONTINUE;
			} else if (evt & FUSB_EVT_TIMER_STATE) {
				device_printf(sc->dev,
				    "VDM DISC_MODES timeout\n");
				sc->work_continue |= FUSB_EVT_CONTINUE;
				return (-ETIMEDOUT);
			}
			break;
		}
	} else {
		sc->val_tmp = 0;
		return (0);
	}
	return (-EINPROGRESS);
}

static int
fusb302_vdm_send_entermode_locked(struct fusb302_softc *sc, uint32_t evt)
{
	int tmp;

	switch (sc->vdm_send_state) {
	case 0:
		fusb302_set_vdm_mesg_locked(sc, VDM_ENTER_MODE, VDM_TYPE_INIT,
		    1);
		sc->notify_is_enter_mode = false;
		sc->tx_state = FUSB_TX_IDLE;
		sc->vdm_send_state++;
		/* FALLTHROUGH */
	case 1:
		tmp = fusb302_policy_send_data_locked(sc);
		if (tmp == FUSB_TX_SUCCESS) {
			sc->vdm_send_state++;
			fusb302_start_state_timer(sc, T_SENDER_RESPONSE);
		} else if (tmp == FUSB_TX_FAILED) {
			device_printf(sc->dev, "VDM ENTER_MODE TX failed\n");
			return (-EIO);
		}
		if (sc->vdm_send_state != 2)
			break;
		/* FALLTHROUGH */
	default:
		if (sc->val_tmp) {
			sc->val_tmp = 0;
			sc->vdm_send_state = 0;
			return (0);
		} else if (evt & FUSB_EVT_TIMER_STATE) {
			device_printf(sc->dev, "VDM ENTER_MODE timeout\n");
			sc->work_continue |= FUSB_EVT_CONTINUE;
			return (-ETIMEDOUT);
		}
		break;
	}
	return (-EINPROGRESS);
}

static int
fusb302_vdm_send_dpstatus_locked(struct fusb302_softc *sc, uint32_t evt)
{
	int tmp;

	switch (sc->vdm_send_state) {
	case 0:
		fusb302_set_vdm_mesg_locked(sc, VDM_DP_STATUS, VDM_TYPE_INIT,
		    1);
		sc->tx_state = FUSB_TX_IDLE;
		sc->vdm_send_state++;
		/* FALLTHROUGH */
	case 1:
		tmp = fusb302_policy_send_data_locked(sc);
		if (tmp == FUSB_TX_SUCCESS) {
			sc->vdm_send_state++;
			fusb302_start_state_timer(sc, T_SENDER_RESPONSE);
		} else if (tmp == FUSB_TX_FAILED) {
			device_printf(sc->dev, "VDM DP_STATUS TX failed\n");
			return (-EIO);
		}
		if (sc->vdm_send_state != 2)
			break;
		/* FALLTHROUGH */
	default:
		if (sc->val_tmp) {
			sc->val_tmp = 0;
			sc->vdm_send_state = 0;
			return (0);
		} else if (evt & FUSB_EVT_TIMER_STATE) {
			device_printf(sc->dev, "VDM DP_STATUS timeout\n");
			sc->work_continue |= FUSB_EVT_CONTINUE;
			return (-ETIMEDOUT);
		}
		break;
	}
	return (-EINPROGRESS);
}

static int
fusb302_vdm_send_dpconfig_locked(struct fusb302_softc *sc, uint32_t evt)
{
	int tmp;

	switch (sc->vdm_send_state) {
	case 0:
		/* Compute pin assignment from discovered DP caps */
		sc->notify_pin_def = fusb302_dp_pin_assignment(
		    sc->notify_dp_caps, sc->notify_dp_status);
		fusb302_set_vdm_mesg_locked(sc, VDM_DP_CONFIG, VDM_TYPE_INIT,
		    0);
		sc->tx_state = FUSB_TX_IDLE;
		sc->vdm_send_state++;
		/* FALLTHROUGH */
	case 1:
		tmp = fusb302_policy_send_data_locked(sc);
		if (tmp == FUSB_TX_SUCCESS) {
			sc->vdm_send_state++;
			fusb302_start_state_timer(sc, T_SENDER_RESPONSE);
		} else if (tmp == FUSB_TX_FAILED) {
			device_printf(sc->dev, "VDM DP_CONFIG TX failed\n");
			return (-EIO);
		}
		if (sc->vdm_send_state != 2)
			break;
		/* FALLTHROUGH */
	default:
		if (sc->val_tmp) {
			sc->val_tmp = 0;
			sc->vdm_send_state = 0;
			return (0);
		} else if (evt & FUSB_EVT_TIMER_STATE) {
			device_printf(sc->dev, "VDM DP_CONFIG timeout\n");
			sc->work_continue |= FUSB_EVT_CONTINUE;
			return (-ETIMEDOUT);
		}
		break;
	}
	return (-EINPROGRESS);
}

/* -----------------------------------------------------------------------
 * Notify CDN-DP of DP Alt Mode state
 * ----------------------------------------------------------------------- */
static void
fusb302_notify_dp_locked(struct fusb302_softc *sc)
{
	bool hpd;

	hpd = DP_STATUS_HPD(sc->notify_dp_status) != 0;

	sc->dp_altmode.valid = true;
	sc->dp_altmode.dp_ready = sc->notify_is_enter_mode && hpd;
	sc->dp_altmode.pin_assignment = sc->notify_pin_def;
	sc->dp_altmode.dp_status = sc->notify_dp_status;
	/* usb_ss: 0 = DP-only (4-lane), 1 = USB3+DP (2-lane) */
	sc->dp_altmode.usb_ss = (sc->notify_pin_def & DP_PIN_MF_MASK) ? 1 : 0;

	device_printf(sc->dev,
	    "DP Alt Mode: dp_ready=%d pin=0x%x usb_ss=%d dp_status=0x%x\n",
	    sc->dp_altmode.dp_ready, sc->dp_altmode.pin_assignment,
	    sc->dp_altmode.usb_ss, sc->dp_altmode.dp_status);
}

/* -----------------------------------------------------------------------
 * VDM state machine (mirrors Linux auto_vdm_machine)
 * ----------------------------------------------------------------------- */
static void
fusb302_auto_vdm_machine_locked(struct fusb302_softc *sc, uint32_t evt)
{
	int cond;

	switch (sc->vdm_state) {
	case VDM_DISC_ID_ST:
		VDM_HANDLE(fusb302_vdm_send_discid_locked, sc, evt, cond);
		break;
	case VDM_DISC_SVID_ST:
		VDM_HANDLE(fusb302_vdm_send_discsvid_locked, sc, evt, cond);
		break;
	case VDM_DISC_MODES_ST:
		VDM_HANDLE(fusb302_vdm_send_discmodes_locked, sc, evt, cond);
		break;
	case VDM_ENTER_MODE_ST:
		VDM_HANDLE(fusb302_vdm_send_entermode_locked, sc, evt, cond);
		break;
	case VDM_DP_STATUS_ST:
		VDM_HANDLE(fusb302_vdm_send_dpstatus_locked, sc, evt, cond);
		break;
	case VDM_DP_CONFIG_ST:
		VDM_HANDLE(fusb302_vdm_send_dpconfig_locked, sc, evt, cond);
		break;
	case VDM_NOTIFY_ST:
		fusb302_notify_dp_locked(sc);
		sc->vdm_state = VDM_READY_ST;
		break;
	default:
		break;
	}
}

/* -----------------------------------------------------------------------
 * PD state functions (DFP / source path)
 * ----------------------------------------------------------------------- */
static void
fusb302_state_attached_source_locked(struct fusb302_softc *sc,
    uint32_t evt __unused)
{
	int polarity;

	/* Derive polarity from STATUS1A TOGSS */
	switch ((sc->status1a & FUSB_ST1A_TOGSS_MASK) >> FUSB_ST1A_TOGSS_SHIFT)
	{
	case 2:	/* src-cc2 */
		polarity = 1;
		break;
	default: /* src-cc1 */
		polarity = 0;
		break;
	}

	sc->notify_is_cc = true;
	sc->notify_power_role = 1;	/* source */
	sc->notify_data_role = 1;	/* DFP */
	sc->hardrst_count = 0;

	fusb302_set_polarity_locked(sc, polarity);
	fusb302_set_state_locked(sc, FUSB_ST_SRC_STARTUP);

	device_printf(sc->dev, "attached as DFP on CC%d\n", polarity + 1);
}

static void
fusb302_state_src_startup_locked(struct fusb302_softc *sc,
    uint32_t evt __unused)
{
	sc->notify_is_pd = false;
	fusb302_reset_pd_params_locked(sc);
	memset(sc->partner_cap, 0, sizeof(sc->partner_cap));

	fusb302_pd_reset_locked(sc);
	fusb302_set_msg_header_locked(sc);
	fusb302_set_polarity_locked(sc, sc->cc_polarity);
	fusb302_enable_rx_locked(sc, true);

	fusb302_set_state_locked(sc, FUSB_ST_SRC_SEND_CAPS);
}

static void
fusb302_state_src_send_caps_locked(struct fusb302_softc *sc, uint32_t evt)
{
	int tmp;

	switch (sc->sub_state) {
	case 0:
		fusb302_set_mesg_srccap_locked(sc);
		sc->tx_state = FUSB_TX_IDLE;
		sc->sub_state++;
		/* FALLTHROUGH */
	case 1:
		tmp = fusb302_policy_send_data_locked(sc);
		if (tmp == FUSB_TX_SUCCESS) {
			sc->hardrst_count = 0;
			sc->caps_counter = 0;
			sc->is_pd_support = true;
			fusb302_start_state_timer(sc, T_SENDER_RESPONSE);
			sc->sub_state++;
		} else if (tmp == FUSB_TX_FAILED) {
			sc->caps_counter++;
			if (sc->caps_counter >= 50)
				fusb302_set_state_locked(sc, FUSB_ST_DISABLED);
			else {
				fusb302_start_state_timer(sc,
				    T_TYPEC_SEND_SRCCAP);
				fusb302_set_state_locked(sc, FUSB_ST_SRC_SEND_CAPS);
			}
			return;
		}
		if (sc->sub_state != 2)
			break;
		/* FALLTHROUGH */
	default:
		if (evt & FUSB_EVT_RX) {
			if (PD_IS_DATA(sc->rec_head, PD_DMT_REQUEST)) {
				fusb302_set_state_locked(sc,
				    FUSB_ST_SRC_NEGOTIATE_CAP);
			} else {
				fusb302_set_state_locked(sc,
				    FUSB_ST_SRC_SEND_SOFTRST);
			}
		} else if (evt & FUSB_EVT_TIMER_STATE) {
			if (sc->hardrst_count <= 0)
				fusb302_set_state_locked(sc,
				    FUSB_ST_SRC_SEND_HARDRST);
			else
				fusb302_set_state_locked(sc, FUSB_ST_DISABLED);
		}
		break;
	}
}

static void
fusb302_state_src_negotiate_cap_locked(struct fusb302_softc *sc,
    uint32_t evt __unused)
{
	/* We only advertise one PDO; any valid REQUEST position is OK */
	fusb302_set_state_locked(sc, FUSB_ST_SRC_TRANSITION_SUPPLY);
}

static void
fusb302_state_src_transition_supply_locked(struct fusb302_softc *sc,
    uint32_t evt)
{
	int tmp;

	switch (sc->sub_state) {
	case 0:
		fusb302_set_mesg_ctrl_locked(sc, PD_CMT_ACCEPT);
		sc->tx_state = FUSB_TX_IDLE;
		sc->sub_state++;
		/* FALLTHROUGH */
	case 1:
		tmp = fusb302_policy_send_data_locked(sc);
		if (tmp == FUSB_TX_SUCCESS) {
			fusb302_start_state_timer(sc, T_SRC_TRANSITION);
			sc->sub_state++;
		} else if (tmp == FUSB_TX_FAILED) {
			fusb302_set_state_locked(sc, FUSB_ST_SRC_SEND_SOFTRST);
		}
		break;
	case 2:
		if (evt & FUSB_EVT_TIMER_STATE) {
			sc->notify_is_pd = true;
			fusb302_set_mesg_ctrl_locked(sc, PD_CMT_PS_RDY);
			sc->tx_state = FUSB_TX_IDLE;
			sc->sub_state++;
			sc->work_continue |= FUSB_EVT_CONTINUE;
		}
		break;
	default:
		tmp = fusb302_policy_send_data_locked(sc);
		if (tmp == FUSB_TX_SUCCESS) {
			device_printf(sc->dev, "PD connected as DFP (5V)\n");
			fusb302_set_state_locked(sc, FUSB_ST_SRC_READY);
		} else if (tmp == FUSB_TX_FAILED) {
			fusb302_set_state_locked(sc, FUSB_ST_SRC_SEND_SOFTRST);
		}
		break;
	}
}

static void
fusb302_state_src_ready_locked(struct fusb302_softc *sc, uint32_t evt)
{
	bool vdm_active;

	vdm_active = (sc->notify_data_role &&
	    sc->vdm_state < VDM_READY_ST);

	if (evt & FUSB_EVT_RX) {
		if (PD_IS_DATA(sc->rec_head, PD_DMT_VDM)) {
			fusb302_process_vdm_msg_locked(sc);
			sc->work_continue |= FUSB_EVT_CONTINUE;
			/* stop state timer so it doesn't race VDM timeout */
			callout_stop(&sc->timer_state);
		} else if (!vdm_active) {
			/* ignore swap requests for now */
		}
	}

	if (!sc->partner_cap[0])
		fusb302_set_state_locked(sc, FUSB_ST_SRC_GET_SINK_CAPS);
	else if (vdm_active)
		fusb302_auto_vdm_machine_locked(sc, evt);
}

static void
fusb302_state_src_get_sink_caps_locked(struct fusb302_softc *sc, uint32_t evt)
{
	int tmp;
	uint32_t i;

	switch (sc->sub_state) {
	case 0:
		fusb302_set_mesg_ctrl_locked(sc, PD_CMT_GETSINKCAP);
		sc->tx_state = FUSB_TX_IDLE;
		sc->sub_state++;
		/* FALLTHROUGH */
	case 1:
		tmp = fusb302_policy_send_data_locked(sc);
		if (tmp == FUSB_TX_SUCCESS) {
			fusb302_start_state_timer(sc, T_SENDER_RESPONSE);
			sc->sub_state++;
		} else if (tmp == FUSB_TX_FAILED) {
			fusb302_set_state_locked(sc, FUSB_ST_SRC_SEND_SOFTRST);
		}
		if (sc->sub_state != 2)
			break;
		/* FALLTHROUGH */
	default:
		if (evt & FUSB_EVT_RX) {
			if (PD_IS_DATA(sc->rec_head, PD_DMT_SINKCAP)) {
				for (i = 0; i < PD_HDR_CNT(sc->rec_head); i++)
					sc->partner_cap[i] = sc->rec_load[i];
			} else {
				sc->partner_cap[0] = 0xffffffff;
			}
			fusb302_set_state_locked(sc, FUSB_ST_SRC_READY);
		} else if (evt & FUSB_EVT_TIMER_STATE) {
			device_printf(sc->dev, "GET_SINK_CAP timeout\n");
			sc->partner_cap[0] = 0xffffffff;
			fusb302_set_state_locked(sc, FUSB_ST_SRC_READY);
		}
		break;
	}
}

static void
fusb302_state_src_send_hardrst_locked(struct fusb302_softc *sc,
    uint32_t evt)
{
	int error;

	switch (sc->sub_state) {
	case 0:
		error = fusb302_update_reg(sc, FUSB_REG_CONTROL3,
		    FUSB_CTL3_SEND_HARDRST, FUSB_CTL3_SEND_HARDRST);
		if (error != 0)
			device_printf(sc->dev, "hard reset send failed\n");
		sc->sub_state++;
		fusb302_start_state_timer(sc, 30);
		break;
	default:
		if (evt & FUSB_EVT_TIMER_STATE) {
			sc->hardrst_count++;
			fusb302_set_state_locked(sc,
			    FUSB_ST_SRC_TRANSITION_DEFAULT);
		}
		break;
	}
}

static void
fusb302_state_src_transition_default_locked(struct fusb302_softc *sc,
    uint32_t evt)
{
	switch (sc->sub_state) {
	case 0:
		sc->notify_is_pd = false;
		fusb302_start_state_timer(sc, 830);	/* T_SRC_RECOVER */
		sc->sub_state++;
		break;
	default:
		if (evt & FUSB_EVT_TIMER_STATE) {
			fusb302_set_state_locked(sc, FUSB_ST_SRC_STARTUP);
		}
		break;
	}
}

static void
fusb302_state_src_softrst_locked(struct fusb302_softc *sc)
{
	int tmp;

	switch (sc->sub_state) {
	case 0:
		fusb302_set_mesg_ctrl_locked(sc, PD_CMT_ACCEPT);
		sc->tx_state = FUSB_TX_IDLE;
		sc->sub_state++;
		/* FALLTHROUGH */
	default:
		tmp = fusb302_policy_send_data_locked(sc);
		if (tmp == FUSB_TX_SUCCESS) {
			fusb302_reset_pd_params_locked(sc);
			fusb302_set_state_locked(sc, FUSB_ST_SRC_SEND_CAPS);
		} else if (tmp == FUSB_TX_FAILED) {
			fusb302_set_state_locked(sc, FUSB_ST_SRC_SEND_HARDRST);
		}
		break;
	}
}

static void
fusb302_state_src_send_softrst_locked(struct fusb302_softc *sc, uint32_t evt)
{
	int tmp;

	switch (sc->sub_state) {
	case 0:
		fusb302_set_mesg_ctrl_locked(sc, PD_CMT_SOFTRESET);
		sc->tx_state = FUSB_TX_IDLE;
		sc->sub_state++;
		/* FALLTHROUGH */
	case 1:
		tmp = fusb302_policy_send_data_locked(sc);
		if (tmp == FUSB_TX_SUCCESS) {
			fusb302_start_state_timer(sc, T_SENDER_RESPONSE);
			sc->sub_state++;
		} else if (tmp == FUSB_TX_FAILED) {
			fusb302_set_state_locked(sc, FUSB_ST_SRC_SEND_HARDRST);
		}
		if (sc->sub_state != 2)
			break;
		/* FALLTHROUGH */
	default:
		if (evt & FUSB_EVT_RX) {
			if (PD_IS_CTRL(sc->rec_head, PD_CMT_ACCEPT)) {
				fusb302_reset_pd_params_locked(sc);
				fusb302_set_state_locked(sc,
				    FUSB_ST_SRC_SEND_CAPS);
			}
		} else if (evt & FUSB_EVT_TIMER_STATE) {
			fusb302_set_state_locked(sc, FUSB_ST_SRC_SEND_HARDRST);
		}
		break;
	}
}

/* Detach: re-init CC toggle and reset everything */
static void
fusb302_set_state_unattached_locked(struct fusb302_softc *sc)
{
	device_printf(sc->dev, "detached, restarting CC detection\n");

	callout_stop(&sc->timer_state);
	callout_stop(&sc->timer_mux);

	sc->notify_is_cc = false;
	sc->notify_is_pd = false;
	sc->notify_is_enter_mode = false;
	sc->is_pd_support = false;
	memset(&sc->dp_altmode, 0, sizeof(sc->dp_altmode));
	memset(sc->partner_cap, 0, sizeof(sc->partner_cap));

	(void)fusb302_write_reg(sc, FUSB_REG_RESET, FUSB_RESET_SW_RES);
	DELAY(1000);
	(void)fusb302_write_reg(sc, FUSB_REG_POWER, FUSB_POWER_ALL);
	(void)fusb302_write_reg(sc, FUSB_REG_CONTROL0, FUSB_CTL0_HOST_CUR_DEF);
	(void)fusb302_write_reg(sc, FUSB_REG_MASK1, FUSB_MASK1_PD);
	(void)fusb302_write_reg(sc, FUSB_REG_MASKA, FUSB_MASKA_PD);
	(void)fusb302_write_reg(sc, FUSB_REG_MASKB, FUSB_MASKB_PD);

	/* Re-enable toggle */
	(void)fusb302_write_reg(sc, FUSB_REG_CONTROL2,
	    FUSB_CTL2_TOG_RD_ONLY | FUSB_CTL2_MODE_DFP | FUSB_CTL2_TOGGLE);

	sc->conn_state = FUSB_ST_UNATTACHED;
	sc->work_continue = 0;
}

/* -----------------------------------------------------------------------
 * Hardware alert → event bits
 * ----------------------------------------------------------------------- */
static void
fusb302_tcpc_alert_locked(struct fusb302_softc *sc, uint32_t *evtp)
{
	uint8_t intr, intra, intrb, status1a;
	int error;

	error = fusb302_read_reg(sc, FUSB_REG_INTERRUPT, &intr);
	if (error != 0)
		return;
	error = fusb302_read_reg(sc, FUSB_REG_INTERRUPTA, &intra);
	if (error != 0)
		return;
	error = fusb302_read_reg(sc, FUSB_REG_INTERRUPTB, &intrb);
	if (error != 0)
		return;

	/* Refresh legacy status for sysctl */
	fusb302_read_reg(sc, FUSB_REG_STATUS0, &sc->status0);
	fusb302_read_reg(sc, FUSB_REG_STATUS1, &sc->status1);
	fusb302_read_reg(sc, FUSB_REG_STATUS0A, &sc->status0a);
	fusb302_read_reg(sc, FUSB_REG_STATUS1A, &sc->status1a);

	if (intra & FUSB_INTRA_TOGDONE) {
		*evtp |= FUSB_EVT_CC;
		status1a = sc->status1a;
		/* Stop toggle; PD code takes manual control of CC */
		(void)fusb302_update_reg(sc, FUSB_REG_CONTROL2,
		    FUSB_CTL2_TOGGLE, 0);
		device_printf(sc->dev, "TOGDONE status1a=0x%02x\n", status1a);
	}

	if ((intr & 0x80) /* VBUSOK */ && sc->notify_is_cc)
		*evtp |= FUSB_EVT_CC;

	if (intra & FUSB_INTRA_TXSENT) {
		*evtp |= FUSB_EVT_TX;
		sc->tx_state = FUSB_TX_SUCCESS;
	}

	if (intra & FUSB_INTRA_RETRYFAIL) {
		*evtp |= FUSB_EVT_TX;
		sc->tx_state = FUSB_TX_FAILED;
	}

	if (intrb & FUSB_INTRB_GCRCSENT)
		*evtp |= FUSB_EVT_RX;

	if (intra & FUSB_INTRA_HARDRST) {
		device_printf(sc->dev, "hard reset received\n");
		fusb302_pd_reset_locked(sc);
		sc->msg_id = 0;
		sc->vdm_state = VDM_DISC_ID_ST;
		fusb302_set_state_locked(sc, FUSB_ST_SRC_TRANSITION_DEFAULT);
		*evtp |= FUSB_EVT_REC_RESET;
	}

	if (intra & FUSB_INTRA_HARDSENT) {
		/* Hard reset transmitted; transition to default state */
		fusb302_pd_reset_locked(sc);
		fusb302_set_state_locked(sc, FUSB_ST_SRC_TRANSITION_DEFAULT);
	}
}

/* -----------------------------------------------------------------------
 * State machine dispatch
 * ----------------------------------------------------------------------- */
static void
fusb302_run_state_locked(struct fusb302_softc *sc, uint32_t evt)
{
	switch (sc->conn_state) {
	case FUSB_ST_DISABLED:
		break;
	case FUSB_ST_ERROR_RECOVERY:
		fusb302_set_state_unattached_locked(sc);
		break;
	case FUSB_ST_UNATTACHED:
		/* Waiting for TOGDONE (EVENT_CC) */
		if (evt & FUSB_EVT_CC) {
			/* TOGSS: 1=src-cc1, 2=src-cc2 */
			uint8_t ts = (sc->status1a & FUSB_ST1A_TOGSS_MASK)
			    >> FUSB_ST1A_TOGSS_SHIFT;
			if (ts == 1 || ts == 2)
				fusb302_set_state_locked(sc, FUSB_ST_ATTACHED_SRC);
		}
		break;
	case FUSB_ST_ATTACHED_SRC:
		fusb302_state_attached_source_locked(sc, evt);
		break;
	case FUSB_ST_SRC_STARTUP:
		fusb302_state_src_startup_locked(sc, evt);
		break;
	case FUSB_ST_SRC_SEND_CAPS:
		fusb302_state_src_send_caps_locked(sc, evt);
		if (sc->conn_state != FUSB_ST_SRC_NEGOTIATE_CAP)
			break;
		/* FALLTHROUGH */
	case FUSB_ST_SRC_NEGOTIATE_CAP:
		fusb302_state_src_negotiate_cap_locked(sc, evt);
		/* FALLTHROUGH */
	case FUSB_ST_SRC_TRANSITION_SUPPLY:
		fusb302_state_src_transition_supply_locked(sc, evt);
		break;
	case FUSB_ST_SRC_CAP_RESPONSE:
		/* reject — hard reset */
		fusb302_set_state_locked(sc, FUSB_ST_SRC_SEND_HARDRST);
		break;
	case FUSB_ST_SRC_TRANSITION_DEFAULT:
		fusb302_state_src_transition_default_locked(sc, evt);
		break;
	case FUSB_ST_SRC_READY:
		fusb302_state_src_ready_locked(sc, evt);
		break;
	case FUSB_ST_SRC_GET_SINK_CAPS:
		fusb302_state_src_get_sink_caps_locked(sc, evt);
		break;
	case FUSB_ST_SRC_SEND_HARDRST:
		fusb302_state_src_send_hardrst_locked(sc, evt);
		break;
	case FUSB_ST_SRC_SEND_SOFTRST:
		fusb302_state_src_send_softrst_locked(sc, evt);
		break;
	case FUSB_ST_SRC_SOFTRST:
		fusb302_state_src_softrst_locked(sc);
		break;
	case FUSB_ST_SRC_DISCOVERY:
		/* simple retry: just go back to send caps */
		fusb302_set_state_locked(sc, FUSB_ST_SRC_SEND_CAPS);
		break;
	default:
		break;
	}
}

/* -----------------------------------------------------------------------
 * Timer callbacks
 * ----------------------------------------------------------------------- */
static void
fusb302_timer_state_cb(void *arg)
{
	struct fusb302_softc *sc;

	sc = arg;
	atomic_set_int(&sc->pending_events, FUSB_EVT_TIMER_STATE);
	taskqueue_enqueue(taskqueue_thread, &sc->irq_task);
}

static void
fusb302_timer_mux_cb(void *arg)
{
	struct fusb302_softc *sc;

	sc = arg;
	atomic_set_int(&sc->pending_events, FUSB_EVT_TIMER_MUX);
	taskqueue_enqueue(taskqueue_thread, &sc->irq_task);
}

/* -----------------------------------------------------------------------
 * IRQ task (runs in taskqueue_thread)
 * ----------------------------------------------------------------------- */
static void
fusb302_irq_task(void *context, int pending __unused)
{
	struct fusb302_softc *sc;
	uint32_t evt;

	sc = context;

	mtx_lock(&sc->mtx);
	if (!sc->initialized) {
		mtx_unlock(&sc->mtx);
		return;
	}

	/* Collect events: hardware alerts + timer expirations */
	evt = 0;
	fusb302_tcpc_alert_locked(sc, &evt);
	evt |= atomic_swap_int(&sc->pending_events, 0);
	evt |= sc->work_continue;
	sc->work_continue = 0;

	if (evt == 0) {
		mtx_unlock(&sc->mtx);
		return;
	}

	/* Detach detection when connected as source: CC open → detach */
	if (sc->notify_is_cc && (evt & FUSB_EVT_CC)) {
		uint8_t st0;
		fusb302_read_reg(sc, FUSB_REG_STATUS0, &st0);
		if (st0 & 0x20 /* COMP bit */) {
			device_printf(sc->dev, "CC open, detaching\n");
			fusb302_set_state_unattached_locked(sc);
			goto out;
		}
	}

	/* Process received PD message */
	if (evt & FUSB_EVT_RX) {
		if (fusb302_fifo_read_locked(sc) == 0) {
			if (PD_IS_CTRL(sc->rec_head, PD_CMT_SOFTRESET))
				fusb302_set_state_locked(sc, FUSB_ST_SRC_SOFTRST);
		}
	}

	/* Increment message ID on successful TX */
	if ((evt & FUSB_EVT_TX) && sc->tx_state == FUSB_TX_SUCCESS)
		sc->msg_id = (sc->msg_id + 1) & 0x7;

	/* Run state machine */
	fusb302_run_state_locked(sc, evt);

	/* Re-enqueue if more work is pending */
	if (sc->work_continue)
		taskqueue_enqueue(taskqueue_thread, &sc->irq_task);

out:
	mtx_unlock(&sc->mtx);
}

static void
fusb302_intr(void *context)
{
	struct fusb302_softc *sc;

	sc = context;
	taskqueue_enqueue(taskqueue_thread, &sc->irq_task);
}

/* -----------------------------------------------------------------------
 * Initialization
 * ----------------------------------------------------------------------- */
static int
fusb302_init_locked(struct fusb302_softc *sc)
{
	uint8_t intr, intra, intrb;
	int error;

	error = fusb302_write_reg(sc, FUSB_REG_RESET, FUSB_RESET_SW_RES);
	if (error != 0)
		return (error);
	DELAY(1000);

	if (sc->vbus_supply != NULL && !sc->vbus_enabled) {
		error = regulator_enable(sc->vbus_supply);
		if (error != 0)
			return (error);
		sc->vbus_enabled = true;
	}

	error = fusb302_write_reg(sc, FUSB_REG_POWER, FUSB_POWER_ALL);
	if (error != 0)
		return (error);
	error = fusb302_write_reg(sc, FUSB_REG_CONTROL0,
	    FUSB_CTL0_HOST_CUR_DEF);
	if (error != 0)
		return (error);

	/* Enable auto-retry with 3 retries for PD TX reliability */
	error = fusb302_update_reg(sc, FUSB_REG_CONTROL3,
	    FUSB_CTL3_AUTO_RETRY | FUSB_CTL3_N_RETRIES,
	    FUSB_CTL3_AUTO_RETRY | FUSB_CTL3_N_RETRIES);
	if (error != 0)
		return (error);

	/* Set interrupt masks for full PD operation */
	error = fusb302_write_reg(sc, FUSB_REG_MASK1, FUSB_MASK1_PD);
	if (error != 0)
		return (error);
	error = fusb302_write_reg(sc, FUSB_REG_MASKA, FUSB_MASKA_PD);
	if (error != 0)
		return (error);
	error = fusb302_write_reg(sc, FUSB_REG_MASKB, FUSB_MASKB_PD);
	if (error != 0)
		return (error);

	/* Clear any stale interrupts */
	fusb302_read_reg(sc, FUSB_REG_INTERRUPT, &intr);
	fusb302_read_reg(sc, FUSB_REG_INTERRUPTA, &intra);
	fusb302_read_reg(sc, FUSB_REG_INTERRUPTB, &intrb);

	/* Enable the FUSB302 interrupt output pin (clear INT_MASK bit) */
	error = fusb302_update_reg(sc, FUSB_REG_CONTROL0,
	    FUSB_CTL0_INT_MASK, 0);
	if (error != 0)
		return (error);

	/* Start CC toggle in DFP-only mode */
	error = fusb302_write_reg(sc, FUSB_REG_CONTROL2,
	    FUSB_CTL2_TOG_RD_ONLY | FUSB_CTL2_MODE_DFP | FUSB_CTL2_TOGGLE);
	if (error != 0)
		return (error);
	DELAY(1000);

	sc->conn_state = FUSB_ST_UNATTACHED;
	sc->initialized = true;

	/* Snapshot legacy status fields */
	fusb302_read_reg(sc, FUSB_REG_POWER,    &sc->power);
	fusb302_read_reg(sc, FUSB_REG_CONTROL2, &sc->control2);
	fusb302_read_reg(sc, FUSB_REG_STATUS0,  &sc->status0);
	fusb302_read_reg(sc, FUSB_REG_STATUS1,  &sc->status1);
	fusb302_read_reg(sc, FUSB_REG_STATUS0A, &sc->status0a);
	fusb302_read_reg(sc, FUSB_REG_STATUS1A, &sc->status1a);
	sc->state_valid = true;

	device_printf(sc->dev,
	    "initialized: irq=0x%02x irqa=0x%02x irqb=0x%02x "
	    "st0=0x%02x st1=0x%02x st0a=0x%02x st1a=0x%02x\n",
	    intr, intra, intrb,
	    sc->status0, sc->status1, sc->status0a, sc->status1a);
	return (0);
}

/* -----------------------------------------------------------------------
 * Public APIs
 * ----------------------------------------------------------------------- */
int
fusb302_get_typec_status(device_t dev, struct fusb302_typec_status *status)
{
	struct fusb302_softc *sc;

	if (dev == NULL || status == NULL)
		return (EINVAL);

	sc = device_get_softc(dev);
	mtx_lock(&sc->mtx);
	status->attached = sc->notify_is_cc;
	status->vbusok = (sc->status0 & FUSB_ST0_VBUSOK) != 0;
	status->has_irq = (sc->irq_res != NULL);
	status->state_valid = sc->state_valid;
	status->togss_raw = (sc->status1a & FUSB_ST1A_TOGSS_MASK) >>
	    FUSB_ST1A_TOGSS_SHIFT;
	status->orientation = (sc->cc_polarity == 0) ?
	    FUSB302_TYPEC_ORIENT_CC1 : FUSB302_TYPEC_ORIENT_CC2;
	status->role = sc->notify_is_cc ?
	    FUSB302_TYPEC_ROLE_SOURCE : FUSB302_TYPEC_ROLE_NONE;
	mtx_unlock(&sc->mtx);

	return (sc->state_valid ? 0 : ENXIO);
}

/*
 * Exported for rk_cdn_dp via linker_file_lookup_symbol("fusb302_get_dp_altmode_state").
 * Returns 0 and fills *status when valid DP Alt Mode state is available.
 */
int
fusb302_get_dp_altmode_state(device_t dev,
    struct rk3399_typec_dp_altmode_status *status)
{
	struct fusb302_softc *sc;

	if (dev == NULL || status == NULL)
		return (EINVAL);

	sc = device_get_softc(dev);
	mtx_lock(&sc->mtx);
	*status = sc->dp_altmode;
	mtx_unlock(&sc->mtx);

	return (status->valid ? 0 : ENXIO);
}

/* -----------------------------------------------------------------------
 * Sysctl
 * ----------------------------------------------------------------------- */
static int
fusb302_sysctl_reg(SYSCTL_HANDLER_ARGS)
{
	struct fusb302_softc *sc;
	uint8_t reg, val;
	int ival, error;

	sc = arg1;
	reg = (uint8_t)arg2;

	mtx_lock(&sc->mtx);
	error = fusb302_read_reg(sc, reg, &val);
	mtx_unlock(&sc->mtx);
	if (error != 0)
		return (error);

	ival = val;
	return (sysctl_handle_int(oidp, &ival, 0, req));
}

static void
fusb302_add_sysctls(struct fusb302_softc *sc)
{
	struct sysctl_ctx_list *ctx;
	struct sysctl_oid *tree;

	ctx = device_get_sysctl_ctx(sc->dev);
	tree = device_get_sysctl_tree(sc->dev);

	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO, "device_id",
	    CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_MPSAFE, sc,
	    FUSB_REG_DEVICE_ID, fusb302_sysctl_reg, "I", "device ID");
	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO, "status0",
	    CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_MPSAFE, sc,
	    FUSB_REG_STATUS0, fusb302_sysctl_reg, "I", "STATUS0");
	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO, "status1",
	    CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_MPSAFE, sc,
	    FUSB_REG_STATUS1, fusb302_sysctl_reg, "I", "STATUS1");
	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO, "status0a",
	    CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_MPSAFE, sc,
	    FUSB_REG_STATUS0A, fusb302_sysctl_reg, "I", "STATUS0A");
	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO, "status1a",
	    CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_MPSAFE, sc,
	    FUSB_REG_STATUS1A, fusb302_sysctl_reg, "I", "STATUS1A");
	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO, "switches0",
	    CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_MPSAFE, sc,
	    FUSB_REG_SWITCHES0, fusb302_sysctl_reg, "I", "SWITCHES0");
	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO, "switches1",
	    CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_MPSAFE, sc,
	    FUSB_REG_SWITCHES1, fusb302_sysctl_reg, "I", "SWITCHES1");
}

/* -----------------------------------------------------------------------
 * OFW IRQ recovery (unchanged from original)
 * ----------------------------------------------------------------------- */
static int
fusb302_try_ofw_irq(struct fusb302_softc *sc)
{
	pcell_t *cells;
	phandle_t node, producer;
	rman_res_t count, start;
	int error, irq, ncells;

	if (bus_get_resource(sc->dev, SYS_RES_IRQ, sc->irq_rid, &start,
	    &count) == 0)
		return (EEXIST);

	node = ofw_bus_get_node(sc->dev);
	if (node <= 0)
		return (ENOENT);

	cells = NULL;
	error = ofw_bus_intr_by_rid(sc->dev, node, sc->irq_rid, &producer,
	    &ncells, &cells);
	if (error != 0)
		return (error);

	irq = ofw_bus_map_intr(sc->dev, producer, ncells, cells);
	free(cells, M_OFWPROP);
	if (irq <= 0)
		return (ENOENT);

	error = bus_set_resource(sc->dev, SYS_RES_IRQ, sc->irq_rid, irq, 1);
	if (error != 0)
		return (error);

	device_printf(sc->dev, "recovered irq %d from OFW\n", irq);
	return (0);
}

/* -----------------------------------------------------------------------
 * Driver methods
 * ----------------------------------------------------------------------- */
static int
fusb302_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "Fairchild FUSB302 Type-C PD controller");
	return (BUS_PROBE_DEFAULT);
}

static int
fusb302_attach(device_t dev)
{
	struct fusb302_softc *sc;
	rman_res_t irq_count, irq_start;
	int error;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->addr = iicbus_get_addr(dev);
	sc->irq_rid = 0;
	mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);
	TASK_INIT(&sc->irq_task, 0, fusb302_irq_task, sc);
	callout_init(&sc->timer_state, 1 /* MPSAFE */);
	callout_init(&sc->timer_mux, 1);

	error = regulator_get_by_ofw_property(dev, 0, "vbus-supply",
	    &sc->vbus_supply);
	if (error != 0 && error != ENOENT) {
		device_printf(dev, "cannot get vbus-supply: %d\n", error);
		goto fail;
	}

	error = fusb302_read_reg(sc, FUSB_REG_DEVICE_ID, &sc->device_id);
	if (error != 0) {
		device_printf(dev, "cannot read device ID: %d\n", error);
		goto fail;
	}
	device_printf(dev, "device id 0x%02x at addr 0x%02x\n",
	    sc->device_id, sc->addr);

	mtx_lock(&sc->mtx);
	error = fusb302_init_locked(sc);
	mtx_unlock(&sc->mtx);
	if (error != 0) {
		device_printf(dev, "init failed: %d\n", error);
		goto fail;
	}

	/* IRQ allocation — same multi-fallback as original */
	if (bus_get_resource(dev, SYS_RES_IRQ, sc->irq_rid, &irq_start,
	    &irq_count) == 0) {
		device_printf(dev, "irq metadata start=%ju count=%ju\n",
		    (uintmax_t)irq_start, (uintmax_t)irq_count);
	}
	sc->irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &sc->irq_rid,
	    RF_ACTIVE);
	if (sc->irq_res == NULL) {
		error = fusb302_try_ofw_irq(sc);
		if (error == 0)
			sc->irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ,
			    &sc->irq_rid, RF_ACTIVE);
	}
	if (sc->irq_res != NULL) {
		error = bus_setup_intr(dev, sc->irq_res,
		    INTR_TYPE_MISC | INTR_MPSAFE, NULL, fusb302_intr, sc,
		    &sc->irq_cookie);
		if (error != 0) {
			device_printf(dev, "cannot setup irq: %d\n", error);
			bus_release_resource(dev, SYS_RES_IRQ, sc->irq_rid,
			    sc->irq_res);
			sc->irq_res = NULL;
			sc->irq_cookie = NULL;
		}
	}
	if (sc->irq_res == NULL)
		device_printf(dev, "no irq, using poll/sysctl only\n");

	fusb302_add_sysctls(sc);
	OF_device_register_xref(OF_xref_from_node(ofw_bus_get_node(dev)), dev);
	return (0);

fail:
	callout_drain(&sc->timer_state);
	callout_drain(&sc->timer_mux);
	if (sc->vbus_enabled)
		regulator_disable(sc->vbus_supply);
	if (sc->vbus_supply != NULL)
		regulator_release(sc->vbus_supply);
	mtx_destroy(&sc->mtx);
	return (ENXIO);
}

static int
fusb302_detach(device_t dev)
{
	struct fusb302_softc *sc;

	sc = device_get_softc(dev);

	if (sc->irq_cookie != NULL)
		bus_teardown_intr(dev, sc->irq_res, sc->irq_cookie);
	if (sc->irq_res != NULL)
		bus_release_resource(dev, SYS_RES_IRQ, sc->irq_rid, sc->irq_res);

	callout_drain(&sc->timer_state);
	callout_drain(&sc->timer_mux);
	taskqueue_drain(taskqueue_thread, &sc->irq_task);

	if (sc->vbus_enabled && sc->vbus_supply != NULL)
		regulator_disable(sc->vbus_supply);
	if (sc->vbus_supply != NULL)
		regulator_release(sc->vbus_supply);

	OF_device_register_xref(OF_xref_from_node(ofw_bus_get_node(dev)), NULL);
	mtx_destroy(&sc->mtx);
	return (0);
}

static device_method_t fusb302_methods[] = {
	DEVMETHOD(device_probe,		fusb302_probe),
	DEVMETHOD(device_attach,	fusb302_attach),
	DEVMETHOD(device_detach,	fusb302_detach),
	DEVMETHOD_END
};

static driver_t fusb302_driver = {
	"fusb302",
	fusb302_methods,
	sizeof(struct fusb302_softc),
};

DRIVER_MODULE(fusb302, iicbus, fusb302_driver, 0, 0);
MODULE_DEPEND(fusb302, iicbus, IICBUS_MINVER, IICBUS_PREFVER, IICBUS_MAXVER);
MODULE_VERSION(fusb302, 2);
