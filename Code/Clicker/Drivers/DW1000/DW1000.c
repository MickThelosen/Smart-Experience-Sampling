//
// Created by marijn on 12/5/24.
//
#include <string.h>
#include "DW1000.h"
#include "DW1000_regs.h"

// ! TODO
uint32_t txFCTRL;
uint32_t status_reg;

/*!<
 * constants
 * */
DW1000_config_t dw1000_cfg = { .chan = 2, .prf = DWT_PRF_64M, .txPreambLength =
DWT_PLEN_1024, .rxPAC = DWT_PAC32, .txCode = 9, .rxCode = 9, .nsSFD = 1,
		.dataRate = DWT_BR_110K, .phrMode = DWT_PHRMODE_STD, .sfdTO = (1025 + 64
				- 32) };

/* measurements */
static double tof;
static double distance;

/*!<
 * RX buffer
 * */
#define RX_BUF_LEN 25
uint8_t rx_buffer[RX_BUF_LEN];

/*!<
 * timestamps
 * */
/* initiatior */
uint64_t poll_tx_ts;
uint64_t resp_rx_ts;
uint64_t final_tx_ts;

/* responder */
uint64_t poll_rx_ts;
uint64_t resp_tx_ts;
uint64_t final_rx_ts;

/*!<
 * messages
 * */
/* initiator */
uint8_t tx_poll_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21,
		0, 0, 0 };
uint8_t rx_resp_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x10,
		0x02, 0, 0, 0, 0, 0 };
uint8_t tx_final_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x23,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
/* responder */
uint8_t rx_poll_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21,
		0, 0, 0 };
uint8_t tx_resp_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x10,
		0x02, 0, 0, 0, 0, 0 };
uint8_t rx_final_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x23,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

uint8_t frame_seq_nb = 0;

/*!<
 * LL functions
 * */

static inline void DW1000_read_reg(DW1000_t *dw1000, uint32_t reg,
		uint16_t offset, uint8_t *buffer, uint32_t size) {
	__disable_irq();
	reg &= 0x3FU;
	HAL_GPIO_WritePin(dw1000->NSS_port, dw1000->NSS_pin, 0);
	if (offset & 0xFF00) {
		reg |= 0x8040U | (offset << 8);
		SPI_write8(dw1000->spi, (void*) &reg, 3, DW1000_TIMEOUT);
	} else if (offset) {
		reg |= 0x40U | (offset << 8);
		SPI_write8(dw1000->spi, (void*) &reg, 2, DW1000_TIMEOUT);
	} else {
		SPI_write8(dw1000->spi, (void*) &reg, 1, DW1000_TIMEOUT);
	}
	SPI_read8(dw1000->spi, buffer, size, DW1000_TIMEOUT);
	HAL_GPIO_WritePin(dw1000->NSS_port, dw1000->NSS_pin, 1);
	__enable_irq();
}

static inline void DW1000_write_reg(DW1000_t *dw1000, uint32_t reg,
		uint16_t offset, uint8_t *buffer, uint32_t size) {
	reg = (reg & 0x3FU) | 0x80;
	HAL_GPIO_WritePin(dw1000->NSS_port, dw1000->NSS_pin, 0);
	if (offset & 0xFF00) {
		reg |= 0x8040U | (offset << 8);
		SPI_write8(dw1000->spi, (void*) &reg, 3, DW1000_TIMEOUT);
	} else if (offset) {
		reg |= 0x40U | (offset << 8);
		SPI_write8(dw1000->spi, (void*) &reg, 2, DW1000_TIMEOUT);
	} else {
		SPI_write8(dw1000->spi, (void*) &reg, 1, DW1000_TIMEOUT);
	}
	SPI_write8(dw1000->spi, buffer, size, DW1000_TIMEOUT);
	HAL_GPIO_WritePin(dw1000->NSS_port, dw1000->NSS_pin, 1);
}

/*!<
 * functions
 * */
void DW1000_init(DW1000_t *dw1000) {
	uint32_t tmp;
	// reset
	HAL_GPIO_WritePin(dw1000->NRST_port, dw1000->NRST_pin, 0);
	HAL_Delay(1);

	HAL_GPIO_WritePin(dw1000->NRST_port, dw1000->NRST_pin, 1);

	// check devid
	DW1000_read_reg(dw1000, DEV_ID_ID, 0, (void*) &tmp, 4);
	if (tmp != 0xDECA0130UL) {
		for (;;)
			;
	}

	// enable clock
	DW1000_read_reg(dw1000, PMSC_ID, PMSC_CTRL0_OFFSET, (void*) &tmp, 2);
	tmp &= 0xFFFCU;	// reset sys_clk select
	tmp |= 0x0001U;	// select XTI as sys_clk
	DW1000_write_reg(dw1000, PMSC_ID, PMSC_CTRL0_OFFSET, (void*) &tmp, 2);

	tmp = PMSC_CTRL1_PKTSEQ_DISABLE;
	DW1000_write_reg(dw1000, PMSC_ID, PMSC_CTRL1_OFFSET, (void*) &tmp, 2); // Disable PMSC ctrl of RF and RX clk blocks

	// Clear any AON auto download bits (as reset will trigger AON download)
	tmp = 0x00;
	DW1000_write_reg(dw1000, AON_ID, AON_WCFG_OFFSET, (void*) &tmp, 2);
	// Clear the wake-up configuration
	DW1000_write_reg(dw1000, AON_ID, AON_CFG0_OFFSET, (void*) &tmp, 1);
	// Upload the new configuration
	DW1000_write_reg(dw1000, AON_ID, AON_CTRL_OFFSET, (void*) &tmp, 1); // Clear the register
	tmp = AON_CTRL_SAVE;
	DW1000_write_reg(dw1000, AON_ID, AON_CTRL_OFFSET, (void*) &tmp, 1);

	// Reset HIF, TX, RX and PMSC (set the reset bits)
	tmp = PMSC_CTRL0_RESET_ALL;
	DW1000_write_reg(dw1000, PMSC_ID, PMSC_CTRL0_SOFTRESET_OFFSET, (void*) &tmp,
			1);

	// DW1000 needs a 10us sleep to let clk PLL lock after reset - the PLL will automatically lock after the reset
	// Could also have polled the PLL lock flag, but then the SPI needs to be < 3MHz !! So a simple delay is easier
	HAL_Delay(1);

	// Clear the reset bits
	tmp = PMSC_CTRL0_RESET_CLEAR;
	DW1000_write_reg(dw1000, PMSC_ID, PMSC_CTRL0_SOFTRESET_OFFSET, (void*) &tmp,
			1);

	// enable clock
	DW1000_read_reg(dw1000, PMSC_ID, PMSC_CTRL0_OFFSET, (void*) &tmp, 2);
	tmp &= 0xFFFCU;	// reset sys_clk select
	tmp |= 0x0001U;	// select XTI as sys_clk
	DW1000_write_reg(dw1000, PMSC_ID, PMSC_CTRL0_OFFSET, (void*) &tmp, 2);

	// configure the CPLL lock detect
	tmp = EC_CTRL_PLLLCK;
	DW1000_write_reg(dw1000, EXT_SYNC_ID, EC_CTRL_OFFSET, (void*) &tmp, 1);

	// LDO tune config
	// Write the address
	tmp = LDOTUNE_ADDRESS;
	DW1000_write_reg(dw1000, OTP_IF_ID, OTP_ADDR, (void*) &tmp, 2);

	// Perform OTP Read - Manual read mode has to be set
	tmp = OTP_CTRL_OTPREAD | OTP_CTRL_OTPRDEN;
	DW1000_write_reg(dw1000, OTP_IF_ID, OTP_CTRL, (void*) &tmp, 1);
	tmp = 0;
	DW1000_write_reg(dw1000, OTP_IF_ID, OTP_CTRL, (void*) &tmp, 1);
	DW1000_read_reg(dw1000, OTP_IF_ID, OTP_RDAT, (void*) &tmp, 4);
	if ((tmp & 0xFF) != 0) {
		tmp = OTP_SF_LDO_KICK;
		DW1000_write_reg(dw1000, OTP_IF_ID, OTP_SF, (void*) &tmp, 1); // Set load LDO kick bit
	}

	// xtrim OTP
	// Write the address
	tmp = XTRIM_ADDRESS;
	DW1000_write_reg(dw1000, OTP_IF_ID, OTP_ADDR, (void*) &tmp, 2);

	// Perform OTP Read - Manual read mode has to be set
	tmp = OTP_CTRL_OTPREAD | OTP_CTRL_OTPRDEN;
	DW1000_write_reg(dw1000, OTP_IF_ID, OTP_CTRL, (void*) &tmp, 1);
	tmp = 0x00;
	DW1000_write_reg(dw1000, OTP_IF_ID, OTP_CTRL, (void*) &tmp, 1); // OTPREAD is self clearing but OTPRDEN is not

	// Read read data, available 40ns after rising edge of OTP_READ

	DW1000_read_reg(dw1000, OTP_IF_ID, OTP_RDAT, (void*) &tmp, 4);
	tmp &= 0xffff;

	if ((tmp & 0x1F) == 0) // A value of 0 means that the crystal has not been trimmed
			{
		tmp = FS_XTALT_MIDRANGE; // Set to mid-range if no calibration value inside
	}
	// Configure XTAL trim
	uint8_t reg_val = (3 << 5) | (tmp & FS_XTALT_MASK);
	DW1000_write_reg(dw1000, FS_CTRL_ID, FS_XTALT_OFFSET, (void*) &reg_val, 1);

	// force enable LDE
	tmp = 0x0301U;
	DW1000_write_reg(dw1000, PMSC_ID, PMSC_CTRL0_OFFSET, (void*) &tmp, 2);
	tmp = OTP_CTRL_LDELOAD;
	DW1000_write_reg(dw1000, OTP_IF_ID, OTP_CTRL, (void*) &tmp, 2); // Set load LDE kick bit
	HAL_Delay(1);
	DW1000_read_reg(dw1000, PMSC_ID, PMSC_CTRL0_OFFSET, (void*) &tmp, 2);
	tmp &= 0xFE00U;
	DW1000_write_reg(dw1000, PMSC_ID, PMSC_CTRL0_OFFSET, (void*) &tmp, 2);

	// AON
	tmp = 0x00;
	DW1000_write_reg(dw1000, AON_ID, AON_CFG1_OFFSET, (void*) &tmp, 1);
}

uint8_t DW1000_config(DW1000_t *dw1000, DW1000_config_t *cfg) {
	uint32_t tmp = 0;
	uint32_t sysCFGreg;
	uint16_t reg16 = 0x28F4; // rx code 9 (lde_replicaCoeff)

	DW1000_read_reg(dw1000, SYS_CFG_ID, 0x00, (void*) &sysCFGreg, 4);
	if (DWT_BR_110K == cfg->dataRate) {
		sysCFGreg |= SYS_CFG_RXM110K;
		reg16 >>= 3; // lde_replicaCoeff must be divided by 8
	} else {
		sysCFGreg &= (~SYS_CFG_RXM110K);
	}

	sysCFGreg &= ~SYS_CFG_PHR_MODE_11;
	sysCFGreg |= (SYS_CFG_PHR_MODE_11
			& ((uint32_t) cfg->phrMode << SYS_CFG_PHR_MODE_SHFT));

	DW1000_write_reg(dw1000, SYS_CFG_ID, 0x00, (void*) &sysCFGreg, 4);

	// Set the lde_replicaCoeff
	DW1000_write_reg(dw1000, LDE_IF_ID, LDE_REPC_OFFSET, (void*) &reg16, 2);

	tmp = LDE_PARAM1;
	DW1000_write_reg(dw1000, LDE_IF_ID, LDE_CFG1_OFFSET, (void*) &tmp, 1); // 8-bit configuration register

	if (dw1000_cfg.prf - DWT_PRF_16M) {
		tmp = LDE_PARAM3_64;
		DW1000_write_reg(dw1000, LDE_IF_ID, LDE_CFG2_OFFSET, (void*) &tmp, 2); // 16-bit LDE configuration tuning register
	} else {
		tmp = LDE_PARAM3_16;
		DW1000_write_reg(dw1000, LDE_IF_ID, LDE_CFG2_OFFSET, (void*) &tmp, 2);
	}

	// Configure PLL2/RF PLL block CFG/TUNE (for a given channel)
	tmp = FS_PLLCFG_CH2;
	DW1000_write_reg(dw1000, FS_CTRL_ID, FS_PLLCFG_OFFSET, (void*) &tmp, 4);
	tmp = FS_PLLTUNE_CH2;
	DW1000_write_reg(dw1000, FS_CTRL_ID, FS_PLLTUNE_OFFSET, (void*) &tmp, 1);

	// Configure RF RX blocks (for specified channel/bandwidth)
	tmp = RF_RXCTRLH_NBW;
	DW1000_write_reg(dw1000, RF_CONF_ID, RF_RXCTRLH_OFFSET, (void*) &tmp, 1);

	// Configure RF TX blocks (for specified channel and PRF)
	// Configure RF TX control
	tmp = RF_TXCTRL_CH2;
	DW1000_write_reg(dw1000, RF_CONF_ID, RF_TXCTRL_OFFSET, (void*) &tmp, 4);

	// Configure the baseband parameters (for specified PRF, bit rate, PAC, and SFD settings)
	// DTUNE0
	tmp = DRX_TUNE0b_110K_NSTD;
	DW1000_write_reg(dw1000, DRX_CONF_ID, DRX_TUNE0b_OFFSET, (void*) &tmp, 2);

	// DTUNE1
	tmp = DRX_TUNE1a_PRF64;
	DW1000_write_reg(dw1000, DRX_CONF_ID, DRX_TUNE1a_OFFSET, (void*) &tmp, 2);
	tmp = DRX_TUNE1b_110K;
	DW1000_write_reg(dw1000, DRX_CONF_ID, DRX_TUNE1b_OFFSET, (void*) &tmp, 2);

	// DTUNE2
	tmp = DRX_TUNE2_PRF64_PAC32;
	DW1000_write_reg(dw1000, DRX_CONF_ID, DRX_TUNE2_OFFSET, (void*) &tmp, 4);

	// DTUNE3 (SFD timeout)
	// Don't allow 0 - SFD timeout will always be enabled
	if (cfg->sfdTO == 0) {
		cfg->sfdTO = DWT_SFDTOC_DEF;
	}
	tmp = cfg->sfdTO;
	DW1000_write_reg(dw1000, DRX_CONF_ID, DRX_SFDTOC_OFFSET, (void*) &tmp, 2);

	// Configure AGC parameters
	tmp = AGC_TUNE2_VAL;
	DW1000_write_reg(dw1000, AGC_CFG_STS_ID, 0xC, (void*) &tmp, 4);
	tmp = AGC_TUNE1_64M;
	DW1000_write_reg(dw1000, AGC_CFG_STS_ID, 0x4, (void*) &tmp, 2);

	// Set (non-standard) user SFD for improved performance,
	uint8_t nsSfd_result = 0;
	uint8_t useDWnsSFD = 0;
	if (cfg->nsSFD) {
		// Write non standard (DW) SFD length
		tmp = DW_NS_SFD_LEN_110K;
		DW1000_write_reg(dw1000, USR_SFD_ID, 0x00, (void*) &tmp, 1);
		nsSfd_result = 3;
		useDWnsSFD = 1;
	}

	uint8_t chan = cfg->chan;
	uint32_t regval = (CHAN_CTRL_TX_CHAN_MASK
			& (chan << CHAN_CTRL_TX_CHAN_SHIFT))
			| // Transmit Channel
			(CHAN_CTRL_RX_CHAN_MASK & (chan << CHAN_CTRL_RX_CHAN_SHIFT))
			| // Receive Channel
			(CHAN_CTRL_RXFPRF_MASK
					& ((uint32_t) cfg->prf << CHAN_CTRL_RXFPRF_SHIFT))
			| // RX PRF
			((CHAN_CTRL_TNSSFD | CHAN_CTRL_RNSSFD)
					& ((uint32_t) nsSfd_result << CHAN_CTRL_TNSSFD_SHIFT))
			| // nsSFD enable RX&TX
			(CHAN_CTRL_DWSFD & ((uint32_t) useDWnsSFD << CHAN_CTRL_DWSFD_SHIFT))
			| // Use DW nsSFD
			(CHAN_CTRL_TX_PCOD_MASK
					& ((uint32_t) cfg->txCode << CHAN_CTRL_TX_PCOD_SHIFT)) | // TX Preamble Code
			(CHAN_CTRL_RX_PCOD_MASK
					& ((uint32_t) cfg->rxCode << CHAN_CTRL_RX_PCOD_SHIFT)); // RX Preamble Code

	DW1000_write_reg(dw1000, CHAN_CTRL_ID, 0x00, (void*) &regval, 4);

	// Set up TX Preamble Size, PRF and Data Rate
	txFCTRL = ((uint32_t) (cfg->txPreambLength | cfg->prf)
			<< TX_FCTRL_TXPRF_SHFT)
			| ((uint32_t) cfg->dataRate << TX_FCTRL_TXBR_SHFT);

	DW1000_write_reg(dw1000, TX_FCTRL_ID, 0x00, (void*) &txFCTRL, 4);

	// Request TX start and TRX off at the same time
	tmp = SYS_CTRL_TXSTRT | SYS_CTRL_TRXOFF;
	DW1000_write_reg(dw1000, SYS_CTRL_ID, SYS_CTRL_OFFSET, (void*) &tmp, 1);

	// set rx antenna delay
	tmp = RX_ANT_DLY;
	DW1000_write_reg(dw1000, LDE_IF_ID, LDE_RXANTD_OFFSET, (void*) &tmp, 2);
	// set tx antenna delay
	tmp = TX_ANT_DLY;
	DW1000_write_reg(dw1000, LDE_IF_ID, TX_ANTD_OFFSET, (void*) &tmp, 2);

	/* tx specific functions */
	if (dw1000->tx) {
		// set rx after tx delay
		uint32_t val;
		DW1000_read_reg(dw1000, ACK_RESP_T_ID, 0x00, (void*) &val, 4); // Read ACK_RESP_T_ID register
		val &= ~(ACK_RESP_T_W4R_TIM_MASK); // Clear the timer (19:0)
		val |= (POLL_TX_TO_RESP_RX_DLY_UUS & ACK_RESP_T_W4R_TIM_MASK); // In UWB microseconds (e.g. turn the receiver on 20uus after TX)
		DW1000_write_reg(dw1000, ACK_RESP_T_ID, 0x00, (void*) &val, 4);

		uint32_t time = RESP_RX_TIMEOUT_UUS;
		// set tx timeout
		DW1000_write_reg(dw1000, RX_FWTO_ID, RX_FWTO_OFFSET, (void*) &time, 2);
		DW1000_read_reg(dw1000, SYS_CFG_ID, 0x03, (void*) &tmp, 1); // Read at offset 3 to get the upper byte only
		tmp |= (SYS_CFG_RXWTOE >> 24); // Shift RXWTOE mask as we read the upper byte only
		// OR in 32bit value (1 bit set), I know this is in high byte.

		DW1000_write_reg(dw1000, SYS_CFG_ID, 0x03, (void*) &tmp, 1); // Write at offset 3 to write the upper byte only

	}
	/* end of tx specific functions */

	// set preamble timeout
	tmp = PRE_TIMEOUT;
	DW1000_write_reg(dw1000, DRX_CONF_ID, DRX_PRETOC_OFFSET, (void*) &tmp, 2);

	uint32_t sys_status;
	DW1000_read_reg(dw1000, SYS_STATUS_ID, 0x00, (void*) &sys_status, 4); // Read at offset 3 to get the upper 2 bytes out of 5
	if (sys_status & 0x2) {
		return 1;
	} else {
		return 0;
	}

}

static uint64_t get_tx_timestamp_u64(DW1000_t *dw1000) {
	uint8_t ts_tab[5];
	uint64_t ts = 0;
	int8_t i;
	DW1000_read_reg(dw1000, TX_TIME_ID, TX_TIME_TX_STAMP_OFFSET,
			(void*) &ts_tab, TX_TIME_TX_STAMP_LEN);
	for (i = 4; i >= 0; i--) {
		ts <<= 8;
		ts |= ts_tab[i];
	}
	return ts;
}

static uint64_t get_rx_timestamp_u64(DW1000_t *dw1000) {
	uint8_t ts_tab[5];
	uint64_t ts = 0;
	int8_t i;
	DW1000_read_reg(dw1000, RX_TIME_ID, RX_TIME_RX_STAMP_OFFSET,
			(void*) &ts_tab, RX_TIME_RX_STAMP_LEN);
	for (i = 4; i >= 0; i--) {
		ts <<= 8;
		ts |= ts_tab[i];
	}
	return ts;
}

static void final_msg_set_ts(uint8_t *ts_field, uint64_t ts) {
	int8_t i;
	for (i = 0; i < FINAL_MSG_TS_LEN; i++) {
		ts_field[i] = (uint8_t) ts;
		ts >>= 8;
	}
}

static void final_msg_get_ts(const uint8_t *ts_field, uint32_t *ts) {
	int8_t i;
	*ts = 0;
	for (i = 0; i < FINAL_MSG_TS_LEN; i++) {
		*ts += ts_field[i] << (i * 8);
	}
}

void DW1000_initiator(DW1000_t *dw1000, uint8_t channel) {
	/* Write frame data to DW1000 and prepare transmission. See NOTE 8 below. */
	tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
	tx_poll_msg[10] = channel;
	DW1000_write_reg(dw1000, TX_BUFFER_ID, 0, tx_poll_msg,
			sizeof(tx_poll_msg) - 2);
	uint32_t tmp = 0;
	tmp = txFCTRL | sizeof(tx_poll_msg)
			| ((uint32_t) 0x0U << TX_FCTRL_TXBOFFS_SHFT)
			| ((uint32_t) 0x1U << TX_FCTRL_TR_SHFT);
	DW1000_write_reg(dw1000, TX_FCTRL_ID, 0x00, (void*) &tmp, 4);

	// start TX
	tmp = SYS_CTRL_WAIT4RESP | SYS_CTRL_TXSTRT;
	DW1000_write_reg(dw1000, SYS_CTRL_ID, SYS_CTRL_OFFSET, (void*) &tmp, 1);

	// poll for reception
	do {
		DW1000_read_reg(dw1000, SYS_STATUS_ID, 0x0, (void*) &status_reg, 4);

	} while (!(status_reg
			& (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)));

	frame_seq_nb++;

	if (status_reg & SYS_STATUS_RXFCG) {
		uint32_t frame_len;

		/* Clear good RX frame event and TX frame sent in the DW1000 status register. */
		tmp = SYS_STATUS_RXFCG | SYS_STATUS_TXFRS;
		DW1000_write_reg(dw1000, SYS_STATUS_ID, 0x00, (void*) &tmp, 4);

		/* A frame has been received, read it into the local buffer. */
		DW1000_read_reg(dw1000, RX_FINFO_ID, 0x00, (void*) &frame_len, 4);
		frame_len &= RX_FINFO_RXFLEN_MASK;
		if (frame_len <= RX_BUF_LEN) {
			DW1000_read_reg(dw1000, RX_BUFFER_ID, 0x00, (void*) &rx_buffer,
					frame_len);
		}
		rx_buffer[ALL_MSG_SN_IDX] = 0;
		if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0
				&& rx_buffer[13] == channel) {
			uint32_t final_tx_time;

			/* Retrieve poll transmission and response reception timestamp. */
			poll_tx_ts = get_tx_timestamp_u64(dw1000);
			resp_rx_ts = get_rx_timestamp_u64(dw1000);

			/* Compute final message transmission time. See NOTE 10 below. */
			final_tx_time = (resp_rx_ts
					+ (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
			DW1000_write_reg(dw1000, DX_TIME_ID, 0x01, (void*) &final_tx_time,
					4);

			/* Final TX timestamp is the transmission time we programmed plus the TX antenna delay. */
			final_tx_ts = (((uint64_t) (final_tx_time & 0xFFFFFFFEUL)) << 8)
					+ TX_ANT_DLY;

			/* Write all timestamps in the final message. See NOTE 11 below. */
			final_msg_set_ts(&tx_final_msg[FINAL_MSG_POLL_TX_TS_IDX],
					poll_tx_ts);
			final_msg_set_ts(&tx_final_msg[FINAL_MSG_RESP_RX_TS_IDX],
					resp_rx_ts);
			final_msg_set_ts(&tx_final_msg[FINAL_MSG_FINAL_TX_TS_IDX],
					final_tx_ts);

			/* Write and send final message. See NOTE 8 below. */
			tx_final_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
			DW1000_write_reg(dw1000, TX_BUFFER_ID, 0, tx_final_msg,
					sizeof(tx_final_msg) - 2);
			tmp = txFCTRL | sizeof(tx_final_msg)
					| ((uint32_t) 0x0U << TX_FCTRL_TXBOFFS_SHFT)
					| ((uint32_t) 0x1U << TX_FCTRL_TR_SHFT);
			DW1000_write_reg(dw1000, TX_FCTRL_ID, 0x00, (void*) &tmp, 4);

			tmp |= (SYS_CTRL_TXDLYS | SYS_CTRL_TXSTRT);
			DW1000_write_reg(dw1000, SYS_CTRL_ID, SYS_CTRL_OFFSET, (void*) &tmp,
					1);

			DW1000_read_reg(dw1000, SYS_STATUS_ID, 0x03, (void*) &tmp, 2); // Read at offset 3 to get the upper 2 bytes out of 5

			if ((tmp & SYS_STATUS_TXERR) == 0) {
				/* Poll DW1000 until TX frame sent event set. See NOTE 9 below. */
				do {
					DW1000_read_reg(dw1000, SYS_STATUS_ID, 0x00,
							(void*) &status_reg, 4);
				} while (!(status_reg & SYS_STATUS_TXFRS));

				/* Clear TXFRS event. */
				tmp = SYS_STATUS_TXFRS;
				DW1000_write_reg(dw1000, SYS_STATUS_ID, 0x00, (void*) &tmp, 4);

				/* Increment frame sequence number after transmission of the final message (modulo 256). */
				frame_seq_nb++;

			}
		}
	} else {
		/* Clear RX error/timeout events in the DW1000 status register. */
		tmp = SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR;
		DW1000_write_reg(dw1000, SYS_STATUS_ID, 0x00, (void*) &tmp, 4);

		/* Reset RX to properly reinitialise LDE operation. */
		tmp = PMSC_CTRL0_RESET_RX;
		DW1000_write_reg(dw1000, PMSC_ID, PMSC_CTRL0_SOFTRESET_OFFSET,
				(void*) &tmp, 1);
		// Clear RX reset
		tmp = PMSC_CTRL0_RESET_CLEAR;
		DW1000_write_reg(dw1000, PMSC_ID, PMSC_CTRL0_SOFTRESET_OFFSET,
				(void*) &tmp, 1);
	}

}

double DW1000_responder(DW1000_t *dw1000, uint8_t channel) {

	/* Clear reception timeout to start next ranging process. */
	uint32_t tmp;

	DW1000_read_reg(dw1000, SYS_CFG_ID, 3, (void*) &tmp, 1); // Read at offset 3 to get the upper byte only
	tmp &= ~(SYS_CFG_RXWTOE >> 24); // Shift RXWTOE mask as we read the upper byte only
	DW1000_write_reg(dw1000, SYS_CFG_ID, 3, (void*) &tmp, 1);

	/* Activate reception immediately. */
	DW1000_read_reg(dw1000, SYS_STATUS_ID, 3, (void*) &tmp, 1); // Read 1 byte at offset 3 to get the 4th byte out of 5

	if ((tmp & (SYS_STATUS_ICRBP >> 24)) !=   // IC side Receive Buffer Pointer
			((tmp & (SYS_STATUS_HSRBP >> 24)) << 1)) // Host Side Receive Buffer Pointer
			{
		tmp = 0x01;
		DW1000_write_reg(dw1000, SYS_CTRL_ID, SYS_CTRL_HRBT_OFFSET,
				(void*) &tmp, 1); // We need to swap RX buffer status reg (write one to toggle internally)
	}

	tmp = SYS_CTRL_RXENAB;
	DW1000_write_reg(dw1000, SYS_CTRL_ID, SYS_CTRL_OFFSET, (void*) &tmp, 2);

	/* Poll for reception of a frame or error/timeout. See NOTE 8 below. */
	do {
		DW1000_read_reg(dw1000, SYS_STATUS_ID, 0x0, (void*) &status_reg, 4);
	} while (!(status_reg
			& (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)));

	if (status_reg & SYS_STATUS_RXFCG) {

		uint32_t frame_len;

		/* Clear good RX frame event in the DW1000 status register. */
		tmp = SYS_STATUS_RXFCG;
		DW1000_write_reg(dw1000, SYS_STATUS_ID, 0x00, (void*) &tmp, 4);
		memset(rx_buffer, 0, sizeof(rx_buffer));
		/* A frame has been received, read it into the local buffer. */
		DW1000_read_reg(dw1000, RX_FINFO_ID, 0x00, (void*) &frame_len, 4);
		frame_len &= RX_FINFO_RXFL_MASK_1023;
		if (frame_len <= RX_BUF_LEN) {
			DW1000_read_reg(dw1000, RX_BUFFER_ID, 0x00, (void*) &rx_buffer,
					frame_len);
		}

		rx_buffer[ALL_MSG_SN_IDX] = 0;

		if (memcmp(rx_buffer, rx_poll_msg, ALL_MSG_COMMON_LEN) == 0
				&& rx_buffer[10] == channel) {
			uint32_t resp_tx_time;

			/* Retrieve poll reception timestamp. */
			poll_rx_ts = get_rx_timestamp_u64(dw1000);

			// set rx after tx delay
			DW1000_read_reg(dw1000, ACK_RESP_T_ID, 0x00, (void*) &tmp, 4); // Read ACK_RESP_T_ID register
			tmp &= ~(ACK_RESP_T_W4R_TIM_MASK); // Clear the timer (19:0)
			tmp |= (RESP_TX_TO_FINAL_RX_DLY_UUS & ACK_RESP_T_W4R_TIM_MASK); // In UWB microseconds (e.g. turn the receiver on 20uus after TX)
			DW1000_write_reg(dw1000, ACK_RESP_T_ID, 0x00, (void*) &tmp, 4);

			// set rx timeout
			tmp = 0;
			DW1000_read_reg(dw1000, SYS_CFG_ID, 0x03, (void*) &tmp, 1); // Read at offset 3 to get the upper byte only

			uint32_t time = FINAL_RX_TIMEOUT_UUS;
			DW1000_write_reg(dw1000, RX_FWTO_ID, RX_FWTO_OFFSET, (void*) &time,
					2);
			tmp |= (SYS_CFG_RXWTOE >> 24); // Shift RXWTOE mask as we read the upper byte only
			// OR in 32bit value (1 bit set), I know this is in high byte.
			DW1000_write_reg(dw1000, SYS_CFG_ID, 0x03, (void*) &tmp, 1); // Write at offset 3 to write the upper byte only

			tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
			tx_resp_msg[13] = channel;
			DW1000_write_reg(dw1000, TX_BUFFER_ID, 0, tx_resp_msg,
					sizeof(tx_resp_msg) - 2);

			tmp = txFCTRL | sizeof(tx_resp_msg)
					| ((uint32_t) 0x0U << TX_FCTRL_TXBOFFS_SHFT)
					| ((uint32_t) 0x1U << TX_FCTRL_TR_SHFT);
			DW1000_write_reg(dw1000, TX_FCTRL_ID, 0x00, (void*) &tmp, 4);

			/* Set send time for response. See NOTE 9 below. */
			resp_tx_time = (poll_rx_ts
					+ (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
			DW1000_write_reg(dw1000, DX_TIME_ID, 0x01, (void*) &resp_tx_time,
					4);

			// start TX
			tmp = SYS_CTRL_WAIT4RESP | SYS_CTRL_TXDLYS | SYS_CTRL_TXSTRT;
			DW1000_write_reg(dw1000, SYS_CTRL_ID, SYS_CTRL_OFFSET, (void*) &tmp,
					1);

			DW1000_read_reg(dw1000, SYS_STATUS_ID, 0x03, (void*) &tmp, 2); // Read at offset 3 to get the upper 2 bytes out of 5

			if (tmp & SYS_STATUS_TXERR) {
				tmp = SYS_CTRL_TRXOFF;
				DW1000_write_reg(dw1000, SYS_CTRL_ID, SYS_CTRL_OFFSET,
						(void*) &tmp, 1);
				goto TXError;
			}
			uint32_t sys_status;
			DW1000_read_reg(dw1000, SYS_STATUS_ID, 0x00, (void*) &sys_status,
					4); // Read at offset 3 to get the upper 2 bytes out of 5
			// poll for reception
			status_reg = 0;
			do {
				DW1000_read_reg(dw1000, SYS_STATUS_ID, 0x0, (void*) &status_reg,
						4);
			} while (!(status_reg
					& (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO
							| SYS_STATUS_ALL_RX_ERR)));
			/* Increment frame sequence number after transmission of the response message (modulo 256). */
			frame_seq_nb++;

			if (status_reg & SYS_STATUS_RXFCG) {
				tmp = SYS_STATUS_RXFCG | SYS_STATUS_TXFRS;
				DW1000_write_reg(dw1000, SYS_STATUS_ID, 0x00, (void*) &tmp, 4);
				/* A frame has been received, read it into the local buffer. */
				DW1000_read_reg(dw1000, RX_FINFO_ID, 0x00, (void*) &frame_len,
						4);
				frame_len &= RX_FINFO_RXFLEN_MASK;
				if (frame_len <= RX_BUF_LEN) {
					DW1000_read_reg(dw1000, RX_BUFFER_ID, 0x00,
							(void*) &rx_buffer, frame_len);
				}
				rx_buffer[ALL_MSG_SN_IDX] = 0;
				if (memcmp(rx_buffer, rx_final_msg, ALL_MSG_COMMON_LEN) == 0) {
					uint32_t poll_tx_ts, resp_rx_ts, final_tx_ts;
					uint32_t poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
					double Ra, Rb, Da, Db;
					int64_t tof_dtu;

					/* Retrieve response transmission and final reception timestamps. */
					resp_tx_ts = get_tx_timestamp_u64(dw1000);
					final_rx_ts = get_rx_timestamp_u64(dw1000);

					/* Get timestamps embedded in the final message. */
					final_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX],
							&poll_tx_ts);
					final_msg_get_ts(&rx_buffer[FINAL_MSG_RESP_RX_TS_IDX],
							&resp_rx_ts);
					final_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX],
							&final_tx_ts);

					/* Compute time of flight. 32-bit subtractions give correct answers even if clock has wrapped. See NOTE 12 below. */
					poll_rx_ts_32 = (uint32_t) poll_rx_ts;
					resp_tx_ts_32 = (uint32_t) resp_tx_ts;
					final_rx_ts_32 = (uint32_t) final_rx_ts;
					Ra = (double) (resp_rx_ts - poll_tx_ts);
					Rb = (double) (final_rx_ts_32 - resp_tx_ts_32);
					Da = (double) (final_tx_ts - resp_rx_ts);
					Db = (double) (resp_tx_ts_32 - poll_rx_ts_32);
					tof_dtu = (int64_t) ((Ra * Rb - Da * Db)
							/ (Ra + Rb + Da + Db));
					tof = tof_dtu * DWT_TIME_UNITS;
					distance = tof * SPEED_OF_LIGHT;
					distance -= 137.34302980478139;
					return distance;
				}
			} else {
				/* Clear RX error/timeout events in the DW1000 status register. */
				tmp = SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR;
				DW1000_write_reg(dw1000, SYS_STATUS_ID, 0x00, (void*) &tmp, 4);
				/* Reset RX to properly reinitialise LDE operation. */
				tmp = PMSC_CTRL0_RESET_RX;
				DW1000_write_reg(dw1000, PMSC_ID, PMSC_CTRL0_SOFTRESET_OFFSET,
						(void*) &tmp, 1);
				// Clear RX reset
				tmp = PMSC_CTRL0_RESET_CLEAR;
				DW1000_write_reg(dw1000, PMSC_ID, PMSC_CTRL0_SOFTRESET_OFFSET,
						(void*) &tmp, 1);
			}
		}
	} else {
		TXError:
		/* Clear RX error/timeout events in the DW1000 status register. */
		tmp = SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR;
		DW1000_write_reg(dw1000, SYS_STATUS_ID, 0x00, (void*) &tmp, 4);
		/* Reset RX to properly reinitialise LDE operation. */
		tmp = PMSC_CTRL0_RESET_RX;
		DW1000_write_reg(dw1000, PMSC_ID, PMSC_CTRL0_SOFTRESET_OFFSET,
				(void*) &tmp, 1);
		// Clear RX reset
		tmp = PMSC_CTRL0_RESET_CLEAR;
		DW1000_write_reg(dw1000, PMSC_ID, PMSC_CTRL0_SOFTRESET_OFFSET,
				(void*) &tmp, 1);

	}
	return 0;
}

