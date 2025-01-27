//
// Created by marijn on 12/5/24.
//

#ifndef STM32WB07_DW1000_H
#define STM32WB07_DW1000_H
#include "SPI.h"

/*!<
 * defines
 * */
#define DW1000_TIMEOUT 10
#define TX_ANT_DLY 16436
#define RX_ANT_DLY 16436
#define PRE_TIMEOUT 32
#define UUS_TO_DWT_TIME 65536
#define RESP_TX_TO_FINAL_RX_DLY_UUS 500
#define FINAL_RX_TIMEOUT_UUS 5000
#define RESP_RX_TO_FINAL_TX_DLY_UUS 4000
#define RESP_RX_TIMEOUT_UUS 5000
#define POLL_TX_TO_RESP_RX_DLY_UUS 300
#define POLL_RX_TO_RESP_TX_DLY_UUS 3000


#define ALL_MSG_COMMON_LEN 10
#define ALL_MSG_SN_IDX 2
#define FINAL_MSG_POLL_TX_TS_IDX 10
#define FINAL_MSG_RESP_RX_TS_IDX 14
#define FINAL_MSG_FINAL_TX_TS_IDX 18
#define FINAL_MSG_TS_LEN 4
#define SPEED_OF_LIGHT 299702547 // m/s

/*!<
 * types
 * */
typedef struct {
	SPI_HandleTypeDef *spi;
	GPIO_TypeDef *NSS_port;
	GPIO_TypeDef *NRST_port;
	uint16_t NSS_pin;
	uint16_t NRST_pin;
	uint8_t tx :1;
} DW1000_t;

typedef struct {
	uint8_t chan;           //!< channel number {1, 2, 3, 4, 5, 7 }
	uint8_t prf;    //!< Pulse Repetition Frequency {DWT_PRF_16M or DWT_PRF_64M}
	uint8_t txPreambLength; //!< DWT_PLEN_64..DWT_PLEN_4096
	uint8_t rxPAC;   //!< Acquisition Chunk Size (Relates to RX preamble length)
	uint8_t txCode;         //!< TX preamble code
	uint8_t rxCode;         //!< RX preamble code
	uint8_t nsSFD; //!< Boolean should we use non-standard SFD for better performance
	uint8_t dataRate;    //!< Data Rate {DWT_BR_110K, DWT_BR_850K or DWT_BR_6M8}
	uint8_t phrMode; //!< PHR mode {0x0 - standard DWT_PHRMODE_STD, 0x3 - extended frames DWT_PHRMODE_EXT}
	uint16_t sfdTO;         //!< SFD timeout value (in symbols)
} DW1000_config_t;

extern DW1000_config_t dw1000_cfg;

/*!<
 * functions
 * */
void DW1000_init(DW1000_t *dw1000);
uint8_t DW1000_config(DW1000_t *dw1000, DW1000_config_t *cfg);

void DW1000_initiator(DW1000_t *dw1000, uint8_t channel);
double DW1000_responder(DW1000_t *dw1000, uint8_t channel);

#endif //STM32WB07_DW1000_H
