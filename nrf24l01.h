#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "debug.h"
#include "stm32f1xx_hal.h"


/******************************************* 
 *              Defines
 ******************************************/

#define NRF_TIMEOUT_MILIS   ((uint32_t)1000)


/******************************************* 
 *              Data types
 ******************************************/

typedef enum {
    NRF_DATA_RATE_250KBPS = 1,
    NRF_DATA_RATE_1MBPS   = 0,
    NRF_DATA_RATE_2MBPS   = 2
} NRF_DATA_RATE;

typedef enum {
    NRF_TX_PWR_M18dBm = 0,
    NRF_TX_PWR_M12dBm = 1,
    NRF_TX_PWR_M6dBm  = 2,
    NRF_TX_PWR_0dBm   = 3
} NRF_TX_PWR;

typedef enum {
    NRF_ADDR_WIDTH_3 = 1,
    NRF_ADDR_WIDTH_4 = 2,
    NRF_ADDR_WIDTH_5 = 3
} NRF_ADDR_WIDTH;

typedef enum {
    NRF_CRC_WIDTH_1B = 0,
    NRF_CRC_WIDTH_2B = 1
} NRF_CRC_WIDTH;

typedef enum {
    NRF_STATE_RX = 1,
    NRF_STATE_TX = 0
} NRF_STATE_RXTX;

typedef enum {
    NRF_OK,
    NRF_ERROR,
    NRF_MAXTX,
    NRF_INVALID_ARGUMENT,
    NRF_BUSY,
    NRF_TIMEOUT
} NRF_RESULT;

typedef enum {
    NRF_NO_PIPE = 0,
    NRF_PIPE_0 = (1 << 0),
    NRF_PIPE_1 = (1 << 1),
    NRF_PIPE_2 = (1 << 2),
    NRF_PIPE_3 = (1 << 3),
    NRF_PIPE_4 = (1 << 4),
    NRF_PIPE_5 = (1 << 5),
    NRF_PIPE_ALL = 0x3F
} NRF_PIPE;

typedef struct {
    NRF_DATA_RATE       data_rate;
    NRF_TX_PWR          tx_power;
    NRF_CRC_WIDTH       crc_width;
    NRF_ADDR_WIDTH      addr_width;

    uint8_t  payload_length;
    uint8_t  retransmit_count;
    uint8_t  retransmit_delay;
    uint8_t  rf_channel;

    uint8_t* tx_address;
    uint8_t* pipe0_address;
    uint8_t* pipe1_address;
    uint8_t  pipe2_address;
    uint8_t  pipe3_address;
    uint8_t  pipe4_address;
    uint8_t  pipe5_address;

    /* Must be sufficient size according to payload_length */
    uint8_t* rx_buffer;

    uint8_t rx_buffer_pip;

    SPI_HandleTypeDef* spi;
    uint32_t           spi_timeout;

    GPIO_TypeDef* csn_port;
    uint16_t      csn_pin;

    GPIO_TypeDef* ce_port;
    uint16_t      ce_pin;

    GPIO_TypeDef* irq_port;
    uint16_t      irq_pin;

} nrf24l01_config;

typedef struct {
    nrf24l01_config     config;

    volatile bool           tx_busy;
    volatile bool           rx_busy;
    volatile NRF_RESULT     tx_result;
    volatile NRF_STATE_RXTX state;
    volatile uint8_t        status;

} nrf24l01;


/****************************************** 
 *          Function prototypes
 ******************************************/

NRF_RESULT nrf_is_connected(nrf24l01* dev);

/* Initialization routine */
NRF_RESULT nrf_init(nrf24l01* dev, nrf24l01_config* config);

/* EXTI Interrupt Handler
 *
 * You must call this function on Falling edge trigger detection interrupt
 * handler, typically, from HAL_GPIO_EXTI_Callback  */
void nrf_irq_handler(nrf24l01* dev);

/* Asynchronous Data Receiving (__weak)
 *
 * Override this function to handle received data asynchronously,
 * default implementation is used in favor of nrf_receive_packet for blocking
 * data receiving */
void nrf_packet_received_callback(nrf24l01* dev);

/* Blocking Data Receiving
 *
 * Blocks until the data has arrived, then returns a pointer to received data.
 * Please note, once nrf_packet_received_callback routine is overridden, this
 * one will stop working. */
uint8_t* nrf_receive_packet(nrf24l01* dev);

/* Blocking Data Sending
 *
 * If the AA is enabled (default), this method will return:
 *   NRF_OK - the data has been acknowledged by other party
 *   NRF_ERROR - the data has not been received (maximum retransmissions has
 * occurred) If the AA is disabled, returns NRF_OK once the data has been
 * transmitted (with no guarantee the data was actually received). */
NRF_RESULT nrf_send_packet(nrf24l01* dev, const uint8_t* data);

/* Blocking Data Sending, with NO_ACK flag
 *
 * Disables the AA for this packet, thus this method always returns NRF_OK */
NRF_RESULT nrf_send_packet_noack(nrf24l01* dev, const uint8_t* data);

/* Non-Blocking Data Sending */
NRF_RESULT nrf_push_packet(nrf24l01* dev, const uint8_t* data);



/*---------------------------------------------------------------

        LOW LEVEL STUFF (you don't have to look in here...)

-----------------------------------------------------------------*/
NRF_RESULT nrf_send_command(nrf24l01* dev, uint8_t cmd, const uint8_t* tx,
                            uint8_t* rx, uint8_t len, volatile uint8_t* status);
/* CMD */
NRF_RESULT nrf_read_register(nrf24l01* dev, uint8_t reg, uint8_t* data);
NRF_RESULT nrf_write_register(nrf24l01* dev, uint8_t reg, uint8_t* data);
NRF_RESULT nrf_read_rx_payload(nrf24l01* dev, uint8_t* data);
NRF_RESULT nrf_write_tx_payload(nrf24l01* dev, const uint8_t* data);
NRF_RESULT nrf_write_tx_payload_noack(nrf24l01* dev, const uint8_t* data);
NRF_RESULT nrf_flush_rx(nrf24l01* dev);
NRF_RESULT nrf_flush_tx(nrf24l01* dev);

/* RF_SETUP */
NRF_RESULT nrf_set_data_rate(nrf24l01* dev, NRF_DATA_RATE rate);
NRF_RESULT nrf_set_tx_power(nrf24l01* dev, NRF_TX_PWR pwr);
NRF_RESULT nrf_set_ccw(nrf24l01* dev, bool activate);
NRF_RESULT nrf_read_rpd(nrf24l01*dev);

/* STATUS */
NRF_RESULT nrf_clear_interrupts(nrf24l01* dev);

/* RF_CH */
NRF_RESULT nrf_set_rf_channel(nrf24l01* dev, uint8_t ch);

/* SETUP_RETR */
NRF_RESULT nrf_set_retransmittion_count(nrf24l01* dev, uint8_t count);
NRF_RESULT nrf_set_retransmittion_delay(nrf24l01* dev, uint8_t delay);

/* SETUP_AW */
NRF_RESULT nrf_set_address_width(nrf24l01* dev, NRF_ADDR_WIDTH width);

/* EN_RXADDR */
NRF_RESULT nrf_set_rx_pipes(nrf24l01* dev, uint8_t pipes);

/* EN_AA */
NRF_RESULT nrf_enable_auto_ack(nrf24l01* dev, uint8_t pipe);

/* CONFIG */
NRF_RESULT nrf_enable_crc(nrf24l01* dev, bool activate);
NRF_RESULT nrf_set_crc_width(nrf24l01* dev, NRF_CRC_WIDTH width);
NRF_RESULT nrf_power_up(nrf24l01* dev, bool power_up);
NRF_RESULT nrf_rx_tx_control(nrf24l01* dev, NRF_STATE_RXTX rx);
NRF_RESULT nrf_enable_rx_data_ready_irq(nrf24l01* dev, bool activate);
NRF_RESULT nrf_enable_tx_data_sent_irq(nrf24l01* dev, bool activate);
NRF_RESULT nrf_enable_max_retransmit_irq(nrf24l01* dev, bool activate);

/* TX_ADDR */
NRF_RESULT nrf_set_tx_address(nrf24l01* dev, uint8_t* address); // 5bytes of address

/* RX_ADDR */
NRF_RESULT nrf_set_rx_address(nrf24l01* dev, uint8_t pipe, uint8_t* address);

/* RX_PW */
NRF_RESULT nrf_set_rx_payload_width(nrf24l01* dev, uint8_t width);

