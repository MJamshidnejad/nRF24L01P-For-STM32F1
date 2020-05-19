#include "nrf24l01.h"
#include "nrf24l01_reg.h"

void ce_set(nrf24l01* dev) {
    //HAL_Delay(1);                   
    HAL_GPIO_WritePin(dev->config.ce_port, dev->config.ce_pin, GPIO_PIN_SET);
}

void ce_reset(nrf24l01* dev) {
    HAL_GPIO_WritePin(dev->config.ce_port, dev->config.ce_pin, GPIO_PIN_RESET);
    //HAL_Delay(1);
}

static void csn_set(nrf24l01* dev) {
    HAL_GPIO_WritePin(dev->config.csn_port, dev->config.csn_pin, GPIO_PIN_SET);
}

static void csn_reset(nrf24l01* dev) {
    HAL_GPIO_WritePin(dev->config.csn_port, dev->config.csn_pin, GPIO_PIN_RESET);
}

GPIO_PinState nrf_ce_state(nrf24l01* dev)
{
    return HAL_GPIO_ReadPin(dev->config.csn_port, dev->config.csn_pin);
}

/************************************************************************/
NRF_RESULT nrf_is_connected(nrf24l01* dev)
{
    uint8_t reg = 0;
    if (nrf_read_register(dev, SETUP_AW, &reg) != NRF_OK) {
        return NRF_ERROR;
    }
    if (reg >= 1 && reg <= 3) {
        return NRF_OK;
    }
    return NRF_ERROR;
}


/************************************************************************/
NRF_RESULT nrf_init(nrf24l01* dev, nrf24l01_config* config)
{
    dev->config = *config;

    ce_reset(dev);
    csn_reset(dev);

    nrf_power_up(dev, true);

    uint8_t config_reg = 0;

    while ((config_reg & 2) == 0) { // wait for powerup
        nrf_read_register(dev, NRF_CONFIG, &config_reg);
    }

    nrf_set_rx_payload_width(dev, dev->config.payload_length);


    nrf_set_tx_address(dev, dev->config.tx_address);
    nrf_set_rx_address(dev, 0, dev->config.pipe0_address);
    nrf_set_rx_address(dev, 1, dev->config.pipe1_address);
    nrf_set_rx_address(dev, 2, &(dev->config.pipe2_address));
    nrf_set_rx_address(dev, 3, &(dev->config.pipe3_address));
    nrf_set_rx_address(dev, 4, &(dev->config.pipe4_address));
    nrf_set_rx_address(dev, 5, &(dev->config.pipe5_address));

    nrf_enable_rx_data_ready_irq(dev, 1);
    nrf_enable_tx_data_sent_irq(dev, 1);
    nrf_enable_max_retransmit_irq(dev, 1);

    nrf_enable_crc(dev, 1);
    nrf_set_crc_width(dev, dev->config.crc_width);

    nrf_set_address_width(dev, dev->config.addr_width);
    nrf_set_rf_channel(dev, dev->config.rf_channel);
    nrf_set_data_rate(dev, dev->config.data_rate);
    nrf_set_retransmittion_count(dev, dev->config.retransmit_count);
    nrf_set_retransmittion_delay(dev, dev->config.retransmit_delay);

    nrf_set_rx_pipes(dev, NRF_PIPE_1 | NRF_PIPE_0);
    nrf_enable_auto_ack(dev, NRF_PIPE_1 | NRF_PIPE_0);

    nrf_clear_interrupts(dev);

    nrf_rx_tx_control(dev, NRF_STATE_RX);

    nrf_flush_rx(dev);
    ce_set(dev);

    return NRF_OK;
}


/************************************************************************/
NRF_RESULT nrf_send_command(nrf24l01* dev, uint8_t cmd, const uint8_t* tx,
                            uint8_t* rx, uint8_t len, volatile uint8_t* status)
{
    uint8_t myTX[len + 1];
    uint8_t myRX[len + 1];
    myTX[0] = cmd;

    int i = 0;
    for (i = 0; i < len; i++) {
        myTX[1 + i] = tx[i];
        myRX[i]     = 0;
    }

    csn_reset(dev);

    if (HAL_SPI_TransmitReceive(dev->config.spi, myTX, myRX, 1 + len,
                                dev->config.spi_timeout) != HAL_OK) {
        return NRF_ERROR;
    }

    csn_set(dev);

    *status = myRX[0];

    for (i = 0; i < len; i++) { rx[i] = myRX[1 + i]; }

    return NRF_OK;
}


/************************************************************************/
void nrf_irq_handler(nrf24l01* dev)
{
    uint8_t status = 0;
    uint8_t temp = 0;
    if (nrf_read_register(dev, NRF_STATUS, &status) != NRF_OK) { return; }

#if(DEBUG == 1)
    sprintf(debug_str, "We have an interrupt: RX: %d, TX: %d, MR: %d",
            status & (1 << 6), status & (1 << 5), status & (1 << 4));
    debug_println(debug_str);
#endif

    if ((status & (1 << RX_DR))) { // RX FIFO Interrupt
        uint8_t fifo_status = 0;
        ce_reset(dev);
        nrf_read_register(dev, FIFO_STATUS, &fifo_status);
        if ((fifo_status & (1 << RX_EMPTY)) == 0) {
            nrf_read_rx_payload(dev, dev->config.rx_buffer);
            temp = 1 << RX_DR;
            nrf_write_register(dev, NRF_STATUS, &temp);  // Clear RX_DR flag
            nrf_packet_received_callback(dev);
        }
        ce_set(dev);
    }
    if (status & (1 << TX_DS)) { // TX Data Sent Interrupt
        ce_reset(dev);
        nrf_rx_tx_control(dev, NRF_STATE_RX);
        ce_set(dev);
        temp = 1 << TX_DS;  // clear the interrupt flag
        nrf_write_register(dev, NRF_STATUS, &temp); // Clear TX_DS flag
        dev->tx_result = NRF_OK;
        dev->tx_busy   = false;
    }
    if ((status & (1 << MAX_RT))) { // MaxRetransmits reached

        nrf_flush_tx(dev);
        nrf_power_up(dev, 0); // power down
        nrf_power_up(dev, 1); // power up

        ce_reset(dev);
        nrf_rx_tx_control(dev, NRF_STATE_RX);
        temp = 1 << MAX_RT;
        nrf_write_register(dev, NRF_STATUS, &temp);
        ce_set(dev);
        dev->tx_result = NRF_MAXTX;
        dev->tx_busy   = false;
    }
}


/************************************************************************/
__weak void nrf_packet_received_callback(nrf24l01* dev) {
    // default implementation (__weak) is used in favor of nrf_receive_packet
    dev->rx_busy = false;
}


/************************************************************************/
NRF_RESULT nrf_read_register(nrf24l01* dev, uint8_t reg, uint8_t* data) {
    uint8_t tx = 0;
    if (nrf_send_command(dev, R_REGISTER | (REGISTER_MASK & reg), &tx, data, 1, &dev->status) !=
            NRF_OK) {
        return NRF_ERROR;
    }
    return NRF_OK;
}


/************************************************************************/
NRF_RESULT nrf_write_register(nrf24l01* dev, uint8_t reg, uint8_t* data) {
    uint8_t rx = 0;
    if (nrf_send_command(dev, W_REGISTER | (REGISTER_MASK & reg), data, &rx, 1, &dev->status) !=
            NRF_OK) {
        return NRF_ERROR;
    }
    return NRF_OK;
}


/************************************************************************/
NRF_RESULT nrf_read_rx_payload(nrf24l01* dev, uint8_t* data) {
    uint8_t tx[dev->config.payload_length];
    if (nrf_send_command(dev, R_RX_PAYLOAD, tx, data,
                         dev->config.payload_length, &dev->status) != NRF_OK) {
        return NRF_ERROR;
    }
    dev->config.rx_buffer_pip = (dev->status & 0x0E) >> RX_P_NO;
    return NRF_OK;
}


/************************************************************************/
NRF_RESULT nrf_write_tx_payload(nrf24l01* dev, const uint8_t* data) {
    uint8_t rx[dev->config.payload_length];
    if (nrf_send_command(dev, W_TX_PAYLOAD, data, rx,
                         dev->config.payload_length, &dev->status) != NRF_OK) {
        return NRF_ERROR;
    }
    return NRF_OK;
}


/************************************************************************/
NRF_RESULT nrf_write_tx_payload_noack(nrf24l01* dev, const uint8_t* data) {
    uint8_t reg = 0;
    uint8_t rx[dev->config.payload_length];

    if (nrf_read_register(dev, FEATURE, &reg) != NRF_OK) {
        return NRF_ERROR;
    }
    reg |= (1 << EN_DYN_ACK);
    if (nrf_write_register(dev, FEATURE, &reg) != NRF_OK) {
        return NRF_ERROR;
    }

    if (nrf_send_command(dev, W_TX_PAYLOAD_NO_ACK, data, rx,
                         dev->config.payload_length, &dev->status) != NRF_OK) {
        return NRF_ERROR;
    }
    return NRF_OK;
}

/************************************************************************/

NRF_RESULT nrf_flush_tx(nrf24l01* dev) {
    uint8_t rx = 0;
    uint8_t tx = 0;
    if (nrf_send_command(dev, FLUSH_TX, &tx, &rx, 0, &dev->status) != NRF_OK) {
        return NRF_ERROR;
    }
    return NRF_OK;
}


/************************************************************************/

NRF_RESULT nrf_flush_rx(nrf24l01* dev) {
    uint8_t rx = 0;
    uint8_t tx = 0;
    if (nrf_send_command(dev, FLUSH_RX, &tx, &rx, 0, &dev->status) != NRF_OK) {
        return NRF_ERROR;
    }
    return NRF_OK;
}


/************************************************************************/

NRF_RESULT nrf_set_data_rate(nrf24l01* dev, NRF_DATA_RATE rate) {
    uint8_t reg = 0;
    if (nrf_read_register(dev, RF_SETUP, &reg) != NRF_OK) {
        return NRF_ERROR;
    }
    if (rate & 1) { // low bit set
        reg |= 1 << RF_DR_LOW;
    } else { // low bit clear
        reg &= ~(1 << RF_DR_LOW);
    }

    if (rate & 2) { // high bit set
        reg |= 1 << RF_DR_HIGH;
    } else { // high bit clear
        reg &= ~(1 << RF_DR_HIGH);
    }
    if (nrf_write_register(dev, RF_SETUP, &reg) != NRF_OK) {
        return NRF_ERROR;
    }
    dev->config.data_rate = rate;
    return NRF_OK;
}


/************************************************************************/

NRF_RESULT nrf_set_tx_power(nrf24l01* dev, NRF_TX_PWR pwr) {
    uint8_t reg = 0;
    if (nrf_read_register(dev, RF_SETUP, &reg) != NRF_OK) {
        return NRF_ERROR;
    }
    reg &= 0xF9;     // clear bits 1,2
    reg |= pwr << 1; // set bits 1,2
    if (nrf_write_register(dev, RF_SETUP, &reg) != NRF_OK) {
        return NRF_ERROR;
    }
    dev->config.tx_power = pwr;
    return NRF_OK;
}


/************************************************************************/

NRF_RESULT nrf_set_ccw(nrf24l01* dev, bool activate) {
    uint8_t reg = 0;
    if (nrf_read_register(dev, RF_SETUP, &reg) != NRF_OK) {
        return NRF_ERROR;
    }

    if (activate) {
        reg |= 0x80;
    } else {
        reg &= 0x7F;
    }

    if (nrf_write_register(dev, RF_SETUP, &reg) != NRF_OK) {
        return NRF_ERROR;
    }
    return NRF_OK;
}


/************************************************************************/

NRF_RESULT nrf_read_rpd(nrf24l01*dev)
{
		uint8_t rpd = 0;
    if (nrf_read_register(dev, RPD, &rpd) != NRF_OK) {
        return NRF_ERROR;
    }
    return ((rpd == 1)? NRF_OK:NRF_ERROR);
}

NRF_RESULT nrf_clear_interrupts(nrf24l01* dev) {
    uint8_t reg = 0;
    if (nrf_read_register(dev, NRF_STATUS, &reg) != NRF_OK) {
        return NRF_ERROR;
    }

    reg |= (1 << RX_DR) | (1 << TX_DS) | (1 << MAX_RT);

    if (nrf_write_register(dev, NRF_STATUS, &reg) != NRF_OK) {
        return NRF_ERROR;
    }
    return NRF_OK;
}


/************************************************************************/

NRF_RESULT nrf_set_rf_channel(nrf24l01* dev, uint8_t ch)
{
    GPIO_PinState state = nrf_ce_state(dev);
    ce_reset(dev);

    ch &= 0x7F;

    if (nrf_write_register(dev, RF_CH, &ch) != NRF_OK) {
        if (state == GPIO_PIN_SET) {
            ce_set(dev);
        }
        return NRF_ERROR;
    }
    dev->config.rf_channel = ch;
    if (state == GPIO_PIN_SET) {
        ce_set(dev);
    }
    return NRF_OK;
}


/************************************************************************/

NRF_RESULT nrf_set_retransmittion_count(nrf24l01* dev, uint8_t count)
{
    count &= 0x0F;
    uint8_t reg = 0;
    if (nrf_read_register(dev, SETUP_RETR, &reg) != NRF_OK) {
        return NRF_ERROR;
    }

    reg &= 0xF0;  // clearing bits 0,1,2,3
    reg |= count; // setting count

    if (nrf_write_register(dev, SETUP_RETR, &reg) != NRF_OK) {
        return NRF_ERROR;
    }
    dev->config.retransmit_count = count;
    return NRF_OK;
}


/************************************************************************/

NRF_RESULT nrf_set_retransmittion_delay(nrf24l01* dev, uint8_t delay)
{
    delay &= 0x0F;
    uint8_t reg = 0;
    if (nrf_read_register(dev, SETUP_RETR, &reg) != NRF_OK) {
        return NRF_ERROR;
    }

    reg &= 0x0F;       // clearing bits 1,2,6,7
    reg |= delay << 4; // setting delay

    if (nrf_write_register(dev, SETUP_RETR, &reg) != NRF_OK) {
        return NRF_ERROR;
    }
    dev->config.retransmit_delay = delay;
    return NRF_OK;
}


/************************************************************************/

NRF_RESULT nrf_set_address_width(nrf24l01* dev, NRF_ADDR_WIDTH width)
{
    uint8_t reg = 0;
    if (nrf_read_register(dev, SETUP_AW, &reg) != NRF_OK) {
        return NRF_ERROR;
    }

    reg &= 0x03;  // clearing bits 0,1
    reg |= width; // setting delay

    if (nrf_write_register(dev, SETUP_AW, &reg) != NRF_OK) {
        return NRF_ERROR;
    }
    dev->config.addr_width = width;
    return NRF_OK;
}


/************************************************************************/

NRF_RESULT nrf_set_rx_pipes(nrf24l01* dev, uint8_t pipes) {
    GPIO_PinState state = nrf_ce_state(dev);
    ce_reset(dev);

    if (nrf_write_register(dev, EN_RXADDR, &pipes) != NRF_OK) {
        if (state == GPIO_PIN_SET) {
            ce_set(dev);
        }
        return NRF_ERROR;
    }
    if (state == GPIO_PIN_SET) {
        ce_set(dev);
    }
    return NRF_OK;
}


/************************************************************************/

NRF_RESULT nrf_enable_auto_ack(nrf24l01* dev, uint8_t pipe) {
    GPIO_PinState state = nrf_ce_state(dev);
    ce_reset(dev);

    if (nrf_write_register(dev, EN_AA, &pipe) != NRF_OK) {
        if (state == GPIO_PIN_SET) {
            ce_set(dev);
        }
        return NRF_ERROR;
    }
    if (state == GPIO_PIN_SET) {
        ce_set(dev);
    }
    return NRF_OK;
}


/************************************************************************/

NRF_RESULT nrf_enable_crc(nrf24l01* dev, bool activate)
{
    uint8_t reg = 0;
    if (nrf_read_register(dev, NRF_CONFIG, &reg) != NRF_OK) {
        return NRF_ERROR;
    }

    if (activate) {
        reg |= 1 << EN_CRC;
    } else {
        reg &= ~(1 << EN_CRC);
    }

    if (nrf_write_register(dev, NRF_CONFIG, &reg) != NRF_OK) {
        return NRF_ERROR;
    }
    return NRF_OK;
}


/************************************************************************/

NRF_RESULT nrf_set_crc_width(nrf24l01* dev, NRF_CRC_WIDTH width)
{
    uint8_t reg = 0;
    if (nrf_read_register(dev, NRF_CONFIG, &reg) != NRF_OK) {
        return NRF_ERROR;
    }

    if (width == NRF_CRC_WIDTH_2B) {
        reg |= 1 << CRCO;
    } else {
        reg &= ~(1 << CRCO);
    }

    if (nrf_write_register(dev, NRF_CONFIG, &reg) != NRF_OK) {
        return NRF_ERROR;
    }
    dev->config.crc_width = width;
    return NRF_OK;
}


/************************************************************************/

NRF_RESULT nrf_power_up(nrf24l01* dev, bool power_up)
{
    uint8_t reg = 0;
    if (nrf_read_register(dev, NRF_CONFIG, &reg) != NRF_OK) {
        return NRF_ERROR;
    }

    if (power_up) {
        reg |= 1 << PWR_UP;
    } else {
        reg &= ~(1 << PWR_UP);
    }

    if (nrf_write_register(dev, NRF_CONFIG, &reg) != NRF_OK) {
        return NRF_ERROR;
    }
    return NRF_OK;
}


/************************************************************************/

NRF_RESULT nrf_rx_tx_control(nrf24l01* dev, NRF_STATE_RXTX rx)
{
    uint8_t reg = 0;
    if (nrf_read_register(dev, NRF_CONFIG, &reg) != NRF_OK) {
        return NRF_ERROR;
    }

    if (rx) {
        reg |= 1 << PRIM_RX;
    } else {
        reg &= ~(1 << PRIM_RX);
    }

    if (nrf_write_register(dev, NRF_CONFIG, &reg) != NRF_OK) {
        return NRF_ERROR;
    }
    dev->state = rx;        // Change state

    return NRF_OK;
}


/************************************************************************/

NRF_RESULT nrf_enable_rx_data_ready_irq(nrf24l01* dev, bool activate)
{
    uint8_t reg = 0;
    if (nrf_read_register(dev, NRF_CONFIG, &reg) != NRF_OK) {
        return NRF_ERROR;
    }

    if (!activate) {
        reg |= 1 << MASK_RX_DR;
    } else {
        reg &= ~(1 << MASK_RX_DR);
    }

    if (nrf_write_register(dev, NRF_CONFIG, &reg) != NRF_OK) {
        return NRF_ERROR;
    }
    return NRF_OK;
}


/************************************************************************/

NRF_RESULT nrf_enable_tx_data_sent_irq(nrf24l01* dev, bool activate)
{
    uint8_t reg = 0;
    if (nrf_read_register(dev, NRF_CONFIG, &reg) != NRF_OK) {
        return NRF_ERROR;
    }
    if (!activate) {
        reg |= 1 << MASK_TX_DS;
    } else {
        reg &= ~(1 << MASK_TX_DS);
    }
    if (nrf_write_register(dev, NRF_CONFIG, &reg) != NRF_OK) {
        return NRF_ERROR;
    }
    return NRF_OK;
}


/************************************************************************/

NRF_RESULT nrf_enable_max_retransmit_irq(nrf24l01* dev, bool activate)
{
    uint8_t reg = 0;
    if (nrf_read_register(dev, NRF_CONFIG, &reg) != NRF_OK) {
        return NRF_ERROR;
    }
    if (!activate) {
        reg |= 1 << MASK_MAX_RT;
    } else {
        reg &= ~(1 << MASK_MAX_RT);
    }
    if (nrf_write_register(dev, NRF_CONFIG, &reg) != NRF_OK) {
        return NRF_ERROR;
    }
    return NRF_OK;
}


/************************************************************************/

NRF_RESULT nrf_set_tx_address(nrf24l01* dev, uint8_t* address)
{
    GPIO_PinState state = nrf_ce_state(dev);
    ce_reset(dev);
    uint8_t rx[5];
    if (nrf_send_command(dev, W_REGISTER | TX_ADDR, address, rx,
                         5, &dev->status) != NRF_OK) {
        if (state == GPIO_PIN_SET) {
            ce_set(dev);
        }
        return NRF_ERROR;
    }
    dev->config.tx_address = address;
    if (state == GPIO_PIN_SET) {
        ce_set(dev);
    }
    return NRF_OK;
}


/************************************************************************/

NRF_RESULT nrf_set_rx_address(nrf24l01* dev, uint8_t pipe, uint8_t* address)
{
    GPIO_PinState state = nrf_ce_state(dev);
    ce_reset(dev);
    uint8_t rx[5];
    if (pipe > 5) {
        return NRF_INVALID_ARGUMENT;
    }
    else if (pipe < 2)
    {
        if (nrf_send_command(dev, W_REGISTER | (RX_ADDR_P0 + pipe), address, rx,
                             5, &dev->status) != NRF_OK) {
            if (state == GPIO_PIN_SET) {
                ce_set(dev);
            }
            return NRF_ERROR;
        }
    }
    else
    {
        if (nrf_send_command(dev, W_REGISTER | (RX_ADDR_P0 + pipe), address, rx,
                             1, &dev->status) != NRF_OK) {
            if (state == GPIO_PIN_SET) {
                ce_set(dev);
            }
            return NRF_ERROR;
        }
    }

    if (state == GPIO_PIN_SET) {
        ce_set(dev);
    }
    switch (pipe)
    {
    case 0:
        dev->config.pipe0_address = address;
        return NRF_OK;
    case 1:
        dev->config.pipe1_address = address;
        return NRF_OK;
    case 2:
        dev->config.pipe2_address = *address;
        return NRF_OK;
    case 3:
        dev->config.pipe3_address = *address;
        return NRF_OK;
    case 4:
        dev->config.pipe4_address = *address;
        return NRF_OK;
    case 5:
        dev->config.pipe5_address = *address;
        return NRF_OK;

    default:
        return NRF_INVALID_ARGUMENT;
    }
}


/************************************************************************/

NRF_RESULT nrf_set_rx_payload_width(nrf24l01* dev, uint8_t width)
{
    width &= 0x3F;

    for (uint8_t i = 0; i < 5; i++)
    {
        if (nrf_write_register(dev, RX_PW_P0 + i, &width) != NRF_OK) {
            dev->config.payload_length = 0;
            return NRF_ERROR;
        }
    }

    dev->config.payload_length = width;
    return NRF_OK;
}


/************************************************************************/

NRF_RESULT nrf_send_packet(nrf24l01* dev, const uint8_t* data)
{
    while (dev->tx_busy || dev->rx_busy);
    dev->tx_busy = true;

    ce_reset(dev);
    nrf_rx_tx_control(dev, NRF_STATE_TX);
    nrf_write_tx_payload(dev, data);
    ce_set(dev);

    uint32_t timeout = HAL_GetTick();
    timeout += NRF_TIMEOUT_MILIS;
    while (dev->tx_busy && (HAL_GetTick() < timeout)) {} // wait for end of transmittion

    if (dev->tx_busy) {
        dev->tx_busy = false;
        return NRF_TIMEOUT;
    } else {
        //dev->tx_busy = false;
        return dev->tx_result;
    }
}


/************************************************************************/

NRF_RESULT nrf_send_packet_noack(nrf24l01* dev, const uint8_t* data)
{
    while (dev->tx_busy || dev->rx_busy);
    dev->tx_busy = true;

    ce_reset(dev);
    nrf_rx_tx_control(dev, NRF_STATE_TX);
    nrf_write_tx_payload_noack(dev, data);
    ce_set(dev);

    while (dev->tx_busy) {} // wait for end of transmittion

    return dev->tx_result;
}


/************************************************************************/

uint8_t* nrf_receive_packet(nrf24l01* dev)
{

    while (dev->tx_busy || dev->rx_busy);
    dev->rx_busy = true;

    ce_reset(dev);
    nrf_rx_tx_control(dev, NRF_STATE_RX);
    ce_set(dev);

    while (dev->rx_busy) {} // wait for reception

    return dev->config.rx_buffer;
}


/************************************************************************/

NRF_RESULT nrf_push_packet(nrf24l01* dev, const uint8_t* data)
{
    if (dev->tx_busy) {
        nrf_flush_tx(dev);
    } else {
        dev->tx_busy;
    }

    ce_reset(dev);
    nrf_rx_tx_control(dev, NRF_STATE_TX);
    nrf_write_tx_payload(dev, data);
    ce_set(dev);

    return NRF_OK;
}


/************************************************************************/

