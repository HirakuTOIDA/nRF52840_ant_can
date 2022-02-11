#include "MCP2515.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

/**
* Low level functions
*/

/**
* @brief Reset
* @param[in] spi spi handler
* @memo fixed length, 1 byte
*/
void MCP2515_Reset(const nrf_drv_spi_t* spi){
  uint8_t m_tx_buf[] = {0xC0};
  uint8_t m_length = sizeof(m_tx_buf);
  APP_ERROR_CHECK(nrf_drv_spi_transfer(spi, m_tx_buf, m_length, NULL, 0));
}

/**
* @brief Read
* @param[in] spi spi handler
* @param[in] address resistor address for reading start
* @param[in] rx_buf initial address of rx_buf
* @param[in] rx_buf_length byte length to be tranfered to rx_buf
* @memo variable length
*/
void MCP2515_Read(const nrf_drv_spi_t* spi, uint8_t address, uint8_t *rx_buf, uint8_t rx_buf_length){
  uint8_t m_tx_buf[] = {0x03, address};
  uint8_t m_length = sizeof(m_tx_buf);
  APP_ERROR_CHECK(nrf_drv_spi_transfer(spi, m_tx_buf, m_length, rx_buf, rx_buf_length));
  //NRF_LOG_HEXDUMP_INFO(rx_buf, rx_buf_length);
}

/**
* @brief Read RX Buffer
* @param[in] spi spi handler
* @param[in] address_pointer 1001 0nm0
* @param[in] rx_buf initial address of rx_buf
* @param[in] rx_buf_length byte length to be tranfered to rx_buf
* @memo variable length
*/
void MCP2515_Read_RX_buffer(const nrf_drv_spi_t* spi, uint8_t address_pointer, uint8_t *rx_buf, uint8_t rx_buf_length){
  address_pointer &= 0x06;
  uint8_t m_tx_buf[] = {0x90 | address_pointer};
  uint8_t m_length = sizeof(m_tx_buf);
  APP_ERROR_CHECK(nrf_drv_spi_transfer(spi, m_tx_buf, m_length, rx_buf, rx_buf_length));
}

// Write
// @todo
// variable length
void MCP2515_Write(const nrf_drv_spi_t* spi, uint8_t address, uint8_t data, uint8_t data_length){
  uint8_t m_tx_buf[] = {0x02, address, data};
  uint8_t m_length = 3;
  APP_ERROR_CHECK(nrf_drv_spi_transfer(spi, m_tx_buf, m_length, NULL, 0));
}

// Load_TX_buffer
// @todo
// variable length
void MCP2515_Load_TX_buffer(const nrf_drv_spi_t* spi, uint8_t address_pointer, uint8_t *data, uint8_t data_length){
  address_pointer &= 0x07;
  uint8_t m_tx_buf[] = {0x40 | address_pointer, data};
  uint8_t m_length = sizeof(m_tx_buf);
  APP_ERROR_CHECK(nrf_drv_spi_transfer(spi, m_tx_buf, m_length, NULL, 0));
}

/**
* @brief Request_to_send
* @param[in] spi spi handler
* @param[in] buffer_number 1000 0nnn
* @memo fixed length, 1 byte
*/
void MCP2515_Request_to_send(const nrf_drv_spi_t* spi, uint8_t buffer_number){
  buffer_number &= 0x07;
  uint8_t m_tx_buf[] = {0x80 | buffer_number};
  uint8_t m_length = sizeof(m_tx_buf);
  APP_ERROR_CHECK(nrf_drv_spi_transfer(spi, m_tx_buf, m_length, NULL, 0));
}

/**
* @brief Read_status
* @param[in] spi spi handler
* @memo fixed length, 1 byte
*/
uint8_t MCP2515_Read_status(const nrf_drv_spi_t* spi){
  uint8_t m_tx_buf[] = {0xA0};
  uint8_t m_length = sizeof(m_tx_buf);
  uint8_t m_rx_buf[3];
  APP_ERROR_CHECK(nrf_drv_spi_transfer(spi, m_tx_buf, m_length, m_rx_buf, m_length));
  return m_rx_buf[2];
}

/**
* @brief RX_status
* @param[in] spi spi handler
* @memo fixed length, 1 byte
*/
uint8_t MCP2515_RX_status(const nrf_drv_spi_t* spi){
  uint8_t m_tx_buf[] = {0xB0};
  uint8_t m_length = sizeof(m_tx_buf);
  uint8_t m_rx_buf[3];
  APP_ERROR_CHECK(nrf_drv_spi_transfer(spi, m_tx_buf, m_length, m_rx_buf, m_length));
  return m_rx_buf[2];
}

/**
* @brief Bit_modify
* @param[in] spi spi handler
* @param[in] address address byte
* @param[in] mask mask byte
* @param[in] data data byte
* @memo fixed length, 4 bytes
*/
void MCP2515_Bit_modify(const nrf_drv_spi_t* spi, uint8_t address, uint8_t mask, uint8_t data){
  uint8_t m_tx_buf[] = {0x05, address, mask, data};
  uint8_t m_length = sizeof(m_tx_buf);
  APP_ERROR_CHECK(nrf_drv_spi_transfer(spi, m_tx_buf, m_length, NULL, 0));
}

/**
* Resistor access functions
*/

/**
* CANCTRL: CAN CONTROL REGISTER (ADDRESS: XFh)
* REQOP, ABAT, OSM, CLKEN, CLKPRE
*/

/**
* @brief mode set
* @param[in] spi spi handler
* @param[in] mode Request Operation Mode bits
*/
void MCP2515_set_mode(const nrf_drv_spi_t* spi, uint8_t mode){
  MCP2515_Bit_modify(spi, MCP2515_CANCTRL, 0xE0, mode);
}

/**
* CNF1: CONFIGURATION REGISTER 1 (ADDRESS: 2Ah)
* SJW, BRP
*/

/**
* @brief 
* @param[in] spi spi handler
* @param[in] sjw Synchronization Jump Width Length bits
* @memo
*/
void MCP2515_set_SJW(const nrf_drv_spi_t* spi, uint8_t sjw){
  sjw --;
  sjw = sjw << 6;
  MCP2515_Bit_modify(spi, MCP2515_CNF1, 0xC0, sjw);
}

/**
* @brief 
* @param[in] spi spi handler
* @param[in] brp
* @memo T_Q = 2 * (brp + 1) / F_OSC
*/
void MCP2515_set_BRP(const nrf_drv_spi_t* spi, uint8_t brp){
  MCP2515_Bit_modify(spi, MCP2515_CNF1, 0x3F, brp);
}

/**
* CNF2: CONFIGURATION REGISTER 2 (ADDRESS: 29h)
* BTLMODE, SAM, PHSEG1, PRSEG
*/

/**
* @brief 
* @param[in] 
* @param[in] 
* @param[in] 
* @param[in] 
* @memo CNF2
*/
void MCP2515_set_BTLMODE(const nrf_drv_spi_t* spi, uint8_t btlmode){
  btlmode = btlmode << 7;
  MCP2515_Bit_modify(spi, MCP2515_CNF2, 0x80, btlmode);
}

/**
* @brief 
* @param[in] 
* @param[in] 
* @param[in] 
* @param[in] 
* @memo
*/
void MCP2515_set_SAM(const nrf_drv_spi_t* spi, uint8_t sam){
  sam = sam << 6;
  MCP2515_Bit_modify(spi, MCP2515_CNF2, 0x40, sam);
}

/**
* @brief 
* @param[in] 
* @param[in] 
* @param[in] 
* @param[in] 
* @memo
*/
void MCP2515_set_PHSEG1(const nrf_drv_spi_t* spi, uint8_t phseg1){
  phseg1 --;
  phseg1 = phseg1 << 3;
  MCP2515_Bit_modify(spi, MCP2515_CNF2, 0x38, phseg1);
}

/**
* @brief 
* @param[in] 
* @param[in] 
* @param[in] 
* @param[in] 
* @memo
*/
void MCP2515_set_PRSEG(const nrf_drv_spi_t* spi, uint8_t prseg){
  prseg --;
  MCP2515_Bit_modify(spi, MCP2515_CNF2, 0x07, prseg);
}

/**
* CNF3: CONFIGURATION REGISTER 3 (ADDRESS: 28h)
* SOF, WAKFIL, PHSEG2
*/

/**
* @brief 
* @param[in] 
* @param[in] 
* @param[in] 
* @param[in] 
* @memo
*/
void MCP2515_set_SOF(const nrf_drv_spi_t* spi, uint8_t sof){
  sof = sof << 7;
  MCP2515_Bit_modify(spi, MCP2515_CNF3, 0x80, sof);
}

/**
* @brief 
* @param[in] 
* @param[in] 
* @param[in] 
* @param[in] 
* @memo
*/
void MCP2515_set_WAKFIL(const nrf_drv_spi_t* spi, uint8_t wakfil){
  wakfil = wakfil << 6;
  MCP2515_Bit_modify(spi, MCP2515_CNF3, 0x40, wakfil);
}

/**
* @brief 
* @param[in] 
* @param[in] 
* @param[in] 
* @param[in] 
* @memo
*/
void MCP2515_set_PHSEG2(const nrf_drv_spi_t* spi, uint8_t phseg2){
  phseg2 --;
  MCP2515_Bit_modify(spi, MCP2515_CNF3, 0x07, phseg2);
}

/**
 * High level functions
 */
 // @memo SYNC [1TQ] + PropSeg + PhaseSeg1 + PhaseSeg2 = NBT
void MCP2515_init(const nrf_drv_spi_t* spi){
  uint8_t m_rx_buf[3];
  uint8_t reg_before;

  NRF_LOG_INFO("----- Initialize MCP2515 -----");
  NRF_LOG_FLUSH();

  MCP2515_Reset(spi);

  // wait?
  // After reset: Configuration Mode
  MCP2515_Read(spi, MCP2515_CANCTRL, m_rx_buf, sizeof(m_rx_buf));
  NRF_LOG_INFO("CANCTRL: 0x%02x", m_rx_buf[2]);
  NRF_LOG_FLUSH();

  // CNF1
  MCP2515_Read(spi, MCP2515_CNF1, m_rx_buf, sizeof(m_rx_buf));
  reg_before = m_rx_buf[2];
  MCP2515_set_SJW(spi, 4); // OK, PS2 > SJW
  MCP2515_set_BRP(spi, 5); // OK, T_Q = 2 * (5 + 1) / 24 MHz = 500 ns
  MCP2515_Read(spi, MCP2515_CNF1, m_rx_buf, sizeof(m_rx_buf));
  NRF_LOG_INFO("CNF1: 0x%02x -> 0x%02x", reg_before, m_rx_buf[2]);
  NRF_LOG_FLUSH();

  // CNF2
  MCP2515_Read(spi, MCP2515_CNF2, m_rx_buf, sizeof(m_rx_buf));
  reg_before = m_rx_buf[2];
  MCP2515_set_BTLMODE(spi, 1);
  MCP2515_set_SAM(spi, 1);
  MCP2515_set_PHSEG1(spi, 7); // OK
  MCP2515_set_PRSEG(spi, 2); // OK, PRSEG + PS1 >= PS2, PRSEG + PS1 >= T_DELAY
  MCP2515_Read(spi, MCP2515_CNF2, m_rx_buf, sizeof(m_rx_buf));
  NRF_LOG_INFO("CNF2: 0x%02x -> 0x%02x", reg_before, m_rx_buf[2]);
  NRF_LOG_FLUSH();

  // CNF3
  MCP2515_Read(spi, MCP2515_CNF3, m_rx_buf, sizeof(m_rx_buf));
  reg_before = m_rx_buf[2];
  MCP2515_set_SOF(spi, 1);
  MCP2515_set_WAKFIL(spi, 0);
  MCP2515_set_PHSEG2(spi, 6); // OK
  MCP2515_Read(spi, MCP2515_CNF3, m_rx_buf, sizeof(m_rx_buf));
  NRF_LOG_INFO("CNF3: 0x%02x -> 0x%02x", reg_before, m_rx_buf[2]);
  NRF_LOG_FLUSH();

  // TXRTSCTRL, not used for initial test 

  // Filter resistors, not used for initial test

  // Mask resistors, not used for initial test

  MCP2515_set_mode(spi, MCP2515_Normal_mode);
  MCP2515_Read(spi, MCP2515_CANCTRL, m_rx_buf, sizeof(m_rx_buf));
  NRF_LOG_INFO("CANCTRL: 0x%02x", m_rx_buf[2]);
  NRF_LOG_FLUSH();
  NRF_LOG_INFO("----- Initialize MCP2515 -----");
  NRF_LOG_FLUSH();
}