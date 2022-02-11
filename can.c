#include "nrf.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "MCP2515.h"

extern nrf_drv_spi_t spi;

void can_send(uint8_t *dat, uint8_t dat_size)
{
  static const uint8_t sylphide_page = 'T';
  uint8_t m_rx_buf[3];    /**< RX buffer. */
  uint8_t MsgBuf[14];
  uint16_t msg = (uint16_t)(sylphide_page - 0x41) << 2; // Standard identifier (11 bits), O page: 0x0E->0x38, L page: 0x0B->0x2C, T page: 0x54->0x4C
  NRF_LOG_INFO("Page %c, SID: 0x%04x", sylphide_page, msg);
  NRF_LOG_FLUSH();
  MsgBuf[0] = 0x08; // TXBnCTRL
  MsgBuf[1] = (uint8_t)(msg >> 3); // TXBnSIDH
  MsgBuf[2] = (uint8_t)(msg << 5); // TXBnSIDL
  MsgBuf[3] = 0; // TXBnEID8
  MsgBuf[4] = 0; // TXBnEID0
  MsgBuf[5] = 0x08; // TXBnDLC, Data Length Code bits
  memcpy(MsgBuf + 6, dat, 8);
  for (uint8_t i = 1; i < 14; i ++) {
    MCP2515_Write(&spi, MCP2515_TXB0CTRL + i, MsgBuf[i], 1);
  }
  MCP2515_Write(&spi, MCP2515_TXB0CTRL, MsgBuf[0], 1);
  // wait for send.
  bool txreq = true;
  bool txerr = false;
  bool mloa = false;
  bool abtf = false;

  uint8_t count = 100; //1000 is too many.
  while (count){
    count --;
    MCP2515_Read(&spi, MCP2515_TXB0CTRL, m_rx_buf, sizeof(m_rx_buf));
    uint8_t TXB0CTRL = m_rx_buf[2];
    txreq = (TXB0CTRL & 0x08) >> 3;
    txerr = (TXB0CTRL & 0x10) >> 4;
    mloa = (TXB0CTRL & 0x20) >> 5;
    abtf = (TXB0CTRL & 0x40) >> 6;
    if(txreq == false){
      //bsp_board_led_invert(BSP_BOARD_LED_3);
      //bsp_board_led_off(BSP_BOARD_LED_0);
      break;
    }
    if(txerr == true){
      //bsp_board_led_invert(BSP_BOARD_LED_0);
      //bsp_board_led_off(BSP_BOARD_LED_3);
      break;
    }
    if(mloa == true){
      //bsp_board_led_invert(BSP_BOARD_LED_0);
      //bsp_board_led_off(BSP_BOARD_LED_3);
      break;
    }
    if(abtf == true){
      //bsp_board_led_invert(BSP_BOARD_LED_0);
      //bsp_board_led_off(BSP_BOARD_LED_3);
      break;
    }
    if(count == 0){
      //bsp_board_led_invert(BSP_BOARD_LED_0);
      //bsp_board_led_off(BSP_BOARD_LED_3);
      break;
    }
  }
  MCP2515_Read(&spi, MCP2515_TXB0CTRL, m_rx_buf, sizeof(m_rx_buf));
  uint8_t TXB0CTRL = m_rx_buf[2];
  MCP2515_Read(&spi, MCP2515_CANCTRL, m_rx_buf, sizeof(m_rx_buf));
  uint8_t CANCTRL = m_rx_buf[2];
  MCP2515_Read(&spi, MCP2515_CANINTF, m_rx_buf, sizeof(m_rx_buf));
  uint8_t CANINTF = m_rx_buf[2];
  MCP2515_Read(&spi, MCP2515_EFLG, m_rx_buf, sizeof(m_rx_buf));
  uint8_t EFLG = m_rx_buf[2];
  MCP2515_Read(&spi, MCP2515_TEC, m_rx_buf, sizeof(m_rx_buf));
  uint8_t TEC = m_rx_buf[2];
  NRF_LOG_INFO("TXB0CTRL: 0x%02x CANCTRL: 0x%02x CANINTF: 0x%02x EFLG: 0x%02x TEC: 0x%02x", TXB0CTRL, CANCTRL, CANINTF, EFLG, TEC);
  NRF_LOG_FLUSH();
  //bsp_board_led_invert(BSP_BOARD_LED_2);
}