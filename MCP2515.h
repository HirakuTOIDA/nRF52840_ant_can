#ifndef MCP2515_H
#define MCP2515_H

#include "nrf_drv_spi.h"
#include <stdint.h>

// registor map
#define MCP2515_BFPCTRL (0x0C)
#define MCP2515_TXRTSCTRL (0x0D)
#define MCP2515_CANSTAT (0x0E)
#define MCP2515_CANCTRL (0x0F)
#define MCP2515_TEC (0x1C)
#define MCP2515_REC (0x1D)
#define MCP2515_CNF3 (0x28)
#define MCP2515_CNF2 (0x29)
#define MCP2515_CNF1 (0x2A)
#define MCP2515_CANINTE (0x2B)
#define MCP2515_CANINTF (0x2C)
#define MCP2515_EFLG (0x2D)
#define MCP2515_TXB0CTRL (0x30)
#define MCP2515_TXB1CTRL (0x40)
#define MCP2515_TXB2CTRL (0x50)
#define MCP2515_RXB0CTRL (0x60)
#define MCP2515_RXB1CTRL (0x70)

void MCP2515_Reset(const nrf_drv_spi_t* spi);
void MCP2515_Read(const nrf_drv_spi_t* spi, uint8_t address, uint8_t *rx_buf, uint8_t rx_buf_length);
void MCP2515_Read_RX_buffer(const nrf_drv_spi_t* spi, uint8_t address_pointer, uint8_t *rx_buf, uint8_t rx_buf_length);
void MCP2515_Write(const nrf_drv_spi_t* spi, uint8_t address, uint8_t data, uint8_t data_length);
void MCP2515_Load_TX_buffer(const nrf_drv_spi_t* spi, uint8_t address_pointer, uint8_t *data, uint8_t data_length);
void MCP2515_Request_to_send(const nrf_drv_spi_t* spi, uint8_t buffer_number);
uint8_t MCP2515_Read_status(const nrf_drv_spi_t* spi);
uint8_t MCP2515_RX_status(const nrf_drv_spi_t* spi);
void MCP2515_Bit_modify(const nrf_drv_spi_t* spi, uint8_t address, uint8_t mask, uint8_t data);

//
#define MCP2515_Normal_mode (0x00)
#define MCP2515_Sleep_mode (0x20)
#define MCP2515_Loopback_mode (0x40)
#define MCP2515_Listen_only_mode (0x60)
#define MCP2515_Configuration_mode (0x80)
void MCP2515_set_mode(const nrf_drv_spi_t* spi, uint8_t mode);

//
#define MCP2515_SJW_4 (0xC0)
#define MCP2515_SJW_3 (0x80)
#define MCP2515_SJW_2 (0x40)
#define MCP2515_SJW_1 (0x00)

void MCP2515_init(const nrf_drv_spi_t* spi);

#endif
