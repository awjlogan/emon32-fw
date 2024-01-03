#ifndef DRIVER_SERCOM_H
#define DRIVER_SERCOM_H

#include <stdint.h>

#include "emon32_samd.h"
#include "driver_DMAC.h"

typedef enum {
    I2CM_ACK    = 0u,
    I2CM_NACK   = 1u
} I2CM_Ack_t;

typedef enum {
    I2CM_ACK_CMD_NONE       = 0u,
    I2CM_ACK_CMD_START      = 1u,
    I2CM_ACK_CMD_CONTINUE   = 2u,
    I2CM_ACK_CMD_STOP       = 3u
} I2CM_AckCmd_t;

typedef enum {
    UART_BAUD_9600      = 9600,
    UART_BAUD_19200     = 19200,
    UART_BAUD_28800     = 28800,
    UART_BAUD_38400     = 38400,
    UART_BAUD_57600     = 57600,
    UART_BAUD_76800     = 76800,
    UART_BAUD_115200    = 115200
} UART_BAUD_t;

typedef struct UART_Cfg_ {
    Sercom       *sercom;
    UART_BAUD_t baud;
    int         apbc_mask;
    int         gclk_id;
    int         gclk_gen;
    int         pad_tx;
    int         pad_rx;
    int         port_grp;
    int         pin_tx;
    int         pin_rx;
    int         pmux;
    int         dmaChannel;
    DMACCfgCh_t dmaCfg;
} UART_Cfg_t;

/*! @brief configure the serial communication module. This function starts the
 *         debug UART and I2C modules. Further SPI and UART modules are
 *         configured separately
 */
void sercomSetup();

/*! @brief Configure a SERCOM module for SPI */
void sercomSetupSPI();

/*! @brief Configure a SERCOM module for UART functions.
 *  @param [in] pCfg : pointer to configuration struct
 */
void sercomSetupUART(const UART_Cfg_t *pCfg);

/*! @brief Configure the DMA for non-blocking transactions
 */
void uartConfigureDMA();

/*! @brief Get a character from the USART data buffer. Only valid when the
 *         INTFLAG.RXC bit it set.
 *  @param [in] sercom : SERCOM instance
 */
char uartGetc(const Sercom *sercom);

/*! @brief Clear the interrupt status for the UART instance
 *  @param [in] sercom : SERCOM instance
 *  @param [in] interrupt : interrupt to clear
 */
void uartInterruptClear(Sercom *sercom, uint32_t interrupt);

/*! @brief Disable the an interrupt for the UART instance
 *  @param [in] sercom : SERCOM instance
 *  @param [in] interrupt : interrupt to disable
 */
void uartInterruptDisable(Sercom *sercom, uint32_t interrupt);

/*! @brief Enable the an interrupt for the UART instance
 *  @param [in] sercom : SERCOM instance
 *  @param [in] interrupt : interrupt to enable
 */
void uartInterruptEnable(Sercom *sercom, uint32_t interrupt);

/*! @brief Return the interrupt status for the UART instance
 *  @param [in] sercom : SERCOM instance
 */
uint32_t uartInterruptStatus(const Sercom *sercom);

/*! @brief Send a single character (blocking) on UART
 *  @param [in] sercom : pointer to the SERCOM instance
 *  @param [in] c : Single character
 */
void uartPutcBlocking(Sercom *sercom, char c);

/*! @brief Send a string (blocking) on UART
 *  @param [in] sercom : pointer to the SERCOM instance
 *  @param [in] s : Pointer to null terminated string
 */
void uartPutsBlocking(Sercom *sercom, const char *s);

/*! @brief Send a string (non-blocking) on UART by DMA
 *  @param [in] dma_chan : DMA channel to send on
 *  @param [in] s : Pointer to the string
 *  @param [in] len : Length of the string (not including NULL)
 */
void uartPutsNonBlocking(unsigned int dma_chan, const char * const s, uint16_t len);

/*! @brief Set up a UART SERCOM to be an interactive device. Characters will
 *         be echoed to the RX channel, and passed to a callback function.
 *  @param [in] sercom : pointer to the SERCOM instance§
 */
void uartSetInteractive(Sercom *sercom);

/*! @brief Set I2C address. If dma is 1, then a packet of len bytes is sent
 *         or received.
 *  @param [in] sercom : SERCOM instance
 *  @param [in] addr : address and RW bit
 */
void i2cActivate(Sercom *sercom, uint8_t addr);

/*! @brief Requester acknowledge command
 *  @param [in] sercom : SERCOM instance
 *  @param [in] ack : 0: ACK, 1: NACK
 *  @param [in] cmd : command
 */
void i2cAck(Sercom *sercom, I2CM_Ack_t ack, I2CM_AckCmd_t cmd);

/*! @brief Write to completer
 *  @param [in] sercom : SERCOM instance
 *  @param [in] data : data byte
 */
void i2cDataWrite(Sercom *sercom, uint8_t data);

/*! @brief Read byte from I2C completer
 *  @param [in] : sercom : SERCOM instance
 *  @return : read data
 */
uint8_t i2cDataRead(Sercom *sercom);

/*! @brief Read a byte from a configured SPI channel
 *  @param [in] sercom : SERCOM instance
 *  @param [in] addr : address to read from
 *  @return : the byte that has been read
 */
uint8_t spiReadByte(Sercom *sercom, const uint8_t addr);

/*! @brief Write a set of bytes to a configured SPI channel.
 *  @param [in] sercom : SERCOM instance
 *  @param [in] pBuf : pointer to data buffer
 *  @param [in] n : size of data buffer
 */
void spiWriteBuffer(Sercom *sercom, const void *pBuf, const unsigned int n);

/*! @brief Write a byte to a configured SPI channel. Blocks until transfer
 *         is complete.
 *  @param [in] sercom : SERCOM instance
 *  @param [in] addr : address to write to
 *  @param [in] data : data to write
 */
void spiWriteByte(Sercom *sercom, const uint8_t addr, const uint8_t data);

#endif
