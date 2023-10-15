#ifndef BOARD_DEF_H
#define BOARD_DEF_H

/* This file contains defines specific to the board that is being used. To
 * extend this, the base SAMD11 configuration can be bracketed in IFDEF
 */

/* Board identification number. If a custom board is used, this should be
 * added to "dbgPutBoard" in emon32.c
  */
#define BOARD_ID_LC         0
#define BOARD_ID_STANDARD   1
#define BOARD_ID_EMONPI     2
#define BOARD_ID_DEV        255
/* LC */
#define BOARD_ID            BOARD_ID_EMONPI

/* Online configuration takes around 2.5 KB of flash. Comment out this define to
 * save space, with only RO access to configuration values.
 */
#define CONFIGURATION_RW

/* Clock frequencies
 *  - Core is on the 48 MHz DFLL
 *  - Peripherals are on the OSC8M / 8 -> 1 MHz
 */
#define F_CORE              48000000ul
#define F_PERIPH            8000000ul
#define F_TIMER1            F_PERIPH / 8
#define F_TIMER2            F_PERIPH / 8

/* Oversampling
 * There will be a fixed anti-aliasing filter on the board. This should be the
 * target oversampling ratio at the ADC
 */
#define OVERSAMPLING_RATIO  2

/* Temperature sensors
 * This is the maximum number of OneWire DS18B20 sensors that can be used
 */
#define TEMP_MAX_ONEWIRE    8u

/* EEPROM */
/* Top of EEPROM address, not including R/W bit */
#define EEPROM_BASE_ADDR    0x50
/* Maximum number of bytes in a single page */
#define EEPROM_PAGE_SIZE    16u
/* Worst case EEPROM write time (microseconds) */
#define EEPROM_WR_TIME      5000ul
/* Size (bytes) of EEPROM */
#define EEPROM_SIZE_BYTES   512u
/* Size (bytes) of wear levelled portion */
#define EEPROM_WL_SIZE      384u
/* Offset of wear levelled area */
#define EEPROM_WL_OFFSET    (EEPROM_SIZE_BYTES - EEPROM_WL_SIZE)

/* SERCOM peripheral defines */
#if (BOARD_ID == BOARD_ID_LC)

#define SERCOM_UART_DBG             SERCOM0
#define SERCOM_I2CM                 SERCOM1
#define SERCOM_UART_DATA            SERCOM2
#define SERCOM_SPI_DATA             SERCOM2

#define SERCOM_UART_DBG_APBCMASK    PM_APBCMASK_SERCOM0
#define SERCOM_I2CM_APBCMASK        PM_APBCMASK_SERCOM1
#define SERCOM_UART_DATA_APBCMASK   PM_APBCMASK_SERCOM2

#define SERCOM_UART_DBG_GCLK_ID     SERCOM0_GCLK_ID_CORE
#define SERCOM_I2CM_GCLK_ID         SERCOM1_GCLK_ID_CORE
#define SERCOM_UART_DATA_GCLK_ID    SERCOM2_GCLK_ID_CORE

#define SERCOM_UART_DBG_DMAC_ID_TX  SERCOM0_DMAC_ID_TX
#define SERCOM_I2CM_DMAC_ID_TX      SERCOM1_DMAC_ID_TX
#define SERCOM_I2CM_DMAC_ID_RX      SERCOM1_DMAC_ID_RX
#define SERCOM_UART_DATA_DMAC_ID_TX SERCOM2_DMAC_ID_TX

#define SERCOM_UART_DBG_NVIC_IRQn   SERCOM0_IRQn

#elif (BOARD_ID == BOARD_ID_EMONPI)

#define SERCOM_UART_DBG             SERCOM2
#define SERCOM_I2CM                 SERCOM3
#define SERCOM_UART_DATA            SERCOM2
#define SERCOM_SPI_DATA             SERCOM4
#define SERCOM_I2CM_EXT             SERCOM5

#define SERCOM_UART_DBG_APBCMASK    PM_APBCMASK_SERCOM2
#define SERCOM_I2CM_APBCMASK        PM_APBCMASK_SERCOM3
#define SERCOM_SPI_APBCMASK         PM_APBCMASK_SERCOM4
#define SERCOM_UART_DATA_APBCMASK   PM_APBCMASK_SERCOM2

#define SERCOM_UART_DBG_GCLK_ID     SERCOM2_GCLK_ID_CORE
#define SERCOM_I2CM_GCLK_ID         SERCOM3_GCLK_ID_CORE
#define SERCOM_UART_DATA_GCLK_ID    SERCOM2_GCLK_ID_CORE
#define SERCOM_SPI_GCLK_ID          SERCOM4_GCLK_ID_CORE

#define SERCOM_UART_DBG_DMAC_ID_TX  SERCOM2_DMAC_ID_TX
#define SERCOM_I2CM_DMAC_ID_TX      SERCOM3_DMAC_ID_TX
#define SERCOM_I2CM_DMAC_ID_RX      SERCOM3_DMAC_ID_RX
#define SERCOM_UART_DATA_DMAC_ID_TX SERCOM2_DMAC_ID_TX

#define PMUX_UART_DBG               PORT_PMUX_PMUXE_C
#define PMUX_UART_DATA              PORT_PMUX_PMUXE_C

#define SERCOM_UART_DBG_NVIC_IRQn   SERCOM2_IRQn
#define SERCOM_I2CM_NVIC_IRQn       SERCOM3_IRQn

/* TIMER1 drives ADC conversion tick, TIMER2 used for delay timing */
#define TIMER1                      TC3
#define TIMER2                      TC4

#define TIMER1_GCLK_ID              TC3_GCLK_ID
#define TIMER1_APBCMASK             PM_APBCMASK_TC3
#define TIMER1_EVT_SRC              EVSYS_ID_GEN_TC3_OVF

#define TIMER2_GCLK_ID              TC4_GCLK_ID
#define TIMER2_APBCMASK             PM_APBCMASK_TC4
#define TIMER2_IRQn                 TC4_IRQn

#endif /* BOARD_ID */

/* Pulse counting */
#define PULSE_MIN_PERIOD_MS         100u    /* Minimum period between pulses */
#define PULSE_EIC_MAP               3u
#define PULSE_EIC_FILTER            EIC_CONFIG_FILTEN3
#define PULSE_EIC_RISING            EIC_CONFIG_SENSE3_RISE
#define PULSE_EIC_INTFLAG           EIC_INTFLAG_EXTINT3
#define PULSE_EIC_INTENSET          EIC_INTENSET_EXTINT3
#define PULSE_EIC_INTENCLR          EIC_INTENCLR_EXTINT3

/* Pin assignments (nb. logical, not physical) */
#if (BOARD_ID == BOARD_ID_LC)

#define GRP_PIN             0u
#define GRP_LED_STATUS      0u
#define PIN_LED_STATUS      16u
#define PIN_LED_PROG        17u

/* SPI related defines */
#define GRP_SERCOM_SPI      0u
#define PIN_SPI_MISO        14u
#define PIN_SPI_MOSI        15u
#define PIN_SPI_SCK         16u
#define PIN_SPI_RFM_SS      17u
#define SPI_DATA_BAUD       4000000ul
#define PMUX_SPI_DATA       PORT_PMUX_PMUXE_D

/* I2C related defines */
#define GRP_SERCOM_I2C      0u
#define PIN_I2C_SDA         22u
#define PIN_I2C_SCL         23u

/* Data UART related defines */
#define GRP_SERCOM_UART_DATA 0u
#define PIN_UART_DATA_RX    21u
#define PIN_UART_DATA_TX    22u
#define UART_DATA_PAD_RX    3u
#define UART_DATA_PAD_TX    1u
#define UART_DATA_BAUD      115200u

/* DMA defines */
#define NUM_CHAN_DMA        4u
#define DMA_CHAN_UART_DATA  3u
#define DMA_CHAN_I2CM       2u
#define DMA_CHAN_UART_DBG   1u
#define DMA_CHAN_ADC        0u

#elif (BOARD_ID == BOARD_ID_EMONPI)

#define GRP_PINA            0u
#define GRP_PINB            1u
#define GRP_LED_STATUS      1u
#define PIN_LED_STATUS      23u
#define PIN_LED_PROG        27u

/* Debug UART related defines */
#define GRP_SERCOM_UART_DBG 0u
#define PIN_UART_DBG_RX     13u
#define PIN_UART_DBG_TX     12u
#define UART_DBG_PAD_RX     1u
#define UART_DBG_PAD_TX     2u
#define UART_DBG_BAUD       38400u

/* SPI related defines */
#define GRP_SERCOM_SPI      0u
#define PIN_SPI_MISO        14u
#define PIN_SPI_MOSI        15u
#define PIN_SPI_SCK         16u
#define PIN_SPI_RFM_SS      17u
#define SPI_DATA_BAUD       4000000ul
#define PMUX_SPI_DATA       PORT_PMUX_PMUXE_D

/* I2C related defines */
#define GRP_SERCOM_I2C      1u
#define PIN_I2C_SDA         12u
#define PIN_I2C_SCL         13u
#define PMUX_I2CM           PORT_PMUX_PMUXE_C

/* Data UART related defines */
#define GRP_SERCOM_UART_DATA 0u
#define PIN_UART_DATA_RX    21u
#define PIN_UART_DATA_TX    22u
#define UART_DATA_PAD_RX    3u
#define UART_DATA_PAD_TX    1u
#define UART_DATA_BAUD      115200u

/* DMA defines */
#define NUM_CHAN_DMA        4u
#define DMA_CHAN_UART_DATA  3u
#define DMA_CHAN_I2CM       2u
#define DMA_CHAN_UART_DBG   1u
#define DMA_CHAN_ADC        0u

#endif /* BOARD_ID */

#endif
