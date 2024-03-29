#pragma once

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
#define BOARD_ID            BOARD_ID_EMONPI

/* Clock frequencies
 *  - Core is on the 48 MHz DFLL
 *  - Peripherals are on the OSC8M / 8 -> 1 MHz
 */
#define F_CORE              48000000ul
#define F_PERIPH            8000000ul
#define F_TIMER_ADC         F_PERIPH / 8
#define F_TIMER2            F_PERIPH / 8

#define NUM_V               3u
#define NUM_CT              11u
#define VCT_TOTAL           (NUM_V + NUM_CT)
#define NUM_CT_ACTIVE_DEF   6u          /* Onboard CTs only*/
#define SAMPLE_RATE         4800u
#define SAMPLES_IN_SET      2u
#define SAMPLE_BUF_DEPTH    2u
#define DOWNSAMPLE_TAPS     19u

/* If there is hardware support of zero-crossing detection, set 1*/
#define ZEROX_HW_SPT        0
#define GRP_ZEROX           0
#define PIN_ZEROX           27u
#define EIC_INTEN_ZEROX     EIC_INTENSET_EXTINT15
#define EIC_INTDIS_ZEROX    EIC_INTENCLR_EXTINT15
#define EIC_INTFLG_ZEROX    EIC_INTFLAG_EXTINT15

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
/* Size of configuration area */
#define EEPROM_CONFIG_SIZE  256
/* Offset of wear levelled area */
#define EEPROM_WL_OFFSET    (EEPROM_CONFIG_SIZE)


/* Serial Communication Instances */

#define SERCOM_UART_DBG             SERCOM2
#define SERCOM_I2CM                 SERCOM3
#define SERCOM_UART_DATA            SERCOM2
#define SERCOM_SPI_DATA             SERCOM4
#define SERCOM_I2CM_EXT             SERCOM5

#define SERCOM_UART_DBG_APBCMASK    PM_APBCMASK_SERCOM2
#define SERCOM_UART_DATA_APBCMASK   PM_APBCMASK_SERCOM2
#define SERCOM_I2CM_INT_APBCMASK    PM_APBCMASK_SERCOM3
#define SERCOM_SPI_APBCMASK         PM_APBCMASK_SERCOM4
#define SERCOM_I2CM_EXT_APBCMASK    PM_APBCMASK_SERCOM5

#define SERCOM_UART_DBG_GCLK_ID     SERCOM2_GCLK_ID_CORE
#define SERCOM_UART_DATA_GCLK_ID    SERCOM2_GCLK_ID_CORE
#define SERCOM_I2CM_INT_GCLK_ID     SERCOM3_GCLK_ID_CORE
#define SERCOM_SPI_GCLK_ID          SERCOM4_GCLK_ID_CORE
#define SERCOM_I2CM_EXT_GCLK_ID     SERCOM5_GCLK_ID_CORE

#define SERCOM_UART_DBG_DMAC_ID_TX  SERCOM2_DMAC_ID_TX
#define SERCOM_I2CM_DMAC_ID_TX      SERCOM3_DMAC_ID_TX
#define SERCOM_I2CM_DMAC_ID_RX      SERCOM3_DMAC_ID_RX
#define SERCOM_UART_DATA_DMAC_ID_TX SERCOM2_DMAC_ID_TX

#define PMUX_UART_DBG0              PORT_PMUX_PMUXE_C
#define PMUX_UART_DBG1              PORT_PMUX_PMUXE_D
#define PMUX_UART_DATA              PORT_PMUX_PMUXE_C

#define SERCOM_UART_INTERACTIVE_HANDLER irq_handler_sercom2()
#define SERCOM_UART_INTERACTIVE     SERCOM2

#define SERCOM_UART_DBG_NVIC_IRQn   SERCOM2_IRQn
#define SERCOM_UART_INTERACTIVE_IRQn SERCOM2_IRQn

/* Timer Instances */

/* TIMER_ADC triggers ADC conversion, TIMER_DELAY used for delay timing.
 * TIMER_DELAY must be a 32 bit timer. For SAMD21, TC4 is combined with TC5
 * (30.6.2.4) to build a 32 bit timer.
 *
 * There is also accurate millisecond/microsecond counters. The 1ms tick is
 * also used to wake up the core for events and USB handling.
 */
#define TIMER_ADC                   TC3
#define TIMER_DELAY                 TC4
#define TIMER_TICK                  TC6
#define IRQ_TIMER_DELAY             irq_handler_tc4
#define IRQ_TIMER_TICK              irq_handler_tc6

#define TIMER_ADC_GCLK_ID           TC3_GCLK_ID
#define TIMER_ADC_APBCMASK          PM_APBCMASK_TC3
#define TIMER_ADC_EVT_SRC           EVSYS_ID_GEN_TC3_MCX_0

#define TIMER_DELAY_GCLK_ID         TC4_GCLK_ID
#define TIMER_DELAY_APBCMASK        PM_APBCMASK_TC4
#define TIMER_DELAY_IRQn            TC4_IRQn

#define TIMER_TICK_GCLK_ID          TC6_GCLK_ID
#define TIMER_TICK_APBCMASK         PM_APBCMASK_TC6
#define TIMER_TICK_IRQn             TC6_IRQn


/* Pin Configuration (nb. logical, not physical) */

#define GRP_PINA            0u
#define GRP_PINB            1u


#define GRP_REV             GRP_PINA
#define PIN_REV0            19u
#define PIN_REV1            20u
#define PIN_REV2            21u

#define GRP_LED_STATUS      GRP_PINB
#define PIN_LED_STATUS      23u
#define GRP_LED_PROG        GRP_PINA
#define PIN_LED_PROG        27u
#define GRP_LED_USER        GRP_PINB
#define PIN_LED_USER0       10u
#define PIN_LED_USER1       11u

#define GRP_ONEWIRE         GRP_PINA
#define PIN_ONEWIRE         16u

#define GRP_PULSE           GRP_PINA
#define PIN_PULSE1          17u
#define PIN_PULSE2          18u

#define GRP_TEST_SENSE      GRP_PINA
#define PIN_TEST_SENSE      28

#define GRP_ADC_VMID        GRP_PINA
#define PIN_ADC_VMID        2u
#define GRP_ADC_VREF        GRP_PINA
#define PIN_ADC_VREF        3u
#define GRP_ADC_VSENS1      GRP_PINB
#define PIN_ADC_VSENS1      8u
#define GRP_ADC_VSENS2      GRP_PINB
#define PIN_ADC_VSENS2      9u
#define GRP_ADC_VSENS3      GRP_PINA
#define PIN_ADC_VSENS3      4u
#define GRP_ADC_CT1         GRP_PINA
#define PIN_ADC_CT1         5u
#define GRP_ADC_CT2         GRP_PINA
#define PIN_ADC_CT2         6u
#define GRP_ADC_CT3         GRP_PINA
#define PIN_ADC_CT3         7u
#define GRP_ADC_CT4         GRP_PINB
#define PIN_ADC_CT4         0u
#define GRP_ADC_CT5         GRP_PINB
#define PIN_ADC_CT5         1u
#define GRP_ADC_CT6         GRP_PINB
#define PIN_ADC_CT6         2u
#define GRP_ADC_CT7         GRP_PINB
#define PIN_ADC_CT7         3u
#define GRP_ADC_CT8         GRP_PINB
#define PIN_ADC_CT8         4u
#define GRP_ADC_CT9         GRP_PINB
#define PIN_ADC_CT9         5u
#define GRP_ADC_CT10        GRP_PINB
#define PIN_ADC_CT10        6u
#define GRP_ADC_CT11        GRP_PINB
#define PIN_ADC_CT11        7u
#define GRP_ADC_AIN         GRP_PINA
#define PIN_ADC_AIN         8u
#define GRP_ADC_VCAL_H      GRP_PINA
#define PIN_ADC_VCAL_H      9u
#define GRP_ADC_VCAL_L      GRP_PINA
#define PIN_ADC_VCAL_L      10u

/* Debug UART related defines */
#define GRP_SERCOM_UART_DBG0    GRP_PINA
#define PIN_UART_DBG_RX0        13u
#define PIN_UART_DBG_TX0        12u
#define GRP_SERCOM_UART_DBG1    GRP_PINB
#define PIN_UART_DBG_RX1        30u
#define PIN_UART_DBG_TX1        31u
#define UART_DBG_PAD_RX     1u
#define UART_DBG_PAD_TX     2u
#define UART_DBG_BAUD       38400u

/* SPI related defines */
#define GRP_SERCOM_SPI      GRP_PINA
#define PIN_SPI_MISO        14u
#define PIN_SPI_MOSI        15u
#define PIN_SPI_SCK         16u
#define PIN_SPI_RFM_SS      17u
#define SPI_DATA_BAUD       4000000ul
#define PMUX_SPI_DATA       PORT_PMUX_PMUXE_D

/* I2C related defines */
#define GRP_SERCOM_I2C_INT  GRP_PINA
#define PIN_I2C_INT_SDA     22u
#define PIN_I2C_INT_SCL     23u
#define PMUX_I2CM           PORT_PMUX_PMUXE_C

#define GRP_SERCOM_I2C_EXT  GRP_PINB
#define PIN_I2C_EXT_SDA     16u
#define PIN_I2C_EXT_SCL     17u
#define PMUX_I2CM           PORT_PMUX_PMUXE_C

/* Data UART related defines */
#define GRP_SERCOM_UART_DATA GRP_PINA
#define PIN_UART_DATA_RX    21u
#define PIN_UART_DATA_TX    22u
#define UART_DATA_PAD_RX    3u
#define UART_DATA_PAD_TX    1u
#define UART_DATA_BAUD      115200u

/* DMA defines */
#define NUM_CHAN_DMA        5u
#define DMA_CHAN_UART_DATA  4u
#define DMA_CHAN_I2CM       3u
#define DMA_CHAN_UART_DBG   2u
#define DMA_CHAN_ADC1       1u
#define DMA_CHAN_ADC0       0u
