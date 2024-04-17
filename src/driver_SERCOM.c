#include "emon32_samd.h"

#include "driver_DMAC.h"
#include "driver_PORT.h"
#include "driver_SERCOM.h"
#include "driver_TIME.h"

#include "configuration.h"


#define I2CM_ACTIVATE_TIMEOUT_US    200u    /* Time to wait for I2C bus */


static void i2cmCommon(Sercom *pSercom);
static void i2cmExtPinsSetup(int enable);
static void spiExtPinsSetup(int enable);

static int extIntfEnabled = 1;

static void
i2cmCommon(Sercom *pSercom)
{
    /* For 400 kHz I2C, SCL T_high >= 0.6 us, T_low >= 1.3 us, with
     * (T_high + T_low) <= 2.5 us, and T_low / T_high ~ 1.8.
     * From I2C->Clock generation (28.6.2.4.1):
     * BAUD.BAUDLOW = (T_low * f_clk) - 5 (1.625 us -> 8 @ 8 MHz)
     * BAUD.BAUD = (T_high * f_clk) - 5 (0.875 us -> 2 @ 8 MHz)
     */
    pSercom->I2CM.BAUD.reg =   SERCOM_I2CM_BAUD_BAUDLOW(8u)
                             | SERCOM_I2CM_BAUD_BAUD(2u);

    /* Configure the master I2C SERCOM */
    pSercom->I2CM.CTRLA.reg = SERCOM_I2CM_CTRLA_MODE_I2C_MASTER;

    /* Enable SERCOM, with sync */
    pSercom->I2CM.CTRLA.reg |= SERCOM_I2CM_CTRLA_ENABLE;
    while (pSercom->I2CM.SYNCBUSY.reg & SERCOM_I2CM_SYNCBUSY_SYSOP);

    /* After enabling the I2C SERCOM, the bus state is UNKNOWN (Table 28-13)
     * Force into IDLE state, with sync
     */
    pSercom->I2CM.STATUS.reg |= SERCOM_I2CM_STATUS_BUSSTATE(0x1u);
    while (pSercom->I2CM.SYNCBUSY.reg & SERCOM_I2CM_SYNCBUSY_SYSOP);

    pSercom->I2CM.INTENSET.reg =   SERCOM_I2CM_INTENSET_MB
                                 | SERCOM_I2CM_INTENSET_SB
                                 | SERCOM_I2CM_INTENSET_ERROR;
}


static void
i2cmExtPinsSetup(int enable)
{
    if (enable)
    {
        portPinMux(GRP_SERCOM_I2C_EXT, PIN_I2C_EXT_SDA, PMUX_I2CM_EXT);
        portPinMux(GRP_SERCOM_I2C_EXT, PIN_I2C_EXT_SCL, PMUX_I2CM_EXT);
    }
    else
    {
        portPinMuxClear(GRP_SERCOM_I2C_EXT, PIN_I2C_EXT_SDA);
        portPinMuxClear(GRP_SERCOM_I2C_EXT, PIN_I2C_EXT_SCL);
    }
}


static void
spiExtPinsSetup(int enable)
{
    if (enable)
    {
        portPinMux(GRP_SERCOM_SPI, PIN_SPI_MISO,    PMUX_SPI_DATA);
        portPinMux(GRP_SERCOM_SPI, PIN_SPI_MOSI,    PMUX_SPI_DATA);
        portPinMux(GRP_SERCOM_SPI, PIN_SPI_SCK,     PMUX_SPI_DATA);
        portPinMux(GRP_SERCOM_SPI, PIN_SPI_RFM_SS,  PMUX_SPI_DATA);
    }
    else
    {
        portPinMuxClear(GRP_SERCOM_SPI, PIN_SPI_MISO);
        portPinMuxClear(GRP_SERCOM_SPI, PIN_SPI_MOSI);
        portPinMuxClear(GRP_SERCOM_SPI, PIN_SPI_SCK);
        portPinMuxClear(GRP_SERCOM_SPI, PIN_SPI_RFM_SS);
    }
}


void
sercomExtIntfDisable(void)
{
    extIntfEnabled = 0;
    i2cmExtPinsSetup(0);
    spiExtPinsSetup(0);
}


void
sercomExtIntfEnable(void)
{
    extIntfEnabled = 1;
    i2cmExtPinsSetup(1);
    spiExtPinsSetup(1);
}


int
sercomExtIntfEnabled(void)
{
    return extIntfEnabled;
}


void
sercomSetup(void)
{
    /*****************
    * Debug UART setup
    ******************/
    unsigned int testSense = portPinValue(GRP_TEST_SENSE, PIN_TEST_SENSE);

    UART_Cfg_t uart_dbg_cfg;
    uart_dbg_cfg.sercom     = SERCOM_UART_DBG;
    uart_dbg_cfg.baud       = UART_DBG_BAUD;
    uart_dbg_cfg.apbc_mask  = SERCOM_UART_DBG_APBCMASK;
    uart_dbg_cfg.gclk_id    = SERCOM_UART_DBG_GCLK_ID;
    uart_dbg_cfg.gclk_gen   = 3u;
    uart_dbg_cfg.pad_tx     = UART_DBG_PAD_TX;
    uart_dbg_cfg.pad_rx     = UART_DBG_PAD_RX;

    /* If the test probe is present, route the debug UART to the tester */
    if (0 == testSense)
    {
        uart_dbg_cfg.port_grp   = GRP_SERCOM_UART_DBG0;
        uart_dbg_cfg.pin_tx     = PIN_UART_DBG_TX0;
        uart_dbg_cfg.pin_rx     = PIN_UART_DBG_RX0;
        uart_dbg_cfg.pmux       = PMUX_UART_DBG0;
    }
    else
    {
        uart_dbg_cfg.port_grp   = GRP_SERCOM_UART_DBG1;
        uart_dbg_cfg.pin_tx     = PIN_UART_DBG_TX1;
        uart_dbg_cfg.pin_rx     = PIN_UART_DBG_RX1;
        uart_dbg_cfg.pmux       = PMUX_UART_DBG1;
    }

    uart_dbg_cfg.dmaChannel = DMA_CHAN_UART_DBG;
    uart_dbg_cfg.dmaCfg.ctrlb =   DMAC_CHCTRLB_LVL(1u)
                                | DMAC_CHCTRLB_TRIGSRC(SERCOM_UART_DBG_DMAC_ID_TX)
                                | DMAC_CHCTRLB_TRIGACT_BEAT;
    sercomSetupUART(&uart_dbg_cfg);

    /*****************
    * I2C Setup
    ******************/
    portPinMux(GRP_SERCOM_I2C_INT, PIN_I2C_INT_SDA, PMUX_I2CM_INT);
    portPinMux(GRP_SERCOM_I2C_INT, PIN_I2C_INT_SCL, PMUX_I2CM_INT);

    PM->APBCMASK.reg |= SERCOM_I2CM_INT_APBCMASK;
    GCLK->CLKCTRL.reg =   GCLK_CLKCTRL_ID(SERCOM_I2CM_INT_GCLK_ID)
                        | GCLK_CLKCTRL_GEN(3u)
                        | GCLK_CLKCTRL_CLKEN;

    i2cmCommon(SERCOM_I2CM);

    if (portPinValue(GRP_nDISABLE_EXT, PIN_nDISABLE_EXT))
    {
        i2cmExtPinsSetup(1);
    }
    else
    {
        extIntfEnabled = 0;
    }

    PM->APBCMASK.reg |= SERCOM_I2CM_EXT_APBCMASK;
    GCLK->CLKCTRL.reg =   GCLK_CLKCTRL_ID(SERCOM_I2CM_EXT_GCLK_ID)
                        | GCLK_CLKCTRL_GEN(3u)
                        | GCLK_CLKCTRL_CLKEN;

    i2cmCommon(SERCOM_I2CM_EXT);

    /*****************
    * SPI Setup
    ******************/
   sercomSetupSPI();
}


void
sercomSetupUART(const UART_Cfg_t *pCfg)
{
    uint16_t baud;
    // const uint64_t br_dbg = (uint64_t)65536 * (F_PERIPH - 16 * pCfg->baud) / F_PERIPH;
    switch (pCfg->baud)
    {
        case (UART_BAUD_9600):
            baud = 64279;
            break;
        case (UART_BAUD_19200):
            baud = 63020;
            break;
        case (UART_BAUD_28800):
            baud = 61762;
            break;
        case (UART_BAUD_38400):
            baud = 60504u;
            break;
        case (UART_BAUD_57600):
            baud = 57987;
            break;
        case (UART_BAUD_76800):
            baud = 55471;
            break;
        case (UART_BAUD_115200):
            baud = 50438u;
            break;
        default:
            /* Default to 9600 if a non-standard baud is entered */
            baud = 64279;
    }

    portPinMux(pCfg->port_grp, pCfg->pin_tx, pCfg->pmux);
    portPinMux(pCfg->port_grp, pCfg->pin_rx, pCfg->pmux);

    /* Configure clocks - runs from the OSC8M clock on gen 3 */
    PM->APBCMASK.reg |= pCfg->apbc_mask;
    GCLK->CLKCTRL.reg =   GCLK_CLKCTRL_ID(pCfg->gclk_id)
                        | GCLK_CLKCTRL_GEN(pCfg->gclk_gen)
                        | GCLK_CLKCTRL_CLKEN;

    /* Configure the USART */
    pCfg->sercom->USART.CTRLA.reg =   SERCOM_USART_CTRLA_DORD
                                    | SERCOM_USART_CTRLA_MODE_USART_INT_CLK
                                    | SERCOM_USART_CTRLA_RXPO(pCfg->pad_rx)
                                    | SERCOM_USART_CTRLA_TXPO(pCfg->pad_tx);

    /* TX/RX enable requires synchronisation */
    pCfg->sercom->USART.CTRLB.reg =   SERCOM_USART_CTRLB_RXEN
                                    | SERCOM_USART_CTRLB_TXEN
                                    | SERCOM_USART_CTRLB_CHSIZE(0);
    while (pCfg->sercom->USART.STATUS.reg & SERCOM_USART_SYNCBUSY_CTRLB);

    pCfg->sercom->USART.BAUD.reg = baud;

    /* Enable requires synchronisation (26.6.6) */
    pCfg->sercom->USART.CTRLA.reg |= SERCOM_USART_CTRLA_ENABLE;
    while (pCfg->sercom->USART.STATUS.reg & SERCOM_USART_SYNCBUSY_ENABLE);

    /* Configure DMA */
    dmacChannelConfigure(pCfg->dmaChannel, &pCfg->dmaCfg);
}


void
sercomSetupSPI(void)
{
    /**********************
    * SPI Setup (for RFM69)
    ***********************/
    if (portPinValue(GRP_nDISABLE_EXT, PIN_nDISABLE_EXT))
    {
        spiExtPinsSetup(1);
    }

    /* Configure clocks - runs from the OSC8M clock on gen 3 */
    PM->APBCMASK.reg |= SERCOM_SPI_APBCMASK;
    GCLK->CLKCTRL.reg =   GCLK_CLKCTRL_ID(SERCOM_SPI_GCLK_ID)
                        | GCLK_CLKCTRL_GEN(3u)
                        | GCLK_CLKCTRL_CLKEN;

    /* Table 25-2 - driven @ F_REF = F_PERIPH. BAUD = F_REF / 2F_BAUD - 1
     * RFM69 maximum SCK is 10 MHz, so can go at maximum 4 MHz SCK easily.
     */
    SERCOM_SPI_DATA->SPI.BAUD.reg = 0;

    /* SPI mode 0: CPOL == 0, CPHA == 0 */
    /* In v0.1 MOSI and !SS are swapped, fix by hand and revise for v0.2 */
    SERCOM_SPI_DATA->SPI.CTRLA.reg =   SERCOM_SPI_CTRLA_MODE_SPI_MASTER
                                     | SERCOM_SPI_CTRLA_DOPO(0x2);

    /* Enable TX and RX interrupts (complete and empty), not routed to NVIC */
    SERCOM_SPI_DATA->SPI.INTENSET.reg |=   SERCOM_SPI_INTENSET_RXC
                                         | SERCOM_SPI_INTENSET_TXC
                                         | SERCOM_SPI_INTENSET_DRE;

    /* While disabled, RXEN will be set immediately. When the SPI SERCOM is
     * enabled, this requires synchronisation before the SPI is ready. See
     * field description in 27.8.2
     */
    SERCOM_SPI_DATA->SPI.CTRLB.reg = SERCOM_SPI_CTRLB_RXEN;
    SERCOM_SPI_DATA->SPI.CTRLA.reg |= SERCOM_SPI_CTRLA_ENABLE;
    while (0 != SERCOM_SPI_DATA->SPI.SYNCBUSY.reg);
}


/*
 * =====================================
 * UART Functions
 * =====================================
 */
void
uartPutcBlocking(Sercom *sercom, char c)
{
    while (!(sercom->USART.INTFLAG.reg & SERCOM_USART_INTFLAG_DRE));
    sercom->USART.DATA.reg = c;
    sercom->USART.INTFLAG.reg = 0;
}


void
uartPutsBlocking(Sercom *sercom, const char *s)
{
    while (*s) uartPutcBlocking(sercom, *s++);
}


void
uartConfigureDMA(void)
{
    volatile DmacDescriptor * dmacDesc = dmacGetDescriptor(DMA_CHAN_UART_DBG);
    dmacDesc->BTCTRL.reg =   DMAC_BTCTRL_VALID
                           | DMAC_BTCTRL_BLOCKACT_NOACT
                           | DMAC_BTCTRL_STEPSIZE_X1
                           | DMAC_BTCTRL_STEPSEL_SRC
                           | DMAC_BTCTRL_SRCINC
                           | DMAC_BTCTRL_BEATSIZE_BYTE;

    dmacDesc->DSTADDR.reg = (uint32_t)&SERCOM_UART_DBG->USART.DATA;
    dmacDesc->DESCADDR.reg = 0u;
}


void
uartPutsNonBlocking(unsigned int dma_chan, const char * const s, uint16_t len)
{
    volatile DmacDescriptor * dmacDesc = dmacGetDescriptor(dma_chan);
    /* Valid bit is cleared when a channel is complete */
    dmacDesc->BTCTRL.reg    |= DMAC_BTCTRL_VALID;
    dmacDesc->BTCNT.reg     = len;
    dmacDesc->SRCADDR.reg   = (uint32_t)s + len;
    dmacChannelEnable(dma_chan);
}


char
uartGetc(const Sercom *sercom)
{
    return sercom->USART.DATA.reg;
}


void
uartInterruptEnable(Sercom *sercom, uint32_t interrupt)
{
    sercom->USART.INTENSET.reg |= interrupt;
}


void
uartInterruptDisable(Sercom *sercom, uint32_t interrupt)
{
    sercom->USART.INTENCLR.reg |= interrupt;
}


uint32_t
uartInterruptStatus(const Sercom *sercom)
{
    return sercom->USART.INTFLAG.reg;
}


void
uartInterruptClear(Sercom *sercom, uint32_t interrupt)
{
    sercom->USART.INTFLAG.reg |= interrupt;
}


/*
 * =====================================
 * I2C Functions
 * =====================================
 */
I2CM_Status_t
i2cActivate(Sercom *sercom, uint8_t addr)
{
    unsigned int t = timerMicros();
    I2CM_Status_t s = I2CM_SUCCESS;

    sercom->I2CM.ADDR.reg = SERCOM_I2CM_ADDR_ADDR(addr);

    /* MB: master on bus, SB: slave on bus */
    while (!(sercom->I2CM.INTFLAG.reg & (SERCOM_I2CM_INTFLAG_MB | SERCOM_I2CM_INTFLAG_SB)))
    {
        if (timerMicrosDelta(t) > I2CM_ACTIVATE_TIMEOUT_US)
        {
            return I2CM_TIMEOUT;
        }
    }

    /* Check for NoAck response from client (28.6.2.4.2) */
    if (sercom->I2CM.STATUS.reg & SERCOM_I2CM_STATUS_RXNACK)
    {
        s = I2CM_NOACK;
    }

    return s;
}


void
i2cAck(Sercom *sercom, I2CM_Ack_t ack, I2CM_AckCmd_t cmd)
{
    sercom->I2CM.CTRLB.reg =   (ack << SERCOM_I2CM_CTRLB_ACKACT_Pos)
                             | SERCOM_I2CM_CTRLB_CMD(cmd);
    while(sercom->I2CM.SYNCBUSY.reg & SERCOM_I2CM_SYNCBUSY_SYSOP);
}


void
i2cDataWrite(Sercom *sercom, uint8_t data)
{
    sercom->I2CM.DATA.reg = data;
    while (!(sercom->I2CM.INTFLAG.reg & SERCOM_I2CM_INTFLAG_MB));
}


uint8_t
i2cDataRead(Sercom *sercom)
{
    while (!(sercom->I2CM.INTFLAG.reg & (SERCOM_I2CM_INTFLAG_MB | SERCOM_I2CM_INTFLAG_SB)));
    return sercom->I2CM.DATA.reg;
}


/*
 * =====================================
 * SPI Functions
 * =====================================
 */
void
spiWriteByte(Sercom *sercom, const uint8_t addr, const uint8_t data)
{
    portPinDrv(GRP_SERCOM_SPI, PIN_SPI_RFM_SS, PIN_DRV_CLR);
    sercom->SPI.DATA.reg = addr;
    while (0 == (sercom->SPI.INTFLAG.reg & SERCOM_SPI_INTFLAG_TXC));
    sercom->SPI.DATA.reg = data;
    while (0 == (sercom->SPI.INTFLAG.reg & SERCOM_SPI_INTFLAG_TXC));
    portPinDrv(GRP_SERCOM_SPI, PIN_SPI_RFM_SS, PIN_DRV_SET);
}


uint8_t
spiReadByte(Sercom *sercom, const uint8_t addr)
{
    /* Set address on first write, then send a dummy byte to provide clock
     * for shifting out the data
     */
    portPinDrv(GRP_SERCOM_SPI, PIN_SPI_RFM_SS, PIN_DRV_CLR);

    sercom->SPI.DATA.reg = addr;
    while (0 == (sercom->SPI.INTFLAG.reg & SERCOM_SPI_INTFLAG_TXC));

    sercom->SPI.DATA.reg = 0;
    while (0 == (sercom->SPI.INTFLAG.reg & SERCOM_SPI_INTFLAG_RXC));
    portPinDrv(GRP_SERCOM_SPI, PIN_SPI_RFM_SS, PIN_DRV_SET);

    return sercom->SPI.DATA.reg;
}


void
spiWriteBuffer(Sercom *sercom, const void *pBuf, const unsigned int n)
{
    /* Send buffer byte-wise from pBuf. Address must be the first entries in
     * pBuf
     */
    uint8_t *data = (uint8_t *)pBuf;

    portPinDrv(GRP_SERCOM_SPI, PIN_SPI_RFM_SS, PIN_DRV_CLR);
    for (unsigned int i = 0; i < n; i++)
    {
        sercom->SPI.DATA.reg = *data++;
        while(0 == (sercom->SPI.INTFLAG.reg & SERCOM_SPI_INTFLAG_TXC));
    }
    portPinDrv(GRP_SERCOM_SPI, PIN_SPI_RFM_SS, PIN_DRV_SET);
}


/* =============================
 * Interrupt handlers
 * ============================= */

void
SERCOM_UART_INTERACTIVE_HANDLER
{
    /* Echo the received character to the TX channel, and send to the command
     * stream. Echo to the console.
     */
    uint8_t rx_char = 0;

    if (SERCOM_UART_INTERACTIVE->USART.INTFLAG.reg & SERCOM_USART_INTFLAG_RXC)
    {
        SERCOM_UART_INTERACTIVE->USART.INTFLAG.reg |= SERCOM_USART_INTFLAG_RXC;
        rx_char = SERCOM_UART_INTERACTIVE->USART.DATA.reg;
        configCmdChar(rx_char);

        /* REVISIT : this may result in lost characters? */
        if (SERCOM_UART_INTERACTIVE->USART.INTFLAG.reg & SERCOM_USART_INTFLAG_DRE)
        {
            SERCOM_UART_INTERACTIVE->USART.DATA.reg = rx_char;
        }
    }
}
