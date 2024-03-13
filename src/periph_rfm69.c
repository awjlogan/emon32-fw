#include <stdint.h>

#include "emon32_samd.h"
#include "board_def.h"
#include "driver_SERCOM.h"
#include "driver_TIME.h"
#include "periph_rfm69.h"

/* Register definitions */
typedef enum {
    REG_FIFO            = 0x00,
    REG_OPMODE          = 0x01,
    REG_FRFMSB          = 0x07,
    REG_AFCFEI          = 0x1E,
    REG_RSSIVALUE       = 0x24,
    REG_DIOMAPPING1     = 0x25,
    REG_IRQFLAGS1       = 0x27,
    REG_IRQFLAGS2       = 0x28,
    REG_SYNCCONFIG      = 0x2E,
    REG_SYNCVALUE1      = 0x2F,
    REG_SYNCVALUE2      = 0x30,
    REG_SYNCVALUE3      = 0x31,
    REG_NODEADRS        = 0x39,
    REG_PACKETCONFIG2   = 0x3D,
    REG_AESKEY1         = 0x3E
} RFM_register_t;

static RFMPkt_t     rfmPkt;

/* Alias to SPI functions */
static void
rfmWriteReg(const RFM_register_t addr, const uint8_t data)
{
    /* Datasheet 5.2.1, Figure 24: "wnr is 1 for write" */
    spiWriteByte(SERCOM_SPI_DATA, ((uint8_t)addr | 0x80), data);
}

static uint8_t
rfmReadReg(const RFM_register_t addr)
{
    return spiReadByte(SERCOM_SPI_DATA, (uint8_t)addr);
}

/* Call back function for the non-blocking timer to set the timed out flag */
static volatile unsigned int timedOut = 0;

void
setTimedOut()
{
    timedOut = 1u;
}

/* Adapted from AVR GCC libc:
 * https://www.nongnu.org/avr-libc/user-manual/group__util__crc.html#ga95371c87f25b0a2497d9cba13190847f
 */
static uint16_t
crc16_update(uint16_t crc, const uint8_t d)
{
    crc ^= d;
    for (unsigned int i = 0; i < 8; i++)
    {
        crc =   (crc & 0x1u)
              ? (crc >> 1) ^ 0xA001u
              : (crc >> 1);
    }

    return crc;
}

static void
rfmSleep()
{
    uint8_t tempRecv;

    /* REG_IRQFLAGS2: IRQ2_PACKETSENT */
    while (0 == (rfmReadReg(REG_IRQFLAGS2) & 0x8u))
    {
        timerDelay_us(1000);
    }

    tempRecv = rfmReadReg(REG_OPMODE);
    tempRecv = (tempRecv & 0xE3u) | 0x1u;
    rfmWriteReg(REG_OPMODE, tempRecv);
}

RFMPkt_t *
rfmGetHandle()
{
    return &rfmPkt;
}

void
rfmInit(RFM_Freq_t freq)
{
    /* Configuration parameters */
    const uint8_t config[][2] =
    {
        {0x01, 0x04}, /* OPMODE: Sequencer, standby, listen off */
        {0x02, 0x00}, /* DataModul: Packet, FSK, no shaping */
        {0x03, 0x02}, /* BitRate MSB: ~49.23 Kbps */
        {0x04, 0x8A}, /* BitRate LSB */
        {0x05, 0x05}, /* FdevMsb: ~90 kHz */
        {0x06, 0xC3}, /* FdevLsb */
        {0x07, (  RF12_868MHz == freq)
                ? 0xD9
                : (   (RF12_915MHz == freq)
                    ? 0xE4
                    : 0x6C)}, /* FrfMsb */
        {0x08, 0x00}, /* FrfMid */
        {0x09, 0x00}, /* FrfLsb */
        {0x11, 0x99}, /* PaLevel */
        {0x1E, 0x2C},
        {0x25, 0x80}, /* DioMapping1 */
        {0x26, 0x03}, /* DioMapping2 */
        {0x28, 0x00}, /* IrqFlags: FIFO overrun */
        {0x2E, 0x88}, /* SyncConfig : On, FIFO fill, 2 bytes, Tol */
        {0x2F, 0x2D}, /* SyncValue1 */
        {0x37, 0x00}, /* PktConfig: fixed, !DC free, !CRC, !CRCClear */
        {0xFF, 0}
    };

    /* Initialise RFM69 */
    while (0xAA != rfmReadReg(REG_SYNCVALUE1))
    {
        rfmWriteReg(REG_SYNCVALUE1, 0xAAu);
    }
    while (0x55u != rfmReadReg(REG_SYNCVALUE1))
    {
        rfmWriteReg(REG_SYNCVALUE1, 0x55u);
    }

    /* Configuration */
    for (unsigned int idxCfg = 0; (0xFF != config[idxCfg][0]); idxCfg++)
    {
        rfmWriteReg(config[idxCfg][0], config[idxCfg][1]);
    }

    rfmSleep();
}

int
rfmSend(const void *pData)
{
    unsigned int    txState = 0;
    int             success = 0;
    uint16_t        crc     = ~0;
    const uint8_t   *data   = (uint8_t *)pData;
    uint8_t         tempRecv;
    uint8_t         writeByte;

    /* Wait for "clear" air to transmit in.
     * 1. Enter receive mode
     * 2. Listen until below threshold
     * 3. If over time, then return with failure. Otherwise proceed
     */
    if (0 != rfmPkt.timeout)
    {
        success = -1;

        tempRecv = rfmReadReg(REG_OPMODE);
        tempRecv = (tempRecv & 0xE3) | 0x10;

        /* Non-blocking timer was busy */
        timedOut = 0;
        if (-1 == timerDelayNB_us(rfmPkt.timeout, &setTimedOut))
        {
            return -1;
        }
        while (0 == timedOut)
        {
            /* Wait for READY */
            while(0 == (rfmReadReg(0x27) & 0x80));

            /* REG_RSSI_CONFIG: RSSI_START */
            rfmWriteReg(0x23u, 0x1u);
            /* RSSI_DONE */
            while (0 == (rfmReadReg(0x23u) & 0x02u));

            /* REG_RSSI_VALUE */
            if (rfmReadReg(0x24u) > (rfmPkt.threshold * -2))
            {
                success = 0;
                break;
            }
            /* Restart receiver */
            /* REG_PACKET_CONFIG2 */
            tempRecv = rfmReadReg(0x3Du);
            tempRecv = (tempRecv & 0xFB) | 0x4u;
            rfmWriteReg(0x3Du, tempRecv);
        }
    }


    /* 1. Check for FIFO full each loop, then push into FIFO:
     *  - Node information (CTL, DST, ACK, ID
     *  - Number of payload bytes
     *  - Data bytes
     *  - CRC16
     * 1.1 If <12 bytes in the FIFO, fill with dummy bytes
     * 2. Send at specified RF power
     * 3. Enter sleep mode
     */
    crc = crc16_update(crc, rfmPkt.grp);
    while (txState < 5)
    {
        if (0 == (rfmReadReg(REG_IRQFLAGS2) & 0x80u))
        {
            switch (txState)
            {
                case 0:
                    writeByte = rfmPkt.node & 0x1F;
                    txState++;
                    break;
                case 1:
                    writeByte = rfmPkt.n;
                    txState++;
                    break;
                case 2:
                    writeByte = *data++;
                    rfmPkt.n--;
                    if (0 == rfmPkt.n)
                    {
                        txState++;
                    }
                    break;
                case 3:
                    writeByte = (uint8_t)crc;
                    txState++;
                    break;
                case 4:
                    writeByte = (uint8_t)(crc >> 8);
                    txState++;
                    break;
            }
            if (txState < 4)
            {
                crc = crc16_update(crc, writeByte);
            }
            rfmWriteReg(REG_FIFO, writeByte);
        }
    }

    /* Pad FIFO out to minimum 17 bytes */
    while (txState < 17u)
    {
        rfmWriteReg(REG_FIFO, 0xAAu);
        txState++;
    }

    writeByte = (rfmPkt.rf_pwr & 0x1F) | 0x80;
    rfmWriteReg(0x11u, writeByte);

    tempRecv = rfmReadReg(REG_OPMODE);
    tempRecv = (tempRecv & 0xE3) | 0xC;
    rfmWriteReg(REG_OPMODE, tempRecv);

    rfmSleep();
    return success;
}
