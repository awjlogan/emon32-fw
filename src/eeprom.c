#include <string.h>

#ifndef HOSTED
#include "emon32_samd.h"
#include "driver_SERCOM.h"
#include "driver_TIME.h"
#include "emon32.h"
#else
#include "test_eeprom.h"
#endif /* HOSTED */

#include "eeprom.h"

/* Local data for interrupt driven EEPROM write */
typedef struct {
    uint16_t        addr;
    unsigned int    n_residual;
    uint8_t         *pData;
} wrLocal_t;

typedef struct {
    uint8_t         msb;
    uint8_t         lsb;
} Address_t;

static Address_t
calcAddress (const uint16_t addrFull)
{
    Address_t address;

    address.msb = EEPROM_BASE_ADDR | (addrFull >> 8);
    address.msb <<= 1u;

    address.lsb = addrFull & 0xFFu;

    return address;
}


/*! @brief Send n bytes over I2C using DMA
 *  @param [in] wr : pointer to local address, data, and remaining bytes
 *  @param [in] n : number of bytes to send in this chunk
 */
static void
writeBytes(wrLocal_t *wr, uint8_t n)
{
    Address_t address = calcAddress(wr->addr);

    /* Setup next transaction */
    wr->addr += n;
    wr->n_residual -= n;

    /* Write to select, then lower address */
    i2cActivate(SERCOM_I2CM, address.msb);
    i2cDataWrite(SERCOM_I2CM, address.lsb);
    while (n--)
    {
        i2cDataWrite(SERCOM_I2CM, *wr->pData++);
    }
    i2cAck(SERCOM_I2CM, I2CM_ACK, I2CM_ACK_CMD_STOP);
}

void
eepromWriteCB()
{
    const eepromWrStatus_t wrStatus = eepromWrite(0, 0, 0);
    if (EEPROM_WR_COMPLETE != wrStatus)
    {
        while(-1 != timerDelayNB_us(EEPROM_WR_TIME, &eepromWriteCB));
    }
    else
    {
        timerDisable();
    }
}

eepromWrStatus_t
eepromWrite(uint16_t addr, const void *pSrc, unsigned int n)
{
    /* Make byte count and address static to allow re-entrant writes */
    static wrLocal_t    wrLocal;
    uint8_t             align_bytes;

    /* If all parameters are 0, then this is a continuation from ISR */
    const unsigned int  continueBlock = (0 == addr) && (0 == pSrc) && (0 == n);

    /* If no ongoing transaction:
     *   - if it is a continuation, then return complete
     *   - otherwise capture required details
     */
    if (0 == wrLocal.n_residual)
    {
        if (0 != continueBlock)
        {
            return EEPROM_WR_COMPLETE;
        }

        wrLocal.n_residual = n;
        wrLocal.pData = (uint8_t *)pSrc;
        wrLocal.addr = addr;
    }
    /* If there is a pending data, and this is new, indicate BUSY */
    else
    {
        if (0 == continueBlock)
        {
            return EEPROM_WR_BUSY;
        }
    }

    /* Writes can not go over EEPROM_PAGE_SIZE byte pages. Align the first
     * transfer to end of EEPROM_PAGE_SIZE byte page.
     */
    align_bytes = (EEPROM_PAGE_SIZE - (wrLocal.addr & (EEPROM_PAGE_SIZE - 1u))) % EEPROM_PAGE_SIZE;
    if (0 != align_bytes)
    {
        if (align_bytes > wrLocal.n_residual)
        {
            align_bytes = wrLocal.n_residual;
        }

        /* Copy data into the write packet's data section */
        writeBytes(&wrLocal, align_bytes);
        return EEPROM_WR_PEND;
    }

    /* Write any whole pages */
    while (wrLocal.n_residual > EEPROM_PAGE_SIZE)
    {
        writeBytes(&wrLocal, EEPROM_PAGE_SIZE);
        return EEPROM_WR_PEND;
    }

    /* Mop up residual data */
    writeBytes(&wrLocal, wrLocal.n_residual);
    return EEPROM_WR_PEND;
}

void
eepromRead(uint16_t addr, void *pDst, unsigned int n)
{
    uint8_t     *pData = pDst;
    Address_t   address = calcAddress(addr);

    /* TODO handle timeouts and other errors gracefully */
    /* Write select with address high and ack with another start, then send low
     * byte of address */
    i2cActivate(SERCOM_I2CM, address.msb);
    i2cDataWrite(SERCOM_I2CM, address.lsb);

    /* Send select with read, and then continue to read until complete. On
     * final byte, respond with NACK */
    address.msb += 1u;

    i2cActivate(SERCOM_I2CM, address.msb);

    do
    {
        *pData++ = i2cDataRead(SERCOM_I2CM);
        i2cAck(SERCOM_I2CM, I2CM_ACK, I2CM_ACK_CMD_CONTINUE);
        n--;
    } while (n);
    i2cAck(SERCOM_I2CM, I2CM_NACK, I2CM_ACK_CMD_STOP);
}

/*! @brief Find the index of the last valid write to a wear levelled block
 *  @param [in] pPkt : pointer to the packet
 */
void
wlFindLast(eepromPktWL_t *pPkt)
{
    /* Step through from the base address in (data) sized steps. The first
     * byte that is different to the 0-th byte is the oldest block. If all
     * blocks are the same, then the 0-th index is the next to be written.
     */

    uint8_t idxNextWr = 0;
    uint8_t firstByte = 0;

    eepromRead(pPkt->addr_base, &firstByte, 1u);
    for (unsigned int idxBlk = 1u; idxBlk < pPkt->blkCnt; idxBlk++)
    {
        uint8_t         validByte;
        unsigned int    addrRd = pPkt->addr_base + (pPkt->dataSize * idxBlk);

        eepromRead(addrRd, &validByte, 1u);
        idxNextWr++;

        if (firstByte != validByte)
        {
            break;
        }
    }
    /* TODO Compare checksum of packet */
    pPkt->idxNextWrite =   (idxNextWr == (pPkt->blkCnt - 1u))
                          ? 0
                          : idxNextWr;
}

uint8_t
nextValidByte(const uint8_t currentValid)
{

    uint8_t validByte = currentValid;

    /* The valid byte is calculated to have even bit 0/1 writes. */
    /* Start filling with 1s */
    if (0 == validByte)
    {
        validByte += 1u;
    }
    /* Start filling with 0s */
    else if (0xFFu == validByte)
    {
        validByte <<= 1;
    }
    /* Continue filling with 1s */
    else if (1u == (validByte & 0x1u))
    {
        validByte <<= 1;
        validByte += 1u;
    }
    /* Continue filling with 0s */
    else
    {
        validByte <<= 1;
    }

    return validByte;
}

void
eepromWriteWL(eepromPktWL_t *pPktWr)
{
    /* Check for correct indexing, find if not yet set; this is indicated by
     * idxNextWrite == -1. Write output to new levelled position.
     */
    uint8_t             validByte;
    int8_t              idxWr;
    uint16_t            addrWr;
    eepromWrStatus_t    wrStatus;

    if (-1 == pPktWr->idxNextWrite) wlFindLast(pPktWr);

    /* Calculate the next valid byte, and store packet */
    addrWr = pPktWr->addr_base + (pPktWr->idxNextWrite * pPktWr->dataSize);
    eepromRead((addrWr - pPktWr->dataSize), &validByte, 1u);

    wrStatus = eepromWrite(addrWr, pPktWr->pData, pPktWr->dataSize);
    if (wrStatus != (EEPROM_WR_PEND || EEPROM_WR_COMPLETE))
    {
        dbgPuts(">  EEPROM write failed!\r\n");
    }

    idxWr = pPktWr->idxNextWrite + 1u;
    if (idxWr == pPktWr->blkCnt)
    {
        idxWr = 0;
        *(uint8_t *)pPktWr->pData = nextValidByte(validByte);
    }
    pPktWr->idxNextWrite = idxWr;
}

void
eepromReadWL(eepromPktWL_t *pPktRd)
{
    /* Check for correct indexing, find it not yet set. Read into struct from
     * correct location.
     */
    int8_t      idxRd;
    uint16_t    addrRd;

    if (-1 == pPktRd->idxNextWrite) wlFindLast(pPktRd);

    idxRd = pPktRd->idxNextWrite - 1u;
    if (idxRd < 0)
    {
        idxRd = pPktRd->blkCnt + idxRd;
    }
    addrRd = pPktRd->addr_base + (idxRd * pPktRd->dataSize);
    eepromRead(addrRd, pPktRd->pData, pPktRd->dataSize);
}

int
eepromInitBlocking(uint16_t startAddr, const uint8_t val, unsigned int n)
{
    Address_t address;

    /* Return a fault if:
     *  - the start address is not on a 16byte boundary
     *  - n is not divisble by 16
     *  - the write is larger than the NVM size
     */
    if (   (0 != (startAddr & 0xF))
        || (0 != (n & 0xF))
        || ((startAddr + n) > EEPROM_SIZE_BYTES))
    {
        return -1;
    }

    while (n)
    {
        address = calcAddress(startAddr);
        i2cActivate(SERCOM_I2CM, address.msb);
        i2cDataWrite(SERCOM_I2CM, address.lsb);
        for (unsigned int i = 0; i < 16; i++)
        {
            i2cDataWrite(SERCOM_I2CM, val);
        }
        i2cAck(SERCOM_I2CM, I2CM_ACK, I2CM_ACK_CMD_STOP);

        timerDelay_us(EEPROM_WR_TIME);
        startAddr += 16;
        n -= 16;
    }
    return 0;
}

void
eepromInitConfig(const void *pSrc, const unsigned int n)
{
    const uint8_t *p = (uint8_t *)pSrc;

    eepromWrite(0, p, n);
    while (EEPROM_WR_COMPLETE != eepromWrite(0, 0, 0))
    {
        timerDelay_us(EEPROM_WR_TIME);
    }
}
