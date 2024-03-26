#include <string.h>

#ifndef HOSTED
#include "emon32_samd.h"
#include "driver_SERCOM.h"
#include "driver_TIME.h"
#include "emon32.h"
#include "util.h"

#include "printf.h"
#else

#include "test_eeprom.h"

#endif /* HOSTED */

#include "eeprom.h"

typedef struct Address_ {
    unsigned int    msb;
    unsigned int    lsb;
} Address_t;

/* Local data for interrupt driven EEPROM write */
typedef struct wrLocal_ {
    unsigned int    addr;
    unsigned int    n_residual;
    uint8_t         *pData;
} wrLocal_t;


/* FUNCTIONS */
static Address_t    calcAddress     (const unsigned int addrFull);
static unsigned int nextValidByte   (const uint8_t currentValid);
static void         wlFindLast      (eepromPktWL_t *pPkt);
static int          writeBytes      (wrLocal_t *wr, unsigned int n);

/* Local values */
static unsigned int eepromSizeBytes = 1024;

/* Precalculate wear limiting addresses */
const unsigned int   blkCnt  = 14;
const unsigned int   blkSize = sizeof(Emon32CumulativeSave_t);


/*! @brief Calculates the LSB and MSB address bytes
 *  @param [in] addrFull : full address of the EEPROM
 *  @return : Address_t struct with MSB and LSB
 */
static Address_t
calcAddress (const unsigned int addrFull)
{
    Address_t address;

    address.msb = EEPROM_BASE_ADDR | (addrFull >> 8);
    address.msb <<= 1u;

    address.lsb = addrFull & 0xFFu;

    return address;
}


/*! @brief Calculate the next "valid" byte. This is chosen for even set/clear
 *         across all bytes
 *  @param [in] currentValid : value of the current "valid" byte
 *  @return The next "valid" byte
 */
static unsigned int
nextValidByte(const uint8_t currentValid)
{
    unsigned int validByte = currentValid;

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


/*! @brief Find the index of the last valid write to a wear levelled block
 *  @param [in] pPkt : pointer to the packet
 */
static void
wlFindLast(eepromPktWL_t *pPkt)
{
    /* Step through from the base address in (data) sized steps. The first
     * byte that is different to the 0-th byte is the oldest block. If all
     * blocks are the same, then the 0-th index is the next to be written.
     */

    unsigned int firstByte = 0;

    pPkt->idxNextWrite = 0;
    eepromRead(EEPROM_WL_OFFSET, &firstByte, 1u);

    for (unsigned int idxBlk = 1u; idxBlk < blkCnt; idxBlk++)
    {
        unsigned int validByte;

        eepromRead((EEPROM_WL_OFFSET + (idxBlk * blkSize)), &validByte, 1u);
        if (firstByte != validByte)
        {
            pPkt->idxNextWrite = idxBlk;
            break;
        }
    }
}


/*! @brief Send n bytes over I2C
 *  @param [in] wr : pointer to local address, data, and remaining bytes
 *  @param [in] n : number of bytes to send in this chunk
 */
static int
writeBytes(wrLocal_t *wr, unsigned int n)
{
    I2CM_Status_t   i2cm_s;
    Address_t       address = calcAddress(wr->addr);

    /* Setup next transaction */
    wr->addr += n;
    wr->n_residual -= n;

    /* Write to select, then lower address */
    i2cm_s = i2cActivate (SERCOM_I2CM, address.msb);
    if (I2CM_SUCCESS != i2cm_s)
    {
        return -1;
    }
    i2cDataWrite(SERCOM_I2CM, address.lsb);

    while (n--)
    {
        i2cDataWrite(SERCOM_I2CM, *wr->pData++);
    }
    i2cAck(SERCOM_I2CM, I2CM_ACK, I2CM_ACK_CMD_STOP);

    return 0;
}


unsigned int
eepromDiscoverSize(void)
{
    /* Read the first 16 bytes as the key value, then search on each power-of-2
     * boundary for a match. Store the found value so it only has to be done
     * once. Start at index 256, minimum possible size.
     */

    uint8_t         keys[16];
    uint8_t         trial[16];
    uint32_t        index = 0x80; /* Shifted at least once */
    unsigned int    matchbytes = 0;

    if (0 != eepromSizeBytes)
    {
        return eepromSizeBytes;
    }

    eepromRead(0, keys, 16);

    while (0xFFFF != matchbytes)
    {
        matchbytes = 0;
        index <<= 1;
        eepromRead(index, trial, 16);
        for (unsigned int i = 0; i < 16; i++)
        {
            if (keys[i] == trial[i])
            {
                matchbytes |= (1 << i);
            }
        }
        timerDelay_ms(1);
    }

    return index;
}


void
eepromDump(void)
{
    /* Write out EEPROM content to debug UART 16 bytes (one page) at a time.
     * Each byte is written out as hex with a space in between
     */
    uint8_t eeprom[16];

    /* Pages */
    for (unsigned int i = 0; i < (eepromSizeBytes / 16); i++)
    {
        /* Bytes in page */
        eepromRead((i * 16), eeprom, 16);
        printf_("%04x: ", (i * 16));
        for (unsigned int j = 0; j < 16; j++)
        {
            printf_("%02x ", eeprom[j]);
        }
        printf_("\r\n");
    }
}

int
eepromInitBlock(unsigned int startAddr, const unsigned int val, unsigned int n)
{
    I2CM_Status_t   i2cm_s;
    Address_t       address;

    /* Return a fault if:
     *  - the start address is not on a 16byte boundary
     *  - n is not divisble by 16
     *  - the write is larger than the NVM size
     */
    if (   (0 != (startAddr & 0xF))
        || (0 != (n & 0xF))
        || ((startAddr + n) > eepromSizeBytes))
    {
        return -1;
    }

    while (n)
    {
        address = calcAddress(startAddr);
        i2cm_s = i2cActivate (SERCOM_I2CM, address.msb);
        if (I2CM_SUCCESS != i2cm_s)
        {
            return -1;
        }

        i2cDataWrite(SERCOM_I2CM, address.lsb);
        for (unsigned int i = 0; i < 16; i++)
        {
            i2cDataWrite(SERCOM_I2CM, (uint8_t)val);
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
    /* Write the first line and wait, then loop through until all n bytes have
     * been written.
     */
    const uint8_t *p = (uint8_t *)pSrc;

    eepromWrite(0, p, n);
    timerDelay_us(EEPROM_WR_TIME);

    while (EEPROM_WR_COMPLETE != eepromWrite(0, 0, 0))
    {
        timerDelay_us(EEPROM_WR_TIME);
    }
}


int
eepromRead(unsigned int addr, void *pDst, unsigned int n)
{
    I2CM_Status_t   i2cm_s;
    uint8_t         *pData = pDst;
    Address_t       address = calcAddress(addr);

    /* Write select with address high and ack with another start, then send low
     * byte of address */
    i2cm_s = i2cActivate (SERCOM_I2CM, address.msb);
    if (I2CM_SUCCESS != i2cm_s)
    {
        return -1;
    }
    i2cDataWrite(SERCOM_I2CM, address.lsb);

    /* Send select with read, and then continue to read until complete. On
     * final byte, respond with NACK */
    address.msb += 1u;

    i2cm_s = i2cActivate(SERCOM_I2CM, address.msb);
    if (I2CM_SUCCESS != i2cm_s)
    {
        return -1;
    }

    while (n--)
    {
        *pData++ = i2cDataRead(SERCOM_I2CM);
        i2cAck(SERCOM_I2CM, I2CM_ACK, I2CM_ACK_CMD_CONTINUE);
    }
    i2cAck(SERCOM_I2CM, I2CM_NACK, I2CM_ACK_CMD_STOP);

    return 0;
}


void
eepromReadWL(eepromPktWL_t *pPktRd)
{
    /* Check for correct indexing, find it not yet set. Read into struct from
     * correct location.
     */
    int             idxRd;
    unsigned int    addrRd;

    if (-1 == pPktRd->idxNextWrite) wlFindLast(pPktRd);

    idxRd = pPktRd->idxNextWrite - 1u;
    if (idxRd < 0)
    {
        idxRd = blkCnt + idxRd;
    }
    addrRd = EEPROM_WL_OFFSET + (idxRd * blkSize);
    eepromRead(addrRd, pPktRd->pData, pPktRd->dataSize);
}


eepromWrStatus_t
eepromWrite(unsigned int addr, const void *pSrc, unsigned int n)
{
    /* Make byte count and address static to allow re-entrant writes */
    static wrLocal_t    wrLocal;
    unsigned int        alignBytes;
    int                 wr_stat;

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
        wrLocal.pData      = (uint8_t *)pSrc;
        wrLocal.addr       = addr;
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
    alignBytes = (EEPROM_PAGE_SIZE - (wrLocal.addr & (EEPROM_PAGE_SIZE - 1u))) % EEPROM_PAGE_SIZE;
    if (0 != alignBytes)
    {
        if (alignBytes > wrLocal.n_residual)
        {
            alignBytes = wrLocal.n_residual;
        }

        /* Copy data into the write packet's data section */
        wr_stat = writeBytes(&wrLocal, alignBytes);
        if (-1 == wr_stat)
        {
            return EEPROM_WR_FAIL;
        }
        return EEPROM_WR_PEND;
    }

    /* Write any whole pages */
    while (wrLocal.n_residual > EEPROM_PAGE_SIZE)
    {
        wr_stat = writeBytes(&wrLocal, EEPROM_PAGE_SIZE);
        if (-1 == wr_stat)
        {
            return EEPROM_WR_FAIL;
        }
        return EEPROM_WR_PEND;
    }

    /* Mop up residual data */
    wr_stat = writeBytes(&wrLocal, wrLocal.n_residual);
    if (-1 == wr_stat)
    {
        return EEPROM_WR_FAIL;
    }
    return EEPROM_WR_PEND;
}


void
eepromWriteCB(void)
{
    const eepromWrStatus_t wrStatus = eepromWrite(0, 0, 0);

    /* If not complete, start next write slot, retry if the timer is busy.
     * When complete, free the timer.
     */
    if (EEPROM_WR_COMPLETE != wrStatus)
    {
        while(-1 == timerDelayNB_us(EEPROM_WR_TIME, &eepromWriteCB));
    }
    else
    {
        /* Disable timer and return mutex to the peripheral */
        timerDelayNB_NotInUse();
    }
}


void
eepromWriteWL(eepromPktWL_t *pPktWr)
{
    /* Check for correct indexing, find if not yet set; this is indicated by
     * idxNextWrite == -1. Write output to new levelled position.
     */
    unsigned int        validByte;
    int                 idxWr;
    unsigned int        addrWr;
    eepromWrStatus_t    wrStatus;

    if (-1 == pPktWr->idxNextWrite) wlFindLast(pPktWr);

    /* Calculate the next valid byte, and store packet */
    addrWr = EEPROM_WL_OFFSET + (pPktWr->idxNextWrite * blkSize);
    /* TODO : this is not correct, unsafe wrap! */
    eepromRead((addrWr - pPktWr->dataSize), &validByte, 1u);

    wrStatus = eepromWrite(addrWr, pPktWr->pData, pPktWr->dataSize);
    if (wrStatus != (EEPROM_WR_PEND || EEPROM_WR_COMPLETE))
    {
        dbgPuts(">  EEPROM write failed!\r\n");
    }

    /* Once all blocks with the same "valid" byte have been written out,
     * generate the next "valid" byte to be used and wrap.
     */
    idxWr = pPktWr->idxNextWrite + 1u;
    if (idxWr == blkCnt)
    {
        idxWr = 0;
        *(uint8_t *)pPktWr->pData = nextValidByte(validByte);
    }
    pPktWr->idxNextWrite = idxWr;
}
