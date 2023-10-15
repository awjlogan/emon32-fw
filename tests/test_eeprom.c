#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <ctype.h>

#include "test_eeprom.h"
#include "emon32.h"
#include "eeprom.h"

typedef enum {
    I2C_IDLE,
    I2C_ACTIVATE_ADDR,
    I2C_ACTIVATE_WR,
    I2C_ACTIVATE_WR_ADDR,
    I2C_ACTIVATE_RD,
    I2C_BYTE_WR,
    I2C_BYTE_RD
} i2c_state_t;

i2c_state_t     state = I2C_IDLE;

uint8_t         eeprom[EEPROM_SIZE_BYTES];
unsigned int    currentAddrHigh;
unsigned int    currentAddrLow;

uint16_t
currentAddr()
{
    return (((currentAddrHigh >> 1) & 0x1) << 8) | currentAddrLow;
}

void
i2cActivate(int inst, uint8_t addr)
{
    unsigned int isRead;

    (void)inst;
    isRead = addr & 0x1 ? 1 : 0;

    if (isRead) state = I2C_ACTIVATE_RD;
    else state = I2C_ACTIVATE_ADDR;

    currentAddrHigh = addr;
}

void
i2cDataWrite(int inst, uint8_t data)
{
    uint16_t lastAddr = currentAddr();

    if (I2C_ACTIVATE_ADDR == state)
    {
        currentAddrLow = data;
        state = I2C_IDLE;
        return;
    }

    (void)inst;

    /* Can't over run a 16 byte boundary */
    if ((currentAddr() & 0xFFF0) ^ (lastAddr & 0xFFF0))
    {
        printf("\nOver ran 16 byte boundary: %d -> %d\n\n", lastAddr, currentAddr());
        assert(0);
    }

    eeprom[(((currentAddrHigh >> 1) & 0x1) << 8) | currentAddrLow++] = data;

}

uint8_t
i2cDataRead(int inst)
{
    (void)inst;
    uint8_t data;
    data =  eeprom[currentAddr()];
    // printf("%#x\tR\t%#x\n", currentAddr(), data);
    currentAddrLow++;
    return data;
}

void
i2cAck(int inst, int action, int cmd)
{

}

int
timerDelayNB_us(int a, void (*p)())
{
    return 0;
}

int
timerDelay_us(int a)
{
    return 0;
}

void
timerDisable()
{}

void
dumpMem(unsigned int start)
{
    const unsigned int lineLen = 16;
    char c;

    printf("\n");
    for (unsigned int i = start / lineLen; i < EEPROM_SIZE_BYTES / lineLen; i++)
    {
        /* Hex dump */
        printf("%02x\t", i);
        for (unsigned int j = 0; j < lineLen; j++)
        {

            printf("%02x ", eeprom[i * lineLen + j]);
        }

        /* Readable ASCII characters */
        printf("\t");
        for (unsigned int j = 0; j < lineLen; j++)
        {
            c = eeprom[i * lineLen + j];
            if (!isprint(c)) c = '.';
            printf("%c", c);
        }
        printf("\n");
    }
}

void
initMem(unsigned int start, unsigned int end, uint8_t val)
{
    for (unsigned int i = start; i < end; i++)
    {
        eeprom[i] = val;
    }
}

/* Check that the lower (non-wear level protected) area is not
 * overwritten by the wear levelling routing */
void
checkStatic()
{
    for (unsigned int i = 0; i < EEPROM_WL_OFFSET; i++)
    {
        if (eeprom[i] != 0xFF)
        {
            printf("\nWear levelling over wrote static area.\n");
            assert(0);
        }
    }
}

int
main(int argc, char *argv[])
{
    char str[] = "This is a very long string that will wrap over writes";
    eepromPktWL_t           wlPkt;
    Emon32Cumulative_t      cumulative;

    /* Fresh EEPROM is all 1s */
    initMem(0, EEPROM_SIZE_BYTES, 0xFFu);

    printf("---- emon32 EEPROM test ----\n\n");

    printf("  > Write over multiple 16 byte pages ...");
    (void)eepromWrite(0, str, strlen(str)+1u);
    while (EEPROM_WR_COMPLETE != eepromWrite(0, 0, 0));

    (void)eepromWrite(63, str, strlen(str)+1u);
    while (EEPROM_WR_COMPLETE != eepromWrite(0, 0, 0));

    printf(" Done!\n");

    printf("  > Wear level routine tests ... ");

    initMem(0, (EEPROM_WL_OFFSET - 1u), 0xFFu);
    /* The wear levelled portion should be set to 0 initially, so that false
     * values are read at the very first time powered on */
    initMem(EEPROM_WL_OFFSET, EEPROM_SIZE_BYTES, 0x0u);

    wlPkt.addrBase = EEPROM_WL_OFFSET;
    assert(wlPkt.addrBase == 128);  /* 512 kB EEPROM, 384 byte WL */
    wlPkt.idxNextWrite = -1;  /* Unknown entrance point */
    wlPkt.blkCnt = EEPROM_WL_SIZE / sizeof(Emon32Cumulative_t);
    wlPkt.dataSize = sizeof(Emon32Cumulative_t);
    wlPkt.pData = &cumulative;

    cumulative.valid = 0;
    cumulative.crc = 0xA5A5;
    cumulative.report.wattHour[0] = 0;
    cumulative.report.wattHour[1] = 0xA;
    cumulative.report.wattHour[2] = 0xF;
    cumulative.report.wattHour[3] = 0xA;
    cumulative.report.pulseCnt = 'a';

    // printf("    Base address: %d\n", wlPkt.addrBase);
    // printf("    Block count:  %d\n", wlPkt.blkCnt);
    // printf("    Data size:    %d\n", wlPkt.dataSize);

    for (unsigned int i = 0; i < 16; i++)
    {
        eepromWriteWL(&wlPkt);
        while (EEPROM_WR_COMPLETE != eepromWrite(0, 0, 0));
        cumulative.report.pulseCnt++;

        /* Check no over run into non-wear levelled portion
         * and correct valid bytes written */

        if ((0 == i) && (1 != wlPkt.idxNextWrite))
        {
            dumpMem(EEPROM_WL_OFFSET);
            printf("  > Incorrect write index found %d, expected 1\n", wlPkt.idxNextWrite);
            assert(0);
        }

        checkStatic();
        if (eeprom[EEPROM_WL_OFFSET + i * sizeof(Emon32Cumulative_t)] != 0x0)
        {
            dumpMem(EEPROM_WL_OFFSET);
            printf("  > Incorrect valid indicator %d, expected 0x0\n", eeprom[EEPROM_WL_OFFSET + i * sizeof(Emon32Cumulative_t)]);
            assert(0);
        }
    }

    eepromWriteWL(&wlPkt);
    while (EEPROM_WR_COMPLETE != eepromWrite(0, 0, 0));
    cumulative.report.pulseCnt++;
    checkStatic();
    if (eeprom[EEPROM_WL_OFFSET] != 0x1)
        {
            dumpMem(EEPROM_WL_OFFSET);
            printf("  > Incorrect valid indicator %d, expected 0x1\n", eeprom[EEPROM_WL_OFFSET]);
            assert(0);
        }

    eepromWriteWL(&wlPkt);
    while (EEPROM_WR_COMPLETE != eepromWrite(0, 0, 0));
    cumulative.report.pulseCnt++;
    checkStatic();

    /* Check identifying the last write is successful
     * initialise EEPROM and seed values into the "valid" positions, then
     * find the latest. */
    const unsigned int lastValidPos = 5;
    initMem(EEPROM_WL_OFFSET, EEPROM_SIZE_BYTES, 0);
    for (unsigned int i = EEPROM_WL_OFFSET; i < EEPROM_SIZE_BYTES; i = i + sizeof(Emon32Cumulative_t))
    {
        eeprom[i] = 1;
    }
    for (unsigned int i = EEPROM_WL_OFFSET + lastValidPos * sizeof(Emon32Cumulative_t); i < EEPROM_SIZE_BYTES; i = i + sizeof(Emon32Cumulative_t))
    {
        eeprom[i] = 0;
    }
    wlPkt.idxNextWrite = -1;
    eepromWriteWL(&wlPkt);
    if ((lastValidPos + 1u) != wlPkt.idxNextWrite)
    {
        dumpMem(EEPROM_WL_OFFSET);
        printf("  > Incorrect wear level entry position %d, expected %d\n", wlPkt.idxNextWrite, (lastValidPos + 1u));
        assert(0);
    }

    printf("Done!\n");
}
