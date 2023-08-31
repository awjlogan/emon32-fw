#include <string.h>

#include "data_pack.h"
#include "qfpio.h"

unsigned int
dataPackageESP_n(const ECMSet_t *pData, char *pDst, unsigned int n)
{
    unsigned int    charCnt = 0;
    char            tmpBuf[16];
    unsigned int    cursor = 0u;
    unsigned int    insLen;

    /* Clear destination buffer */
    memset(pDst, 0, n);

    /* Message number */
    charCnt += 4;
    if (charCnt <= n)
    {
        cursor = utilStrInsert(pDst, "MSG:", 0, 4);
    }
    insLen = utilItoa(tmpBuf, pData->msgNum, ITOA_BASE10) - 1u;
    charCnt += insLen;
    if (charCnt <= n)
    {
        cursor = utilStrInsert(pDst, tmpBuf, cursor, insLen);
    }

    /* V RMS for each channel.
     * REVISIT : how should this look for multiple V channels?
     */
    charCnt += 6;
    if (charCnt <= n)
    {
        cursor = utilStrInsert(pDst, ",Vrms:", cursor, 6);
    }
    qfp_float2str(pData->rmsV[0], tmpBuf, 0);
    insLen = utilStrlen(tmpBuf);
    charCnt += insLen;
    if (charCnt <= n)
    {
        cursor = utilStrInsert(pDst, tmpBuf, cursor, insLen);
    }

    /* CT channels */
    for (unsigned int idxCT = 0; idxCT < NUM_CT; idxCT++)
    {
        charCnt += 2;
        if (charCnt <= n)
        {
            cursor = utilStrInsert(pDst, ",P", cursor, 2);
        }
        insLen = utilItoa(tmpBuf, (idxCT + 1u), ITOA_BASE10) - 1u;
        charCnt += insLen;
        if (charCnt <= n)
        {
            cursor = utilStrInsert(pDst, tmpBuf, cursor, insLen);
        }
        charCnt += 1;
        if (charCnt <= n)
        {
            cursor = utilStrInsert(pDst, ":", cursor, 1);
        }
        qfp_float2str(pData->CT[idxCT].realPower, tmpBuf, 0);
        insLen = utilStrlen(tmpBuf);
        charCnt += insLen;
        if (charCnt <= n)
        {
            cursor = utilStrInsert(pDst, tmpBuf, cursor, insLen);
        }
        charCnt += 2;
        if (charCnt <= n)
        {
            cursor = utilStrInsert(pDst, ",E", cursor, 2);
        }
        insLen = utilItoa(tmpBuf, (idxCT + 1u), ITOA_BASE10) - 1u;
        charCnt += insLen;
        if (charCnt <= n)
        {
            cursor = utilStrInsert(pDst, tmpBuf, cursor, insLen);
        }
        charCnt += 1;
        if (charCnt <= n)
        {
            cursor = utilStrInsert(pDst, ":", cursor, 1);
        }
        insLen = utilItoa(tmpBuf, pData->CT[idxCT].wattHour, ITOA_BASE10) - 1u;
        charCnt += insLen;
        if (charCnt <= n)
        {
            cursor = utilStrInsert(pDst, tmpBuf, cursor, insLen);
        }
    }

    /* TODO : temperature and pulse count are not implemented */

    return charCnt;
}

void
dataPackagePacked(const ECMSet_t *pData, PackedData_t *pPacked)
{
    pPacked->msg = pData->msgNum;
    for (unsigned int v = 0; v < NUM_V; v++)
    {
        pPacked->V[v] = (int16_t)pData->rmsV[v];
    }
}
