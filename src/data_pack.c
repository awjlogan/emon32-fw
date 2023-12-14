#include <string.h>
#include <inttypes.h>

#include "data_pack.h"
#include "util.h"
#include "qfpio.h"
#include "printf.h"

unsigned int
dataPackageESP_n(const Emon32Dataset_t *pData, char *pDst, const unsigned int n)
{
    unsigned int    bufLen = 0;

    /* Clear destination buffer */
    memset(pDst, 0, n);

    bufLen = snprintf_(pDst, n, "MSG:%u", (unsigned int)pData->msgNum);
    
    /* V channels */
    for (unsigned int i = 0; i < NUM_V; i++)
    {
        bufLen = snprintf_(pDst, n, "%s,V%d:%.2f", 
                           pDst, i, pData->pECM->rmsV[i]);
    }

    /* CT channels */
    for (unsigned int i = 0; i < NUM_CT; i++)
    {
        bufLen = snprintf_(pDst, n, "%s,P%d:%.2f",
                           pDst, i, pData->pECM->CT[i].realPower);
    }

    for (unsigned int i = 0; i < NUM_CT; i++)
    {
        bufLen = snprintf_(pDst, n, "%s,E%d:%u",
                           pDst, i, (unsigned int)pData->pECM->CT[i].wattHour);
    }

    for (unsigned int i = 0; i < NUM_PULSECOUNT; i++)
    {
        bufLen = snprintf_(pDst, n, "%s,pulse%d:%"PRIu64"",
                           pDst, i, pData->pulseCnt[i]);
    }

    return bufLen;
}

void
dataPackagePacked(const Emon32Dataset_t *pData, PackedData_t *pPacked)
{
    pPacked->msg = pData->msgNum;
    for (unsigned int v = 0; v < NUM_V; v++)
    {
        pPacked->V[v] = (int16_t)pData->pECM->rmsV[v];
    }
}
