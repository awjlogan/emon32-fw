#include <string.h>
#include <inttypes.h>

#include "qfplib-m0-full.h"
#include "qfpio.h"

#include "dataPack.h"
#include "util.h"


#define CONV_STR_W  16
#define STR_MSG     0
#define STR_V       1
#define STR_P       2
#define STR_E       3
#define STR_PULSE   4
#define STR_TEMP    5
#define STR_COLON   6
#define STR_CRLF    7


/* "Fat" string with current length and buffer size. */
typedef struct StrN {
    char    *str;   /* Pointer to the string */
    int     n;      /* Length of the string  */
    int     m;      /* Buffer length */
} StrN_t;


static void catId(StrN_t *strD, unsigned int id, unsigned int field);
static int  strnFtoa(StrN_t *strD, const float v);
static int  strnItoa(StrN_t *strD, const uint32_t v);
static int  strnCat(StrN_t *strD, const StrN_t *strS);
static int  strnLen(StrN_t *str);


static char     tmpStr[CONV_STR_W] = {0};
static StrN_t   strConv;    /* Fat string for conversions */

/* Strings that are inserted in the transmitted message */
const StrN_t baseStr[8] = { {.str = "MSG:",     .n = 4, .m = 5},
                            {.str = ",V",       .n = 2, .m = 3},
                            {.str = ",P",       .n = 2, .m = 3},
                            {.str = ",E",       .n = 2, .m = 3},
                            {.str = ",pulse",   .n = 6, .m = 7},
                            {.str = ",t",       .n = 2, .m = 3},
                            {.str = ":",        .n = 1, .m = 2},
                            {.str = "\r\n",     .n = 2, .m = 3}};


/*! @brief "Append <field><id>:" to the string
 *  @param [out] strD : pointer to the fat string
 *  @param [in] id : numeric index
 *  @param [in] field : field name index, e.g. "STR_V"
 */
static void
catId(StrN_t *strD, unsigned int id, unsigned int field)
{
    strD->n += strnCat(strD, &baseStr[field]);

    (void)strnItoa(&strConv, id);
    strD->n += strnCat(strD, &strConv);
    strD->n += strnCat(strD, &baseStr[STR_COLON]);
}


static int
strnFtoa(StrN_t *strD, const float v)
{
    /* REVISIT : check formatting parameter */
    const uint32_t fmt = 0;

    /* Zero the destination buffer then convert */
    memset(strD->str, 0, strD->m);
    qfp_float2str(v, strD->str, fmt);
    strD->n = strnLen(strD);

    /* Truncate if it exceeds the length of the string */
    if (-1 == strD->n)
    {
        strD->n = strD->m;
    }
    return strD->n;
}

static int
strnItoa(StrN_t *strD, const uint32_t v)
{
    /* Zero the destination buffer then convert */
    memset(strD->str, 0, strD->m);

    strD->n = utilItoa(strD->str, v, ITOA_BASE10);
    return strD->n;
}


static int
strnCat(StrN_t *strD, const StrN_t *strS)
{
    /* Check bounds to make sure it won't go over the end. If so, return the
     * actual number of bytes that are copied.
     */
    int newLen;
    int bytesToCopy;

    bytesToCopy  = strS->n;
    newLen      = strS->n + strD->n;
    if (newLen >= strD->m)
    {
        bytesToCopy = strD->m - strD->n;
    }

    memcpy((strD->str + strD->n), strS->str, bytesToCopy);
    return bytesToCopy;
}


static int
strnLen(StrN_t *str)
{
    int i = 0;
    while (str->str[i++])
    {
        /* Terminate if exceeded the maximum length */
        if (i >= str->m)
        {
            return -1;
        }
    }
    return i;
}


/* Pack the data into a format to send out. Do not use the printf functions
 * here as the conversions, particularly floats, are too slow.
 */
unsigned int
dataPackageESP_n(const Emon32Dataset_t *pData, char *pDst, const unsigned int m)
{
    StrN_t  strn;       /* Fat string for processed data */

    /* Setup destination string */
    strn.str    = pDst;
    strn.n      = 0;
    strn.m      = m;

    /* Setup conversion string */
    strConv.str = tmpStr;
    strConv.n   = 0;
    strConv.m   = CONV_STR_W;

    /* Clear destination buffer */
    memset(strn.str, 0, m);

    /* "MSG:<xx>"*/
    strn.n += strnCat(&strn, &baseStr[STR_MSG]);
    (void)strnItoa(&strConv, pData->msgNum);
    strn.n += strnCat(&strn, &strConv);

    /* V channels: "[..],V<x>:<yy.y>" */
    for (unsigned int i = 0; i < NUM_V; i++)
    {
        catId(&strn, (i + 1), STR_V);
        (void)strnFtoa(&strConv, pData->pECM->rmsV[i]);
        strn.n += strnCat(&strn, &strConv);
    }

    /* CT channels "[..],P<x>:<yy.y>" */
    for (unsigned int i = 0; i < NUM_CT; i++)
    {
        catId(&strn, (i + 1), STR_P);
        (void)strnItoa(&strConv, pData->pECM->CT[i].realPower);
        strn.n += strnCat(&strn, &strConv);
    }

    /* "[..],E<x>:<yy>" */
    for (unsigned int i = 0; i < NUM_CT; i++)
    {
        catId(&strn, (i + 1), STR_E);
        (void)strnItoa(&strConv, pData->pECM->CT[i].wattHour);
        strn.n += strnCat(&strn, &strConv);
    }

    /* Pulse channels: "[..],pulse<x>:<yy>" */
    for (unsigned int i = 0; i < NUM_PULSECOUNT; i++)
    {
        catId(&strn, (i + 1), STR_PULSE);
        (void)strnItoa(&strConv, pData->pulseCnt[i]);
        strn.n += strnCat(&strn, &strConv);
    }

    /* Temperature sensors: "[..],t<x>:<yy.y>" */
    for (unsigned int i = 0; i < pData->numTempSensors; i++)
    {
        catId(&strn, (i + 1), STR_TEMP);
        (void)strnFtoa(&strConv, pData->temp[i]);
        strn.n += strnCat(&strn, &strConv);
    }

    /* CR-LF */
    strn.n += strnCat(&strn, &baseStr[STR_CRLF]);

    return strn.n;
}


void
dataPackagePacked(const Emon32Dataset_t *pData, PackedData_t *pPacked)
{
    pPacked->msg = pData->msgNum;
    for (unsigned int v = 0; v < NUM_V; v++)
    {
        pPacked->V[v] = qfp_float2int(pData->pECM->rmsV[v]);
    }
}
