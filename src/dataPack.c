#include <inttypes.h>
#include <stdbool.h>
#include <string.h>

#include "dataPack.h"
#include "emon32_assert.h"
#include "util.h"

#include "qfplib-m0-full.h"
#include "qfpio.h"


#define CONV_STR_W  16
#define STR_MSG     0
#define STR_V       1
#define STR_P       2
#define STR_E       3
#define STR_PULSE   4
#define STR_TEMP    5
#define STR_COLON   6
#define STR_CRLF    7
#define STR_DQUOTE  8
#define STR_LCURL   9
#define STR_RCURL   10
#define STR_COMMA   11


/* "Fat" string with current length and buffer size. */
typedef struct StrN {
    char    *str;   /* Pointer to the string */
    int     n;      /* Length of the string  */
    int     m;      /* Buffer length */
} StrN_t;


static void catId(StrN_t *strD, int id, int field, bool json);
static void catMsg(StrN_t *strD, int msg, bool json);
static void initFields(StrN_t *pD, char *pS, const int m);
static int  strnFtoa(StrN_t *strD, const float v);
static int  strnItoa(StrN_t *strD, const uint32_t v);
static int  strnCat(StrN_t *strD, const StrN_t *strS);
static int  strnLen(StrN_t *str);


static char     tmpStr[CONV_STR_W] = {0};
static StrN_t   strConv;    /* Fat string for conversions */

/* Strings that are inserted in the transmitted message */
const StrN_t baseStr[12] = { {.str = "MSG",     .n = 3, .m = 4},
                            {.str = "V",        .n = 1, .m = 2},
                            {.str = "P",        .n = 1, .m = 2},
                            {.str = "E",        .n = 1, .m = 2},
                            {.str = "pulse",    .n = 5, .m = 6},
                            {.str = "t",        .n = 1, .m = 2},
                            {.str = ":",        .n = 1, .m = 2},
                            {.str = "\r\n",     .n = 2, .m = 3},
                            {.str = "\"",       .n = 1, .m = 2},
                            {.str = "{",        .n = 1, .m = 2},
                            {.str = "}",        .n = 1, .m = 2},
                            {.str = ",",        .n = 1, .m = 2}};


/*! @brief "Append <field><id>:" to the string
 *  @param [out] strD : pointer to the fat string
 *  @param [in] id : numeric index
 *  @param [in] field : field name index, e.g. "STR_V"
 */
static void
catId(StrN_t *strD, int id, int field, bool json)
{
    strD->n += strnCat(strD, &baseStr[STR_COMMA]);
    if (json)
    {
        strD->n += strnCat(strD, &baseStr[STR_DQUOTE]);
    }
    strD->n += strnCat(strD, &baseStr[field]);
    (void)strnItoa(&strConv, id);
    strD->n += strnCat(strD, &strConv);
    if (json)
    {
        strD->n += strnCat(strD, &baseStr[STR_DQUOTE]);
    }
    strD->n += strnCat(strD, &baseStr[STR_COLON]);
}


static void
catMsg(StrN_t *strD, int msg, bool json)
{
    /* <{">MSG<">:<"><#><"> */

    if (json)
    {
        strD->n += strnCat(strD, &baseStr[STR_LCURL]);
        strD->n += strnCat(strD, &baseStr[STR_DQUOTE]);
    }
    strD->n += strnCat(strD, &baseStr[STR_MSG]);
    if (json)
    {
        strD->n += strnCat(strD, &baseStr[STR_DQUOTE]);
    }
    strD->n += strnCat(strD, &baseStr[STR_COLON]);
    (void)strnItoa(&strConv, msg);
    strD->n += strnCat(strD, &strConv);
}


static void
initFields(StrN_t *pD, char *pS, const int m)
{
    /* Setup destination string */
    pD->str = pS;
    pD->n   = 0;
    pD->m   = m;
    memset(pD->str, 0, m);

    /* Setup conversion string */
    strConv.str = tmpStr;
    strConv.n   = 0;
    strConv.m   = CONV_STR_W;
}

static int
strnFtoa(StrN_t *strD, const float v)
{
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


int
dataPackSerial(const Emon32Dataset_t *pData, char *pDst, int m, bool json)
{
    EMON32_ASSERT(pData);
    EMON32_ASSERT(pDst);

    StrN_t strn;
    initFields(&strn, pDst, m);

    catMsg(&strn, pData->msgNum, json);

    /* V channels */
    for (int i = 0; i < NUM_V; i++)
    {
        catId(&strn, (i + 1), STR_V, json);
        /* Voltages are scaled 0.01 for transmission */
        (void)strnFtoa(&strConv, qfp_fmul(pData->pECM->rmsV[i], 0.01f));
        strn.n += strnCat(&strn, &strConv);
    }

    /* CT channels (power and energy) */
    for (int i = 0; i < NUM_CT; i++)
    {
        catId(&strn, (i + 1), STR_P, json);
        (void)strnItoa(&strConv, pData->pECM->CT[i].realPower);
        strn.n += strnCat(&strn, &strConv);
    }
    for (int i = 0; i < NUM_CT; i++)
    {
        catId(&strn, (i + 1), STR_E, json);
        (void)strnItoa(&strConv, pData->pECM->CT[i].wattHour);
        strn.n += strnCat(&strn, &strConv);
    }

    /* REVIST : pulse and temperature */

    /* Terminate with } for JSON and \r\n */
    if (json)
    {
        strn.n += strnCat(&strn, &baseStr[STR_RCURL]);
    }
    strn.n += strnCat(&strn, &baseStr[STR_CRLF]);
    return strn.n;
}


void
dataPackPacked(const Emon32Dataset_t *pData, PackedData_t *pPacked)
{
    pPacked->msg = pData->msgNum;
    /* Vrms is sent as 0.01 scaling */
    for (unsigned int v = 0; v < NUM_V; v++)
    {
        pPacked->V[v] = qfp_float2int(qfp_fmul(pData->pECM->rmsV[v], 100.0f));
    }

}
