#include <stdio.h>

#include "util.h"

void
utilStrReverse(char *pBuf, unsigned int len)
{
    char tmp;
    unsigned int idxEnd = len - 1u;
    for (unsigned int idx = 0; idx < (len / 2); idx++)
    {
        tmp = pBuf[idx];
        pBuf[idx] = pBuf[idxEnd];
        pBuf[idxEnd] = tmp;
        idxEnd--;
    }
}


unsigned int
utilStrlen(const char *pBuf)
{
    unsigned int charCnt = 0;
    while (*pBuf++)
    {
        charCnt++;
    }
    return charCnt;
}


unsigned int
utilItoa(char *pBuf, int32_t val, ITOA_BASE_t base)
{
    unsigned int charCnt = 0;
    unsigned int isNegative = 0;
    char * const pBase = pBuf;

    /* Handle 0 explicitly */
    if (0 == val)
    {
        *pBuf++ = '0';
        *pBuf = '\0';
        return 2u;
    }

    /* Base 10 can be signed, and has a divide in */
    if (ITOA_BASE10 == base)
    {
        if (val < 0)
        {
            isNegative = 1u;
            val = -val;
        }

        while (0 != val)
        {
            *pBuf++ = (val % 10u) + '0';
            val = val / 10u;
            charCnt++;
        }

        if (0 != isNegative)
        {
            *pBuf++ = '-';
            charCnt++;
        }
    }
    else
    {
        const char itohex[] = "0123456789abcdef";
        uint32_t val_u = (uint32_t)val;

        while (0 != val_u)
        {
            *pBuf++ = itohex[(val_u & 0xFu)];
            val_u >>= 4;
            charCnt++;
        }
    }

    /* Terminate and return */
    *pBuf = '\0';
    charCnt++;

    utilStrReverse(pBase, charCnt - 1u);
    return charCnt;
}


int32_t
utilAtoi(char *pBuf, ITOA_BASE_t base)
{
    unsigned int isNegative = 0u;
    unsigned int len;
    unsigned int mulCnt = 1;
    int32_t val = 0;

    if ('-' == *pBuf)
    {
        isNegative = 1u;
        pBuf++;
    }

    /* Reverse string and convert */
    len = utilStrlen(pBuf);
    utilStrReverse(pBuf, len);

    if (ITOA_BASE10 == base)
    {
        while (*pBuf)
        {
            val += ((*pBuf++) - '0') * mulCnt;
            mulCnt *= 10;
        }
        if (1u == isNegative)
        {
            val = -val;
        }
    }
    else
    {
        while (*pBuf)
        {
            if ('a' <= *pBuf)
            {
                val += ((*pBuf) - 'a' + 10u) * mulCnt;
            }
            else
            {
                val += ((*pBuf) - '0') * mulCnt;
            }
            pBuf++;
            mulCnt *= 16;
        }
    }

    return val;
}


unsigned int
utilFtoa(char *pBuf, float val)
{
    unsigned int charCnt = 0;
    unsigned int isNegative = 0;
    char * const pBase = pBuf;

    uint16_t    decimals;
    int         units;

    if (val < 0.0)
    {
        isNegative = 1u;
        val = -val;
    }
    decimals = (int)(val * 100) % 100;
    units = (int)val;

    charCnt += 3u;
    *pBuf++ = (decimals % 10) + '0';
    decimals = decimals / 10;
    *pBuf++ = (decimals % 10) + '0';
    *pBuf++ = '.';

    if (0 == units)
    {
        *pBuf++ = '0';
        charCnt++;
    }

    while (0 != units)
    {
        *pBuf++ = (units % 10) + '0';
        units = units / 10;
        charCnt++;
    }

    if (0 != isNegative)
    {
        *pBuf++ = '-';
        charCnt++;
    }
    utilStrReverse(pBase, charCnt);
    return charCnt;
}


float
utilAtof(char *pBuf)
{
    unsigned int isNegative = 0u;
    unsigned int len = 0;
    unsigned int mulCnt = 1u;
    unsigned int fraction = 0u;
    float val = 0.0f;

    if ('-' == *pBuf)
    {
        isNegative = 1u;
        pBuf++;
    }
    len = utilStrlen(pBuf);
    utilStrReverse(pBuf, len);

    while (*pBuf)
    {
        const char c = *pBuf++;
        /* Allow period/comma delimit, divide down if found */
        if ('.' != c && ',' != c)
        {
            const float toAdd = (float)(c - '0') * mulCnt;
            val += toAdd;
            mulCnt *= 10;
        }
        else
        {
            fraction = mulCnt;
        }
    }

    if (0 != fraction)
    {
        val = val / fraction;
    }

    if (isNegative)
    {
        val = -val;
    }
    return val;
}
