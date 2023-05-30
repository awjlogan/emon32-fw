#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <stdint.h>
#include <limits.h>
#include <stdlib.h>
#include <time.h>

#include "util.h"

/* After directed tests, run some random values */
#define RANDOM_TESTS    1024ul

void
util_strlen(const char *str)
{
    unsigned int strlen_gold, strlen_util;

    strlen_gold = strlen(str);
    strlen_util = utilStrlen(str);
    if (strlen_util != strlen_gold)
    {
        printf("    %s : %d\n", str, strlen_util);
        assert(strlen_util == strlen_gold);
    }
}

void
util_itoa()
{
    int32_t vals[5] = {0, -1, 1, INT32_MAX, INT32_MIN};
    uint32_t valtohex[4] = {0, 1, 255, UINT32_MAX};
    char abuf_gold[24], abuf_util[24];

    printf("    base 10 ... ");
    for (unsigned int idx = 0; idx < 5; idx++)
    {
        int32_t val = vals[idx];
        memset(abuf_util, 0, 24);
        memset(abuf_gold, 0, 24);
        snprintf(abuf_gold, 24, "%d", val);
        utilItoa(abuf_util, val, ITOA_BASE10);
        if (0 != strcmp(abuf_gold, abuf_util))
        {
            printf("    %d : %s\n", val, abuf_util);
            assert(!(strcmp(abuf_gold, abuf_util)));
        }
    }

    for (unsigned int idx = 0; idx < RANDOM_TESTS; idx++)
    {
        int32_t val = rand() % UINT32_MAX - INT32_MAX;
        memset(abuf_util, 0, 24);
        memset(abuf_util, 0, 24);
        snprintf(abuf_gold, 24, "%d", val);
        utilItoa(abuf_util, val, ITOA_BASE10);
        if (0 != strcmp(abuf_gold, abuf_util))
        {
            printf("    %d : %s\n", val, abuf_util);
            assert(!(strcmp(abuf_gold, abuf_util)));
        }
    }
    printf(" PASSED\n");

    printf("    base 16 ... ");
    for (unsigned int idx = 0; idx < 4; idx++)
    {
        uint32_t val = valtohex[idx];
        memset(abuf_util, 0, 24);
        memset(abuf_gold, 0, 24);
        snprintf(abuf_gold, 24, "%x", val);
        utilItoa(abuf_util, val, ITOA_BASE16);
        if (0 != strcmp(abuf_gold, abuf_util))
        {
            printf("  %d : %s\n", val, abuf_util);
            assert(!(strcmp(abuf_gold, abuf_util)));
        }
    }

    for (unsigned int idx = 0; idx < RANDOM_TESTS; idx++)
    {
        uint32_t val = rand() % UINT32_MAX;
        memset(abuf_util, 0, 24);
        memset(abuf_util, 0, 24);
        snprintf(abuf_gold, 24, "%x", val);
        utilItoa(abuf_util, val, ITOA_BASE16);
        if (strcmp(abuf_gold, abuf_util))
        {
            printf("    %d : %s\n", val, abuf_util);
            assert(!(strcmp(abuf_gold, abuf_util)));
        }
    }
    printf(" PASSED\n\n");
}

void
atoiTest10(char *pBuf)
{
    int gold = atoi(pBuf);
    int test = utilAtoi(pBuf, ITOA_BASE10);
    if (gold != test)
    {
        printf("  %s : %d\n", pBuf, test);
        assert(0);
    }
}

void
atoiTest16(char *pBuf)
{
    int gold = strtoul(pBuf, NULL, 16);
    int test = utilAtoi(pBuf, ITOA_BASE16);
    if (gold != test)
    {
        printf("  %s : %d (%d)\n", pBuf, test, gold);
        assert(0);
    }
}

void
util_atoi()
{
    char txt0[] = "0";
    char txt1[] = "1";
    char txt_1[] = "-1";
    char txt1000[] = "1000";
    char txt_1000[] = "-1000";

    printf("    base 10 ... ");
    atoiTest10(txt0);
    atoiTest10(txt1);
    atoiTest10(txt_1);
    atoiTest10(txt1000);
    atoiTest10(txt_1000);
    printf(" PASSED \n");

    printf("    base 16 ... ");
    char txtxFF[] = "ff";
    atoiTest10(txt0);
    atoiTest16(txt1);
    atoiTest16(txtxFF);

    printf(" PASSED \n\n");
}

void
atofTest(char *pBuf)
{
    float gold = strtof(pBuf, NULL);
    float test = utilAtof(pBuf);
    if (gold != test)
    {
        printf("  %s : %.6f (%.6f)\n", pBuf, test, gold);
        assert(0);
    }
}

void
util_atof()
{
    char txt0[] = "0";
    char txt1[] = "1";
    char txt2[] = "240.64";
    char txt3[] = "-100";
    atofTest(txt0);
    atofTest(txt1);
    atofTest(txt2);
    atofTest(txt3);
}

void
util_ftoa()
{
    float tests[5] = {0.0, -100.0, 100.0, 123.45, -123.45};
    unsigned int testLen, goldLen;
    char testBuf[16], goldBuf[16];
    for (unsigned int i = 0; i < 5; i++)
    {
        memset(testBuf, 0, 16);
        memset(goldBuf, 0, 16);
        testLen = utilFtoa(testBuf, tests[i]);
        goldLen = snprintf(goldBuf, 16, "%.2f", tests[i]);

        if ( strcmp(goldBuf, testBuf) || (testLen != goldLen) )
        {
            printf("\n\n  Test: %s (%d)\n", testBuf, testLen);
            printf("  Gold: %s (%d)\n", goldBuf, goldLen);
            assert(0);
        }
    }
}

void
util_insert(const char *ins, const char *gold, unsigned int pos, unsigned int len, unsigned int gold_cursor)
{
    char base_str[] = "This is an existing string to insert into";
    unsigned int cursor = utilStrInsert(base_str, ins, pos, len);

    if ((strcmp(gold, base_str)) || (gold_cursor != cursor))
    {
        printf("\n    %s (%s) | %d (%d) \n", base_str, gold, cursor, gold_cursor);
        assert(0);
    }
}

int
main(int argc, char *argv[])
{
    srand(time(0));

    printf("---- emon32 util test ----\n\n");

    /* Returns the length of a NULL terminated string excluding termination */
    printf("  utilStrlen ... ");
    const char str0[] = "";
    const char str5[] = "TEST5";
    const char str32[] = "01234567890123456789012345678901";
    util_strlen(str0);
    util_strlen(str5);
    util_strlen(str32);
    printf("PASSED\n\n");

    /* Insert a string into an existing buffer */
    printf("  utilStrInsert ... ");
    util_insert("MSG", "MSGs is an existing string to insert into", 0, 3, 3);
    util_insert("012345", "This is an existing 012345 to insert into", 20, 6, 26);
    util_insert("IS", "This IS an existing string to insert into", 5, 2, 7);
    printf("PASSED\n\n");

    /* Convert an up to 32 bit value to string */
    printf("  utilItoa:\n");
    util_itoa();

    /* Convert a string to 32 bit value */
    printf("  utilAtoi:\n");
    util_atoi();

    /* Convert strings to float */
    printf("  utilAtof ... ");
    util_atof();
    printf("PASSED\n\n");

    /* Convert float to string */
    printf("  utilFtoa ... ");
    util_ftoa();
    printf("PASSED\n\n");
}
