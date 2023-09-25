#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <float.h>

#include "data_pack.h"
#include "util.h"

int
main(int argc, char *argv[])
{
    ECMSet_t dataset = {0};
    char txBuffer[1024];
    char goldBuffer[1024];
    unsigned int dataCnt;
    unsigned int goldCnt;

    printf("----- emon32 data test -----\n\n");
    printf("  Num V:  %d\n", NUM_V);
    printf("  Num CT: %d\n\n", NUM_CT);

    printf("  dataPackage(_n) ... ");

    dataset.msgNum = 0;
    dataset.rmsV[0] = 0.0f;
    for (unsigned int i = 0; i < NUM_CT; i++)
    {
        dataset.CT[i].realPower = 0.0f;
        dataset.CT[i].wattHour = 0;
    }

    dataCnt = dataPackageESP_n(&dataset, txBuffer, 1024);
    goldCnt = snprintf(goldBuffer, 1024, "MSG:%d,Vrms:%.2f,P1:%.2f,E1:%d,P2:%.2f,E2:%d,P3:%.2f,E3:%d,P4:%.2f,E4:%d",
                   dataset.msgNum, dataset.rmsV[0], dataset.CT[0].realPower, dataset.CT[0].wattHour, dataset.CT[1].realPower, dataset.CT[1].wattHour, dataset.CT[2].realPower, dataset.CT[2].wattHour, dataset.CT[3].realPower, dataset.CT[3].wattHour);
    if (0 != strcmp(txBuffer, goldBuffer) || dataCnt != goldCnt)
    {
        printf("  all data == 0:\n");
        printf("    Test: %s\n", txBuffer);
        printf("    Gold: %s\n", goldBuffer);
        printf("    %d | %d\n", dataCnt, goldCnt);
        assert(0);
    }

    memset(&dataset, 'F', sizeof(ECMSet_t));
    dataset.msgNum = UINT32_MAX;
    dataset.rmsV[0] = 2E6;
    for (unsigned int i = 0; i < NUM_CT; i++)
    {
        dataset.CT[i].realPower = 2E6;
        dataset.CT[i].wattHour = UINT32_MAX;
    }

    dataCnt = dataPackageESP_n(&dataset, txBuffer, 1024);
    goldCnt = snprintf(goldBuffer, 1024, "MSG:%d,Vrms:%.2f,P1:%.2f,E1:%d,P2:%.2f,E2:%d,P3:%.2f,E3:%d,P4:%.2f,E4:%d",
                   dataset.msgNum, dataset.rmsV[0], dataset.CT[0].realPower, dataset.CT[0].wattHour, dataset.CT[1].realPower, dataset.CT[1].wattHour, dataset.CT[2].realPower, dataset.CT[2].wattHour, dataset.CT[3].realPower, dataset.CT[3].wattHour);
    if (0 != strcmp(txBuffer, goldBuffer) || dataCnt != goldCnt)
    {
        printf("  all data == MAX:\n");
        printf("    Test: %s\n", txBuffer);
        printf("    Gold: %s\n", goldBuffer);
        printf("    %d | %d\n", dataCnt, goldCnt);
        assert(0);
    }

    printf("PASSED\n\n");

}
