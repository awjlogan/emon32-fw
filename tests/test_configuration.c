#include <stdio.h>
#include <string.h>

#include "configuration.h"

int
main(int argc, char *argv[])
{
    Emon32Config_t config;
    config.baseCfg.nodeID = 15u;
    config.baseCfg.reportCycles = 500u;
    config.baseCfg.mainsFreq = 50u;
    config.voltageCfg[0].voltageCal = 240.63;

    printf("---- emon32 configuration test ----\n\n");
    configEnter(&config);
}
