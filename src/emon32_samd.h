#ifndef EMON32_SAMD_H
#define EMON32_SAMD_H

#include "board_def.h"

#if (BOARD_ID == BOARD_ID_LC)
    #include "samd11.h"
#else
    #include "samd21.h"
#endif /* BOARD_ID */

#include "emon32.h"

#include "driver_ADC.h"
#include "driver_CLK.h"
#include "driver_DMAC.h"
#include "driver_EIC.h"
#include "driver_EVSYS.h"
#include "driver_PORT.h"
#include "driver_SAMD.h"
#include "driver_SERCOM.h"
#include "driver_TIME.h"
#include "driver_WDT.h"

#include "configuration.h"
#include "data_pack.h"
#include "eeprom.h"
#include "emon_CM.h"
#include "periph_rfm69.h"
#include "util.h"

#endif
