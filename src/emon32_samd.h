#pragma once

#include "board_def.h"

#if (BOARD_ID == BOARD_ID_LC)
    #include "samd11.h"
#else
    #include "samd21.h"
#endif /* BOARD_ID */

