#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#define CRYSTALLESS 1

#define VENDOR_NAME "OpenEnergyMonitor"
#define PRODUCT_NAME "emonPi3"
#define VOLUME_LABEL "EMONBOOT"
#define INDEX_URL "https://openenergymonitor.org"

#define BOARD_ID "SAMD21J17A-emon32"

//#define USB_VID 0x2341
//#define USB_PID 0x024D

#define LED_PIN PIN_PB23

// Only have 128KB flash, rather than x18A's 256KB
#define FLASH_NUM_ROWS 512

#endif
