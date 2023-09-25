#ifndef PERIPH_DS18B20_H
#define PERIPH_DS18B20_H

/* The DS18B20 can be configured for 9-12 bit resolution conversions. This
 * changes the conversion time (AC Electrical Characteristics)
 */
typedef enum {
    DS18B20_CONV_9,
    DS18B20_CONV_10,
    DS18B20_CONV_11,
    DS18B20_CONV_12
} DS18B20_CONV_t;

typedef struct ds18b20_conf {
    unsigned int    grp;
    unsigned int    pin;
    unsigned int    *tempAddr;
    DS18B20_CONV_t  conv;
} DS18B20_conf_t;

typedef enum {
    DS18B20_IDLE,
    DS18B20_RESET,
    DS18B20_WRITE,
    DS18B20_READ
} DS18B20_STATE_t;

/*! @brief Configure the OneWire port
 *  @param [in] pCfg: pointer to the configuration struct
 */
void tempSetup(const DS18B20_conf_t *pCfg);

void tempConfigure(const DS18B20_conf_t *pCfg);

/*! @brief Start a temperature conversion
 *  @param [in] addr: DS18B20 address
 *  @return : -1 if already in use, 0 if complete, 1 if on-going
 */
int tempSense(const uint64_t *addr);

#endif