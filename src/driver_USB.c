#include "emon32_samd.h"
#include "board_def.h"
#include "driver_PORT.h"
#include "driver_USB.h"

#include "configuration.h"
#include "tusb.h"


bool
usbCDCIsConnected(void)
{
    return tud_cdc_connected();
}


bool
usbCDCPutsBlocking(const char *s)
{
    /* REVISIT does this need timeout protection? */
    int count_c = 0;
    while (*s)
    {
        tud_cdc_write_char(*s++);

        /* Flush every 64 characters */
        if (63 == (count_c % 64))
        {
            tud_cdc_write_flush();
        }
        count_c++;
    }
    tud_cdc_write_flush();

    /* REVISIT more informative exit status */
    return true;
}


bool usbCDCRxAvailable(void)
{
    return (tud_cdc_available() > 0);
}


uint8_t usbCDCRxGetChar(void)
{
    uint8_t c = 0;
    int ch = tud_cdc_read_char();
    if (-1 != ch)
    {
        c = (uint8_t)ch;
    }

    return c;
}


void
usbCDCTask(void)
{
    tud_task();
    if (tud_cdc_available())
    {
        uint8_t buf[64];
        int     count = tud_cdc_read(buf, sizeof(buf));
        for (int i = 0; i < count; i++)
        {
            configCmdChar(buf[i]);

            /* Exit early if reached a line break as this indicates a command */
            if ('\n' == buf[i])
            {
                break;
            }
        }
    }

    /* Flush any outstanding writes */
    if (tud_cdc_write_available())
    {
        tud_cdc_write_flush();
    }
}


bool
usbCDCTxAvailable(void)
{
    return tud_cdc_write_available() > 0;
}


void
usbCDCTxChar(uint8_t c)
{
    tud_cdc_write_char(c);
}


void
usbCDCTxFlush(void)
{
    tud_cdc_write_flush();
}


void
usbSetup(void)
{
    /* Clocking:
     *  - AHB is enabled by default (16.8.7)
     *  - APB is enabled by default (16.8.9)
     *  - Use the 48 MHz PLL for required accuracy
     */
    GCLK->CLKCTRL.reg =   GCLK_CLKCTRL_ID_USB
                        | GCLK_CLKCTRL_GEN_GCLK0
                        | GCLK_CLKCTRL_CLKEN;

    /* Configure ports (Table 7-1) */
    portPinMux(GRP_USB_DM, PIN_USB_DM, PMUX_USB);
    portPinMux(GRP_USB_DP, PIN_USB_DP, PMUX_USB);

    tusb_init();
}
