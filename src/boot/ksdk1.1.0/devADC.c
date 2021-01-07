#include <stdlib.h>

#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
//#include "fsl_i2c_master_driver.h"
//#include "fsl_spi_master_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"

/*
As the ADC isn't I2C, half of the usual stuff should be able to be ignored

I think it's just a case of reading the two adresses 0x4003B010 and 0x4003B014
for the outputs, but I could be spectacularly wrong

It seems PTA12 is the default input, but there could be some dodgy stuff with a 
resistor to some part of the accelerometer package - hopefully this can be
sorted if it is an issue

May need to set ADCH to 0000 and MODE to 01 - forgotten what these mean but
this will go in the init function if any
*/
void
initADC(void)
{
    
}

void
configureADC(void)
{

}

printSensorDataADC(bool hexModeFlag)
{
    /*
    What we know so far:
        Straight-up trying to read the supposed addresses of the ADC completely
        nuggets everything. So, don't do that
    */
    SEGGER_RTT_printf(0, "Start printing...\n");
    uint16_t readSensorRegisterValueLSB;
    uint16_t readSensorRegisterValueMSB;
    int *LSBaddress = (int *) 0x4003B010;
    int *MSBaddress = (int *) 0x4003B010;
    int16_t readSensorRegisterValueCombined;

    SEGGER_RTT_printf(0, "Attempting to read adresses...\n");
    readSensorRegisterValueLSB = *LSBaddress;
    readSensorRegisterValueMSB = *MSBaddress;
    /*
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 8) | (readSensorRegisterValueLSB);

    if (hexModeFlag)
    {
        SEGGER_RTT_printf(0, " 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
    }
    else
    {
        SEGGER_RTT_printf(0, " %d,", readSensorRegisterValueCombined);
    }
    */
    SEGGER_RTT_printf(0, "End printing...\n");

}