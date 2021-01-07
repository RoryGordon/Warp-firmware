#include <stdlib.h>

#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"

#include "fsl_smc_hal.h"
#include "fsl_pmc_hal.h"
//#include "fsl_adc16.h" 
#include "fsl_adc16_driver.h"
//#include "board.h"

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

#define ADC_0                   (0U)
#define CHANNEL_0               (0U)

// Define array to keep run-time callback set by application
void (* volatile g_AdcTestCallback[HW_ADC_INSTANCE_COUNT][HW_ADC_SC1n_COUNT])(void);
volatile uint16_t g_AdcValueInt[HW_ADC_INSTANCE_COUNT][HW_ADC_SC1n_COUNT];

///////////////////////////////////////////////////////////////////////////////
// Variables
///////////////////////////////////////////////////////////////////////////////

static uint32_t adcValue = 0;               /*! ADC value */
static uint32_t adcrTemp25 = 0;             /*! Calibrated ADCR_TEMP25 */
static uint32_t adcr100m = 0;
volatile bool conversionCompleted = false;  /*! Conversion is completed Flag */
//const uint32_t gSimBaseAddr[] = SIM_BASE_ADDRS;
static smc_power_mode_config_t smcConfig;


/* User-defined function to install callback. */
void ADC_TEST_InstallCallback(uint32_t instance, uint32_t chnGroup, void (*callbackFunc)(void) )
{
    g_AdcTestCallback[instance][chnGroup] = callbackFunc;
}

/* User-defined function to read conversion value in ADC ISR. */
uint16_t ADC_TEST_GetConvValueRAWInt(uint32_t instance, uint32_t chnGroup)
{
    return g_AdcValueInt[instance][chnGroup];
}


/* ADC Interrupt Handler */
void ADC1IRQHandler(void)
{
    // Get current ADC value
    adcValue = ADC_TEST_GetConvValueRAWInt (ADC_0, CHANNEL_0);
    // Set conversionCompleted flag. This prevents an wrong conversion in main function
    conversionCompleted = true;
}

static int32_t initADC(uint32_t instance)
{

#if FSL_FEATURE_ADC16_HAS_CALIBRATION
    adc16_calibration_param_t adcCalibraitionParam;
#endif
    adc16_user_config_t adcUserConfig;
    adc16_chn_config_t adcChnConfig;

#if FSL_FEATURE_ADC16_HAS_CALIBRATION
    // Auto calibration
    ADC16_DRV_GetAutoCalibrationParam(instance, &adcCalibraitionParam);
    ADC16_DRV_SetCalibrationParam(instance, &adcCalibraitionParam);
#endif

    // Initialization ADC for
    // 16bit resolution, interrupt mode, hw trigger enabled.
    // normal convert speed, VREFH/L as reference,
    // disable continuous convert mode.
    ADC16_DRV_StructInitUserConfigDefault(&adcUserConfig);
    adcUserConfig.intEnable = true;
    adcUserConfig.resolutionMode = kAdcResolutionBitOf12or13;
    adcUserConfig.hwTriggerEnable = true;
    adcUserConfig.continuousConvEnable = false;
    adcUserConfig.clkSrcMode = kAdcClkSrcOfAsynClk;
    ADC16_DRV_Init(instance, &adcUserConfig);

    /*
    // Install Callback function into ISR
    ADC_TEST_InstallCallback(instance, CHANNEL_0, ADC1IRQHandler);

    //adcChnConfig.chnNum = kAdcChannelTemperature;
    adcChnConfig.diffEnable = false;
    adcChnConfig.intEnable = true;
    //adcChnConfig.chnMux = kAdcChnMuxOfA;

    // Configure channel0
    ADC16_DRV_ConfigConvChn(instance, CHANNEL_0, &adcChnConfig);
    */
    return 0;
}


void configureADC(void)
{
    initADC(ADC_0);
    SEGGER_RTT_printf(0, "Have you drank your milk today?");
}

printSensorDataADC(bool hexModeFlag)
{
    /*
    What we know so far:
        Straight-up trying to read the supposed addresses of the ADC completely
        nuggets everything. So, don't do that
    */
    adcValue = ADC_TEST_GetConvValueRAWInt (ADC_0, CHANNEL_0);
    SEGGER_RTT_printf(0, "%d", adcValue);
    /*
    SEGGER_RTT_printf(0, "Start printing...\n");
    uint16_t readSensorRegisterValueLSB;
    uint16_t readSensorRegisterValueMSB;
    int *LSBaddress = (int *) 0x4003B010;
    int *MSBaddress = (int *) 0x4003B010;
    int16_t readSensorRegisterValueCombined;

    SEGGER_RTT_printf(0, "Attempting to read adresses...\n");
    readSensorRegisterValueLSB = *LSBaddress;
    readSensorRegisterValueMSB = *MSBaddress;
    SEGGER_RTT_printf(0, "%d %d,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
    */
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