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

#include "devADC.h"
#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"


#define ADC_0                   (0U)
#define CHANNEL_0               (0U) // Now PTA8 (2 for PTA9, 0 for PTA12) - this works
#define LED_ON                  (0U)
#define LED_OFF                 (1U)

/*!
 * @brief These values are used to get the temperature. DO NOT MODIFY
 * The method used in this demo to calculate temperature of chip is mapped to
 * Temperature Sensor for the HCS08 Microcontroller Family document (Document Number: AN3031)
 */

#define ADCR_VDD                (0xFFF)    /*! Maximum value when use 12b resolution */
#define V_BG                    (1000U)     /*! BANDGAP voltage in mV (trim to 1.0V) */
#define V_TEMP25                (716U)      /*! Typical VTEMP25 in mV */
#define M                       (1620U)     /*! Typical slope: (mV x 1000)/oC */
#define STANDARD_TEMP           (25)

#define UPPER_VALUE_LIMIT       (1U)        /*! This value/10 is going to be added to current Temp to set the upper boundary*/
#define LOWER_VALUE_LIMIT       (1U)        /*! This Value/10 is going to be subtracted from current Temp to set the lower boundary*/
#define UPDATE_BOUNDARIES_TIME  (20U)       /*! This value indicates the number of cycles needed to update boundaries. To know the Time it will take, multiply this value times LPTMR_COMPARE_VALUE*/
#define kAdcChannelADC0_SE3     (3U)
#define kAdcChannelTemperature  (26U)       /*! ADC channel of temperature sensor */
#define kAdcChannelBandgap      (27U)       /*! ADC channel of BANDGAP */

/* board led color mapping */
#define BOARD_GPIO_LED_RED              kGpioLED1
#define BOARD_GPIO_LED_GREEN            kGpioLED2
#define BOARD_GPIO_LED_BLUE             kGpioLED3

// Define array to keep run-time callback set by application
void (* volatile g_AdcTestCallback[HW_ADC_INSTANCE_COUNT][HW_ADC_SC1n_COUNT])(void);
volatile uint16_t g_AdcValueInt[HW_ADC_INSTANCE_COUNT][HW_ADC_SC1n_COUNT];

///////////////////////////////////////////////////////////////////////////////
// Variables
///////////////////////////////////////////////////////////////////////////////

static uint32_t adcValue = 0;               /*! ADC value */
static uint32_t adcrTemp25 = 0;             /*! Calibrated ADCR_TEMP25 */
static uint32_t adcr100m = 0;
static uint32_t printCounter = 0;
volatile bool conversionCompleted = false;  /*! Conversion is completed Flag */
//const uint32_t gSimBaseAddr[] = SIM_BASE_ADDRS;
static smc_power_mode_config_t smcConfig;

adc16_user_config_t adcUserConfig;
adc16_chn_config_t adcChnConfig;

//extern void init_trigger_source(uint32_t instance);

/* User-defined function to install callback. */
ADC_TEST_InstallCallback(uint32_t instance, uint32_t chnGroup, void (*callbackFunc)(void) )
{
    g_AdcTestCallback[instance][chnGroup] = callbackFunc;
    //conversionCompleted = true;
}

//uint16_t ADC_TEST_GetConvValueRAWInt(uint32_t instance, uint32_t chnGroup)
//{
//    return g_AdcValueInt[instance][chnGroup];
//}
//
///* User-defined ADC ISR. */
//static void ADC16_TEST_IRQHandler(uint32_t instance)
//{
//    uint32_t chnGroup;
//    for (chnGroup = 0U; chnGroup < HW_ADC_SC1n_COUNT; chnGroup++)
//    {
//        if (   ADC16_DRV_GetChnFlag(instance, chnGroup, kAdcChnConvCompleteFlag) )
//        {
//            g_AdcValueInt[instance][chnGroup] = ADC16_DRV_GetConvValueRAW(instance, chnGroup);
//            if ( g_AdcTestCallback[instance][chnGroup] )
//            {
//                (void)(*(g_AdcTestCallback[instance][chnGroup]))();
//            }
//        }
//    }
//}

/* ADC Interrupt Handler */
ADC1IRQHandler(void)
{
    SEGGER_RTT_printf(0, "ping!\n");
    // Get current ADC value
    adcValue = ADC16_DRV_GetConvValueRAW(ADC_0, CHANNEL_0);
    //adcValue = ADC_TEST_GetConvValueRAWInt (ADC_0, CHANNEL_0);
    //ADC16_TEST_IRQHandler(ADC_0);

    // Set conversionCompleted flag. This prevents an wrong conversion in main function
    conversionCompleted = true;
}

/*!
 * Parameters calibration: VDD and ADCR_TEMP25
 */
calibrateParams(void)
{

#if FSL_FEATURE_ADC16_HAS_CALIBRATION
    adc16_calibration_param_t adcCalibraitionParam;
    SEGGER_RTT_printf(0, "You Shouldn't be here...\n");
#endif
    adc16_user_config_t adcUserConfig;
    adc16_chn_config_t adcChnConfig;
    uint32_t bandgapValue = 0;  /*! ADC value of BANDGAP */
    uint32_t vdd = 0;           /*! VDD in mV */

#if FSL_FEATURE_ADC16_HAS_CALIBRATION
    // Auto calibration
    ADC16_DRV_GetAutoCalibrationParam(ADC_0, &adcCalibraitionParam);
    ADC16_DRV_SetCalibrationParam(ADC_0, &adcCalibraitionParam);
#endif

    // Enable BANDGAP reference voltage
    PMC_HAL_SetBandgapBufferCmd(PMC_BASE, true);

    // Initialization ADC for
    // 16bit resolution, interrupt mode, hw trigger disabled.
    // normal convert speed, VREFH/L as reference,
    // disable continuous convert mode.
    ADC16_DRV_StructInitUserConfigDefault(&adcUserConfig);
    adcUserConfig.resolutionMode = kAdcResolutionBitOf12or13;
    adcUserConfig.hwTriggerEnable = false;
    adcUserConfig.continuousConvEnable = false;
    adcUserConfig.clkSrcMode = kAdcClkSrcOfAsynClk;
    ADC16_DRV_Init(ADC_0, &adcUserConfig);

#if FSL_FEATURE_ADC16_HAS_HW_AVERAGE
    ADC16_DRV_EnableHwAverage(ADC_0, kAdcHwAverageCountOf32);
#endif // FSL_FEATURE_ADC16_HAS_HW_AVERAGE

    adcChnConfig.chnNum = kAdcChannelBandgap;
    adcChnConfig.diffEnable = false;
    adcChnConfig.intEnable = false;
    //adcChnConfig.chnMux = kAdcChnMuxOfA;
    ADC16_DRV_ConfigConvChn(ADC_0, CHANNEL_0, &adcChnConfig);

    // Wait for the conversion to be done
    ADC16_DRV_WaitConvDone(ADC_0, CHANNEL_0);

    // Get current ADC BANDGAP value
    bandgapValue = ADC16_DRV_GetConvValueRAW(ADC_0, CHANNEL_0);
    bandgapValue = ADC16_DRV_ConvRAWData(bandgapValue, false, adcUserConfig.resolutionMode);

    // ADC stop conversion
    ADC16_DRV_PauseConv(ADC_0, CHANNEL_0);

#if FSL_FEATURE_ADC16_HAS_HW_AVERAGE
    ADC16_DRV_DisableHwAverage(ADC_0);
#endif // FSL_FEATURE_ADC16_HAS_HW_AVERAGE

    // Disable BANDGAP reference voltage
    PMC_HAL_SetBandgapBufferCmd(PMC_BASE, false);

}

static int32_t initADC(uint32_t instance)
{

#if FSL_FEATURE_ADC16_HAS_CALIBRATION
    adc16_calibration_param_t adcCalibraitionParam;
#endif


#if FSL_FEATURE_ADC16_HAS_CALIBRATION
    // Auto calibration
    ADC16_DRV_GetAutoCalibrationParam(instance, &adcCalibraitionParam);
    ADC16_DRV_SetCalibrationParam(instance, &adcCalibraitionParam);
#endif

    // Initialization ADC for
    // 12bit resolution, interrupt mode, hw trigger enabled.
    // normal convert speed, VREFH/L as reference,
    // disable continuous convert mode.
    ADC16_DRV_StructInitUserConfigDefault(&adcUserConfig);
    adcUserConfig.intEnable = true;
    adcUserConfig.resolutionMode = kAdcResolutionBitOf12or13;
    adcUserConfig.hwTriggerEnable = false; //prev. true
    adcUserConfig.continuousConvEnable = false;
    adcUserConfig.clkSrcMode = kAdcClkSrcOfAsynClk;
    ADC16_DRV_Init(instance, &adcUserConfig);


    // Install Callback function into ISR
    ADC_TEST_InstallCallback(instance, CHANNEL_0, ADC1IRQHandler);
    
    adcChnConfig.chnNum = kAdcChannelADC0_SE3;
    //adcChnConfig.chnNum = kAdcChannelBandgap;
    adcChnConfig.diffEnable = false;
    adcChnConfig.intEnable = false; 

    // Configure channel0
    ADC16_DRV_ConfigConvChn(instance, CHANNEL_0, &adcChnConfig);

    // Wait for the conversion to be done
    ADC16_DRV_WaitConvDone(instance, CHANNEL_0);
    // Get current ADC BANDGAP value
    adcValue = ADC16_DRV_GetConvValueRAW(instance, CHANNEL_0);
    adcValue = ADC16_DRV_ConvRAWData(adcValue, false, adcUserConfig.resolutionMode);
    SEGGER_RTT_printf(0, " adcValue: %d\n", adcValue);
    return 0;
}

void configureADC(void)
{
    calibrateParams();

    // Initialize ADC
    if (initADC(ADC_0))
    {
        printf("Failed to do the ADC init\n");
        return -1;
    }

}

uint32_t printSensorDataADC(bool hexModeFlag)
{
    SEGGER_RTT_printf(0, "%4d | ", printCounter);

    ADC16_DRV_ConfigConvChn(ADC_0, CHANNEL_0, &adcChnConfig);

    SEGGER_RTT_printf(0, "C | ");

    // Wait for the conversion to be done
    ADC16_DRV_WaitConvDone(ADC_0, CHANNEL_0);
    
    adcValue = ADC16_DRV_GetConvValueRAW(ADC_0, CHANNEL_0);
//    adcValue = ADC16_DRV_ConvRAWData(adcValue, false, kAdcResolutionBitOf12or13);

    SEGGER_RTT_printf(0, "bandgap method: %4d | ", adcValue);

    //uint16_t readSensorRegisterValueLSB;
    //uint16_t readSensorRegisterValueMSB;
    //int *LSBaddress = (int *) 0x4003B010;
    //int *MSBaddress = (int *) 0x4003B014;
    //int16_t readSensorRegisterValueCombined;
//
//    //SEGGER_RTT_printf(0, "Attempting to read addresses...\n");
//    readSensorRegisterValueLSB = *LSBaddress;
//    readSensorRegisterValueMSB = *MSBaddress;
//    SEGGER_RTT_printf(0, " Pointer method: %4x %4x | ", readSensorRegisterValueMSB, readSensorRegisterValueLSB);

    SEGGER_RTT_printf(0, "End");
    printCounter++;

    // Clear conversionCompleted flag
    conversionCompleted = false;
    return adcValue;
}

uint32_t getSensorDataADC(bool hexModeFlag)
{
    // Prevents the use of wrong values
    ADC16_DRV_ConfigConvChn(ADC_0, CHANNEL_0, &adcChnConfig);

    // Wait for the conversion to be done
    ADC16_DRV_WaitConvDone(ADC_0, CHANNEL_0);

    adcValue = ADC16_DRV_GetConvValueRAW(ADC_0, CHANNEL_0);

    // Clear conversionCompleted flag
    conversionCompleted = false;
    return adcValue;
}