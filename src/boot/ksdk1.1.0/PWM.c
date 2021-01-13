/*
 * functions regarding the generation of the PWM signal
 */
#include <stdlib.h>

//#include "fsl_device_registers.h"
#include "fsl_port_hal.h"
#include "fsl_tpm_driver.h"


#include "SEGGER_RTT.h"

#define TPM_0 (0U)
#define PWM_CHANNEL (0U)

#define PWM_BASE_ADDRESS  (0x4003)
#define PWM_VALUE_ADDRESS (0x8010)

tpm_general_config_t PwmGConfig = {
    .isDBGMode = false,
    .isGlobalTimeBase = true,
    .isTriggerMode = false,
    .isStopCountOnOveflow = false,
    .isCountReloadOnTrig = true,
    .triggerSource = kTpmExtTrig,
};

tpm_pwm_param_t PwmParams = {
    .mode = kTpmEdgeAlignedPWM,
    .edgeMode = kTpmHighTrue,
    .uFrequencyHZ = 240000U,
    .uDutyCyclePercent = 50U,
};


void initPWM(void)
{
    TPM_DRV_Init(TPM_0, &PwmGConfig);

    TPM_DRV_SetClock(TPM_0, kTpmClockSourceModuleClk, kTpmDividedBy1);
    SEGGER_RTT_printf(0, "\tInit complete - duty cycle = %d\n",
        PwmParams.uDutyCyclePercent);
    TPM_DRV_PwmStart(TPM_0, &PwmParams, PWM_CHANNEL);
    SEGGER_RTT_printf(0, "Clock mode: %d", TPM_HAL_GetClockMode(g_tpmBaseAddr[TPM_0]));
    //TPM_DRV_PwmStart(TPM_0, &PwmParams, PWM_CHANNEL);
}

void writeToPWM(uint16_t output)
{
    PwmParams.uDutyCyclePercent = (10*output) >> 10; // times 10 div 1024 is easier than  div 100 :/
    
    if(TPM_DRV_PwmStart(TPM_0, &PwmParams, PWM_CHANNEL))
    {
        //TPM_DRV_SetTimeOverflowIntCmd(TPM_0, true);
        SEGGER_RTT_printf(0, "\tfreq: %d, Input val: %3d, Channel val: %3d\n",
            TPM_DRV_GetClock(TPM_0), PwmParams.uDutyCyclePercent, TPM_DRV_GetChnVal(TPM_0, PWM_CHANNEL));
    }
    else
    {
        SEGGER_RTT_WriteString(0,"PWMStart failed\n");
    }
    
    
}