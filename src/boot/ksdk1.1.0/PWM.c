/*
 * functions regarding the generation of the PWM signal
 */
#include <stdlib.h>

//#include "fsl_device_registers.h"
#include "fsl_port_hal.h"
#include "fsl_tpm_driver.h"


#include "SEGGER_RTT.h"

#define TPM_0 (0U)
#define TPM_1 (1U)
#define PWM_CHANNEL (1U)

#define PWM_BASE_ADDRESS  (0x4003)
#define PWM_VALUE_ADDRESS (0x8010)


tpm_general_config_t PwmGConfig = {
    .isDBGMode = false,
    .isGlobalTimeBase = true,
    .isTriggerMode = false,
    .isStopCountOnOveflow = false,
    .isCountReloadOnTrig = false,
    .triggerSource = kTpmTpm1Trig,
};

tpm_general_config_t CountConfig = {
    .isDBGMode = false,
    .isGlobalTimeBase = false,
    .isTriggerMode = false,
    .isStopCountOnOveflow = false,
    .isCountReloadOnTrig = false,
};

tpm_counting_mode_t CountMode = kTpmCountingUp;

tpm_pwm_param_t PwmParams = {
    .mode = kTpmEdgeAlignedPWM,
    .edgeMode = kTpmLowTrue,
    .uFrequencyHZ = 240000,
    .uDutyCyclePercent = 100,
};


void initPWM(void)
{
    TPM_DRV_Init(TPM_0, &PwmGConfig);
    //TPM_DRV_Init(TPM_1, &CountConfig);
    //TPM_DRV_CounterStart(TPM_1, CountMode, 4U, false);
    TPM_DRV_SetClock(TPM_0, 2U,kTpmDividedBy1);
    SEGGER_RTT_printf(0, "\tInit complete - duty cycle = %d\n",
        PwmParams.uDutyCyclePercent);
    
    //TPM_DRV_PwmStart(TPM_0, &PwmParams, PWM_CHANNEL);
    /* When switching mode, disable channel first  */
    //TPM_HAL_DisableChn(g_tpmBaseAddr[TPM_0], PWM_CHANNEL);

    /* Set the requested PWM mode */
    //TPM_HAL_EnablePwmMode(g_tpmBaseAddr[TPM_0], &PwmParams, PWM_CHANNEL);
}

void writeToPWM(uint16_t output)
{
    //SEGGER_RTT_WriteString(0, "\tpush val\n");
    //PwmParams.uDutyCyclePercent = (10*output) >> 10; // times 10 div 1024 is easier than  div 100 :/
    PwmParams.uDutyCyclePercent = 100;

    //TPM_HAL_SetMod(g_tpmBaseAddr[TPM_0], 199);
    //TPM_HAL_SetChnCountVal(g_tpmBaseAddr[TPM_0], PWM_CHANNEL, 200);
    //SEGGER_RTT_printf(0, "\t channel value: %3d\n", TPM_DRV_GetChnVal(TPM_0, PWM_CHANNEL));
    
    if(TPM_DRV_PwmStart(TPM_0, &PwmParams, PWM_CHANNEL))
    {
        TPM_DRV_SetTimeOverflowIntCmd(TPM_0, true);
        SEGGER_RTT_printf(0, "\tfreq: %d, Input val: %3d, Channel val: %3d\n",
            TPM_DRV_GetClock(TPM_0), PwmParams.uDutyCyclePercent, TPM_DRV_GetChnVal(TPM_0, PWM_CHANNEL));
    }
    else
    {
        SEGGER_RTT_WriteString(0,"PWMStart failed\n");
    }
    
    
}