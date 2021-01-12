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



/*
// FTM configuration structure
typedef struct FtmUserConfig {
    uint8_t tofFrequency;
    bool isFTMMode;
    uint8_t BDMMode;
    bool isWriteProtection;
    //bool isTimerOverFlowInterrupt;
    //bool isFaultInterrupt;
} ftm_user_config_t;

typedef struct FtmPwmParam
{
    ftm_user_config_t mode;
    ftm_pwm_edge_mode_t edgeMode;
    uint32_t uFrequencyHz
    uint32_t uDutyCyclePercent;
    uint16_t uFirstEdgeDelayPercent;
} ftm_pwm_param_t;
*/

tpm_general_config_t PwmGConfig = {
    .isDBGMode = true,
    .isGlobalTimeBase = false,
    .isTriggerMode = true,
    .isStopCountOnOveflow = false,
    .isCountReloadOnTrig = true,
    .triggerSource = kTpmTpm1Trig,
};

tpm_general_config_t CountConfig = {
    .isDBGMode = true,
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
    .uDutyCyclePercent = 50,
};


void initPWM(void)
{
    TPM_DRV_Init(TPM_0, &PwmGConfig);
    TPM_DRV_Init(TPM_1, &CountConfig);
    TPM_DRV_CounterStart(TPM_1, CountMode, 4U, false);
    SEGGER_RTT_WriteString(0, "\tInit complete\n");
    //TPM_DRV_PwmStart(TPM_0, &PwmParams, PWM_CHANNEL);
}

void writeToPWM(uint16_t output)
{
    //SEGGER_RTT_WriteString(0, "\tpush val\n");
    PwmParams.uDutyCyclePercent = (10*output) >> 10; // times 10 div 1024 is easier than  div 100 :/
    if(TPM_DRV_PwmStart(TPM_0, &PwmParams, PWM_CHANNEL))
    {
        SEGGER_RTT_printf(0, "\tStart: %d, Input val: %3d, Channel val: %3d\n",
            PwmParams.uDutyCyclePercent, TPM_DRV_GetChnVal(TPM_0, PWM_CHANNEL));
    }
    else
    {
        SEGGER_RTT_WriteString(0,"PWMStart failed\n")
    }
    
}