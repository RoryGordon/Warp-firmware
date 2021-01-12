/*
 * functions regarding the generation of the PWM signal
 */
#include <stdlib.h>

//#include "fsl_device_registers.h"
#include "fsl_port_hal.h"
#include "fsl_tpm_driver.h"

#include "SEGGER_RTT.h"

#define TPM_0 (0U)
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
};

tpm_pwm_param_t PwmParams = {
    .mode = kTpmEdgeAlignedPWM,
    .edgeMode = kTpmLowTrue,
    .uFrequencyHZ = 240000,
    .uDutyCyclePercent = 50,
};

void initPWM(void)
{
    // TODO: find config values (trust default?)
    //PwmGConfig.isDBGMode = false;            //Debug mode
    //PwmGConfig.isGlobalTimeBase = false;     //not sure what this does, but false feels safer
    PwmGConfig.isTriggerMode = false;        //we don't have hardware triggers
    //PwmGConfig.isStopCountOnOveflow = false; //again another guess
    //PwmGConfig.isCountReloadOnTrig = false;  //probably doesn't matter if we arent using a trigger
    //PwmGConfig.triggerSource;              //Don't need

    TPM_DRV_Init(TPM_0, &PwmGConfig);
    SEGGER_RTT_WriteString(0, "\tInit complete\n");
}

void writeToPWM(uint16_t output)
{
    //SEGGER_RTT_WriteString(0, "\tpush val\n");
    PwmParams.uDutyCyclePercent = (10*output) >> 10; // times 10 div 1024 is easier than  div 100 :/
    TPM_DRV_PwmStart(TPM_0, &PwmParams, PWM_CHANNEL);
    SEGGER_RTT_printf(0, "\tChannel val: %d\n", TPM_DRV_GetChnVal(TPM_0, PWM_CHANNEL));
    SEGGER_RTT_printf(0, "TOF: %d\n", TPM_HAL_GetTimerOverflowStatus(0x4003800));
}