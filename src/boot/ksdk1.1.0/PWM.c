/*
 * functions regarding the generation of the PWM signal
 */
#include <stdlib.h>

//#include "fsl_device_registers.h"
#include "fsl_port_hal.h"
#include "fsl_tpm_driver.h"

#define TPM_0 (0U)
#define PWM_CHANNEL (1U)




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

tpm_general_config_t PwmGConfig;

tpm_pwm_param_t PwmParams;

void initPWM(void)
{
    // TODO: find config values
    PwmGConfig.isDBGMode = false;            //Debug mode
    PwmGConfig.isGlobalTimeBase = false;     //not sure what this does, but false feels safer
    PwmGConfig.isTriggerMode = false;        //we don't have hardware triggers
    PwmGConfig.isStopCountOnOveflow = false; //again another guess
    PwmGConfig.isCountReloadOnTrig = false;  //probably doesn't matter if we arent using a trigger
    //PwmGConfig.triggerSource;              //Don't need

    PwmParams.mode = kTpmEdgeAlignedPWM;
    PwmParams.edgeMode = kTpmHighTrue; //Not sure what this means
    PwmParams.uFrequencyHZ = 22000U;   //A guess at the speed of the program
    PwmParams.uDutyCyclePercent = 50U; //Default to midrange

    PORT_HAL_SetMuxMode(PORTB_BASE,10u,kPortMuxAlt2);
    TPM_DRV_Init(TPM_0, &PwmGConfig);
}

void writeToPWM(uint16_t output)
{

    PwmParams.uDutyCyclePercent = (10*output) >> 10; // times 10 div 1024 is easier than  div 100 :/
    TPM_DRV_PwmStart(TPM_0, &PwmParams, PWM_CHANNEL);
}