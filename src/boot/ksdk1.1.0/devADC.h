#ifndef WARP_BUILD_ENABLE_DEVADC
#define WARP_BUILD_ENABLE_DEVADC
#endif

/*!
* @brief Boundaries struct
*/
typedef struct lowPowerAdcBoundaries
{
    int32_t upperBoundary;
    int32_t lowerBoundary;
} lowPowerAdcBoundaries_t;

/*!
 * @brief Low Power Timer Interrupt handler. Clear LPT Compare flag.
 */
void LowPowerTimerIRQHandler(void);

/*!
 * @brief ADC Interrupt handler. Get current ADC value and set conversionCompleted flag.
 */
void ADC1IRQHandler(void);

/*!
 * @brief Initialize Low Power Timer. Use 1 kHz LPO with no preescaler and enable LPT interrupt.
 */
void InitLowPowerTmr(void);

/*!
 * @brief Calculate current temperature.
 *
 * @return uint32_t Returns current temperature.
 */
//int32_t GetCurrentTempValue(void);

/*!
 * @brief Calculate current temperature.
 *
 * @param updateBoundariesCounter Indicate number of values into tempArray.
 *
 * @param tempArray Store temperature value.
 *
 * @return lowPowerAdcBoundaries_t Returns upper and lower temperature boundaries.
 */
lowPowerAdcBoundaries_t TempSensorCalibration(uint32_t updateBoundariesCounter,
                                                     int32_t *tempArray);

/*!
 * @brief User-defined function to install callback.
 */
void ADC_TEST_InstallCallback(uint32_t instance, uint32_t chnGroup, void (*callbackFunc)(void) );

/*!
 * @brief User-defined function to read conversion value in ADC ISR. 
 */
uint16_t ADC_TEST_GetConvValueRAWInt(uint32_t instance, uint32_t chnGroup);
//static int32_t        initADC(uint32_t instance);
void                  configureADC(void);
//uint32_t		      printSensorDataADC(bool hexModeFlag);