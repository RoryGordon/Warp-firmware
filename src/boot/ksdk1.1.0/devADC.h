#ifndef WARP_BUILD_ENABLE_DEVADC
#define WARP_BUILD_ENABLE_DEVADC
#endif

static int32_t        initADC(uint32_t instance);
void                  configureADC(void);
void		          printSensorDataADC(bool hexModeFlag);