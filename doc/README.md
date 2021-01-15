# Delay Pedal: rarg2 coursework 5

## Overview

The purpose of this program is to accept a high-impedance audio signal and pass this signal through a delay scheme in order to emulate the effect of a delay-style guitar pedal. As the output was never achieved, the functionality has been split into various menu options in order to demonstrate the work done so far.

Three extra files have been created in order to produce input and output to the pedal, and most irrelevant files from the original Warp firmware have been removed.

## Extra `.c` and `.h` files

### `ADC.h` and `ADC.c`

These files handle the ADC so that the main function has minimal setup, and are based off of a set of demo functions within the SDK.

#### `ADC_TEST_InstallCallback(uint32_t instance, uint32_t chnGroup, void (*callbackFunc)(void) )`

Installs the callback funtion

#### `ADC1IRQHandler(void)`

Interrupt handler for the ADC

#### `calibrateParams(void)`

called by `configureADC()`, this function sets `config` structures and does a test read of the bandgap voltage (was printed in testing).

#### `static int32_t initADC(uint32_t instance)`

Sets the ADC up in a similar manner to `calibrateParams()`, but for pin `PTA8`, and does a test read, printing the value.

#### `void configureADC(void)`

Called by the main function to set up the ADC, and itself calls upon `calibrateParams()` and `initADC()`.

#### `uint32_t printSnesorDataADC(bool hexModeFlag)`

Print the current value of the ADC, and return the value as well.

#### uint32_t getSensorDataADC(bool hexModeFlag)

The same as above, but without the print statements.

### `PWM.c`
Controls the TPM module for PWM mode.

#### `void initPWM(void)`

Initialises the TPM unit and sets the clock.

#### `void writeToPWM(uint16_t output)`

Attempts to write a value to the TPM unit, and reads back the `COUNT` register to ensure the value is recieved. This is successful, however no response is seen from the output LED, even though the register seems to be accepting the changes.

## Added Menu Options

### `z`: Print ADC Value

Prints values from the ADC (reading `PTA8`), with a specified delay between each read

### `0`: Speed Test

A speed test to ascertain how quickly 40,000 reads of the ADC can be achieved.

### `1`: Delay Demonstration

Runs a printed demonstration of the delay line at a significantly lower speed to allow the user to understand the mechanism.

### `2`: PWM Test

Attempts to set values to the PWM, to make the green LED alternate between half-on and full-on, however, this is not observed.

### `3`: Input Hardware Test

Reads the ADC and prints the maximum and minimum values for every 40,000 values it recieves, before resetting and repeating. Use this to varify that the input hardware is reading a sufficient amplitude.

### `4`: Integrating All Parts

Assuming the PWM was working, this would be the final product, producing the effected signal at the PWM output.

## Extra hardware

In order to interface between the KL03 and an instrument, I have built a simple amplifier circuit, as described in the report. This is to lower the input impedance adn amplify the incoming signal to the KL03 to allow values to be read successfully.