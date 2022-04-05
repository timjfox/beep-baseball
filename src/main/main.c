/*
    main.c :: Sound Sports Beep Baseball

    Summary:

      *  Atmel SAMD21 Arduino compatible C code for generating and controlling 2 identical PWM outputs.

      *  PWM outputs are low-pass filtered, input to a class-D amplifier, and output through 2 speakers of a beep baseball.

    Features and operation:

      *  Loads previously saved PWM parameters from memory.

      *  Initializes and configures a single PWM channel to be output from 2 GPIO pins.

      *  Allows connection of USB numpad or keyboard to change PWM parameters and update saved default settings if desired.

      *  Plays charging started and completed audio indicators using ACP and CHRG signals from charging IC.

*/


#include "lib/boot.h"
#include "lib/pwm.c"
#include "lib/usb_input.h"


static pwm_settings pwm_idx;


void setup()
{
    // EnableBOD33();  // This seems to cause issues.
    Initialize48MHzClock();
    InitializeUSB();
    ConfigureGPIO();
    
    pwm_idx = LoadPWMParameters();
    InitializeTCC0(pwm_idx);
}


void loop()
{   
    // Check for LTC-4060 ACP high charging signal.
    CheckChargingStatus(pwm_idx);

    // Check for USB keyboard connection and input.
    CheckUSBStatus(&pwm_idx);

    Beep(pwm_idx.beep_rate);
}

