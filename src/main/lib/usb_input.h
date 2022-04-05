/*
    usb.h :: USB initialization, configuration, and operation.

    USB input is used to change the configuration of the PWM signal.
*/

#ifndef USB_INPUT_H
#define USB_INPUT_H


#include <Usb.h>
#include <usbhub.h>
#include "KeyboardController.h"  // modified
#include "pwm.h"


// Characters that we might receive from KeyboardController.getKey(), to be used in pwm.c UpdatePWM().
#define VOLUME_UP     '7'
#define VOLUME_DOWN   '1'
#define PITCH_UP      '8'
#define PITCH_DOWN    '2'
#define INTERVAL_UP   '9'
#define INTERVAL_DOWN '3'
#define SAVE_SETTINGS '\r'

static USBHost            usb;
static KeyboardController keyboard(usb);
static uint8_t            usb_signal = 0;


// Set up the USBHost functionality.
void InitializeUSB()
{
    while (usb.Init());
}


// Check for changes in USB connection and initiate updates to PWM configuration based on USB inputs.
void CheckUSBStatus(pwm_settings* pwm_idx)
{
    // Check for USB keyboard connection and input.
    // Testing: Only when beep is OFF?
    // Result: More consistent behavior, we no longer pause with the beep on during initial USB connection.
    //         Slightly less responsive changes to keyboard input (does not happen while beep is ON), but it still takes effect on the next beep.
    if (TCC0->STATUS.bit.STOP)
    {
        usb.Task();
        /*
        usb_task_state
        - starts at 17 (USB_DETACHED_SUBSTATE_INITIALIZE)
        - after first usb.Task(), changes to 18 (USB_DETACHED_SUBSTATE_WAIT_FOR_DEVICE) 
        - when USB is connected, changes to 32 (USB_ATTACHED_SUBSTATE_SETTLE) 
        - back to 18 after USB disconnected
        */
    }

    // Apply changes based on input.
    if (usb_signal)
    {
        UpdatePWM(usb_signal, pwm_idx);
        usb_signal = 0;
    }
}


// Called automatically when a key is pressed on a connected keyboard.
void keyPressed()
{
    usb_signal = keyboard.getKey();
}


// Called automatically when a key is released on a connected keyboard.
void keyReleased()
{
    usb_signal = 0;
}


#endif

