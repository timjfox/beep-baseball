/*
    pwm.c :: PWM initialization, configuration, and operation.

    PWM is used to control the beep baseball's 2 beeping speakers.

    TCC0/WO[1] is configured for use on both PA05 and PA09.

    Sources:
      -  SAMD21 Datasheet
      -  https://shawnhymel.com/1710/arduino-zero-samd21-raw-pwm-using-cmsis/
      -  https://blog.thea.codes/phase-shifted-pwm-on-samd/
      -  https://forum.arduino.cc/t/please-help-understand-pwm-on-samd21/373173
      -  https://forum.arduino.cc/t/disable-stop-pwm-in-arduino-samd21/658852/2
*/

#define NUM_PULSE_WIDTHS 4
#define NUM_BEEP_RATES 11
#define NUM_PERIODS 8

#include <FlashStorage.h>
#include "usb_input.h"
#include "pwm.h"
#include "util.h"

// Create a section of flash to store a pwm_settings struct (data will be lost if MC is flashed again).
FlashStorage(pwm_settings_memory, pwm_settings);

// Arrays of preset values for PWM settings which are indexed by the pwm_settings struct.
static float    PULSE_WIDTHS [NUM_PULSE_WIDTHS] = {0.025, 0.05, 0.100, 0.300};
static uint32_t BEEP_RATES   [NUM_BEEP_RATES] = {500, 450, 400, 350, 300, 250, 200, 150, 100, 50, 25};
static uint32_t PERIODS      [NUM_PERIODS] = {81920, 65536, 57344, 49152, 40960, 32768, 24576, 16384};


/*************************************\
* PWM configuration and manipulation. *
\*************************************/

// Load pwm_settings from memory; apply default settings and save to memory on first boot.
pwm_settings LoadPWMParameters()
{
    pwm_settings pwm_idx = pwm_settings_memory.read();

    // Assign and write default PWM settings if memory has not been written.
    if (!pwm_idx.valid)
    {
        pwm_idx.pulse_width = 2;
        pwm_idx.beep_rate = 8;
        pwm_idx.period = 5;
        pwm_idx.valid = true;

        pwm_settings_memory.write(pwm_idx);
    }

    return pwm_idx;
}


// Initialize TCC0/1 to use a 48 MHz clock without prescaling, and apply loaded pwm_settings.
void InitializeTCC0(pwm_settings pwm_idx)
{
    // Enable the APB clock for TCC0.
    PM->APBCMASK.reg |= PM_APBCMASK_TCC0;

    // Enable GCLK0 (48 MHz), connect it to TCC0, and wait for clock bus to sync.
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TCC0_TCC1;
    while (GCLK->STATUS.bit.SYNCBUSY);

    // Make TCC0 count slower than the clock with prescaler (currently disabled with DIV1).
    TCC0->CTRLA.reg |= TCC_CTRLA_PRESCALER(TCC_CTRLA_PRESCALER_DIV1_Val);

    // Configure TCC0 wave generation to use Normal PWM and wait for clock bus to sync.
    TCC0->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;
    while (TCC0->SYNCBUSY.bit.WAVE);

    // Set PWM frequency and wait for clock bus to sync.
    // PER register sets frequency f = f_GCLK / (TCC_prescaler * (1 + PER)).
    // e.g. f = 48 MHz / (1 * (1 + 81920)) ~ 585.9 Hz
    // TCC0 is a 24-bit counter, so the period is masked.
    TCC0->PER.reg = PERIODS[pwm_idx.period] & 0x00ffffff;
    while (TCC0->SYNCBUSY.bit.PER);

    // Configure the pulse width (or duty cycle) and wait for clock bus to sync.
    // CC[n] has n = x % 4 where x is from WO[x] where WO[x] comes from datasheet Table 7-1.
    // pulse_width = 0.5 -> 50% duty cycle
    TCC0->CC[1].reg = TCC0->PER.reg * PULSE_WIDTHS[pwm_idx.pulse_width];
    while (TCC0->SYNCBUSY.bit.CC1);

    // Enable setting WO[1] LOW when TCC0 is stopped.
    TCC0->DRVCTRL.bit.NRE1 = 1;                      

    // Start PWM on TCC0, stop timer, and wait for clock bus to sync.
    TCC0->CTRLA.reg |= TCC_CTRLA_ENABLE;
    StopTimer();  // In case MC is being powered via charger.
    while (TCC0->SYNCBUSY.bit.ENABLE);
}


// Update PWM settings based on the received usb_signal.
// Clamp changes to indices to ensure they stay within the bounds of the preset arrays.
void UpdatePWM(uint8_t usb_signal, pwm_settings* pwm_idx)
{
    switch (usb_signal)
    {
        case VOLUME_UP:
            pwm_idx->pulse_width = clamp_idx(pwm_idx->pulse_width + 1, NUM_PULSE_WIDTHS);
            UpdatePulseWidth(pwm_idx->pulse_width);
            break;

        case VOLUME_DOWN:
            pwm_idx->pulse_width = clamp_idx(pwm_idx->pulse_width - 1, NUM_PULSE_WIDTHS);
            UpdatePulseWidth(pwm_idx->pulse_width);
            break;

        case PITCH_UP:
            pwm_idx->period = clamp_idx(pwm_idx->period + 1, NUM_PERIODS);
            UpdatePeriod(pwm_idx->period, pwm_idx->pulse_width);
            break;

        case PITCH_DOWN:
            pwm_idx->period = clamp_idx(pwm_idx->period - 1, NUM_PERIODS);
            UpdatePeriod(pwm_idx->period, pwm_idx->pulse_width);
            break;

        case INTERVAL_UP:
            pwm_idx->beep_rate = clamp_idx(pwm_idx->beep_rate + 1, NUM_BEEP_RATES);
            break;

        case INTERVAL_DOWN:
            pwm_idx->beep_rate = clamp_idx(pwm_idx->beep_rate - 1, NUM_BEEP_RATES);
            break;
        
        case SAVE_SETTINGS:
            pwm_settings_memory.write(*pwm_idx);
            break;

        default:
            break;
    }
}


// Update the period of the PWM waveform.
void UpdatePeriod(size_t period_idx, size_t pulse_width_idx)
{
    // Because we are using TCC0, limit period to 24 bits.
    // Uses buffer (PERB) to prevent glitches when changing on the fly.
    TCC0->PERB.reg = PERIODS[period_idx] & 0x00ffffff;
    while (TCC0->SYNCBUSY.bit.PERB);

    // Update PW too since it depends on the period.
    UpdatePulseWidth(pulse_width_idx);
}


// Update the pulse width of the PWM waveform.
void UpdatePulseWidth(size_t pulse_width_idx)
{
    // Change the TCC0 duty cycle on channel 1 and wait for clock bus to sync.
    // Uses buffer (CCB) to prevent glitches when changing on the fly.
    TCC0->CCB[1].reg = TCC0->PER.reg * PULSE_WIDTHS[pulse_width_idx];  
    while (TCC0->SYNCBUSY.bit.CCB1);
}


void StopTimer()
{
    TCC0->CTRLBSET.reg = TCC_CTRLBCLR_CMD_STOP;
    while (TCC0->SYNCBUSY.bit.CTRLB);
}


void RestartTimer()
{
    TCC0->CTRLBSET.reg = TCC_CTRLBCLR_CMD_RETRIGGER;
    while (TCC0->SYNCBUSY.bit.CTRLB); 
}


// Toggle TCC0 on or off, opposite of its current state.
// Returns the current time in ms to allow for control of the beep rate.
unsigned long TogglePWM()
{
    if (TCC0->STATUS.bit.STOP)
    {
        RestartTimer();
    }
    else
    {
        StopTimer();
    }

    return millis();
}


// When run in main while loop, toggles TCC0 on and off to produce a beep at the desired beep rate.
void Beep(size_t beep_rate_idx)
{
    static unsigned long last_toggle = 0;

    if (millis() - last_toggle >= BEEP_RATES[beep_rate_idx])
    {
        last_toggle = TogglePWM();
    }
}


/***********************************************\
* Charging status checking and indicator audio. *
\***********************************************/
#define NUM_OCTAVES 4

static uint32_t C  [NUM_OCTAVES] = {183465, 91734, 45867, 22934};
static uint32_t Db [NUM_OCTAVES] = {173173, 86585, 43293, 21646};
static uint32_t D  [NUM_OCTAVES] = {163454, 81726, 40863, 20431};
static uint32_t Eb [NUM_OCTAVES] = {154276, 77139, 38569, 19285};
static uint32_t E  [NUM_OCTAVES] = {145618, 72810, 36405, 18202};
static uint32_t F  [NUM_OCTAVES] = {137445, 68723, 34362, 17181};
static uint32_t Gb [NUM_OCTAVES] = {129733, 64866, 32433, 16216};
static uint32_t G  [NUM_OCTAVES] = {122449, 61225, 30613, 15306};
static uint32_t Ab [NUM_OCTAVES] = {115579, 57789, 28894, 14447};
static uint32_t A  [NUM_OCTAVES] = {109091, 54545, 27273, 13636};
static uint32_t Bb [NUM_OCTAVES] = {102969, 51484, 25742, 12871};
static uint32_t B  [NUM_OCTAVES] = {97190, 48594, 24297, 12149};


// Updates PWM period and pulse width based on music notes.
void UpdatePWMMusical(char note, size_t octave, bool flat, size_t volume_idx)
{   
    uint32_t period = 0;
    size_t idx = octave - NUM_OCTAVES;
    if (idx > NUM_OCTAVES) idx = 0;  // default

    switch (note)
    {
        case 'C':
            period = flat ? B[idx] : C[idx];
            break;

        case 'D':
            period = flat ? Db[idx] : D[idx];
            break;

        case 'E':
            period = flat ? Eb[idx] : E[idx];
            break;
        
        case 'F':
            period = flat ? E[idx] : F[idx];
            break;
        
        case 'G':
            period = flat ? Gb[idx] : G[idx];
            break;
        
        case 'A':
            period = flat ? Ab[idx] : A[idx];
            break;
        
        case 'B':
            period = flat ? Bb[idx] : B[idx];
            break;

        default:
            period = A[0];
            break;
    }

    // Because we are using TCC0, limit period to 24 bits.
    // Uses buffer (PERB) to prevent glitches when changing on the fly.
    TCC0->PERB.reg = period & 0x00ffffff;
    while (TCC0->SYNCBUSY.bit.PERB);

    // Update PW for max volume too since it depends on the period.
    UpdatePulseWidth(volume_idx);
}


// Uses PWM to play a specific music note with customizable duration and volume.
void PlayNote(char note, size_t octave, bool flat, uint32_t off_time, uint32_t on_time, size_t volume_idx)
{   
    unsigned long time = millis();

    UpdatePWMMusical(note, octave, flat, volume_idx);
    while (millis() - time < off_time);
    time = TogglePWM();
    while (millis() - time < on_time);
    time = TogglePWM();
}


// Plays build up of "Charge" when charger is plugged in.
void ChargingStartedSound()
{   
    // note, octave, flat, off_time, on_time, volume_idx
    PlayNote('B', 4, true, 150, 300, NUM_PULSE_WIDTHS - 1);
    PlayNote('F', 4, false, 150, 300, NUM_PULSE_WIDTHS - 1);
    PlayNote('G', 4, false, 150, 300, NUM_PULSE_WIDTHS - 1);
    PlayNote('A', 4, false, 150, 300, NUM_PULSE_WIDTHS - 1);

    PlayNote('B', 5, true, 250, 250, NUM_PULSE_WIDTHS - 1);
    PlayNote('F', 5, false, 125, 250, NUM_PULSE_WIDTHS - 1);
    PlayNote('G', 5, false, 125, 250, NUM_PULSE_WIDTHS - 1);
    PlayNote('A', 5, false, 125, 250, NUM_PULSE_WIDTHS - 1);
}


// Plays charge part of "Charge" when batteries are finished charging.
unsigned long ChargingCompleteSound()
{
    static size_t octave = 5;

    PlayNote('G', octave - 1, false, 100, 125, NUM_PULSE_WIDTHS - 1);
    PlayNote('C', octave, false, 75, 125, NUM_PULSE_WIDTHS - 1);
    PlayNote('E', octave, false, 75, 125, NUM_PULSE_WIDTHS - 1);
    PlayNote('G', octave, false, 75, 300, NUM_PULSE_WIDTHS - 1);
    PlayNote('E', octave, false, 75, 100, NUM_PULSE_WIDTHS - 1);
    PlayNote('G', octave, false, 75, 350, NUM_PULSE_WIDTHS - 1);

    if (octave == 5)
    {
        octave++;
    } 
    else
    {
        octave--;
    }

    return millis();
}


// Checks if device is being powered by the USB charger and plays status indicators using PWM.
void CheckChargingStatus(pwm_settings pwm_idx)
{   
    if (ReadPA(16))
    {   
        if (!TCC0->STATUS.bit.STOP)
        {
            StopTimer();
        }
        ChargingStartedSound();

        // Stall until CHRG is high, then continue to stall while ACP is high.
        while (ReadPA(16) && !ReadPA(17));
        while (ReadPA(16))
        {   
            // If CHRG goes low, charging has finished.
            if (!ReadPA(17))
            {
                static unsigned long last_indicator = 0;

                // Play every 10 seconds.
                if (millis() - last_indicator >= 10000)
                {
                    last_indicator = ChargingCompleteSound();
                }
            }
        }

        // Stall for a short time to avoid immediately retriggering the ACP branch.
        for (unsigned long time = millis(); millis() - time < 200;);

        // Reload saved beep settings:
        UpdatePeriod(pwm_idx.period, pwm_idx.pulse_width);
    }
}


/*****************************\
* PWM & audio output testing. *
\*****************************/
void FrequencySweep(uint32_t start, uint32_t range, uint32_t step)
{
    for (uint32_t period = start; period < start + range; period += step)
    {
        TCC0->PERB.reg = period & 0x00ffffff;
        while (TCC0->SYNCBUSY.bit.PERB);
        // UpdatePulseWidth(NUM_PULSE_WIDTHS - 1);
        UpdatePulseWidth(3);
        // for (unsigned long time = millis(); millis() - time < 50;);
    }
    for (uint32_t period = start + range; period > start; period -= step)
    {
        TCC0->PERB.reg = period & 0x00ffffff;
        while (TCC0->SYNCBUSY.bit.PERB);
        // UpdatePulseWidth(NUM_PULSE_WIDTHS - 1);
        UpdatePulseWidth(3);
        // for (unsigned long time = millis(); millis() - time < 50;)
    }
}


void VolumeSweep(float start, float range, float step)
{
    for (float volume = start; volume < start + range; volume += step)
    {
        TCC0->CCB[1].reg = TCC0->PER.reg * volume;  
        while (TCC0->SYNCBUSY.bit.CCB1);
        // for (unsigned long time = millis(); millis() - time < 2;);
    }
    for (float volume = start + range; volume > start; volume -= step)
    {
        TCC0->CCB[1].reg = TCC0->PER.reg * volume;  
        while (TCC0->SYNCBUSY.bit.CCB1);
        // for (unsigned long time = millis(); millis() - time < 2;);
    }
}

