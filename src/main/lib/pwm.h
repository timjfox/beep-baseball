/*
    pwm.h :: PWM initialization, configuration, and operation.

    PWM is used to control the beep baseball's 2 beeping speakers.

    TCC0/WO[1] is configured for use on both pins PA05 (QT Py A3) and PA09 (QT Py MI).

    Sources:
      -  SAMD21 Datasheet
      -  https://shawnhymel.com/1710/arduino-zero-samd21-raw-pwm-using-cmsis/
      -  https://blog.thea.codes/phase-shifted-pwm-on-samd/
      -  https://forum.arduino.cc/t/please-help-understand-pwm-on-samd21/373173
      -  https://forum.arduino.cc/t/disable-stop-pwm-in-arduino-samd21/658852/2
*/

#ifndef PWM_H
#define PWM_H


// Struct to store indices of current PWM settings, which are used to index the preset arrays defined in pwm.c.
typedef struct pwm_settings {
	size_t beep_rate;
    size_t period;
    size_t pulse_width;
    bool   valid;
} pwm_settings;


/*************************************\
* PWM configuration and manipulation. *
\*************************************/
pwm_settings LoadPWMParameters();

void InitializeTCC0(pwm_settings pwm_idx);
void UpdatePWM(uint8_t usb_signal, pwm_settings* pwm_idx);

void UpdatePeriod(size_t period_idx, size_t pulse_width_idx);
void UpdatePulseWidth(size_t pulse_width_idx);

void StopTimer();
void RestartTimer();
unsigned long TogglePWM();
void Beep(size_t beep_rate_idx);


/***********************************************\
* Charging status checking and indicator audio. *
\***********************************************/
void UpdatePWMMusical(char note, size_t octave, bool flat, size_t volume_idx);
void PlayNote(char note, size_t octave, bool flat, uint32_t off_time, uint32_t on_time, size_t volume_idx);
void ChargingStartedSound();
unsigned long ChargingCompleteSound();
void CheckChargingStatus(pwm_settings pwm_idx);


/*****************************\
* PWM & audio output testing. *
\*****************************/
void FrequencySweep(uint32_t start, uint32_t range, uint32_t step);
void VolumeSweep(float start, float range, float step);


#endif

