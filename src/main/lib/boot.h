/*
    boot.h :: Initialization and configuration of MC at boot.

    Sources:
      -  SAMD21 Datasheet
      -  https://blog.thea.codes/understanding-the-sam-d21-clocks/
      -  https://blog.thea.codes/sam-d21-brown-out-detector/
*/

#ifndef BOOT_H
#define BOOT_H


void EnableBOD33()
{
    // Disable brown-out detector during configuration, or it could misbehave and reset the microcontroller.
    SYSCTRL->BOD33.bit.ENABLE = 0;
    while (!SYSCTRL->PCLKSR.bit.B33SRDY);

    // Configure brown-out detector so that the program can use it to watch the power supply voltage.
    SYSCTRL->BOD33.reg = SYSCTRL_BOD33_LEVEL(48) | SYSCTRL_BOD33_ACTION_NONE | SYSCTRL_BOD33_HYST;

    // Enable brown-out detector and wait for voltage level to settle.
    SYSCTRL->BOD33.bit.ENABLE = 1;
    while (!SYSCTRL->PCLKSR.bit.BOD33RDY);

    // BOD33DET is set when the voltage is too low, so wait for it to be cleared.
    while (SYSCTRL->PCLKSR.bit.BOD33DET);

    // Let brown-out detector reset the microcontroller when voltage drops too low.
    SYSCTRL->BOD33.bit.ENABLE = 0;
    while (!SYSCTRL->PCLKSR.bit.B33SRDY);

    SYSCTRL->BOD33.reg |= SYSCTRL_BOD33_ACTION_RESET;
    SYSCTRL->BOD33.bit.ENABLE = 1;
}


void Initialize48MHzClock()
{
    // Set the correct number of wait states for 48 MHz @ 3.3v.
    NVMCTRL->CTRLB.bit.RWS = 1;

    /*
    External 32.768 KHz crystal oscillator start-up time can vary based on the component chosen.
    Wait time is defined by a value in 0x[0, 7] (see Table 17-6 in datasheet), e.g.:
        0x4: 500 ms, 0x5: 1 s, 0x7: 4 s
    */
    SYSCTRL->XOSC32K.reg = SYSCTRL_XOSC32K_STARTUP(0x5) | SYSCTRL_XOSC32K_EN32K | SYSCTRL_XOSC32K_XTALEN;

    // Enable the crystal, and wait for it to be ready.
    SYSCTRL->XOSC32K.bit.ENABLE = 1;
    while (!SYSCTRL->PCLKSR.bit.XOSC32KRDY);

    // Configure GCLK1's divider (currently disabled with DIV(1)).
    GCLK->GENDIV.reg = GCLK_GENDIV_ID(1) | GCLK_GENDIV_DIV(1);

    // Setup GCLK1 using the external oscillator, and wait for the register write to complete.
    GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(1) | GCLK_GENCTRL_SRC_XOSC32K | GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN;
    while (GCLK->STATUS.bit.SYNCBUSY);

    // Connect GCLK1's output to the DFLL's reference clock.
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_DFLL48 | GCLK_CLKCTRL_GEN_GCLK1 | GCLK_CLKCTRL_CLKEN;

    // This works around a quirk in the hardware (errata 1.2.1) - the DFLLCTRL register must be manually reset to this value before configuration.
    while (!SYSCTRL->PCLKSR.bit.DFLLRDY);
    SYSCTRL->DFLLCTRL.reg = SYSCTRL_DFLLCTRL_ENABLE;
    while (!SYSCTRL->PCLKSR.bit.DFLLRDY);

    /*
    Set the DFLL multiplier to multiply the 32.768 kHz reference clock to 48 MHz.

    DFLLMUL = output freq. / ref. clock freq.
     1464.8 = (48 MHz / 32.768 kHz)

    Coarse (10-bit) and fine (6-bit) step are used by the DFLL to lock on to the target frequency.
    Lower values mean less overshoot, whereas higher values typically result in some overshoot but faster locking.
    */
    SYSCTRL->DFLLMUL.reg = SYSCTRL_DFLLMUL_MUL(1465) | SYSCTRL_DFLLMUL_FSTEP(511) | SYSCTRL_DFLLMUL_CSTEP(31);
    while (!SYSCTRL->PCLKSR.bit.DFLLRDY);
    
    // Load factory calibration value for the coarse register to start the DFLL at a frequency close to 48 MHz and reduce locking time.
    SYSCTRL->DFLLVAL.bit.COARSE = (*((uint32_t *)FUSES_DFLL48M_COARSE_CAL_ADDR) & FUSES_DFLL48M_COARSE_CAL_Msk) >> FUSES_DFLL48M_COARSE_CAL_Pos;
    while (!SYSCTRL->PCLKSR.bit.DFLLRDY);

    // Set the DFLL to closed loop mode and turn it on, then wait for the frequency to lock.
    SYSCTRL->DFLLCTRL.reg |= SYSCTRL_DFLLCTRL_MODE | SYSCTRL_DFLLCTRL_WAITLOCK | SYSCTRL_DFLLCTRL_ENABLE;
    while (!SYSCTRL->PCLKSR.bit.DFLLLCKC || !SYSCTRL->PCLKSR.bit.DFLLLCKF);

    // Set up GCLK0 using the DFLL @ 48 MHz.
    GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(0) | GCLK_GENCTRL_SRC_DFLL48M | GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN;
    while (GCLK->STATUS.bit.SYNCBUSY);
}


void ConfigureGPIO()
{
    // Configure PA05 and PA09 to be outputs and clear them to LOW.
    PORT->Group[PORTA].DIRSET.reg = PORT_PA05 | PORT_PA09;
    PORT->Group[PORTA].OUTCLR.reg = PORT_PA05 | PORT_PA09;
    
    // Enable the peripheral multiplexer for the pins.
    PORT->Group[PORTA].PINCFG[5].reg |= PORT_PINCFG_PMUXEN;
    PORT->Group[PORTA].PINCFG[9].reg |= PORT_PINCFG_PMUXEN;

    // Set drive strength of output pins. (0 -> 2 mA; 1 -> 7 mA)
    // TODO: Make sure this is working, see: https://github.com/arduino/ArduinoCore-samd/issues/158
    PORT->Group[PORTA].PINCFG[5].bit.DRVSTR = 1;
    PORT->Group[PORTA].PINCFG[9].bit.DRVSTR = 1;

    // Set the multiplexer function for the pins from datasheet Table 7-1.
    // Odd pins get PMUX[{pin_num} - 1 / 2].reg = PORT_PMUX_PMUXO_{function_letter}
    // Even pins get PMUX[{pin_num} / 2].reg = PORT_PMUX_PMUXE_{function_letter}
    // Setting PA05 and PA09 to function E to utilize TCC0/WO[1] for PWM.
    PORT->Group[PORTA].PMUX[2].reg = PORT_PMUX_PMUXO_E;
    PORT->Group[PORTA].PMUX[4].reg = PORT_PMUX_PMUXO_E;

    // Configure input pins for LTC-4060 charging signals.
    PORT->Group[PORTA].DIRCLR.reg = PORT_PA16 | PORT_PA17;
    PORT->Group[PORTA].OUTCLR.reg = PORT_PA16 | PORT_PA17;
    PORT->Group[PORTA].PINCFG[16].bit.INEN = 1;
    PORT->Group[PORTA].PINCFG[17].bit.INEN = 1;
    PORT->Group[PORTA].PINCFG[16].bit.PULLEN = 1;
    PORT->Group[PORTA].PINCFG[17].bit.PULLEN = 1;
}


#endif

