/*
    util.h

    Utility functions.
*/

#ifndef UTIL_H
#define UTIL_H


// Clamps idx to the range [0, num_presets) to limit preset adjustments.
size_t clamp_idx(size_t idx, uint8_t num_presets)
{
    // size_t underflow
    if (idx == (size_t) -1) return 0;

    // preset overflow
    else if (idx == num_presets) return num_presets - 1;

    // otherwise
    return idx;
}


// Returns the current value read on a PORTA input pin.
bool ReadPA(uint8_t pin)
{
    return (PORT->Group[PORTA].IN.reg >> pin) & 0x1;
}


#endif

