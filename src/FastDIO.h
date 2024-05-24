#pragma once

#define digitalWrite(a, b) FastDigitalWrite(a, b)
#define digitalRead(a) FastDigitalRead(a)

#include <Arduino.h>

// write value to register
#ifndef REGISTER_WRITE
#define REGISTER_WRITE(_r, _v) ((*(volatile uint32_t*)(_r)) = (_v))
#endif

//get bit or get bits from register
#ifndef REGISTER_GET_BIT
#define REGISTER_GET_BIT(_r, _b) (*(volatile uint32_t*)(_r) & (_b))
#endif

inline void FastDigitalWrite(uint8_t pin, uint8_t val)
{
    if (pin < 32) {
        val ? (REGISTER_WRITE(GPIO_OUT_W1TS_REG, 1 << pin)) : (REGISTER_WRITE(GPIO_OUT_W1TC_REG, 1 << pin));
    } else {
        val ? (REGISTER_WRITE(GPIO_OUT1_W1TS_REG, 1 << (pin - 32))) : (REGISTER_WRITE(GPIO_OUT1_W1TC_REG, 1 << (pin - 32)));
    }
}

inline int FastDigitalRead(uint8_t pin)
{
    if (pin < 32) {
        return (REGISTER_GET_BIT(GPIO_IN_REG, 1 << pin)) ? 1 : 0;
    } else {
        return (REGISTER_GET_BIT(GPIO_IN1_REG, 1 << (pin - 32))) ? 1 : 0;
    }
}