#ifndef UINT64_MATH_H
#define UINT64_MATH_H

#include <stdint.h>

void add64(uint32_t aHigh, uint32_t aLow, uint32_t bHigh, uint32_t bLow, uint32_t &resultHigh, uint32_t &resultLow);
void sub64(uint32_t aHigh, uint32_t aLow, uint32_t bHigh, uint32_t bLow, uint32_t &resultHigh, uint32_t &resultLow);
void mul64(uint32_t aHigh, uint32_t aLow, uint32_t bHigh, uint32_t bLow, uint32_t &resultHigh, uint32_t &resultLow);
void div64(uint32_t aHigh, uint32_t aLow, uint32_t bHigh, uint32_t bLow, uint32_t &resultHigh, uint32_t &resultLow);

#endif // UINT64_MATH_H
