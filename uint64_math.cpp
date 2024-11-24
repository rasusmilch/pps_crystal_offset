#include "uint64_math.h"
#include <Arduino.h>

void add64(uint32_t aHigh, uint32_t aLow, uint32_t bHigh, uint32_t bLow,
           uint32_t &resultHigh, uint32_t &resultLow) {
  uint32_t carry = 0;
  resultLow = aLow + bLow;
  if (resultLow < aLow && resultLow < bLow) {
    carry = 1;
  }
  resultHigh = aHigh + bHigh + carry;
}

void sub64(uint32_t aHigh, uint32_t aLow, uint32_t bHigh, uint32_t bLow,
           uint32_t &resultHigh, uint32_t &resultLow) {
  uint32_t borrow = 0;
  if (aLow < bLow) {
    borrow = 1;
  }
  resultLow = aLow - bLow;
  resultHigh = aHigh - bHigh - borrow;
}

void mul64(uint32_t aHigh, uint32_t aLow, uint32_t bHigh, uint32_t bLow,
           uint32_t &resultHigh, uint32_t &resultLow) {
  uint32_t aLowLow = aLow & 0xFFFF;
  uint32_t aLowHigh = aLow >> 16;
  uint32_t bLowLow = bLow & 0xFFFF;
  uint32_t bLowHigh = bLow >> 16;

  // Calculate intermediate products
  uint32_t lowLow = aLowLow * bLowLow;
  uint32_t lowHigh = aLowLow * bLowHigh;
  uint32_t highLow = aLowHigh * bLowLow;
  uint32_t highHigh = aLowHigh * bLowHigh;

  // Calculate combined low part
  uint32_t midLow = lowLow + ((lowHigh & 0xFFFF) << 16);
  uint32_t carry = (midLow < lowLow) ? 1 : 0;

  midLow += ((highLow & 0xFFFF) << 16);
  carry += (midLow < lowLow) ? 1 : 0;

  resultLow = midLow;

  // Calculate combined high part
  uint32_t midHigh = (lowHigh >> 16) + (highLow >> 16) + carry;
  resultHigh = highHigh + midHigh;

  // Add contributions from aHigh and bHigh
  resultHigh += (aHigh * bLow) + (aLow * bHigh);
}

void leftShift64(uint32_t &high, uint32_t &low) {
  // Shift the high part left by 1 and add the carry bit from the low part
  high = (high << 1) | (low >> 31);
  // Shift the low part left by 1
  low <<= 1;
}

bool getBit64(uint32_t high, uint32_t low, uint8_t i) {
  if (i < 32) {
    // Return the bit from the low part
    return (low >> i) & 1;
  } else if (i < 64UL) {
    // Return the bit from the high part
    return (high >> (i - 32UL)) & 1;
  } else {
    // If 'i' is out of range, return false
    return false;
  }
}

bool isGreaterOrEqual64(uint32_t rHigh, uint32_t rLow, uint32_t dHigh,
                        uint32_t dLow) {
  if (rHigh > dHigh) {
    return true;
  } else if (rHigh < dHigh) {
    return false;
  } else {
    return rLow >= dLow;
  }
}

void setBit64(uint32_t &high, uint32_t &low, uint8_t i) {
  if (i < 32) {
    // Set the bit in the low part
    low |= (1UL << i);
  } else if (i < 64) {
    // Set the bit in the high part
    high |= (1UL << (i - 32));
  }
}

__attribute__((noinline)) void div64(uint32_t aHigh, uint32_t aLow,
                                     uint32_t bHigh, uint32_t bLow,
                                     uint32_t &resultHigh,
                                     uint32_t &resultLow) {
  uint32_t tempHigh, tempLow;

  if (bHigh == 0 && bLow == 0) {
    // Division by zero error handling
    resultHigh = 0;
    resultLow = 0;
    return;
  }

  // Initialize result
  resultHigh = 0;
  resultLow = 0;

  // Initialize remainder
  uint32_t rHigh = 0;
  uint32_t rLow = 0;

  // Perform bit-wise division
  for (int i = 63; i >= 0; i--) {
    // Left shift rHigh:rLow by 1
    leftShift64(rHigh, rLow);

    // Set the least significant bit of rLow equal to bit i of the numerator
    rLow |= getBit64(aHigh, aLow, i);

    // Check if R >= D
    if (isGreaterOrEqual64(rHigh, rLow, bHigh, bLow)) {
      sub64(rHigh, rLow, bHigh, bLow, tempHigh, tempLow);
      rHigh = tempHigh;
      rLow = tempLow;

      setBit64(resultHigh, resultLow, i);
    }
  }
}