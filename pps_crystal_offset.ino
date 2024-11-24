#include "uint64_math.h"
#include <Arduino.h>
#include <LibPrintf.h>
#include <EEPROMWearLevel.h>

#define EEPROM_LAYOUT_VERSION 0
#define AMOUNT_OF_INDEXES 1

#define EEPROM_INDEX_TICKS 0

#define SKIP_PULSES 5
#define TIMER_OVERFLOW_VECTOR TIMER1_OVF_vect
#define TIMER_CAPTURE_VECTOR TIMER1_CAPT_vect

#define INTERVAL_15_MINS (60UL*15UL)
#define INTERVAL_60_MINS (60UL*60UL)
#define INTERVAL_1_DAY   (INTERVAL_60_MINS*24UL)
#define INTERVAL_7_DAYS  (INTERVAL_1_DAY*7UL)

// Number of seconds (Pulse-per-second input)
uint32_t num_intervals = INTERVAL_7_DAYS;

volatile uint32_t capture_msw = 0; // 32-bit overflow counter
volatile uint32_t first_msw = 0;   // First timestamp high word
volatile uint16_t first_lsw = 0;   // First timestamp low word
volatile uint32_t last_msw = 0;    // Last timestamp high word
volatile uint16_t last_lsw = 0;    // Last timestamp low word
volatile uint16_t capture_lsw = 0; // Captured timer value
volatile uint32_t pulse_count = 0;
volatile bool finished = false;
volatile bool calibration_drift_error = false;

static uint8_t saveTCCR1A, saveTCCR1B;

volatile uint32_t previous_lsw, previous_msw;
volatile uint32_t previous_diff_lsw, previous_diff_msw;
volatile uint32_t current_diff_lsw, current_diff_msw;

float max_jitter_ppm = 20;
uint16_t max_jitter_ticks;

// Timer settings
uint32_t ticks_per_second = 16000000; // 16 MHz for Arduino Uno, prescaler = 1

void test_calculate_oscillator_offset() {
    struct TestCase {
        uint32_t first_msw;
        uint16_t first_lsw;
        uint32_t last_msw;
        uint16_t last_lsw;
        uint32_t num_intervals; // Number of intervals (seconds)
    };

    // Define test cases for a variety of intervals up to 24 hours
    TestCase tests[] = {
        {0x00000000, 0x0000, 0x00000001, 0x0000, 1},           // 1 second
        {0x00000000, 0x0000, 0x0000003C, 0x0000, 60},          // 1 minute
        {0x00000000, 0x0000, 0x00000E10, 0x0000, 3600},        // 1 hour
        {0x00000000, 0x0000, 0x00015180, 0x0000, 86400},      // 24 hours
        {0x00000000, 0x0001, 0x00015181, 0x0000, 86400},      // Near 24 hours with fractional start
        {0x00000001, 0x0000, 0x00015181, 0xFFFF, 86400},      // Across multiple MSWs
        {0x00000000, 0x0000, 0x00000001, 0xFFFF, 1},           // Small delta near overflow
        {0x12345678, 0x9ABC, 0x12345679, 0x9ABD, 1},           // Random test case
        {0xFFFFFFFF, 0x0000, 0x00000000, 0x0001, 1},           // Delta across overflow
        {0x00000000, 0x0001, 0x00000000, 0xFFFF, 1},           // Minimum measurable delta
    };

    // Run each test case
    for (int i = 0; i < sizeof(tests) / sizeof(TestCase); i++) {
        // Set global variables for test
        first_msw = tests[i].first_msw;
        first_lsw = tests[i].first_lsw;
        last_msw = tests[i].last_msw;
        last_lsw = tests[i].last_lsw;
        num_intervals = tests[i].num_intervals;

        // Print test case information
        Serial.print("Test ");
        Serial.print(i + 1);
        Serial.print(": First = 0x");
        Serial.print(first_msw, HEX);
        Serial.print(first_lsw, HEX);
        Serial.print(", Last = 0x");
        Serial.print(last_msw, HEX);
        Serial.print(last_lsw, HEX);
        Serial.print(", Intervals = ");
        Serial.println(num_intervals);

        // Call the function under test
        calculate_oscillator_offset();

        // Print a separator for clarity
        Serial.println("---------------------------");
    }
}

void convertTo64Bit(uint32_t msw, uint16_t lsw, uint32_t &resultHigh,
                    uint32_t &resultLow) {
  // Shift the lower 16 bits of msw into the higher word of the lsw
  resultHigh = msw >> 16; // High 32 bits of the 64-bit number
  resultLow = (msw << 16) |
              (uint32_t)lsw; // Combine the remaining 16 bits of msw with lsw
}

static inline void capture_init(void) {
  cli();
  finished = false;
  pulse_count = 0;
  saveTCCR1A = TCCR1A;
  saveTCCR1B = TCCR1B;
  TCCR1B = 0;
  TCCR1A = 0;
  TCNT1 = 0;
  TIFR1 = (1 << ICF1) | (1 << TOV1);
  TIMSK1 = (1 << ICIE1) | (1 << TOIE1);
  sei();
}
static inline void capture_start(void) {
  TCCR1B = (1 << ICNC1) | (1 << ICES1) | (1 << CS10);
}
static inline uint16_t capture_read(void) { return ICR1; }
static inline uint8_t capture_overflow(void) { return TIFR1 & (1 << TOV1); }
static inline void capture_overflow_reset(void) { TIFR1 = (1 << TOV1); }
static inline void capture_shutdown(void) {
  TCCR1B = 0;
  TIMSK1 = 0;
  TCCR1A = saveTCCR1A;
  TCCR1B = saveTCCR1B;
}



ISR(TIMER_OVERFLOW_VECTOR) { capture_msw++; }

ISR(TIMER_CAPTURE_VECTOR) {
  uint16_t capture_lsw;
  uint16_t interval_tick_diff;
  uint32_t convert_msw, convert_lsw;
  
  cli(); // Disable interrupts to ensure consistent readings

  // get the timer capture
  capture_lsw = capture_read();

  // Handle the case where but capture and overflow interrupts were pending
  // (eg, interrupts were disabled for a while), or where the overflow occurred
  // while this ISR was starting up.  However, if we read a 16 bit number that
  // is very close to overflow, then ignore any overflow since it probably
  // just happened.
  if (capture_overflow() && capture_lsw < 0xFF00) {
    capture_overflow_reset();
    capture_msw++;
  }

  convertTo64Bit(capture_msw, capture_lsw, convert_msw, convert_lsw);

  uint32_t temp_current_diff_msw = current_diff_msw;
  uint32_t temp_current_diff_lsw = current_diff_lsw;

  sub64(convert_msw,  convert_lsw, previous_msw, previous_lsw, temp_current_diff_msw, temp_current_diff_lsw);

  // Copy back results to volatile variables
  current_diff_msw = temp_current_diff_msw;
  current_diff_lsw = temp_current_diff_lsw;
  
  if (pulse_count == SKIP_PULSES) {
    // Record the first timestamp
    first_msw = capture_msw;
    first_lsw = capture_lsw;
  } else if (pulse_count > SKIP_PULSES + 1) {

    if (current_diff_lsw > previous_diff_lsw) {
      interval_tick_diff = current_diff_lsw - previous_diff_lsw;
    } else if (current_diff_lsw < previous_diff_lsw) {
      interval_tick_diff = previous_diff_lsw - current_diff_lsw;
    } else {
      interval_tick_diff = 0;
    }
    

    if (interval_tick_diff > max_jitter_ticks) {
      // We have too much drift or lost signal!
      calibration_drift_error = true;
      finished = true;
      capture_shutdown();
      sei();
      return;
    }
  } 
  
  if (pulse_count >= num_intervals + SKIP_PULSES) {
    // We need to count number of "intervals" between pulses, so add an extra
    // pulse so we have X intervals Record the last timestamp
    last_msw = capture_msw;
    last_lsw = capture_lsw;
    finished = true;
    capture_shutdown();
    sei();
    return;
  }

  pulse_count++;

  previous_diff_lsw = current_diff_lsw;
  previous_diff_msw = current_diff_msw;
  previous_lsw = convert_lsw;
  previous_msw = convert_msw;
  sei();
}


// Initialize Timer1
void timer1_init() {
  TCCR1A = 0;            // Normal mode
  TCCR1B = (1 << CS10);  // Prescaler 1
  TCNT1 = 0;             // Reset timer counter
  TIMSK1 = (1 << TOIE1); // Enable overflow interrupt
  sei();                 // Enable global interrupts
}

// Get the current ticks in 64-bit representation
void get_ticks(uint32_t &high, uint32_t &low) {
  uint32_t msw;
  uint16_t lsw;

  cli(); // Disable interrupts to ensure consistent readings
  // get the timer capture
  msw = capture_msw;
  lsw = capture_read();
  // Handle the case where but capture and overflow interrupts were pending
  // (eg, interrupts were disabled for a while), or where the overflow occurred
  // while this ISR was starting up.  However, if we read a 16 bit number that
  // is very close to overflow, then ignore any overflow since it probably
  // just happened.
  if (capture_overflow() && lsw < 0xFF00) {
    capture_overflow_reset();
    msw++;
  }
  sei(); // Re-enable interrupts

  convertTo64Bit(msw, lsw, high, low); // Convert to 64-bit representation
}

// Convert ticks to seconds (as a float)
float ticks_to_seconds(uint32_t high, uint32_t low) {
  float total_ticks = (float)high * 4294967296.0 + low; // high * 2^32 + low
  return total_ticks / ticks_per_second;
}

// Convert ticks to milliseconds (as an integer)
uint32_t ticks_to_milliseconds(uint32_t high, uint32_t low) {
  uint32_t ticks_high_ms = (high * 4294967296UL) / ticks_per_second * 1000;
  uint32_t ticks_low_ms = (low * 1000UL) / ticks_per_second;
  return ticks_high_ms + ticks_low_ms;
}

// Convert ticks to microseconds (as an integer)
uint32_t ticks_to_microseconds(uint32_t high, uint32_t low) {
  uint32_t ticks_high_us = (high * 4294967296UL) / ticks_per_second * 1000000;
  uint32_t ticks_low_us = (low * 1000000UL) / ticks_per_second;
  return ticks_high_us + ticks_low_us;
}

float calculate_ppm(uint32_t actual, uint32_t expected) {
  uint32_t difference;
  float ppm;

  if (actual > expected) {
    difference = actual - expected;
    ppm = difference * 1000000.0 / expected;
  } else {
    difference = expected - actual;
    ppm = difference * -1000000.0 / expected;
  }

  return ppm;
}

uint32_t calculate_ticks_ppm(uint32_t ticks, float ppm) {
  return ticks / 1000000.0 * ppm;
}

void calculate_oscillator_offset() {
  uint32_t last_64_msw, last_64_lsw, first_64_msw, first_64_lsw;

  convertTo64Bit(last_msw, last_lsw, last_64_msw, last_64_lsw);

  convertTo64Bit(first_msw, first_lsw, first_64_msw, first_64_lsw);

  uint32_t delta_msw, delta_lsw; // Difference between timestamps
  sub64(last_64_msw, last_64_lsw, first_64_msw, first_64_lsw, delta_msw,
        delta_lsw);

  // Calculate ticks per pulse
  uint32_t ticks_per_pulse_high, ticks_per_pulse_low;
  div64(delta_msw, delta_lsw, 0, num_intervals, ticks_per_pulse_high,
        ticks_per_pulse_low);

  // Calculate PPM offset
    float ppm = calculate_ppm(ticks_per_pulse_low, ticks_per_second);

  // Print results
  Serial.println("Oscillator Characterization Results:");
  printf("   First timestamp: 0x%08lX, 0x%08lX\n", first_64_msw, first_64_lsw);
  printf("    Last timestamp: 0x%08lX, 0x%08lX\n", last_64_msw, last_64_lsw);
  printf("       Delta Ticks: 0x%08lX, 0x%08lX\n", delta_msw, delta_lsw);
  printf("Ticks per Interval: 0x%08lX, %lu\n", ticks_per_pulse_high, ticks_per_pulse_low);
  printf("Expected Oscillator Frequency: %ld\n", ticks_per_second);
  printf("PPM Offset: %.2f ppm\n", ppm);

  ticks_per_second = ticks_per_pulse_low;
}

void write_calibration() {
  EEPROMwl.put(EEPROM_INDEX_TICKS, ticks_per_second);
  Serial.println(F("Saving configuration"));
}

void read_calibration() {
  
  EEPROMwl.get(EEPROM_INDEX_TICKS, ticks_per_second);
  Serial.println(F("Reading configuration"));
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(115200);

  while (!Serial) {
    delay(1);
  }

  EEPROMwl.begin(EEPROM_LAYOUT_VERSION, AMOUNT_OF_INDEXES);

  Serial.print(F("Max number of EEPROM indices: "));
  Serial.println(EEPROMwl.length());

  Serial.print(F("Max data length for index ["));
  Serial.print(EEPROM_INDEX_TICKS);
  Serial.print("]:");
  Serial.println(EEPROMwl.getMaxDataLength(EEPROM_INDEX_TICKS));

  EEPROMwl.printStatus(Serial);

  read_calibration();

  Serial.print("EEPROM Calibration: ");
  Serial.print(ticks_per_second);
  Serial.println(" Hz");

  if (ticks_per_second < 14000000UL || ticks_per_second > 18000000UL) {
    Serial.println("Ticks value out of range! Resetting!");
    ticks_per_second = F_CPU;
  }

  max_jitter_ticks = calculate_ticks_ppm(ticks_per_second, max_jitter_ppm);
  Serial.print("Max tick deviation: ");
  Serial.println(max_jitter_ticks);

  Serial.println("PPS Capturing starting...");

  capture_init();
  
  capture_start();

  //test_calculate_oscillator_offset();

}

void loop() {
  uint32_t previous_count = 0;
  bool LED_VALUE = LOW;

  while (!finished) {
    if (previous_count != pulse_count && pulse_count > SKIP_PULSES) {
      previous_count = pulse_count;
      LED_VALUE = !LED_VALUE;
      digitalWrite(LED_BUILTIN, LED_VALUE);
      Serial.print("Count [");
      Serial.print(pulse_count - SKIP_PULSES);
      Serial.print("/");
      Serial.print(num_intervals);
      Serial.print("], Difference: ");
      Serial.print(previous_diff_lsw);
      Serial.print(", Previous: 0x");
      Serial.print(previous_msw, HEX);
      Serial.print(", 0x");
      Serial.println(previous_lsw, HEX);
      
    }
  }
  if (finished) {
    digitalWrite(LED_BUILTIN, HIGH);
    if (calibration_drift_error) {
      Serial.println("Calibration drift error!");
      Serial.print("Previous difference: ");
      Serial.println(previous_diff_lsw);
      Serial.print("Current difference: ");
      Serial.println(current_diff_lsw);
    } else {
      calculate_oscillator_offset();
      write_calibration();
    }
    while (1)
      ; // Stop further execution
  }
}
