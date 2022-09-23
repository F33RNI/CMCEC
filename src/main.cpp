/*
 * Copyright (C) 2022 Fern Lane, CMCEC Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 */

// Arduino library
#include <Arduino.h>

// EEPROM library to store settings (mode and clock divider)
#include <EEPROM.h>

// SPI library
#include <SPI.h>

// WS LED library
#include <Adafruit_NeoPixel.h>

// MIDI library
#include <MIDI.h>

// Arduino PINs
const uint8_t PIN_DAC_LATCH_1 PROGMEM = 8;
const uint8_t PIN_DAC_LATCH_2 PROGMEM = 9;
const uint8_t PIN_DAC_LATCH_3 PROGMEM = 10;
const uint8_t PIN_TRIG_1 PROGMEM = 4;
const uint8_t PIN_GATE_1 PROGMEM = 5;
const uint8_t PIN_TRIG_2 PROGMEM = 6;
const uint8_t PIN_GATE_2 PROGMEM = 7;
const uint8_t PIN_WS_LED PROGMEM = 3;
const uint8_t PIN_BTN PROGMEM = A0;
const uint8_t PIN_CLK_OUT PROGMEM = A1;

// Measured real 1.1V reference in mV
const uint32_t VREF_ACTUAL_MV PROGMEM = 1099;

// Final gains (DAC Voltage = Target V / Gain)
const float GAIN_1 PROGMEM = 1.82907440;
const float GAIN_2 PROGMEM = 1.83076190;

// Before using system it needs to be calibrated
// Uncomment and set CALIBRATION to 0 and after set to 1
// Open Serial monitor on 9600 and follow the instructions
// 0 - Vfref, 1 - Gain
//#define CALIBRATION 1

// Serial port for calibration
#ifdef CALIBRATION
#define CALIBRATION_PORT Serial
#define CALIBRATION_PORT_BAUD_RATE 9600
#endif

// Constants
#define DAC_MAX 4095
#define OUT_MAX_MV 8000
#define NOTE_MIN_NUM 21
#define NOTE_MAX_NUM 108
#define NOTE_START 60 // C4
#define NOTE_MIN_MV 0
#define NOTE_MAX_MV 7250
#define PULSE_DURATION_MS 20
#define LED_SHOW_STATE_DURATION_MS 500
#define BTN_SHORT_DURATION_MS 100
#define BTN_LONG_DURATION_MS 700
#define MIDI_CHANNEL_1 1
#define MIDI_CHANNEL_2 2
#define NOTE_COLOR_LOW 0
#define NOTE_COLOR_HIGH 54613
#define LED_NOTE_OFF_BRIGHTNESS 40
#define LED_NOTE_ON_BRIGHTNESS 80
#define LED_CLOCK_BRIGHTNESS 110
#define MODE_SEPARATE_CHANNELS 0
#define MODE_POLY 1
#define MODE_MIRROR 2
#define MODE_MAX MODE_MIRROR
#define MODE_DEFAULT MODE_SEPARATE_CHANNELS
const uint16_t MODE_COLORS_HUE[MODE_MAX + 1] PROGMEM = { 0, 21845, 43691 };
#define CLK_DIVIDER_MAX 192 // 2 notes
#define CLK_DIVIDER_MIN 6 // 1/4 note
#define CLK_DIVIDER_DEFAULT 24
const uint16_t CLK_DIVIDER_COLORS_VAL[] PROGMEM = { 192, 96, 48, 24, 12, 6 };
const uint16_t CLK_DIVIDER_COLORS_HUE[] PROGMEM = { 0, 10923, 21845, 32768, 43691, 54613 };
#define LED_STATE_POSITION_NONE 0
#define LED_STATE_POSITION_MODE 1
#define LED_STATE_POSITION_CLK_DIVIDER 2
#define EEPROM_MODE_ADR 0
#define EEPROM_CLK_DIVIDER_ADR 1

// DAC variables
volatile uint8_t *dac_1_port_output, *dac_2_port_output, *dac_3_port_output;
uint8_t dac_1_mask, dac_2_mask, dac_3_mask;
uint16_t dac_1_mv_target, dac_2_mv_target;
uint16_t dac_1_value, dac_2_value;

// VCC variables
uint32_t vcc, vcc_prev;
uint64_t vcc_read_timer;

// Note variables
uint8_t note_number_channel_1, note_number_channel_2;

// Gate variables
boolean gate_1_value, gate_2_value;

// Mode variable
uint8_t mode;

// Clock variables
uint16_t clock_count;
uint64_t clock_out_timer, clock_timeout_timer;
uint16_t clock_divider;

// Trig variables
uint64_t trig_1_timer, trig_2_timer;

// LED variables
uint16_t note_1_color, note_2_color, state_color;
uint64_t led_state_timer;
uint8_t led_mode_current = 255;
uint16_t led_clock_divider_current = 65535;
uint8_t led_divider_color_index;
uint8_t led_state_position;

// Button variables
uint64_t button_start_timer;
boolean button_state;
boolean button_long_triggered;
boolean button_short_press, button_long_press;
boolean button_clk_increment;

// Initialize Adafruit_NeoPixel library
Adafruit_NeoPixel leds = Adafruit_NeoPixel(2, PIN_WS_LED, NEO_GRB + NEO_KHZ800);

// Initialize MIDI library
MIDI_CREATE_DEFAULT_INSTANCE();

// Voids
void notes_write(void);
void led_handler(boolean from_clock = false);
void button_read(void);
void button_handle(void);
void gate_1_write(void);
void gate_2_write(void);
void dac_write(void);
void vcc_read(void);

void setup() {
  // Initialize pins
  pinMode(PIN_DAC_LATCH_1, OUTPUT);
  pinMode(PIN_DAC_LATCH_2, OUTPUT);
  pinMode(PIN_DAC_LATCH_3, OUTPUT);
  pinMode(PIN_TRIG_1, OUTPUT);
  pinMode(PIN_GATE_1, OUTPUT);
  pinMode(PIN_TRIG_2, OUTPUT);
  pinMode(PIN_GATE_2, OUTPUT);
  pinMode(PIN_WS_LED, OUTPUT);
  pinMode(PIN_BTN, INPUT_PULLUP);
  pinMode(PIN_CLK_OUT, OUTPUT);

  // Read settings from EEPROM
  EEPROM.begin();
  mode = EEPROM.read(EEPROM_MODE_ADR);
  if (mode > MODE_MAX) {
    mode = MODE_DEFAULT;
    EEPROM.write(EEPROM_MODE_ADR, MODE_DEFAULT);
  }
  clock_divider = (EEPROM.read(EEPROM_CLK_DIVIDER_ADR) << 8) | EEPROM.read(EEPROM_CLK_DIVIDER_ADR + 1);
  if (clock_divider > CLK_DIVIDER_MAX || clock_divider < CLK_DIVIDER_MIN) {
    clock_divider = CLK_DIVIDER_DEFAULT;
    EEPROM.write(EEPROM_CLK_DIVIDER_ADR, (clock_divider >> 8) & 0xFF);
    EEPROM.write(EEPROM_CLK_DIVIDER_ADR + 1, clock_divider & 0xFF);
  }

  // Store ports and masks for fast digital write
  dac_1_port_output = portOutputRegister(digitalPinToPort(PIN_DAC_LATCH_1));
  dac_2_port_output = portOutputRegister(digitalPinToPort(PIN_DAC_LATCH_2));
  dac_3_port_output = portOutputRegister(digitalPinToPort(PIN_DAC_LATCH_3));
  dac_1_mask = digitalPinToBitMask(PIN_DAC_LATCH_1);
  dac_2_mask = digitalPinToBitMask(PIN_DAC_LATCH_2);
  dac_3_mask = digitalPinToBitMask(PIN_DAC_LATCH_3);

  // Initialize SPI as master
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV2);

  // Initialize DAC with first writing
  dac_write();

  // Initialize serial port for calibration
#ifdef CALIBRATION
  CALIBRATION_PORT.begin(CALIBRATION_PORT_BAUD_RATE);
#endif

  // Initialize WS LED
  leds.begin();
#ifdef CALIBRATION
  // Calibration mode - orange color
  leds.setPixelColor(0, 5, 2, 0);
  leds.setPixelColor(1, 5, 2, 0);
#else
  // Default mode - white color
  leds.setPixelColor(0, 5, 5, 5);
  leds.setPixelColor(1, 5, 5, 5);
#endif
	leds.show();

  // Initialize midi
#ifndef CALIBRATION
  MIDI.begin(MIDI_CHANNEL_OMNI);
#endif

  // Wait for settle
  delay(100);

  // Initialize VCC reading
  vcc_read();
  delay(10);
  vcc_read();

  // Calibration
#ifdef CALIBRATION
  // Calibration type 0 - measure 1.1V reference
#if CALIBRATION == 0
  // Read VCC some times
  vcc_read();
  delay(10);
  vcc_read();
  delay(10);
  vcc_read();
  delay(10);

  // Infinite loop
  while (true) {
    vcc_read();
    CALIBRATION_PORT.print(F("VREF_ACTUAL_MV: "));
    CALIBRATION_PORT.print(VREF_ACTUAL_MV);
    CALIBRATION_PORT.print("\tVCC: ");
    CALIBRATION_PORT.println(vcc / 1000.);
    CALIBRATION_PORT.println();
    delay(1000);
  }

  // Calibration type 1 - measure gain
#elif CALIBRATION == 1
  // Debug variables
  uint64_t print_timer = 0;
  uint16_t target_voltage = 500;

  // Infinite loop
  while (true) {
    // Read VCC
    vcc_read();

    // Print and increment voltage every 5s
    if (millis() - print_timer >= 5000) {
      // Increment by 0.5V from 0.5V to 4V
      target_voltage += 500;
      if (target_voltage > 4000)
        target_voltage = 500;

      // Print info
      CALIBRATION_PORT.print(F("----- "));
      CALIBRATION_PORT.print(target_voltage / 1000., 1);
      CALIBRATION_PORT.println(F("V -----"));
      CALIBRATION_PORT.println(F("Use a meter to measure the voltage between GND and CV outputs (with disabled portamento)"));
      CALIBRATION_PORT.print(F("Divide these voltages by "));
      CALIBRATION_PORT.print(target_voltage / 1000., 3);
      CALIBRATION_PORT.println(F(" to get the values of the GAIN_1 and GAIN_2 constants"));
      CALIBRATION_PORT.println();
      
      // Reset timer
      print_timer = millis();
    }

    // Set target voltage
    dac_1_mv_target = target_voltage;
    dac_2_mv_target = target_voltage;

    // Calculate DAC 1 value
    dac_1_value = map(dac_1_mv_target, 0, vcc, 0, DAC_MAX);
    if (dac_1_value > DAC_MAX)
      dac_1_value = DAC_MAX;

    // Calculate DAC 2 value
    dac_2_value = map(dac_2_mv_target, 0, vcc, 0, DAC_MAX);
    if (dac_2_value > DAC_MAX)
      dac_2_value = DAC_MAX;

    // Write data to DAC
    dac_write();
  }
#endif
#endif

  // Default note - C4
  note_number_channel_1 = NOTE_START;
  note_number_channel_2 = NOTE_START;

  // Write initial notes
  notes_write();
  dac_write();
}

void loop() {
  // Trig 1 handler
  if (trig_1_timer != 0 && millis() - trig_1_timer >= PULSE_DURATION_MS) {
    // Put TRIG 1 to LOW
    digitalWrite(PIN_TRIG_1, LOW);
    
    // Stop trig timer
    trig_1_timer = 0;
  }

  // Trig 2 handler
  if (trig_2_timer != 0 && millis() - trig_2_timer >= PULSE_DURATION_MS) {
    // Put TRIG 2 to LOW
    digitalWrite(PIN_TRIG_2, LOW);
    
    // Stop trig timer
    trig_2_timer = 0;
  }

  // Clock out handler
  if (clock_out_timer != 0 && millis() - clock_out_timer >= PULSE_DURATION_MS) {
    // Put CLK to LOW
    digitalWrite(PIN_CLK_OUT, LOW);
    
    // Stop clock timer
    clock_out_timer = 0;
  }

  // Read VCC
  vcc_prev = vcc;
  vcc_read();

  // VCC changed -> Recalculate DAC
  if (vcc != vcc_prev) {
    notes_write();
    vcc_prev = vcc;
  }

  // Read and parse button states
  button_read();
  button_handle();

  // Read data from MIDI
  if (MIDI.read()) {
    switch (MIDI.getType()) {
      // Note ON event
      case midi::NoteOn:
        // Channel 1
        if (MIDI.getChannel() == MIDI_CHANNEL_1) {
          // Get note number
          note_number_channel_1 = MIDI.getData1();
          if (mode == MODE_MIRROR)
            note_number_channel_2 = note_number_channel_1;

          // Write note to DAC
          notes_write();

          // Start trig 1
          digitalWrite(PIN_TRIG_1, HIGH);
          trig_1_timer = millis();

          // Start trig 2
          if (mode == MODE_MIRROR) {
            digitalWrite(PIN_TRIG_2, HIGH);
            trig_2_timer = millis();
          }

          // Start gate 1
          gate_1_value = true;
          gate_1_write();

          // Start gate 2
          if (mode == MODE_MIRROR) {
            gate_2_value = true;
            gate_2_write();
          }
        }

        // Channel 2
        else if (MIDI.getChannel() == MIDI_CHANNEL_2) {
          // Get note number
          note_number_channel_2 = MIDI.getData1();
          if (mode == MODE_MIRROR)
            note_number_channel_1 = note_number_channel_2;

          // Write note to DAC
          notes_write();

          // Start trig 2
          digitalWrite(PIN_TRIG_2, HIGH);
          trig_2_timer = millis();

          // Start trig 1
          if (mode == MODE_MIRROR) {
            digitalWrite(PIN_TRIG_1, HIGH);
            trig_1_timer = millis();
          }

          // Start gate 2
          gate_2_value = true;
          gate_2_write();

          // Start gate 1
          if (mode == MODE_MIRROR) {
            gate_1_value = true;
            gate_1_write();
          }
        }
        break;

      // Note OFF event
      case midi::NoteOff:
        // Channel 1
        if (MIDI.getChannel() == MIDI_CHANNEL_1 && MIDI.getData1() == note_number_channel_1) {
          // Stop gate 1
          gate_1_value = false;
          gate_1_write();

          // Stop gate 2
          if (mode == MODE_MIRROR) {
            gate_2_value = false;
            gate_2_write();
          }
        }

        // Channel 2
        if (MIDI.getChannel() == MIDI_CHANNEL_2 && MIDI.getData1() == note_number_channel_2) {
          // Stop gate 2
          gate_2_value = false;
          gate_2_write();

          // Stop gate 1
          if (mode == MODE_MIRROR) {
            gate_1_value = false;
            gate_1_write();
          }
        }
        break;

      case midi::PitchBend:
        break;

      // Clock event
      case midi::Clock:
        // Prevents clock from starting in between quarter notes after clock is restarted
        if (millis() - clock_timeout_timer > 300)
          clock_count = 0;
        clock_timeout_timer = millis();

        // Note (new clock pulse)
        if (clock_count == 0) {
          // Start CLK output
          digitalWrite(PIN_CLK_OUT, HIGH);
          clock_out_timer = millis();

          // Blink with LED
          led_handler(true);
        }

        // Increment clock counter
        clock_count++;

        // Send pulse once every divider pulses
        clock_count = clock_count % clock_divider;

        break;

      case midi::ActiveSensing: 
          break;

      default:
        break;
    }
  }

  // Update LED
  led_handler(false);
}

/**
 * @brief Calculates voltage for note and writes it to the DAC
 * 
 */
void notes_write(void) {
  // Check channel 1 note range
  if (note_number_channel_1 >= NOTE_MIN_NUM && note_number_channel_1 <= NOTE_MAX_NUM) {
    // Calculate DAC voltage
    dac_1_mv_target = (uint16_t)((float)map(note_number_channel_1, NOTE_MIN_NUM, NOTE_MAX_NUM, NOTE_MIN_MV, NOTE_MAX_MV) / GAIN_1);

    // Calculate DAC value
    dac_1_value = map(dac_1_mv_target, 0, vcc, 0, DAC_MAX);
      if (dac_1_value > DAC_MAX)
        dac_1_value = DAC_MAX;
  }
  
  // Check channel 2 note range
  if (note_number_channel_2 >= NOTE_MIN_NUM && note_number_channel_2 <= NOTE_MAX_NUM) {
    // Calculate DAC voltage
    dac_2_mv_target = (uint16_t)((float)map(note_number_channel_2, NOTE_MIN_NUM, NOTE_MAX_NUM, NOTE_MIN_MV, NOTE_MAX_MV) / GAIN_2);

    // Calculate DAC value
    dac_2_value = map(dac_2_mv_target, 0, vcc, 0, DAC_MAX);
      if (dac_2_value > DAC_MAX)
        dac_2_value = DAC_MAX;
  }

  // Write data to the DAC
  dac_write();
}

/**
 * @brief Shows current states with WS LEDs
 * 
 */
void led_handler(boolean from_clock) {
  // Mode changed
  if (led_state_position == LED_STATE_POSITION_NONE && mode != led_mode_current) {
    // Get color
    state_color = pgm_read_word_near(MODE_COLORS_HUE + mode);

    // Set state to LED_STATE_POSITION_MODE
    led_state_position = LED_STATE_POSITION_MODE;

    // Store current value for next loop
    led_mode_current = mode;

    // Start timer
    led_state_timer = millis();
  }

  // Clock divider changed
  if (led_state_position == LED_STATE_POSITION_NONE && clock_divider != led_clock_divider_current) {
    // Find color index
    led_divider_color_index = 0;
    for (uint8_t i = 0; i < sizeof(CLK_DIVIDER_COLORS_VAL); i++) {
      if (pgm_read_word_near(CLK_DIVIDER_COLORS_VAL + i) == clock_divider) {
        led_divider_color_index = i;
        break;
      }
    }
    
    // Get color
    state_color = pgm_read_word_near(CLK_DIVIDER_COLORS_HUE + led_divider_color_index);

    // Set state to LED_STATE_POSITION_CLK_DIVIDER
    led_state_position = LED_STATE_POSITION_CLK_DIVIDER;

    // Store current value for next loop
    led_clock_divider_current = clock_divider;

    // Start timer
    led_state_timer = millis();
  }

  // Check state timer
  if (led_state_timer != 0 && millis() - led_state_timer >= LED_SHOW_STATE_DURATION_MS) {
    led_state_timer = 0;
    led_state_position = LED_STATE_POSITION_NONE;
  }

  // Calculate note 1 color
  if (note_number_channel_1 >= NOTE_MIN_NUM && note_number_channel_1 <= NOTE_MAX_NUM)
    note_1_color = map((note_number_channel_1 - NOTE_MIN_NUM) % 12, 0, 11, NOTE_COLOR_LOW, NOTE_COLOR_HIGH);

  // Calculate note 2 color
  if (note_number_channel_2 >= NOTE_MIN_NUM && note_number_channel_2 <= NOTE_MAX_NUM)
    note_2_color = map((note_number_channel_2 - NOTE_MIN_NUM) % 12, 0, 11, NOTE_COLOR_LOW, NOTE_COLOR_HIGH);

  // Show state (mode or clock divider)
  if (led_state_timer != 0 && led_state_position > 0) {
    leds.setPixelColor(led_state_position - 1, leds.gamma32(leds.ColorHSV(state_color, 255, LED_CLOCK_BRIGHTNESS)));
    leds.setPixelColor(1 - (led_state_position - 1), 0);
  }

  // Default state
  else {
    // Clock pulse -> max led brightness
    if (from_clock || clock_out_timer != 0) {
      leds.setPixelColor(0, leds.gamma32(leds.ColorHSV(note_1_color, 255, LED_CLOCK_BRIGHTNESS)));
      leds.setPixelColor(1, leds.gamma32(leds.ColorHSV(note_2_color, 255, LED_CLOCK_BRIGHTNESS)));
    }

    // No clock pulse
    else {
      leds.setPixelColor(0, leds.gamma32(leds.ColorHSV(note_1_color, 255, gate_1_value ? LED_NOTE_ON_BRIGHTNESS : LED_NOTE_OFF_BRIGHTNESS)));
      leds.setPixelColor(1, leds.gamma32(leds.ColorHSV(note_2_color, 255, gate_2_value ? LED_NOTE_ON_BRIGHTNESS : LED_NOTE_OFF_BRIGHTNESS)));
    }
  }

  // Update LEDs
  leds.show();
}

/**
 * @brief Handles button press and changes mode or clock divider
 *
 */
void button_handle(void) {
  // Short press
  if (button_short_press) {
    // Increment or decrement clock divider
    if (button_clk_increment)
      clock_divider *= 2;
    else
      clock_divider /= 2;

    // > MAX -> change direction
    if (clock_divider > CLK_DIVIDER_MAX) {
      clock_divider = CLK_DIVIDER_MAX / 2;
      button_clk_increment = false;
    }

    // < MIN -> change direction
    if (clock_divider < CLK_DIVIDER_MIN) {
      clock_divider = CLK_DIVIDER_MIN * 2;
      button_clk_increment = true;
    }

    // Write to EEPROM
    EEPROM.write(EEPROM_CLK_DIVIDER_ADR, (clock_divider >> 8) & 0xFF);
    EEPROM.write(EEPROM_CLK_DIVIDER_ADR + 1, clock_divider & 0xFF);

    // Clear button press flag
    button_short_press = false;
  }

  // Long press
  if (button_long_press) {
    // Mode < MAX -> Increment by 1
    if (mode < MODE_MAX)
      mode++;

    // Mode >= MAX -> Set to 0
    else
      mode = 0;

    // Write to EEPROM
    EEPROM.write(EEPROM_MODE_ADR, mode);

    // Clear button press flag
    button_long_press = false;
  }
}

/**
 * @brief Read button state (short / long)
 *
 */
void button_read(void) {
  // Read button state
  button_state = !digitalRead(PIN_BTN);

  // Button pressed and no timer -> start timer
  if (button_state && button_start_timer == 0)
    button_start_timer = millis();

  // Button pressed, timer > BTN_LONG_DURATION_MS and long press not triggered -> set long press
  if (button_state && millis() - button_start_timer >= BTN_LONG_DURATION_MS && !button_long_triggered) {
    button_long_press = true;
    button_long_triggered = true;
  }

  // Button not pressed and timer started
  if (!button_state && button_start_timer != 0) {
    // No long press triggered and pressed for >= BTN_SHORT_DURATION_MS -> set short press
    if (!button_long_triggered && millis() - button_start_timer >= BTN_SHORT_DURATION_MS)
      button_short_press = true;

    // Stop timer
    button_start_timer = 0;

    // Clear long triggered flag
    button_long_triggered = false;
  }
}

/**
 * @brief Writes HIGh or LOW to the GATE 1
 * 
 */
void gate_1_write(void) {
  digitalWrite(PIN_GATE_1, gate_1_value);
}

/**
 * @brief Writes HIGh or LOW to the GATE 2
 * 
 */
void gate_2_write(void) {
  digitalWrite(PIN_GATE_2, gate_2_value);
}

/**
 * @brief Measures VCC my measuring 1.1V reference against AVcc
 * 
 */
void vcc_read(void) {
  if (vcc_read_timer == 0) {
    // Read 1.1V reference against AVcc
    #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
      ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
    #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
      ADMUX = _BV(MUX5) | _BV(MUX0) ;
    #else
      ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
    #endif

    // Start timer
    vcc_read_timer = millis();
  }

  else if (millis() - vcc_read_timer >= 2) {
    // Start conversion
    ADCSRA |= _BV(ADSC);

    // Measuring
    while (bit_is_set(ADCSRA, ADSC));
    vcc = (ADCH << 8) | ADCL;

    // Print calibration instructions
#if defined(CALIBRATION) && CALIBRATION == 0
    CALIBRATION_PORT.println(F("Use a meter to measure the voltage between GND and AREF pins (~5V)"));
    CALIBRATION_PORT.print(F("Multiply that voltage by "));
    CALIBRATION_PORT.print(vcc / 1024., 7);
    CALIBRATION_PORT.println(F(" to get the value of the internal voltage reference."));
    CALIBRATION_PORT.println(F("Then multiply that by 1000 to get the VREF_ACTUAL_MV constant (in mV)"));
    CALIBRATION_PORT.println();
#endif

    // Back-calculate AVcc in mV
    vcc = (VREF_ACTUAL_MV * ((uint32_t) 1024)) / vcc;

    // Restore default ADC state
    analogReference(DEFAULT);
    analogRead(A5);

    // Reset timer
    vcc_read_timer = 0;
  }
}

/**
 * @brief Writes dac_1_value and dac_2_value to shift registers
 * 
 */
void dac_write(void) {
  // Send lowest 8 bits of first dac value
  *dac_1_port_output &= ~dac_1_mask;
  SPI.transfer(dac_1_value & 0xFF);
  *dac_1_port_output |= dac_1_mask;

  // Send highest 4 bits of first dac value and lowest 4 bits of second dac value
  *dac_2_port_output &= ~dac_2_mask;
  SPI.transfer(((dac_1_value >> 8) & 0xFF) | (((dac_2_value & 0b00001111) << 4) & 0xFF));
  *dac_2_port_output |= dac_2_mask;

  // Send highest 8 bits of second dac value
  *dac_3_port_output &= ~dac_3_mask;
  SPI.transfer((dac_2_value >> 4) & 0xFF);
  *dac_3_port_output |= dac_3_mask;
}
