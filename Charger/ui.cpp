// UI handling for single charger (button + LED)

#include "defs.h"
#include "EEPROM.h"

#ifdef HAS_UI

#define BUTTON_PIN          4

#define LED_COMMON_ANODE      // Define if LED has common anode (i.e. active low)

#define LED_PIN_R           3
#define LED_PIN_G           7
#define LED_PIN_B           8

#undef  PWM_LED_PIN_R

#ifdef SERIAL_DEBUG
#undef  PWM_LED_PIN_G   // Cannot PWM while attiny serial runs
#undef  PWM_LED_PIN_B
#else
#define PWM_LED_PIN_G       LED_PIN_G
#define PWM_LED_PIN_B       LED_PIN_B
#endif

#define DEBOUNCE_COUNT  3

#define CURRENT_6A        0
#define CURRENT_8A        1
#define CURRENT_10A       2
#define CURRENT_13A       3
#define CURRENT_16A       4
#if MAX_CURRENT == 32
#define CURRENT_20A       5
#define CURRENT_24A       6
#define CURRENT_32A       7
#define CURRENT_SETTINGS  8
#else
#define CURRENT_SETTINGS  5
#endif

#ifdef  LED_COMMON_ANODE
#define LED_ON  LOW
#define LED_OFF HIGH
#define LED_INT(x) (255-(x))
#else
#define LED_ON  HIGH
#define LED_OFF LOW
#define LED_INT(x) (x)
#endif

static byte current = CURRENT_6A;   // If not stored; init at lowest current
static byte chargeCurrent = CURRENT_6A;

static const byte currents[CURRENT_SETTINGS] PROGMEM = { 6, 8, 10, 13, 16
#if MAX_CURRENT == 32
  , 20, 24, 32
#endif
};
static const byte intensity[CURRENT_SETTINGS] PROGMEM = {
#if MAX_CURRENT == 32
4, 10, 20, 44, 80, 116, 160, 255
#else
16, 32, 64, 128, 255
#endif
};

#define UI_STATE_ACTIVE   0
#define UI_STATE_PAUSED   1
#define UI_STATE_CURRENT  2

static byte   curState = UI_STATE_ACTIVE;    // What are we doing currently

/* Initialization */
static void debounceTimer();
static void symbolTimer();
static void colorOut(byte r, byte g, byte b);
static void showCurrent(bool clearQueue);
static void bufferEmpty();
static void uiActiveState();
static void uiPausedState();
static void uiCurrentState();
static void buttonChange(bool pressed, int duration);
static void changeState(byte newState);
void setCurrent();
void saveCurrent();
static void indicateChargerState();
static void readEEPROM();
static void writeEEPROM();

void initUI()
{
  // Read current setting:
  readEEPROM();
  
  // Setup button:
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  addSimpleTimer(TIMER_CS, &debounceTimer);
  addSimpleTimer(TIMER_DS, &symbolTimer);

  // We use timer 0 (OC0A, OC0B = pin 5 and 6) for PWM control of UI led. However we need to invert one output to allow all colors.
  // However we need the output of one pin to be inverted, so we manipulate the registers.
  digitalWrite(LED_PIN_R, LED_OFF);
  digitalWrite(LED_PIN_G, LED_OFF);
  digitalWrite(LED_PIN_B, LED_OFF);
  pinMode(LED_PIN_R, OUTPUT);
  pinMode(LED_PIN_G, OUTPUT);
  pinMode(LED_PIN_B, OUTPUT);
  colorOut(0, 0, 0);
  showCurrent(true);
}

/* Button debouncing and handling */
static byte stateCount = 0;
static bool buttonState = false;
static int currentButtonChangeTime = 0;
static int buttonChangeTime = 0;

static void debounceTimer()
{
  if ((digitalRead(BUTTON_PIN) == LOW) == buttonState) {
    stateCount = 0;
    if (!(currentButtonChangeTime&0x8000)) {
      currentButtonChangeTime++;
    }
  } else {
    if (++stateCount == DEBOUNCE_COUNT) {
      buttonState = !buttonState;
      buttonChangeTime = currentButtonChangeTime; // Reset duration counter
      currentButtonChangeTime = 0; // Reset duration counter
      // TODO: Determine what to do with new state...
      stateCount = 0;
    } else {
      if (!(currentButtonChangeTime&0x8000)) {
        currentButtonChangeTime++;
      }
    }
  }
}

/* LED output  buffer */
#define BUFFER_LENGTH 16
#define BUFFER_MASK 0x0f

static struct Symbol {
  // Intensities:
  byte r;
  byte g;
  byte b;
  byte duration;  // Duration in ds - 255 means forever
  byte pause;     // Pause until next symbol
} outputBuffer[BUFFER_LENGTH];

static byte outputBufferIn = 0;
static byte outputBufferOut = 0;
static Symbol* currentSymbol = nullptr;
static unsigned int symbolStamp = 0;

static void nextSymbol() // Must be called with interrupts disabled...
{
  if (outputBufferOut != outputBufferIn) {
    currentSymbol = outputBuffer + outputBufferOut;
    colorOut(currentSymbol->r, currentSymbol->g, currentSymbol->b);
    if (currentSymbol->duration < 255) {
      // Get everything ready for our decisecond interrupt routine
      outputBufferOut = (outputBufferOut + 1)&BUFFER_MASK; // Point out next symbol
      symbolStamp = dsCount; // For timer reference.
    } else {
      // Keep this until further - kill rest of buffer
      outputBufferIn = outputBufferOut = 0; // Reset output
      currentSymbol = nullptr;
    }
  } else {
    currentSymbol = nullptr;
    colorOut(0, 0, 0);
    bufferEmpty();
  }
}

static void symbolTimer()
{
  if (currentSymbol == nullptr) return; // Nothing to do...
  unsigned int timePassed = dsCount - symbolStamp;
  if (currentSymbol->duration != 0) {
    if (timePassed >= currentSymbol->duration) {
      // We're done displaying initial color. Maybe we need to pause:
      if (currentSymbol->pause > 0) {
        colorOut(0, 0, 0);
        currentSymbol->duration = 0;  // Indicate we're timing pause now...
        symbolStamp = dsCount;
      } else {
        nextSymbol();
      }
    }
  } else {
    if (timePassed >= currentSymbol->pause) {
      nextSymbol();
    }
  }
}

static bool enQueue(const Symbol& symbol, bool clear = false)
{
  if (clear) {
    outputBufferIn = outputBufferOut = 0; // Reset output
    currentSymbol = nullptr;
  }
  if (((outputBufferIn + 1)&BUFFER_MASK) == outputBufferOut) {
    // No room (we have one unused guard entry)
    return false;
  }
  outputBuffer[outputBufferIn] = symbol;
  outputBufferIn = (outputBufferIn + 1)&BUFFER_MASK;
  cli();
  if (currentSymbol == nullptr && ((outputBufferOut + 1) % BUFFER_LENGTH) == outputBufferIn) {
    nextSymbol();
  }
  sei();
}

static void colorOut(byte r, byte g, byte b)
{
  #ifdef PWM_LED_PIN_R
  analogWrite(PWM_LED_PIN_R, LED_INT(r));
  #else
  digitalWrite(LED_PIN_R, r ? LED_ON : LED_OFF);
  #endif
  #ifdef PWM_LED_PIN_G
  analogWrite(PWM_LED_PIN_G, LED_INT(g));
  #else
  digitalWrite(LED_PIN_G, g ? LED_ON : LED_OFF);
  #endif
  #ifdef PWM_LED_PIN_B
  analogWrite(PWM_LED_PIN_B, LED_INT(b));
  #else
  digitalWrite(LED_PIN_B, b ? LED_ON : LED_OFF);
  #endif
}

void showCurrent(bool clearQueue)
{
  byte amps = pgm_read_byte(&currents[current]);
  DEBUGF_CSTR_P("Current: %d\n\r", amps);
  if (clearQueue) enQueue({0, 0, 2, 2}, true);
  while (amps>0) {
    if (amps >= 10) {
      amps -= 10;
      enQueue({0, 0, 255, 6, 3});
    } else {
      amps--;
      enQueue({0, 255, 255, 2, amps ? 2 : 3});
    }
  }
}

// Button detection state
static bool bttn = false;
static unsigned int tm = 0;

static bool sb = false;


void uiState()
{
  DEBUGF_CSTR_P("Button state: %d\n", digitalRead(BUTTON_PIN));
  if (bttn != buttonState) {
    bttn = buttonState;
    cli();
    auto duration = buttonChangeTime; // cs
    buttonChangeTime = 0;
    sei();
    buttonChange(bttn, (duration+5)/10); // convert cs to ds...
  }
}

#define PRESS_SHORT       5     // Shorter than 0.5s: Short press
#define PRESS_LONG       20     // Shorter than 1.5s: Long press
#define PRESS_EXTRA_LONG 35     // Shorter than 2.5s: Extra long press
#define PRESS_ULTRA_LONG 50     // Shorter than 2.5s: Extra long press
#define PRESS_TIMEOUT   100     // Longer than 10s: Ignore

void buttonChange(bool pressed, int duration)
{
  static byte shortPressed = 0;
  switch (curState) {
  case UI_STATE_ACTIVE:
    if (pressed) {
      // Button pressed... check if we should reset short press counter
      if (duration >= PRESS_LONG) {
        shortPressed=0; // Ignore short presses after 2s
      }
      enQueue({0, 255, 0, PRESS_SHORT, 0}, true);    // Green light indicate short press
      enQueue({0, 0, 255, PRESS_LONG-PRESS_SHORT, 0}); // Blue light indicates charge setup menu. After that, go back to normal indication
    } else {
      // Serial.print("Duration: ");
      // Serial.print(duration, DEC);
      // Serial.println(".");

      if (duration <= PRESS_SHORT) {
        // Serial.println("Active -> short...");
        shortPressed++;
        if (shortPressed == 2) {
          // Serial.println("Active -> PAUSED...");
          changeState(UI_STATE_PAUSED);
          shortPressed = 0;
        } else {
          indicateChargerState();
        }
      } else {
        shortPressed = 0;
        if (duration <= PRESS_LONG) {
          // Serial.println("Active -> long...");
          // Long press.
          // Serial.println("Active -> CURRENT...");
          changeState(UI_STATE_CURRENT);
        } // Else nothing...
      }
    }
    break;
  case UI_STATE_PAUSED:
    if (pressed) {
      enQueue({0, 255, 0, PRESS_SHORT, 0}, true);    // Green light indicate restart charging
      enQueue({0, 0, 255, PRESS_LONG-PRESS_SHORT, 0}); // Blue light indicates charge setup menu. After that, go back to normal indication
    } else {
      if (duration < PRESS_SHORT) {
        changeState(UI_STATE_ACTIVE);  // Short press: Resume charging
      } else
      if (duration < PRESS_LONG) {
        changeState(UI_STATE_CURRENT); // Long press: Go to charge setup
      }
    }
    break;
  case UI_STATE_CURRENT:
    if (pressed) {
      // Set up LED to reflect button press length
      enQueue({0, pgm_read_byte(&intensity[(current + 1) % CURRENT_SETTINGS]), 0, PRESS_SHORT, 0}, true); // Within short press: Next current level
      enQueue({0, 0, 255, PRESS_LONG-PRESS_SHORT, 0}); // Exit using new current: Blue light
      enQueue({0, 255, 255, PRESS_EXTRA_LONG-PRESS_LONG, 0}); // Exit and save new current: 
      enQueue({0, 0, 0, PRESS_ULTRA_LONG-PRESS_EXTRA_LONG, 0}); // Exit and save new current: 
      enQueue({0, pgm_read_byte(&intensity[current]), 0, 255, 0}); // Back to selection
    } else {
      if (duration <= PRESS_SHORT) {
        current = (current + 1) % CURRENT_SETTINGS;
        enQueue({0, pgm_read_byte(&intensity[current]), 0, 5, 0}, true);
        showCurrent(false);
        enQueue({0, pgm_read_byte(&intensity[current]), 0, 255, 0});
      } else
      if (duration < PRESS_LONG) {
        setCurrent();
        changeState(UI_STATE_ACTIVE);
      } else
      if (duration < PRESS_EXTRA_LONG) {
        setCurrent();
        saveCurrent();
        changeState(UI_STATE_ACTIVE);
      } else
      if (duration < PRESS_ULTRA_LONG) {
        current = chargeCurrent;
        changeState(UI_STATE_ACTIVE);
      } // Else: Go back to toggling.
    }
    break;
  }
}

static void changeState(byte newState)
{
  curState = newState;
  switch (curState) {
  case UI_STATE_ACTIVE:
    chargerPaused[0] = false;  // Restart charging if it was paused
    indicateChargerState();
    break;
  case UI_STATE_CURRENT:
    current = chargeCurrent;
    showCurrent(true);
    enQueue({0, pgm_read_byte(&intensity[current]), 0, 255, 0});
    break;
  case UI_STATE_PAUSED:
    chargerPaused[0] = true;  // Will be picked up by state logic
    enQueue({0, 255, 255, 4, 4}, true); // Do five slow cyan 1Hz blinks while state changes
    enQueue({0, 255, 255, 4, 4});
    enQueue({0, 255, 255, 4, 5});
    break;
  }
}

byte getCurrent(byte port)
{
  return pgm_read_byte(&currents[chargeCurrent]);
}

void setCurrent()
{
  chargeCurrent = current;
  updateCurrent();  // Have charger update current indication
}

void saveCurrent()
{
  writeEEPROM();
}

static void indicateChargerState()
{
  switch (portStates[0]) {
  case STATE_UNDEF:
    enQueue({255, 0, 0, 1, 1}, true); // Red fast blink for undefined state
    break;
  case STATE_IDLE:
  case STATE_DISCONN:
    enQueue({0, 255, 0, 1, 14}, true); // Green slow blink for idle state
    break;
  case STATE_CONNECT:
  case STATE_WAIT:
    enQueue({0, 0, 255, 1, 14}, true); // Blue slow blink for waiting state
    break;
  case STATE_CHARGE:
  case STATE_FAN:
    enQueue({0, 0, 255, 255, 0}, true); // Blue constant light for charging state
    break;
  }
}

void updateChargerState()
{
  // This is called whenever charger changes state.
  if (curState == UI_STATE_ACTIVE) {
    indicateChargerState();
  }
}

static void bufferEmpty()
{
  // LED buffer empty. What to add?
  switch (curState) {
  case UI_STATE_ACTIVE:
    indicateChargerState();
    break;
  case UI_STATE_PAUSED:
    enQueue({255, 255, 0, 1, 9}, true); // Yellow short slow blink for paused state
    break;
  case UI_STATE_CURRENT:
    break;
  }
}

static void readEEPROM()
{
  if (EEPROM[0] == 'F' && EEPROM[1] == 'E' && EEPROM[2] == 'F' && EEPROM[3] < CURRENT_SETTINGS) {
    chargeCurrent = current = EEPROM[3];
  }
}

static void writeEEPROM()
{
  if (EEPROM[0] != 'F') EEPROM[0] = 'F';
  if (EEPROM[1] != 'E') EEPROM[1] = 'E';
  if (EEPROM[2] != 'F') EEPROM[2] = 'F';
  if (EEPROM[3] != chargeCurrent) EEPROM[3] = chargeCurrent;
}

#endif // HAS_UI

