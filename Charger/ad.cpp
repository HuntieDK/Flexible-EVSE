#include "defs.h"

#include <limits.h>

#define ADREF ( /*_BV(REFS1) |*/ _BV(REFS0))    // We use AVCC as reference (5V) as this allows us to monitor full range -12 to 12.

#define SAMPLE_COUNT  3   // # of samples to be correct in a row to change state

// Legal state values:

#define LEVEL_UNDEF     0
#define LEVEL_LOW12     1
#define LEVEL_HIGH3     2
#define LEVEL_HIGH6     3
#define LEVEL_HIGH9     4
#define LEVEL_HIGH12    5
#define N_LEVELS        6

uint16_t adResult[N_PORTS][2];

#ifdef UNTETHERED
uint16_t cableResults[N_PORTS];
#endif

static const byte levelMap[1023*3/100] PROGMEM = {
  LEVEL_LOW12,  // -12 - -11
  LEVEL_UNDEF,  // -11 - -10
  LEVEL_UNDEF,  // -10 - -9
  LEVEL_UNDEF,  // -9  - -8
  LEVEL_UNDEF,  // -8  - -7
  LEVEL_UNDEF,  // -7  - -6
  LEVEL_UNDEF,  // -6  - -5
  LEVEL_UNDEF,  // -5  - -4
  LEVEL_UNDEF,  // -4  - -3
  LEVEL_UNDEF,  // -3  - -2
  LEVEL_UNDEF,  // -2  - -1
  LEVEL_UNDEF,  // -1  -  0
  LEVEL_UNDEF,  //  0  -  1
  LEVEL_UNDEF,  //  1  -  2
  LEVEL_HIGH3,  //  2  -  3
  LEVEL_HIGH3,  //  3  -  4
  LEVEL_UNDEF,  //  4  -  5
  LEVEL_HIGH6,  //  5  -  6
  LEVEL_HIGH6,  //  6  -  7
  LEVEL_UNDEF,  //  7  -  8
  LEVEL_HIGH9,  //  8  -  9
  LEVEL_HIGH9,  //  9  - 10
  LEVEL_UNDEF,  // 10  - 11
  LEVEL_HIGH12, // 11  - 12
  LEVEL_HIGH12, // 12  - 13
  LEVEL_UNDEF,  // 13  - 14
  LEVEL_UNDEF,  // 14  - 15
  LEVEL_UNDEF,  // 15  - 16
  LEVEL_UNDEF,  // 16  - 17
  LEVEL_UNDEF,  // 17  - 18
};

byte portLevels[N_PORTS][2];
byte inputStates[N_PORTS];
unsigned int inputStateAges[N_PORTS];
static byte stateCount[N_PORTS][N_STATES];

static const byte stateMap[N_LEVELS][N_LEVELS] PROGMEM = {
//  LEVEL_UNDEF,  LEVEL_LOW12,  LEVEL_HIGH3,  LEVEL_HIGH6,  LEVEL_HIGH9,  LEVEL_HIGH12   <-- High half
  { STATE_UNDEF,  STATE_UNDEF,  STATE_UNDEF,  STATE_UNDEF,  STATE_UNDEF,  STATE_UNDEF   }, // Low half LEVEL_UNDEF
  { STATE_UNDEF,  STATE_UNDEF,  STATE_FAN,    STATE_CHARGE, STATE_WAIT,   STATE_DISCONN }, // Low half LEVEL_LOW12
  { STATE_UNDEF,  STATE_UNDEF,  STATE_CONNECT,STATE_UNDEF,  STATE_UNDEF,  STATE_UNDEF   }, // Low half LEVEL_HIGH3
  { STATE_UNDEF,  STATE_UNDEF,  STATE_UNDEF,  STATE_CONNECT,STATE_UNDEF,  STATE_UNDEF   }, // Low half LEVEL_HIGH6
  { STATE_UNDEF,  STATE_UNDEF,  STATE_UNDEF,  STATE_UNDEF,  STATE_CONNECT,STATE_UNDEF   }, // Low half LEVEL_HIGH9
  { STATE_UNDEF,  STATE_UNDEF,  STATE_UNDEF,  STATE_UNDEF,  STATE_UNDEF,  STATE_IDLE    }  // Low half LEVEL_HIGH12
};

// AD stuff
byte adPort = 0;
byte adPart = 0;
int adStart = 0;
int adDone = 0;

void initAD()
{
  memset(adResult, 0, sizeof(adResult));
  memset(portLevels, LEVEL_UNDEF, sizeof(portLevels));
  memset(inputStates, STATE_UNDEF, sizeof(inputStates));
  memset(inputStateAges, 0, sizeof(inputStateAges));
  memset(stateCount, 0, sizeof(stateCount));
  
  ADCSRA = orBits(ADEN, ADIE, ADPS2, /*ADPS1, ADPS0,*/ -1);
#ifdef MINI
  ADCSRB &= ~_BV(MUX5);
#endif
  ADMUX = ADREF | adPort;
  DIDR0 |= orBits(ADC5D, ADC4D, ADC3D, ADC2D, ADC1D, ADC0D);
}

void startAD(int half)
{
  ADCSRA |= _BV(ADSC); // Start conversion
  adPart = half;
  adStart++;
}

ISR(ADC_vect)
{
  // Conversion done. Read data, set MUX to next...
  ADCSRA &= ~_BV(ADIF); // Reset interrupt flag

  uint16_t level = ADCW;

#ifdef UNTETHERED
  if (adPart == 2) {
    // Cable resistor measurement on proximity pin done
    cableResults[adPort] = level;
    adPort = (adPort + 1) % N_PORTS;
    ADMUX = ADREF | adPort;
    return;
  }
#endif

  int half = adPart^(adPort>2);

  adResult[adPort][half] = level; // TODO: Remove...
  byte* portLevel = &portLevels[adPort][0];
  
  portLevel[half] = pgm_read_byte(&levelMap[level * 3 / 100]);

  if (adPart) {
    // Just got top/bottom measurement of adPort - calculate state.
    byte state = pgm_read_byte(&stateMap[portLevel[0]][portLevel[1]]);
    byte& inputState = inputStates[adPort];
    byte oldState = inputState;
    for (int i=0; i<N_STATES; i++) {
      byte& count = stateCount[adPort][i];
      if (i == state) {
        if (count < SAMPLE_COUNT) {
          count++;
          if (count == SAMPLE_COUNT) {
            inputState = state;
          }
        }
      } else {
        if (count > 0) {
          count--;
          if (count == 0 && i == inputState) {
            inputState = STATE_UNDEF;
          }
        }
      }
    }
    
    register unsigned int* age = inputStateAges;   // Update all port ages (1 ms older now)
    for (byte port = 0; port < N_PORTS; port++, age++) {
      if (port == adPort && inputState != oldState) {
        *age = 0;
      } else {
        if (!(*age & 0x8000)) {
          (*age)++;
        }
      }
    }
    
    // Set mux for next conversion (do it in advance to let input settle)
#ifndef UNTETHERED
    // If cable is tethered, we're done.
    adPort = (adPort + 1) % N_PORTS;
    ADMUX = ADREF | adPort;
#else
    // Cable is untethered - do resistor measurement
    ADMUX = ADREF | (adPort+N_PORTS); // MUX to proximity input
    ADCSRA |= _BV(ADSC); // Start conversion
    adPart = 2;   // Remember we're doing resistor measurement
#endif
  }
  adDone++;
}

