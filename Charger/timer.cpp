#include "defs.h"

#define UNWRAP(X) X

#define INITTIMER_8_WRAP(N,I) \
  TCCR##N##A = orBits(WGM##N##0, -1); \
  TCCR##N##B = orBits(WGM##N##2, CS##N##2, -1); \
  TIMSK##N = I; \
  TIFR##N = orBits(-1); \
  OCR##N##A = TIMER_TOP_8;

#define INITTIMER_16_WRAP(N, I) \
  TCCR##N##A = orBits(WGM##N##1, -1); \
  TCCR##N##B = orBits(WGM##N##3, CS##N##1, /*CS##N##0,*/ -1); \
  TCCR##N##C = orBits(-1); \
  TIMSK##N = I; \
  TIFR##N = orBits(-1); \
  ICR##N = TIMER_TOP;

#define TIMER_BITS_0  8
#define TIMER_BITS_1  16
#define TIMER_BITS_2  8
#define TIMER_BITS_3  16
#define TIMER_BITS_4  16
#define TIMER_BITS_6  16

#define TIMER_BITS_WRAP(timer) TIMER_BITS_##timer
#define TIMER_BITS(timer) TIMER_BITS_WRAP(timer)

#define TIMER_REG_WRAP(reg,timer) reg##timer
#define TIMER_REG(reg,timer) TIMER_REG_WRAP(reg,timer)

#define INITTIMER_WRAP3(X,B,I) INITTIMER_##B##_WRAP(X,I)
#define INITTIMER_WRAP2(X,B,I) INITTIMER_WRAP3(X,B,I)
#define INITTIMER_WRAP(X, I) INITTIMER_WRAP2(X, TIMER_BITS_##X, I)
#define INITTIMER(X, I) INITTIMER_WRAP(X, I)

static int lastOutput[N_PORTS];

#ifdef SINGLE_CHARGER

#define TIMER_A  1

static int timerMap[N_PORTS] = { TIMER_A };
static int masks[N_PORTS] = { // Map port to timer ports as port connections are twisted
  (1<<COM1B1)|(1<<COM1B0)
};

// The following must indicate the same timer pin.
static int pinMap[N_PORTS] = { 10 };
static int portMap[N_PORTS] = { PORT_B_ID };
static int dataBits[N_PORTS] = { // Map port to pin output mask
  1<<PORTB2
};

#else // MULTI_CHARGER

#define TIMER_A  3
#define TIMER_B  4

static const int timerMap[N_PORTS] /*PROGMEM*/ = { TIMER_A, TIMER_A, TIMER_A, TIMER_B, TIMER_B, TIMER_B };
static const int masks[N_PORTS] /*PROGMEM*/ = { // Map port to timer ports as port connections are twisted
  (1<<COM3B1)|(1<<COM3B0),
  (1<<COM3C1)|(1<<COM3C0),
  (1<<COM3A1)|(1<<COM3A0),
  (1<<COM4A1)|(1<<COM4A0),
  (1<<COM4B1)|(1<<COM4B0),
  (1<<COM4C1)|(1<<COM4C0)
};

// The following must indicate the same timer pins arduino style and atmel style.

static const int pinMap[N_PORTS] /*PROGMEM*/ = { 2, 3, 5, 6, 7, 8 };
static const int portMap[N_PORTS] /*PROGMEM*/ = { PORT_E_ID, PORT_E_ID, PORT_E_ID, PORT_H_ID, PORT_H_ID, PORT_H_ID };
static const int dataBits[N_PORTS] /*PROGMEM*/ = { // Map port to pin output mask
  1<<PORTE4,
  1<<PORTE5,
  1<<PORTE3,
  1<<PORTH3,
  1<<PORTH4,
  1<<PORTH5
};

#endif

volatile uint16_t msCount = 0;
volatile uint16_t csCount = 0;
volatile uint16_t dsCount = 0;
volatile uint16_t sCount = 0;

static byte ms10 = 0;
static byte cs10 = 0;
static byte ds10 = 0;

struct Timer {
  TimerFunc call;
  uint16_t  rest;
  uint16_t  count;
  bool      recurring;
};

static TimerFunc simpleTimerCalls[TIMER_CNT][MAX_TIMERS];
static byte simpleTimerCount[TIMER_CNT] = { 0, 0, 0, 0 };

static Timer complexTimers[TIMER_CNT][MAX_TIMERS];
static byte complexTimerCount[TIMER_CNT] = { 0, 0, 0, 0 };

#define REG2_WRAP(R, N) R##N
#define REG2(R,N) REG2_WRAP(R,N)
#define REG3_WRAP(R, N, C) R##N##C
#define REG3(R,N,C) REG3_WRAP(R,N,C)

void initTimers()
{
  memset(lastOutput, 0xff, sizeof(lastOutput));
  
  for (int port=0; port<N_PORTS; port++) {
    digitalWrite(pinMap[port], HIGH);
    pinMode(pinMap[port], OUTPUT);
  }

  cli();  // Stop interrupts

#ifdef PSRSYNC
  GTCCR = orBits(TSM, PSRSYNC, -1); // Stop timers for sync 
#else
  GTCCR = orBits(TSM, -1); // Stop timers for sync 
#endif

  // Set up timers to use phase-correct PWM, top register
  INITTIMER(TIMER_A, orBits(TIMER_REG(TOIE, TIMER_A), TIMER_REG(ICIE, TIMER_A), -1));
  REG2(TCNT,TIMER_A) = 0;  // Init timer A value to 0

#ifdef TIMER_B
  INITTIMER(TIMER_B, 0);  // Timer 2/4 used for car 4, 5, 6
  REG2(TCNT,TIMER_B) = 0;  // Init timer B value to 0
#endif

  GTCCR = 0;  // Release timers

  sei();  // Re-enable interrupts
}

void addSimpleTimer(byte timer, TimerFunc func)
{
  byte& cnt = simpleTimerCount[timer];
  if (cnt < MAX_TIMERS) {
    simpleTimerCalls[timer][cnt++] = func;
  }
}

Timer* addComplexTimer(byte unit, TimerFunc call)
{
  byte& cnt = complexTimerCount[unit];
  if (cnt < MAX_TIMERS) {
    Timer* timer = &complexTimers[unit][cnt];
    memset(timer, 0, sizeof(Timer));
    timer->call = call;
    cnt++;
    return timer;
  }
  return NULL;
}

void setComplexTimer(Timer* timer, uint16_t count, bool recurring)
{
  if (timer) {
    cli();
    timer->count = timer->rest = count;
    timer->recurring = recurring;
    sei();
  }
}

inline void runTimerCalls(byte timerUnit)
{
  TimerFunc* func = simpleTimerCalls[timerUnit];
  for (register byte cnt = simpleTimerCount[timerUnit]; cnt>0; cnt--) {
    (**(func++))();
  }
  Timer* timer = complexTimers[timerUnit];
  for (register byte cnt = complexTimerCount[timerUnit]; cnt>0; cnt--) {
    if (timer->count != 0 && timer->rest != 0) {
      timer->rest--;
      if (timer->rest == 0) {
        (*timer->call)();
        if (timer->recurring) {
          timer->rest = timer->count;
        }
      }
    }
    timer++;
  }
}

void MsTimer()
{
  runTimerCalls(TIMER_MS);
  if (ms10 != 9) {
    ms10++;
  } else {
    ms10 = 0;
    runTimerCalls(TIMER_CS);
    if (cs10 != 9) {
      cs10++;
    } else {
      cs10 = 0;
      runTimerCalls(TIMER_DS);
      if (ds10 != 9) {
        ds10++;
      } else {
        ds10 = 0;
        runTimerCalls(TIMER_S);
        sCount++;
      }
      dsCount++;
    }
    csCount++;
  }
  msCount++;
}

#define ISR_ROUTINE_WRAP(N,H,I,F) \
ISR(TIMER##N##_##I##_vect) \
{ \
  TIFR##N = 0; \
  startAD(H); \
  F \
}
#define ISR_ROUTINE(N,H,I,F) ISR_ROUTINE_WRAP(N,H,I,F)

ISR_ROUTINE(TIMER_A, 0, OVF, MsTimer(););
ISR_ROUTINE(TIMER_A, 0, CAPT, );

// General routines
void setOutput(byte port, unsigned int dutyCycle)
{
  int& lastCycle = lastOutput[port];
  if (dutyCycle == lastCycle) {
    return;
  }

  /*
  Serial.print("setOutput(");
  Serial.print(port);
  Serial.print(", ");
  Serial.print(dutyCycle);
  Serial.println(")");
  */
  
  switch (dutyCycle) {
  case 0:
  case TIMER_TOP:
    {
      // Set output register first...
      switch (portMap[port]) {
#ifdef PORTB
      case PORT_B_ID:
        if (dutyCycle) {
          PORTB = PORTB | dataBits[port];
        } else {
          PORTB = PORTB & ~dataBits[port];
        }
        break;
#endif
#ifdef PORTE
      case PORT_E_ID:
        if (dutyCycle) {
          PORTE = PORTE | dataBits[port];
        } else {
          PORTE = PORTE & ~dataBits[port];
        }
        break;
#endif
#ifdef PORTH
      case PORT_H_ID:
        if (dutyCycle) {
          PORTH = PORTH | dataBits[port];
        } else {
          PORTH = PORTH & ~dataBits[port];
        }
        break;
#endif
      }
      if (lastCycle == -1 || (lastCycle != 0 && lastCycle != TIMER_TOP)) {
        // We need to change output mode to permanent output
        switch (timerMap[port]) {
#ifdef TCCR1A
        case 1: TCCR1A = TCCR1A & ~masks[port]; break;
#endif
#ifdef TCCR2A
        case 2: TCCR2A = TCCR2A & ~masks[port]; break;
#endif
#ifdef TCCR3A
        case 3: TCCR3A = TCCR3A & ~masks[port]; break;
#endif
#ifdef TCCR4A
        case 4: TCCR4A = TCCR4A & ~masks[port]; break;
#endif
        }
      }
    }
    break;
  default:
    {
      if (lastCycle == -1 || lastCycle == 0 || lastCycle == TIMER_TOP) {
        // We need to change output mode to PWM output
        switch (timerMap[port]) {
#ifdef TCCR1A
        case 1: TCCR1A = TCCR1A | masks[port]; break;
#endif
#ifdef TCCR2A
        case 2: TCCR2A = TCCR2A | masks[port]; break;
#endif
#ifdef TCCR3A
        case 3: TCCR3A = TCCR3A | masks[port]; break;
#endif
#ifdef TCCR4A
        case 4: TCCR4A = TCCR4A | masks[port]; break;
#endif
        }
      }
      register unsigned int value = TIMER_TOP - dutyCycle;
#if PWM_SLOPE_CORRECTION != 0
      if (value > PWM_SLOPE_CORRECTION) {
        value -= PWM_SLOPE_CORRECTION;
      } else {
        value = 0;
      }
#endif
      switch (port) {
      case 0: REG3(OCR, TIMER_A, B) = value; break;
#ifdef MULTI_CHARGER
      case 1: REG3(OCR, TIMER_A, C) = value; break;
      case 2: REG3(OCR, TIMER_A, A) = value; break;
      case 3: REG3(OCR, TIMER_B, A) = value; break;
      case 4: REG3(OCR, TIMER_B, B) = value; break;
      case 5: REG3(OCR, TIMER_B, C) = value; break;
#endif
      }
    }
    break;
  }
  lastCycle = dutyCycle;
}

