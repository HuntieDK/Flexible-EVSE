#include <stdarg.h>
#include <Arduino.h>

#ifdef ATTINYX4

#define SINGLE_CHARGER

#ifdef F_CPU
#undef F_CPU
#endif
#define F_CPU 15360000  // In our design cpu runs at 15.36 MHz

#else

#define MULTI_CHARGER
#include <Controllino.h>

#endif

#if !defined(SINGLE_CHARGER) && !defined(MULTI_CHARGER)
#error "Unknown hardware"
#endif

#ifdef SINGLE_CHARGER

#define N_PORTS 1
#define ACTUAL_PORTS  1

#define SERIAL_DEBUG        // Enable ATTINY84 debug output

#define HAS_UI              // We got button and led UI (manages current setting)
#undef  HAS_CURRENT_MGMT    // No active current management

#define MAX_CURRENT 16

#define PWM_SLOPE_CORRECTION  5   // Slope takes about 6ms in single charger opamp output - correct by that much

#undef  MONITOR_RELAYS
#define UNTETHERED
#undef  HAS_METERING
#undef  LOCKABLE

#else

#define N_PORTS 6
#define ACTUAL_PORTS  2

#undef  HAS_UI              // No button and led UI
#define HAS_CURRENT_MGMT    // Active current management enabled (controls current setting)

#define PWM_SLOPE_CORRECTION  0   // Currently no slope correction in multi-charger

#define MONITOR_RELAYS
#define UNTETHERED
#define HAS_LOCKING
#define HAS_METERING

#endif

// Some Atmel port Id's used throughout the code
#define PORT_A_ID  'a'
#define PORT_B_ID  'b'
#define PORT_D_ID  'd'
#define PORT_E_ID  'e'
#define PORT_H_ID  'h'
#define PORT_J_ID  'j'
#define PORT_L_ID  'l'

struct AtmelPort {
  volatile uint8_t* port;
  volatile uint8_t* ddr;
  volatile uint8_t* pin;
  byte pno;
  void input(bool pullup = false) const;
  void output(bool high = false) const;
  bool read() const;
  void write(bool high) const;
};

#define APORT_WRAP(ID,PNO) { &PORT##ID, &DDR##ID, &PIN##ID, PNO }
#define APORT(ID, PNO) APORT_WRAP(ID,PNO)

#define TIMER_TOP_16  (F_CPU/8/1000/2)  // Top value for 16-bit timers for 1kHz with factor 8 prescaling
#define TIMER_TOP_8   (F_CPU/64/1000/2) // Top value for 8-bit timers for 1kHz with factor 64 prescaling
#define TIMER_TOP TIMER_TOP_16          // Value used for PWM calculation always relates to 16-bit timer to get good correct resolution

// Charging states:
#define STATE_UNDEF       0   // State is undefined / in error
#define STATE_IDLE        1   // No car connected: 12/12v
#define STATE_CONNECT     2   // Car just connected: 9/9v
#define STATE_WAIT        3   // Charger indicates current; car idle: 9/-12v
#define STATE_CHARGE      4   // Car indicates charging is to begin w/o fan: 6/-12v
#define STATE_FAN         5   // Car indicates charging is to begin w/ fan: 3/-12v
#define STATE_DISCONN     6   // Car disconnected (12/-12v)
#define STATE_PAUSED      7   // Pause charging
#define N_STATES          8   // Total number of states
#define STATE_ANY         N_STATES    // Pseudo state indicating state transition for any state

// Timer type identifiers
#define TIMER_MS    0
#define TIMER_CS    1
#define TIMER_DS    2
#define TIMER_CNT   3

#define RELAY_TIME  3   // 3/10 sec for flipping bistable relay

// Types
typedef void (*TimerFunc)();
typedef bool (*PortCondition)(byte port, byte newState, byte oldState);
typedef void (*PortTransition)(byte port, byte newState, byte oldState);
struct Timer;

// Vars
extern byte portCount;
extern byte inputStates[N_PORTS];
extern unsigned int inputStateAges[N_PORTS];
extern byte portStates[N_PORTS];
extern bool chargerPaused[N_PORTS];
extern volatile unsigned int msCount;
extern volatile unsigned int csCount;
extern volatile unsigned int dsCount;
extern volatile unsigned int sCount;
#ifdef MONITOR_RELAYS
extern bool relayMonitor[N_PORTS];
#endif

// util.cpp
int orBits(int bit, ...);
void crc16Update(uint16_t& crc, uint8_t a);
uint16_t calcCRC16(const byte* data, byte length);

// timer.cpp

struct SimpleTimer {
  inline SimpleTimer(TimerFunc call) : call(call) {};
  TimerFunc call;
  struct SimpleTimer* next;
};

struct ComplexTimer {
  inline ComplexTimer(TimerFunc call, uint16_t count, bool recurring) : call(call), count(count), recurring(recurring), rest(0) {};
  TimerFunc call;
  uint16_t  rest;
  uint16_t  count;
  bool      recurring;
  struct ComplexTimer* next;
};

void initTimers();
void setOutput(byte port, unsigned int dutyCycle);
void addSimpleTimer(byte unit, SimpleTimer& timer);
void addComplexTimer(byte unit, ComplexTimer& timer);
void setComplexTimer(ComplexTimer& timer, uint16_t count, bool recurring);

// ad.cpp
void initAD();
void startAD(int half);
byte cableCurrentRestriction(byte port, byte current);

// state.cpp
void initState();
void chargerState();
void updateCurrent();
bool addCondition(byte state, PortCondition condition);
bool addTransition(byte state, PortTransition transition);

// ui.cpp
#ifdef HAS_UI
void initUI();
void uiState();
void updateChargerState();
byte getCurrent(byte port);
#endif

// current.cpp
#ifdef HAS_CURRENT_MGMT
void initCurrent();
void currentState();
void updateChargerState();
byte getCurrent(byte port);
#endif

// metering.cpp
#ifdef HAS_METERING
void initMetering();
void stateMetering();
#endif

// locking.cpp
#ifdef HAS_LOCKING
void initLocking();
void stateLocking();
#endif

// debug.cpp
void initDebug();
void debug_ch(char c);
void debug_str(const char* s);
void debug_str_p(const char* s);
void debugf_p(const char* progmem_fmt, ...);
void debugf(const char* fmt, ...);

#define DEBUGF_CSTR_P(str, ...) { static const char _loc_p_str[] PROGMEM = str; debugf_p(_loc_p_str, __VA_ARGS__); }
#define DEBUG_CSTR_P(str) { static const char _loc_p_str[] PROGMEM = str; debug_str_p(_loc_p_str); }

