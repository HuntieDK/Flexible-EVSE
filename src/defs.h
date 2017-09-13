#include <stdarg.h>
#include <Arduino.h>

#ifdef ARDUINO_AVR_MINI
#define SINGLE_CHARGER
#else
#define MULTI_CHARGER
#include <Controllino.h>
#endif

#ifdef SINGLE_CHARGER

#define N_PORTS 1
#define ACTUAL_PORTS  1

#define LATCHING_RELAY      // Define this if a latching relay is used.
#define N_PORT_RELAYS   2   // 2 relays per port in this config (must be drawn individually)
#define N_LATCH_STATES  2   // 2 for latching relays (0=off, 1=on), 1 for non-latching relay

#define HAS_UI              // We got button and led UI (manages current setting)
#undef  HAS_CURRENT_MGMT    // No active current management

#define PWM_SLOPE_CORRECTION  5   // Slope takes about 6ms in single charger opamp output - correct by that much

#undef  MONITOR_RELAYS
#undef  UNTETHERED
#undef  HAS_METERING
#undef  LOCKABLE

#else

#define N_PORTS 6
#define ACTUAL_PORTS  2

#undef LATCHING_RELAY       // Define this if a latching relay is used.
#define N_PORT_RELAYS   1   // 1 contactor per port in this config
#define N_LATCH_STATES  1   // 2 for latching relays (0=off, 1=on), 1 for non-latching relay
#undef  HAS_UI              // No button and led UI
#define HAS_CURRENT_MGMT    // Active current management enabled (controls current setting)

#define PWM_SLOPE_CORRECTION  0   // Currently no slope correction in multi-charger

#define MONITOR_RELAYS
#define UNTETHERED
#define HAS_LOCKING
#define HAS_METERING

#endif

// Some Atmel port Id's used throughout the code
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

#define TIMER_TOP_16 1000   // Top value for 16-bit timers for 1kHz with factor 8 prescaling
#define TIMER_TOP_8   125   // Top value for 8-bit timers for 1kHz with factor 64 prescaling (not used for PWM output, only sampling interrupt due to resolution)

#define TIMER_TOP TIMER_TOP_16   // Value used for PWM calculation always relates to 16-bit timer to get good correct resolution

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
#define TIMER_S     3
#define TIMER_CNT   4

// Max number of timer calls for each
#define MAX_TIMERS  10

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
extern char str[];
#ifdef MONITOR_RELAYS
extern bool relayMonitor[N_PORTS];
#endif

// util.cpp
int orBits(int bit, ...);
void crc16Update(uint16_t& crc, uint8_t a);
uint16_t calcCRC16(const byte* data, byte length);

// timer.cpp
void initTimers();
void setOutput(byte port, unsigned int dutyCycle);
void addSimpleTimer(byte timer, TimerFunc func);
Timer* addComplexTimer(byte unit, TimerFunc func);
void setComplexTimer(Timer* timer, uint16_t count, bool recurring);

// ad.cpp
void initAD();
void startAD(int half);

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

