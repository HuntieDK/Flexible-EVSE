#include "defs.h"

// Blå -- kontakt -- Rød -- spole -- Sort

byte portStates[N_PORTS];
bool chargerPaused[N_PORTS];

#ifdef SINGLE_CHARGER

static const byte relayPorts[N_PORTS] = { 2 };  // Attiny84 port PA2

#else

static const byte relayPorts[N_PORTS] = { 4, 9, 10, 11, 12, 13 };    // Controllino D2,7,8,9,10,11 (order due to timer connections)

#ifdef MONITOR_RELAYS
static const AtmelPort relayMonitorPins[N_PORTS] /*PROGMEM*/ = { APORT(L,1), APORT(L,0), APORT(D,4), APORT(D,5), APORT(D,6), APORT(J,4) };   // D18-23 port blocks on Controllino Mega (PL1, PL0, PD4, PD5, PD6, PJ4)
bool relayMonitor[N_PORTS] = { false, false, false, false, false, false };
#endif

#endif

#define N_HOOKS 3

static PortCondition conditions[N_STATES+1][N_HOOKS];
static byte conditionCount[N_STATES+1];
static PortTransition transitions[N_STATES+1][N_HOOKS];
static byte transitionCount[N_STATES+1];

static bool relayStates[N_PORTS];

struct stateDef {
  byte  nextState;  // Next state from this
  int   minAge;     // Centiseconds
  // TODO: Add function pointer to state change verification machine
};

static const stateDef nextStates[N_STATES][N_STATES] PROGMEM = {
  // CURRENTvv MEASURED>> STATE_UNDEF          STATE_IDLE           STATE_CONNECT           STATE_WAIT          STATE_CHARGE          STATE_FAN           STATE_DISCONN      STATE_PAUSED
  /* STATE_UNDEF   */ { { STATE_UNDEF,  0 }, { STATE_IDLE, 100 }, { STATE_UNDEF,     0 }, { STATE_UNDEF, 0 }, { STATE_UNDEF,    0 }, { STATE_UNDEF, 0 }, { STATE_IDLE, 10 }, { STATE_UNDEF,   0 } },
  /* STATE_IDLE    */ { { STATE_IDLE,   0 }, { STATE_IDLE,   0 }, { STATE_CONNECT, 100 }, { STATE_UNDEF, 0 }, { STATE_UNDEF,    0 }, { STATE_UNDEF, 0 }, { STATE_IDLE,  0 }, { STATE_IDLE,    0 } },
  /* STATE_CONNECT */ { { STATE_UNDEF,  0 }, { STATE_IDLE,   0 }, { STATE_WAIT,    100 }, { STATE_WAIT,  0 }, { STATE_UNDEF,    0 }, { STATE_UNDEF, 0 }, { STATE_IDLE,  0 }, { STATE_IDLE,    0 } },
  /* STATE_WAIT    */ { { STATE_UNDEF,  0 }, { STATE_IDLE,   0 }, { STATE_WAIT,      0 }, { STATE_WAIT,  0 }, { STATE_CHARGE, 100 }, { STATE_FAN, 100 }, { STATE_IDLE,  0 }, { STATE_IDLE,    0 } },
  /* STATE_CHARGE  */ { { STATE_UNDEF,  0 }, { STATE_UNDEF,  0 }, { STATE_UNDEF,     0 }, { STATE_WAIT,  0 }, { STATE_CHARGE,   0 }, { STATE_FAN,   0 }, { STATE_IDLE, 10 }, { STATE_PAUSED,  0 } },
  /* STATE_FAN     */ { { STATE_UNDEF,  0 }, { STATE_UNDEF,  0 }, { STATE_UNDEF,     0 }, { STATE_WAIT,  0 }, { STATE_CHARGE,   0 }, { STATE_FAN,   0 }, { STATE_IDLE, 10 }, { STATE_PAUSED,  0 } },
  /* STATE_DISCONN */ { { STATE_UNDEF,  0 }, { STATE_IDLE,   0 }, { STATE_CONNECT,  10 }, { STATE_UNDEF, 0 }, { STATE_UNDEF,    0 }, { STATE_UNDEF, 0 }, { STATE_IDLE,  0 }, { STATE_IDLE,    0 } },
  /* STATE_PAUSED */  { { STATE_UNDEF,  0 }, { STATE_IDLE,   0 }, { STATE_CONNECT,  10 }, { STATE_UNDEF, 0 }, { STATE_UNDEF,    0 }, { STATE_UNDEF, 0 }, { STATE_IDLE, 10 }, { STATE_IDLE, 2500 } },
};


static const byte indicatePower[N_STATES] PROGMEM = {
  /* STATE_UNDEF   */ false,
  /* STATE_IDLE    */ false,
  /* STATE_CONNECT */ false,
  /* STATE_WAIT    */ true,
  /* STATE_CHARGE  */ true,
  /* STATE_FAN     */ true,
  /* STATE_DISCONN */ false,
  /* STATE_PAUSED  */ false,
};

static const byte relayOn[N_STATES] PROGMEM = {
  /* STATE_UNDEF   */ false,
  /* STATE_IDLE    */ false,
  /* STATE_CONNECT */ false,
  /* STATE_WAIT    */ false,
  /* STATE_CHARGE  */ true,
  /* STATE_FAN     */ true,
  /* STATE_DISCONN */ false,
  /* STATE_PAUSED  */ true,
};

static void relayState(byte relay, bool on);
static void relayTimer();
static bool checkConditions(byte port, byte newState, byte oldState);
static void runTransitions(byte port, byte newState, byte oldState);

void initState()
{
  byte port;
  memset(portStates, STATE_IDLE, sizeof(portStates));
  memset(chargerPaused, 0, sizeof(chargerPaused));
  memset(relayStates, 0, sizeof(relayStates));
  memset(conditionCount, 0, sizeof(conditionCount));
  memset(transitionCount, 0, sizeof(transitionCount));

  for (port = 0; port < N_PORTS; port++) {
    setOutput(port, TIMER_TOP);
    digitalWrite(relayPorts[port], LOW);
    pinMode(relayPorts[port], OUTPUT);
    relayState(port, false);
    // Set up ports to input, pull up.
#ifdef MONITOR_RELAYS
    relayMonitorPins[port].input(true);
#endif
  }
}

static bool switchActive = false;
static bool switchOn = false;
static byte switchPort = 0;
static byte switchRelay = 0;
static int switchStart = 0;

static void relayState(byte port, bool on)
{
  DEBUGF_CSTR_P("Charging on port %d: %s\n", port, on?"ON...":"OFF...");
  digitalWrite(relayPorts[port], on ? HIGH : LOW);
}

extern uint16_t adResult[N_PORTS][2];

#define PWM_DENOMINATOR_PWMLOW   60
#define PWM_OFFSET_PWMLOW        0
#define PWM_REDUCTION_PWMLOW     5    // Must be divisor in TIMER_TOP, offset and denominator (to reduce number of bits needed in calculation)

#define PWM_DENOMINATOR_PWMHIGH  250
#define PWM_OFFSET_PWMHIGH       160
#define PWM_REDUCTION_PWMHIGH    10   // Must be divisor in TIMER_TOP, offset and denominator (to reduce number of bits needed in calculation)

#define PWM_RESULT_WRAP(part) ((current*(TIMER_TOP/PWM_REDUCTION_##part) + PWM_OFFSET_##part*(TIMER_TOP/PWM_REDUCTION_##part) + PWM_DENOMINATOR_##part/PWM_REDUCTION_##part/2) / (PWM_DENOMINATOR_##part/PWM_REDUCTION_##part))
#define PWM_RESULT(part) PWM_RESULT_WRAP(part)

static unsigned int calcPWM(unsigned int current)
{
  unsigned int result;
  if (current < 50) {
    result = PWM_RESULT(PWMLOW);
  } else {
    result = PWM_RESULT(PWMHIGH);
  }
  // sprintf(str, "Current: %u PWM: %u\n", current, result);
  // Serial.println(str);
  return result;
}

#ifdef UNTETHERED
#define RESTRICT_CURRENT(p, c) cableCurrentRestriction(p, c)
#else
#define RESTRICT_CURRENT(p, c) (c)
#endif

void chargerState()
{
  byte port;
  stateDef nextState;
  for (port = 0; port < N_PORTS; port++) {
    byte& portState = portStates[port];
    memcpy_P(&nextState, &nextStates[portState][chargerPaused[port]?STATE_PAUSED:inputStates[port]], sizeof(stateDef));
    if (inputStateAges[port] >= nextState.minAge) {
      if (portState != nextState.nextState && checkConditions(port, nextState.nextState, portState)) {
        runTransitions(port, nextState.nextState, portState);
        portState = nextState.nextState;
        setOutput(port, pgm_read_byte(&indicatePower[portState])?calcPWM(RESTRICT_CURRENT(port, getCurrent(port))):TIMER_TOP);
        if (pgm_read_byte(&relayOn[portState]) != relayStates[port]) {
          relayState(port, relayStates[port] = pgm_read_byte(&relayOn[portState]));
        }
#ifdef HAS_UI
        updateChargerState();
#endif
        // sprintf(str, "Port %d: New state %d, relay: %s (level L %d, H %d)", port, portState, relayOn[portState] ? "ON" : "OFF", adResult[0][0], adResult[0][1]);
        // Serial.println(str);
      }
    }
#ifdef MONITOR_RELAYS
    relayMonitor[port] = relayMonitorPins[port].read();
#endif
  }
}

void updateCurrent()
{
  byte port;
  for (port = 0; port < N_PORTS; port++) {
    if (pgm_read_byte(&indicatePower[portStates[port]])) {
      setOutput(port, calcPWM(RESTRICT_CURRENT(port, getCurrent(port))));
    }
  }
}

// Transition management:
bool addCondition(byte state, PortCondition condition)
{
  if (conditionCount[state] >= N_HOOKS) {
    return false;
  }
  byte& count = conditionCount[state];
  conditions[state][count] = condition;
  count++;
  return true;
}

// Transition management:
bool addTransition(byte state, PortTransition transition)
{
  if (transitionCount[state] >= N_HOOKS) {
    return false;
  }
  byte& count = transitionCount[state];
  transitions[state][count] = transition;
  count++;
  return true;
}

bool checkConditions(byte port, byte newState, byte oldState)
{
  byte n;
  for (n=0; n<conditionCount[newState]; n++) {
    if (!conditions[newState][n](port, newState, oldState)) {
      return false;
    }
  }
  for (n=0; n<conditionCount[STATE_ANY]; n++) {
    if (!conditions[STATE_ANY][n](port, newState, oldState)) {
      return false;
    }
  }
  return true;
}

void runTransitions(byte port, byte newState, byte oldState)
{
  byte n;
  for (n=0; n<transitionCount[newState]; n++) {
    transitions[newState][n](port, newState, oldState);
  }
  for (n=0; n<transitionCount[STATE_ANY]; n++) {
    transitions[STATE_ANY][n](port, newState, oldState);
  }
}

