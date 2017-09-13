#include "defs.h"

// Blå -- kontakt -- Rød -- spole -- Sort

byte portStates[N_PORTS];
bool chargerPaused[N_PORTS];

#ifdef SINGLE_CHARGER

static const byte relayPorts[N_PORTS][N_LATCH_STATES][N_PORT_RELAYS] /*PROGMEM*/ = { 9, 8, 12, 13 };

#else

static const byte relayPorts[N_PORTS][N_PORT_RELAYS][N_LATCH_STATES] /*PROGMEM*/ = { 4, 9, 10, 11, 12, 13 };    // Controllino D2,7,8,9,10,11 (order due to timer connections)

#ifdef MONITOR_RELAYS
static const AtmelPort relayMonitorPins[N_PORTS] /*PROGMEM*/ = { APORT(L,1), APORT(L,0), APORT(D,4), APORT(D,5), APORT(D,6), APORT(J,4) };   // D18-23 port blocks on Controllino Mega (PL1, PL0, PD4, PD5, PD6, PJ4)
bool relayMonitor[N_PORTS] = { false, false, false, false, false, false };
#endif

#endif

#define N_HOOKS 5

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

static const stateDef nextStates[N_STATES][N_STATES] /*PROGMEM*/ = {
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


static const bool indicatePower[N_STATES] /*PROGMEM*/ = {
  /* STATE_UNDEF   */ false,
  /* STATE_IDLE    */ false,
  /* STATE_CONNECT */ false,
  /* STATE_WAIT    */ true,
  /* STATE_CHARGE  */ true,
  /* STATE_FAN     */ true,
  /* STATE_DISCONN */ false,
  /* STATE_PAUSED  */ false,
};

static const bool relayOn[N_STATES] /*PROGMEM*/ = {
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
    for (byte latch = 0; latch < N_LATCH_STATES; latch++) {
      for (byte relay = 0; relay < N_PORT_RELAYS; relay++) {
        digitalWrite(relayPorts[port][latch][relay], LOW);
        pinMode(relayPorts[port][latch][relay], OUTPUT);
      }
    }
    relayState(port, false);
    // Set up ports to input, pull up.
#ifdef MONITOR_RELAYS
    relayMonitorPins[port].input(true);
#endif
  }
#if N_LATCH_STATES > 1
  addSimpleTimer(TIMER_DS, &relayTimer);  // For latching relays, we need timer callbacks every 0.1s to turn coils on/off
#endif
}

static bool switchActive = false;
static bool switchOn = false;
static byte switchPort = 0;
static byte switchRelay = 0;
static int switchStart = 0;

static void relayTimer()
{
  // Handle latching relays - called every 0.1 s
  if (!switchActive) return;
  if (dsCount-switchStart >= RELAY_TIME + (switchRelay==0 ? 1 : 0)) {
    // Turn off this output; maybe turn on next.
    digitalWrite(relayPorts[switchPort][switchOn][switchRelay], LOW);
    if (++switchRelay == N_PORT_RELAYS) {
      switchActive = false; // All done
    } else {
      digitalWrite(relayPorts[switchPort][switchOn][switchRelay], HIGH);  // Start flipping next relay
      switchStart = dsCount;  // Keep timing from here
    }
  }
}

static void relayState(byte port, bool on)
{
  Serial.println(on?"ON...":"OFF...");
#if N_LATCH_STATES == 1
  // Monostable relay
  for (byte relay = 0; relay < N_PORT_RELAYS; relay++) {
    digitalWrite(relayPorts[port][0][relay], on ? HIGH : LOW);
  }
#else
  // Bistable relay: Turn off all coils but relevant on/off coil on first relay:
  for (byte latch = 0; latch < N_LATCH_STATES; latch++) {
    digitalWrite(relayPorts[port][latch][0], (latch==(on?1:0))?HIGH:LOW);  // Set first relevant relay active, rest inactive
    for (byte relay = 1; relay < N_PORT_RELAYS; relay++) {
      digitalWrite(relayPorts[port][latch][relay], LOW);
    }
  }
  cli();
  // Set up info for timer function:
  switchPort = port;
  switchRelay = 0;
  switchStart = dsCount;
  switchOn = on;
  switchActive = true;
  sei();
#endif
}

extern uint16_t adResult[N_PORTS][2];

#define PWM_DENOMINATOR_PWMLOW   60
#define PWM_OFFSET_PWMLOW        0
#define PWM_REDUCTION_PWMLOW     5    // Must be divisor in TIMER_TOP, offset and denominator (to reduce number of bits needed in calculation)

#define PWM_DENOMINATOR_PWMHIGH  250
#define PWM_OFFSET_PWMHIGH       160
#define PWM_REDUCTION_PWMHIGH    10   // Must be divisor in TIMER_TOP, offset and denominator (to reduce number of bits needed in calculation)

// #define PWM_RESULT_WRAP(part) (((current + PWM_OFFSET_##part)*TIMER_TOP + PWM_DENOMINATOR_##part/2) / PWM_DENOMINATOR_##part)

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
  sprintf(str, "Current: %u PWM: %u\n", current, result);
  Serial.println(str);
  return result;
}

void chargerState()
{
  byte port;
  for (port = 0; port < N_PORTS; port++) {
    byte& portState = portStates[port];
    const stateDef& nextState = nextStates[portState][chargerPaused[port]?STATE_PAUSED:inputStates[port]];
    if (inputStateAges[port] >= nextState.minAge) {
      if (portState != nextState.nextState && checkConditions(port, nextState.nextState, portState)) {
        runTransitions(port, nextState.nextState, portState);
        portState = nextState.nextState;
        setOutput(port, indicatePower[portState]?calcPWM(getCurrent(port)):TIMER_TOP);
        if (relayOn[portState] != relayStates[port]) {
          relayState(port, relayStates[port] = relayOn[portState]);
        }
#ifdef HAS_UI
        updateChargerState();
#endif
        sprintf(str, "Port %d: New state %d, relay: %s (level L %d, H %d)", port, portState, relayOn[portState] ? "ON" : "OFF", adResult[0][0], adResult[0][1]);
        Serial.println(str);
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
    if (indicatePower[portStates[port]]) {
      setOutput(port, calcPWM(getCurrent(port)));
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

