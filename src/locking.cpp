#include "defs.h"

#ifdef HAS_LOCKING

static const AtmelPort lockReading[N_PORTS] = { APORT(K,4), APORT(K,5), APORT(K,6), APORT(K,7), APORT(D,7), APORT(G,2) };   // A12-16,I17-18 port blocks on Controllino Mega
static const AtmelPort actuatorOn[N_PORTS] = { APORT(A,0), APORT(A,1), APORT(A,2), APORT(A,3), APORT(A,4), APORT(A,5) };   // R0-5 starts actuators (PA0, PA1, PA2, PA3, PA4, PA5)
static const AtmelPort actuatorLock = APORT(A,7);
static const AtmelPort actuatorUnlock = APORT(A,6);

static bool lockCondition(byte port, byte newState, byte oldState);
static void lockTransition(byte port, byte newState, byte oldState);
static void lockTimer();

#define NO_LOCKING 0x80
#define AWAIT_IDLE 0x81

static byte actLocked = (1<<N_PORTS)-1;
static byte newLocked = 0;
static byte locking = NO_LOCKING;
static bool lockEngaging = false;
static Timer* timer = NULL;
static volatile bool timeOut = false;

void initLocking()
{
  addCondition(STATE_ANY, lockCondition);
  addTransition(STATE_ANY, lockTransition);
  timer = addComplexTimer(TIMER_CS, lockTimer);
  actuatorLock.output(false);
  actuatorUnlock.output(false);
  for (byte port = 0; port < N_PORTS; port++) {
    lockReading[port].input();
    actuatorOn[port].output(false);
  }
}

void stateLocking()
{
  if (locking == NO_LOCKING && actLocked != newLocked) {
    byte port;
    for (port = 0; port < N_PORTS; port++) {
      if ((actLocked ^ newLocked) & (1<<port)) {
        break;
      }
    }
    if (port < N_PORTS) {
      // Start locking port...
      locking = port;
      lockEngaging = (newLocked & (1<<port)) != 0;
      actuatorOn[locking].write(true);
      actuatorLock.write(lockEngaging);
      actuatorUnlock.write(!lockEngaging);
      setComplexTimer(timer, 60, false);
      timeOut = false;
      Serial.print("Enable lock motor: "); Serial.println(locking);
    } else {
      // Shouldn't get here: Fix bad state.
      actLocked = newLocked;
    }
  }
  if (timeOut) {
    if (locking != NO_LOCKING) {
      if (locking != AWAIT_IDLE) {
        Serial.print("Diable lock motor: "); Serial.println(locking);
        actuatorOn[locking].write(false);
        actuatorLock.write(false);
        actuatorUnlock.write(false);
        if (lockEngaging) {
          actLocked |= (1<<locking);
        } else {
          actLocked &= ~(1<<locking);
        }
        locking = AWAIT_IDLE;
        setComplexTimer(timer, 10, false);
      } else {
        locking = NO_LOCKING;
      }
    }
    timeOut = false;
  }
  
  {
    static int lastS = -1;
    if (lastS != sCount) {
      sprintf(str, "Lock status: %d/%d - %x/%x %d %d %d %d %d %d\n", 
        timer, locking,
        newLocked, actLocked,
        lockReading[0].read(),
        lockReading[1].read(),
        lockReading[2].read(),
        lockReading[3].read(),
        lockReading[4].read(),
        lockReading[5].read());
      Serial.println(str);
      lastS = sCount;
    }
  }
}

static bool lockCondition(byte port, byte newState, byte oldState)
{
  switch (newState) {
  case STATE_CHARGE:
  case STATE_FAN:
    // Cannot go here until locked:
    return (actLocked & (1<<port)) != 0;
  }
  return true;
}

static void lockTransition(byte port, byte newState, byte oldState)
{
  switch (newState) {
  case STATE_UNDEF:
    // No change here
    break;
    
  case STATE_IDLE:
  case STATE_DISCONN:
    // Need to unlock
    Serial.print("Unlock port "); Serial.println(port);
    newLocked &= ~(1<<port);
    break;

  case STATE_CONNECT:
  case STATE_WAIT:
  case STATE_CHARGE:
  case STATE_FAN:
  case STATE_PAUSED:
    // Need to lock
    Serial.print("Lock port "); Serial.println(port);
    newLocked |= (1<<port);
    break;
  }
}

static void lockTimer()
{
  timeOut = true;
}

#endif

