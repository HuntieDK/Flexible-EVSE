/*
 * Initial test for charger code
 */

#include "defs.h"

static int i = 0;

extern uint16_t adResult[][2];

// the setup function runs once when you press reset or power the board
void setup()
{
  
  initDebug();

  DEBUG_CSTR_P("           \nInitializing...\n");

  DEBUG_CSTR_P("Init AD...\n");
  initAD();

  DEBUG_CSTR_P("Init Timers...\n");
  initTimers();

  DEBUG_CSTR_P("Init State...\n");
  initState();

#ifdef HAS_UI
  DEBUG_CSTR_P("Init UI...\n");
  initUI();
#endif
#ifdef HAS_METERING
  initMetering();
#endif
#ifdef HAS_LOCKING
  initLocking();
#endif

  DEBUG_CSTR_P("Mini EVSE is running.\n");
}

// the loop function runs over and over again forever

byte portCount = ACTUAL_PORTS;

extern int adStart, adDone;
extern byte portLevels[][2];
extern byte portStates[];
extern unsigned int portStateAges[];

static unsigned int lastS = 0;

void loop()
{
  chargerState();
#ifdef HAS_UI
  uiState();
#endif
#ifdef HAS_METERING
  stateMetering();
#endif
#ifdef HAS_LOCKING
  stateLocking();
#endif
  if (lastS != sCount) {
    lastS = sCount;
    DEBUGF_CSTR_P("Tick %d\n", lastS);
  }
}

