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

  DEBUG_CSTR_P("Init State...\n");
  initState();

  DEBUG_CSTR_P("Init AD...\n");
  initAD();

  DEBUG_CSTR_P("Init Timers...\n");
  initTimers();

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
#ifdef UNTETHERED
extern byte cableStates[];
#endif
extern unsigned int portStateAges[];

static unsigned int lastS = 0;

extern uint16_t cableLevel;

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
    DEBUGF_CSTR_P("Measured state %d/%d\n", adResult[0][0], adResult[0][1])
#ifdef UNTETHERED
    DEBUGF_CSTR_P("Cable level %d\n", cableStates[0])
#endif
  }
}
