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
  initAD();
  initTimers();
  initState();
#ifdef HAS_UI
  initUI();
#endif
#ifdef HAS_METERING
  initMetering();
#endif
#ifdef HAS_LOCKING
  initLocking();
#endif
  // Serial.println("We are running!");
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
     // Serial.print("Tick ");
     // Serial.println(lastS);
     //sprintf(str, "Level L %d, H %d", adResult[0][0], adResult[0][1]);
     //Serial.println(str);

//     sprintf(str, "PORTL %x, PINL %x, DDRL %x", PORTL, PINL, DDRL);
//     Serial.println(str);

/*     if (relayMonitor[0])
       Serial.println("D18 = HIGH");
     else
       Serial.println("D18 = LOW"); */

//   sprintf(str, "TCCR3A: %x, TCCR3B: %x, TCCR3C: %x", TCCR3A, TCCR3B, TCCR3C);
//   Serial.println(str);
  }
  
  // delay(100);
  
  /* 
  setOutput(0, i*10);
  setOutput(1, logLevel[(i+4)%11]);
  setOutput(2, logLevel[(i+3)%11]);
  setOutput(3, logLevel[(i+2)%11]);
  setOutput(4, logLevel[(i+1)%11]);
  setOutput(5, logLevel[(i+0)%11]);

  int c3 = TCNT3;
  int c4 = TCNT4;

  Serial.print("Output 5 = ");
  Serial.println(i*10);

  Serial.print("TCNT3 = ");
  Serial.print(c3);
  Serial.print(", TCNT4 = ");
  Serial.println(c4);

  Serial.print("Sum = ");
  Serial.println(c4+c3);

  sprintf(str, "adStart = %d, adDone = %d", adStart, adDone);
  Serial.println(str);

  sprintf(str, "Values = %d/%d %d/%d %d/%d %d/%d %d/%d %d/%d", adResult[0][0], adResult[0][1], adResult[1][0], adResult[1][1], adResult[2][0], adResult[2][1], adResult[3][0], adResult[3][1], adResult[4][0], adResult[4][1], adResult[5][0], adResult[5][1]);
  Serial.println(str);

  sprintf(str, "Levels = %d/%d %d/%d %d/%d %d/%d %d/%d %d/%d", portLevels[0][0], portLevels[0][1], portLevels[1][0], portLevels[1][1], portLevels[2][0], portLevels[2][1], portLevels[3][0], portLevels[3][1], portLevels[4][0], portLevels[4][1], portLevels[5][0], portLevels[5][1]);
  Serial.println(str);

  sprintf(str, "States = %d %d %d %d %d %d", portStates[0], portStates[1], portStates[2], portStates[3], portStates[4], portStates[5]);
  Serial.println(str);

  i = (i+1) % 11;
 
  delay(500);              // wait for a second

  sprintf(str, "Values = %d/%d %d/%d %d/%d %d/%d %d/%d %d/%d", adResult[0][0], adResult[0][1], adResult[1][0], adResult[1][1], adResult[2][0], adResult[2][1], adResult[3][0], adResult[3][1], adResult[4][0], adResult[4][1], adResult[5][0], adResult[5][1]);
  Serial.println(str);

  sprintf(str, "Levels = %d/%d %d/%d %d/%d %d/%d %d/%d %d/%d", portLevels[0][0], portLevels[0][1], portLevels[1][0], portLevels[1][1], portLevels[2][0], portLevels[2][1], portLevels[3][0], portLevels[3][1], portLevels[4][0], portLevels[4][1], portLevels[5][0], portLevels[5][1]);
  Serial.println(str);

  sprintf(str, "States = %d %d %d %d %d %d", portStates[0], portStates[1], portStates[2], portStates[3], portStates[4], portStates[5]);
  Serial.println(str);

  sprintf(str, "Ages = %d %d %d %d %d %d", portStateAges[0], portStateAges[1], portStateAges[2], portStateAges[3], portStateAges[4], portStateAges[5]);
  Serial.println(str);

  delay(1500);
  */
}

