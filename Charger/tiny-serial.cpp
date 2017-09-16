#include <Arduino.h>
#include "tiny-serial.h"
#include "defs.h"
/* Simple interrupt-based serial library for ATtinyx4 for debugging purposes */

/* Supported combinations:
 *   F_CPU 1000000   BAUDRATE 1200, 2400 
 *   F_CPU 8000000   BAUDRATE 9600, 19200
 *   F_CPU 16000000  BAUDRATE 9600, 19200, 28800, 38400
 */

// Set your baud rate and number of stop bits here
#define BAUDRATE            2400
#define STOPBITS            1

// If bit width in cpu cycles is greater than 255 then divide by 8 to fit in timer
// Calculate prescaler setting
#define CYCLES_PER_BIT       ( (F_CPU) / (BAUDRATE) )
#if (CYCLES_PER_BIT >= 256)
  #if (CYCLES_PER_BIT >= 2048)
    #define DIVISOR             64
    #define CLOCKSELECT         3
  #else
    #define DIVISOR             8
    #define CLOCKSELECT         2
  #endif
#else
  #define DIVISOR             1
  #define CLOCKSELECT         1
#endif
#define FULL_BIT_TICKS      ( (CYCLES_PER_BIT) / (DIVISOR) )

// Send state variable and accessors
static volatile byte state = 0;
#define STATE_IDLE      0
#define STATE_STARTBIT  1
#define STATE_DATABIT   2
#define STATE_STOPBIT   3

// Transmit data persistent between USI OVF interrupts
static volatile uint8_t serial_tx_data;
static volatile uint8_t bitcount;

void usiserial_send_byte(uint8_t data)
{
  while (state);
  serial_tx_data = data;
  state = STATE_STARTBIT;
}

// USI overflow interrupt indicates we have sent a buffer

#ifndef TIMER0_COMPA_vect
#error "Invalid vector TIMER0_COMPA_vect"
#endif

ISR(TIMER0_COMPA_vect)  // Every time we pass top send next bit
{
  switch (state) {
  case STATE_STARTBIT:
    PORTA = PORTA & ~(1<<PA5);
    bitcount = 0;
    state = STATE_DATABIT;
    break;
  case STATE_DATABIT:
    PORTA = PORTA & (~(1<<PA5)) | ((serial_tx_data&1)<<PA5);
    serial_tx_data >>=1;
    if (++bitcount == 8) {
      state = STATE_STOPBIT;
      bitcount = 0;
    }
    break;
  case STATE_STOPBIT:
    PORTA = PORTA | (1<<PA5);
    if (++bitcount == STOPBITS) {
      state = STATE_IDLE;
    }
    break;
  case STATE_IDLE:
    break;
  }
}

void serialSetup() {
  pinMode(5,OUTPUT);                // Configure USI_DO as output.
  digitalWrite(5,HIGH);           // Ensure serial output is high when idle

  // Configure Timer0
  TCCR0A = 2<<WGM00;                      // CTC mode
  TCCR0B = CLOCKSELECT;                   // Set prescaler to clk or clk /8
  GTCCR |= _BV(PSR10);                    // Reset prescaler
  OCR0A = FULL_BIT_TICKS;                 // Reset every full bit width
  TCNT0 = 0;                              // Count up from 0 
  TIMSK0 |= 1<<OCIE0A;                  // Interrupt on match for our simple timer
}

void serialWrite(const char* str)
{
  for (; *str; ++str) {
    while (state);
    usiserial_send_byte(*str);
  }
}

void serialWrite(const char ch)
{
  while (state);
  usiserial_send_byte(ch);
}


