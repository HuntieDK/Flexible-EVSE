// MODBUS communication with electricity meters
#include "defs.h"
#include "HardwareSerial.h"
#include "HardwareSerial.cpp"

#ifdef HAS_METERING

#define MODBUS_READ_HOLDING_REGISTER  3
#define MODBUS_READ_INPUT_REGISTER    4

#define SDM630_MEASURES_START   1
#define SDM630_MEASURES_COUNT   9

#define SDM630_ENERGY_START     172
#define SDM630_ENERGY_COUNT     1

#define MODBUS_RESULT_PROTOCOL_HEADER 3   // Result has three bytes header
#define MODBUS_RESULT_PROTOCOL_SUM    2   // Result has two bytes CRC
#define MODBUS_RESULT_PROTOCOL_BYTES  (MODBUS_RESULT_PROTOCOL_HEADER+MODBUS_RESULT_PROTOCOL_SUM)   // Total protocol bytes
#define MODBUS_OFFSET_ADDRESS   0   // Request/reply address
#define MODBUS_OFFSET_FUNCTION  1   // Request/reply function code
#define MODBUS_OFFSET_LENGTH    2   // Reply length
#define MODBUS_OFFSET_DATA      3   // First data byte
#define MODBUS_REQUEST_SIZE     8   // Standard request size

#define METERING_IDLE               0
#define METERING_REQUEST_SENDING    1
#define METERING_REQUEST_RECEIVING  2
#define METERING_BAD_DATA           3

#define TIME_AWAIT_IDLE             20      // Delay between measurements
#define TIME_AWAIT_SENDING          100     // Max wait time for sending
#define TIME_AWAIT_INITIAL_RESPONSE 200     // Max wait time for first char in reply
#define TIME_AWAIT_NEXT_CHAR        50      // Max wait time for next char in reply

// static uint16_t registers[][2] = { { 0, 18 }, { 172, 2 } };
// #define N_REGISTERS (sizeof(registers)/sizeof(registers[0]));

static float  measurements[N_PORTS][SDM630_MEASURES_COUNT];
static byte   measurementsReady = 0;
static float  energy[N_PORTS][SDM630_ENERGY_COUNT];
static byte   energyReady = 0;

static byte   needStartReading = 0;
static byte   needStopReading = 0;

static const byte meterAddress[N_PORTS] = { 1, 2, 0, 0, 0, 0 };

static byte state;

static void transmitDone();

// We implement hardware serial 3 ourselves as we need access to interrupt routine:
class MeterSerial : public HardwareSerial
{
public:
  inline MeterSerial(volatile uint8_t *ubrrh, volatile uint8_t *ubrrl,
                     volatile uint8_t *ucsra, volatile uint8_t *ucsrb,
                     volatile uint8_t *ucsrc, volatile uint8_t *udr)
      : HardwareSerial(ubrrh, ubrrl, ucsra, ucsrb, ucsrc, udr)
  {
  }
  void _tx_udr_empty_irq(void)
  {
    if (_tx_buffer_head == _tx_buffer_tail) {
      // Buffer empty, so disable interrupts
      cbi(*_ucsrb, UDRIE0);
      sbi(*_ucsra, TXC0);
      sbi(*_ucsrb, TXCIE0);
      return;
    }

    // If interrupts are enabled, there must be more data in the output
    // buffer. Send the next byte
    unsigned char c = _tx_buffer[_tx_buffer_tail];
    _tx_buffer_tail = (_tx_buffer_tail + 1) % SERIAL_TX_BUFFER_SIZE;
  
    *_udr = c;
  
    // clear the TXC bit -- "can be cleared by writing a one to its bit
    // location". This makes sure flush() won't return until the bytes
    // actually got written
    sbi(*_ucsra, TXC0);
  }
  inline void _transmitDone(void)
  {
      cbi(*_ucsrb, TXCIE0);
  }
};

MeterSerial ModbusSerial(&UBRR3H, &UBRR3L, &UCSR3A, &UCSR3B, &UCSR3C, &UDR3);

static uint16_t needMeasurement = 0;   // Bit flag indicating needed energy measurement (TODO: Max 16 chargers here)
static byte     measuredPort = N_PORTS-1;
static bool     measuringEnergy = false;  //!< True when measuring energy, false when measuring values
static Timer*   timerHandle;
static bool     timeOut = false;
static byte     receiveBuffer[SDM630_MEASURES_COUNT*sizeof(float)+3+2]; // Header, 3 bytes, CRC 2 bytes, data, N floats.
static byte     receiveIndex = 0;
static byte     expectedSize;
static byte     requestBuffer[MODBUS_REQUEST_SIZE];

static void meterTimer();
static void dataReceived(byte count);
static void await(uint16_t time);
static bool meterCondition(byte port, byte newState, byte oldState);
static void meterTransition(byte port, byte newState, byte oldState);

void initMetering()
{
  // Init RS485 connection to modbus meters
  ModbusSerial.begin(9600, SERIAL_8N2);
  Controllino_RS485Init();
  addCondition(STATE_ANY, meterCondition);
  addTransition(STATE_ANY, meterTransition);
  timerHandle = addComplexTimer(TIMER_CS, meterTimer);
  await(TIME_AWAIT_IDLE);
}

static void await(uint16_t time)
{
  setComplexTimer(timerHandle, time, false);
  timeOut = false;
}

static void request(byte port, uint16_t startRegister, byte registerCount)
{
  startRegister = (startRegister-1) * 2;
  registerCount <<= 1;  // Each SDM630 register occupies two modbus registers
  expectedSize = registerCount<<1;  // Expected data size (2 bytes per modbus register = 4 bytes per SDM630 register)
  requestBuffer[0] = meterAddress[port];
  requestBuffer[1] = MODBUS_READ_INPUT_REGISTER;
  requestBuffer[2] = highByte(startRegister);
  requestBuffer[3] = lowByte(startRegister);
  requestBuffer[4] = highByte(registerCount);
  requestBuffer[5] = lowByte(registerCount);
  uint16_t crc = calcCRC16(requestBuffer, 6);
  requestBuffer[6] = lowByte(crc);
  requestBuffer[7] = highByte(crc);
  while (ModbusSerial.available()) { ModbusSerial.read(); } // Flush buffer
  Controllino_SwitchRS485DE(HIGH);
  Controllino_SwitchRS485RE(HIGH);
  // Serial.print("Writing request for port");
  // Serial.println(port);
  ModbusSerial.write(requestBuffer, sizeof(requestBuffer));
}

static void transmitDone()
{
  // Transmitting is done; turn of transmitter, start reading
  Controllino_SwitchRS485DE(LOW);
  Controllino_SwitchRS485RE(LOW);
  receiveIndex = 0;
  state = METERING_REQUEST_RECEIVING;
  await(TIME_AWAIT_INITIAL_RESPONSE);
}

static void requestMeasures(byte port)
{
  measuringEnergy = false;
  state = METERING_REQUEST_SENDING;
  request(port, SDM630_MEASURES_START, SDM630_MEASURES_COUNT);
  await(TIME_AWAIT_SENDING);
}

static void requestEnergy(byte port)
{
  measuringEnergy = true;
  state = METERING_REQUEST_SENDING;
  request(port, SDM630_ENERGY_START, SDM630_ENERGY_COUNT);
  await(TIME_AWAIT_SENDING);
}

static bool receiveData()
{
  bool avail = ModbusSerial.available();
  while (ModbusSerial.available()) {
    byte data = ModbusSerial.read();
    if (receiveIndex < sizeof(receiveBuffer)) {
      receiveBuffer[receiveIndex++] = data;
      if (receiveIndex>MODBUS_RESULT_PROTOCOL_BYTES && receiveIndex == receiveBuffer[MODBUS_OFFSET_LENGTH]+MODBUS_RESULT_PROTOCOL_BYTES) {
        // Data buffer reaches received length indication - check buffer.
        dataReceived(receiveIndex);
      }
    } else {
      ModbusSerial.read();
    }
  }
  return avail;
}
char *ftoa(char *a, double f, int precision)
{
 long p[] = {0,10,100,1000,10000,100000,1000000,10000000,100000000};
 
 char *ret = a;
 long heiltal = (long)f;
 itoa(heiltal, a, 10);
 while (*a != '\0') a++;
 *a++ = '.';
 long desimal = abs((long)((f - heiltal) * p[precision]));
 itoa(desimal, a, 10);
 return ret;
}

static void dataReceived(byte count)
{
  uint16_t crc = calcCRC16(receiveBuffer, count - MODBUS_RESULT_PROTOCOL_SUM);
  if (receiveBuffer[MODBUS_OFFSET_LENGTH] == expectedSize &&
      receiveBuffer[count-2] == lowByte(crc) &&
      receiveBuffer[count-1] == highByte(crc) &&
      requestBuffer[MODBUS_OFFSET_ADDRESS] == receiveBuffer[MODBUS_OFFSET_ADDRESS] &&
      requestBuffer[MODBUS_OFFSET_FUNCTION] == receiveBuffer[MODBUS_OFFSET_FUNCTION])
  {
    // CRC is correct, reply is for correct address and correct size. Copy to relevant buffer.
    /*
    Serial.print("Data: ");
    *str = 0;
    for (int i=0; i<count; i++) {
      sprintf(str+strlen(str), "%-2.2x ", receiveBuffer[i]);
    }
    Serial.println(str);
    */
  
    if (measuringEnergy) {
      memcpy(energy[measuredPort], receiveBuffer+MODBUS_OFFSET_DATA, expectedSize);    // TODO: Avoid this; write directly to float buffer
      energyReady |= _BV(measuredPort);

    } else {
      memcpy(measurements[measuredPort], receiveBuffer+MODBUS_OFFSET_DATA, expectedSize);    // TODO: Avoid this; write directly to float buffer
      measurementsReady |= _BV(measuredPort);

/*      ftoa(str, measurements[measuredPort][0], 100);
      sprintf(str, "%d: %d %d %d - %d %d %d - %d %d %d",
                measuredPort,
                (int)(measurements[measuredPort][0]*100),
                (int)measurements[measuredPort][1],
                (int)measurements[measuredPort][2],
                (int)measurements[measuredPort][3],
                (int)measurements[measuredPort][4],
                (int)measurements[measuredPort][5],
                (int)measurements[measuredPort][6],
                (int)measurements[measuredPort][7],
                (int)measurements[measuredPort][8]);
      Serial.print("Hmmm: ");
      Serial.println(str); */
    }
    
    state = METERING_IDLE;
    await(TIME_AWAIT_IDLE);
    return;
  }
  // Wrong data received - read until timeout, then start next
  state = METERING_BAD_DATA;
}

void stateMetering()
{
  static byte lastState = 255;
/*  
  if (state != lastState) {
    Serial.print("New state: ");
    Serial.println(lastState = state);
  }
*/
  byte lastPort = measuredPort;
  switch (state) {
  case METERING_IDLE:
    if (timeOut) {
      // Decide what to read next
      if (needMeasurement != 0) {
        for (measuredPort = 0; measuredPort < N_PORTS; measuredPort++) {
          if (needMeasurement & _BV(measuredPort)) {
            requestEnergy(measuredPort);
            break;
          }
        }
        if (measuredPort != N_PORTS) {
          break;
        }
        // Bug... Shouldn't get here. Revert:
        needMeasurement = 0;
        measuredPort = lastPort;
      }
      measuredPort = (measuredPort+1)%N_PORTS;
      while (meterAddress[measuredPort] == 0 && measuredPort != lastPort) {
        measuredPort = (measuredPort+1)%N_PORTS;
      }
      if (measuredPort != lastPort) {
        requestMeasures(measuredPort);
      }
    }
    break;
  case METERING_REQUEST_SENDING:
    if (timeOut) {
      // Didn't get through sending buffer... strange. What to do?
      // Clear buffer
      // Restart
    }
    break;
  case METERING_REQUEST_RECEIVING:
    if (receiveData()) {
      // Got data, give more time slack:
      await(TIME_AWAIT_NEXT_CHAR);
    } else if (timeOut) {
      // TODO: Mark failed
      state = METERING_IDLE;
    }
    break;
  case METERING_BAD_DATA:
    // Read bad data until timeout 
    if (ModbusSerial.available()) {
      ModbusSerial.read();
      await(TIME_AWAIT_NEXT_CHAR);
    } else if (timeOut) {
      // Restart metering
      state = METERING_IDLE;
    }
    break;
  }
}

static void meterTimer()
{
  timeOut = true;
}

static bool meterCondition(byte port, byte newState, byte oldState)
{
  switch (newState) {
  case STATE_CHARGE:
  case STATE_FAN:
    // Cannot go here until start has been read:
    return true;
  case STATE_IDLE:
    // Cannot go here until end has been read:
    return true;
  }
  return true;
}

static void meterTransition(byte port, byte newState, byte oldState)
{
  switch (newState) {
  case STATE_UNDEF:
    // No change here
    break;
    
  case STATE_IDLE:
  case STATE_DISCONN:
    // Need to unlock
    Serial.print("Do stop-reading of port "); Serial.println(port);
    needStopReading |= (1<<port);
    needStartReading &= ~(1<<port);
    break;

  case STATE_CONNECT:
  case STATE_WAIT:
  case STATE_CHARGE:
  case STATE_FAN:
  case STATE_PAUSED:
    // Start meter reading
    Serial.print("Do start-reading of port "); Serial.println(port);
    needStartReading |= (1<<port);
    needStopReading &= ~(1<<port);
    break;
  }
}

// Interrupt routines as seen in HardwareSerial3:
ISR(USART3_RX_vect)
{
  ModbusSerial._rx_complete_irq();
}

ISR(USART3_UDRE_vect)
{
  ModbusSerial._tx_udr_empty_irq();
}

ISR(USART3_TX_vect)
{
  ModbusSerial._transmitDone();
  transmitDone();
}

#endif // HAS_METERING

