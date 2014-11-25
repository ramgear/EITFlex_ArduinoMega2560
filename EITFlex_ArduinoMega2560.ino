#include <NilRTOS.h>
#include <NilSerial.h>
#include <EEPROM.h>
#include <stdarg.h>

#define _INLINE __inline

// Serial communications
//#define SERIAL_BT_DEV
#define BTSerial Serial2
#define SysSerial Serial

#define BT_MODE_PIN 15          // Enable programming mode for bluetooth device
#define DEFUALT_BT_SERIAL_SPEED  38400  // defualt = 38400 for HC-05
#define DEFUALT_SERIAL_SPEED  115200

// Injector declaration
#define INJ_COUNT 4
#define INJ_ON_STAT LOW

// MAP declaration
#define MAP_MAX 100
#define MAP_MIN 0
#define MAP_COUNT_MAX 20
#define MAP_COUNT MAP_COUNT_MAX
#define MAP_START MAP_MIN
#define MAP_STEP 5
#define MAP_ADJ_STEP 20
#define MAP_SENSOR_PIN A0

// RPM declaration
#define RPM_MAX 7500
#define RPM_MIN 600
#define RPM_COUNT_MAX 25
#define RPM_COUNT RPM_COUNT_MAX
#define RPM_START 800
#define RPM_STEP 100
#define RPM_ADJ_STEP 100

// User define
#define DEFAULT_FEATURE_EN B00000001;
#define DEFAULT_USER_ADJ  20
#define DEFAULT_RPM_ACC_ADJ  3
#define DEFAULT_MAP_ACC_ADJ  3
#define FUEL_ADJ_MAX_DUTY  80
#define FUEL_ADJ_MIN  0
#define FUEL_ADJ_MAX  35
#define REV_PER_PULSE  2

// Timing define
#define THRD_SLEEP_MIN_US 10
#define INJ_OFF_CHK_TIME_US 100
#define TIMER_CHK_PERIOD_US 100000
#define FUEL_ADJ_TIME_MS 500
#define SYS_MON_PERIOD_MS 500
#define WARMUP_TIME_S 30
#define WARMUP_RPM_LIMIT 1800

#define CMD_HEADER_SIZE 4
#define CMD_BUF_SIZE 32

#define MAX_CONFIG_COUNT 4

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#define CRITICAL_TIME_LIMIT FALSE

enum Injs
{
  INJ_1 = 0,
  INJ_2 = 1,
  INJ_3 = 2,
  INJ_4 = 3,
};

// Support command declaration
enum CMDs
{
  CMD_START = 0,
  CMD_PRINT_INFO,           // Print system monitoring information
  CMD_READ_SYSMON,            // Read system monitoring information
  CMD_READ_CONFIGS,         // Read feature enable flags
  CMD_WRITE_CONFIGS,        // Write feature enable flags
  CMD_READ_MAPPING,         // Read fuel mapping in %
  CMD_WRITE_MAPPING,        // Write fuel mapping in %
  CMD_WRITE_BT_CFG,         // Write bluetooth config
  CMD_MONITOR_INFO,
  CMD_INJECTOR_INFO,
  CMD_END,
};

// Error code declaration
enum ErrCodes
{
  ERR_NONE = 0,                  // No error
  ERR_CMD_NOT_FOUND,             // Command not found error
  ERR_CHK_SUM,                   // Checksum error
  ERR_CMD_INDEX_OUT_OF_BOUNDS,   // MAP/RPM mapping index out of bounds error
};

enum Prescalars
{
  PRESCAL_2 = 1,
  PRESCAL_4,
  PRESCAL_8,
  PRESCAL_16,
  PRESCAL_32,
  PRESCAL_64,
  PRESCAL_128,
};

enum Frequencies
{
  FREQ_8MHz = 1,
  FREQ_4MHz,
  FREQ_2MHz,
  FREQ_1MHz,
  FREQ_500kHz,
  FREQ_250kHz,
  FREQ_125kHz,
};

enum Configs
{
  CFG_SYSTEM,
  CFG_E20,
  CFG_E85,
  CFG_CUSTOM,
};

typedef struct
{
    uint8_t en;
    uint8_t map;
    uint8_t rpm_acc;
    uint8_t map_acc;
    uint8_t mon;
    uint8_t inj_mon;
} Feature_t;

typedef struct
{
  // User config
  Feature_t feature;
  uint8_t rpmAccAdj;
  uint8_t mapAccAdj;
  uint8_t userAdj;
  uint8_t maxAdj;
  
  // engine config
  uint8_t warmup_time_s;
  uint16_t warmup_rpm;
  uint8_t rev_per_pulse;
  
  // mapping config
  uint8_t rpm_count;
  uint16_t rpm_start;
  uint16_t rpm_end;
  uint16_t rpm_step;
  uint16_t rpm_adj_step;
  
  uint8_t map_count;
  uint8_t map_start;
  uint8_t map_end;
  uint8_t map_step;
  uint8_t map_adj_step;
} Config_t;

typedef struct
{
  uint8_t current;
  Config_t configs[MAX_CONFIG_COUNT];
  uint8_t fuel_maps[MAX_CONFIG_COUNT][MAP_COUNT_MAX][RPM_COUNT_MAX];
} EEPROM_Data_t;

typedef struct
{
  uint8_t InjectorNo;
  uint8_t Stat;
  uint64_t StartTime;
  uint32_t Period;
  uint16_t CurrentRPM;
  uint8_t CurrentDuty;
  uint8_t CurrentRPMAcc;
  uint32_t CriticalTimeUS;
} InjectorInfo_t;

// System monitoring variables
typedef struct
{
  // declare system changeable fist
  uint32_t AdjustTimeMs;
  uint8_t FuelAdjust;
  uint8_t CurrentMAP;
  uint8_t CurrentMAPAcc;
  uint8_t EngineStarted;
  uint8_t EngineWarming;
  uint64_t EngineStartTime;
} SysMon_t;

typedef void (*NoArgFuncPtr)();

// Function prototype
// -------------------------
void internalCommand(CMDs cmd,const void *data, const uint8_t len);
void enableTimerS(uint8_t timeout, NoArgFuncPtr callback);
   
// End of Function prototype
// -------------------------

// data type for fifo item
typedef struct
{
  uint8_t  inj_no;
  uint32_t period;
  uint64_t start_time;
} PWMItem_t;

// #########################################################################
// -------------------------------------------------------------------------
// Semaphore declaration
// -------------------------------------------------------------------------
// #########################################################################
SEMAPHORE_DECL(semPWM, 0);
SEMAPHORE_DECL(semFuelAdjust, 0);
SEMAPHORE_DECL(semSendData, 0);

EEPROM_Data_t gEEPROM;
Config_t *gCurrentConfig = NULL;
uint8_t *gCurrentMapping = NULL;

// #########################################################################
// -------------------------------------------------------------------------
// System functions
// -------------------------------------------------------------------------
// #########################################################################

/*
Analog/Digital Convertion sampling rate
16 MHz / 2 = 8 MHz
16 MHz / 4 = 4 MHz
16 MHz / 8 = 2 MHz
16 MHz / 16 = 1 MHz
16 MHz / 32 = 500 kHz
16 MHz / 64 = 250 kHz
16 MHz / 128 = 125 kHz
*/
void setAdcPrescal(uint8_t scal)
{
  ADCSRA &= B11111000;  // Clear prescale values
  ADCSRA |= scal;
}

void setAdcFreq(uint8_t freq)
{  
  switch((Frequencies)freq)
  {
    case FREQ_8MHz:
      setAdcPrescal(PRESCAL_2);
    break;
    case FREQ_4MHz:
      setAdcPrescal(PRESCAL_4);
    break;
    case FREQ_2MHz:
      setAdcPrescal(PRESCAL_8);
    break;
    case FREQ_1MHz:
      setAdcPrescal(PRESCAL_16);
    break;
    case FREQ_500kHz:
      setAdcPrescal(PRESCAL_32);
    break;
    case FREQ_250kHz:
      setAdcPrescal(PRESCAL_64);
    break;
    case FREQ_125kHz:
      setAdcPrescal(PRESCAL_128);
    break;
  }
}

bool setActiveConfig(uint8_t index)
{
  if(index >= MAX_CONFIG_COUNT)
    return false;
    
  gCurrentConfig = &gEEPROM.configs[index];
  gCurrentMapping = &gEEPROM.fuel_maps[index][0][0];
  
  return true;
}

// #########################################################################
// -------------------------------------------------------------------------
// Queue class
// -------------------------------------------------------------------------
// #########################################################################
#define QUEUE_SIZE  10
#ifndef BOOL
#define BOOL boolean
#endif
#ifndef NULL
#define NULL (void *)0
#endif

static const uint8_t led = 13;
static volatile uint8_t led_stat = LOW;

class Queue
{
private:
  size_t _count;
  uint8_t  _head;
  uint8_t  _tail;
  void *_items;
  size_t _size;
  
public:
  Queue(void *items, size_t item_size)
  {
    _count = 0;
    _head = 0;
    _tail = 0;
    _items = items;
    _size = item_size;
  }
  
  _INLINE BOOL Push(const void *item)
  {    
    static char *ptr;
    
    if(!IsFull())
    {
      ptr = (char *)_items;
      ptr += (_tail * _size);
      memcpy(ptr, item, _size);
      
      // increase tail index
      _tail = (_tail + 1 == QUEUE_SIZE) ? 0 : _tail + 1;
      
      // increase item count
      ++_count;
      
      return TRUE;
    }
    
    return FALSE;
  }
  
  const _INLINE void *Pop()
  {
    static char *ptr;
    
    // check empty queue
    if(!IsEmpty())
    {
      ptr = (char *)_items;
      ptr += _head * _size;
      
      // increase head index
      _head = (_head + 1 == QUEUE_SIZE) ? 0 : _head + 1;
      
      // decrease item count
      --_count;
    }
    
    return ptr;
  }
  
  _INLINE size_t Count()
  {
    return _count;
  }
  
  _INLINE BOOL IsEmpty()
  {
    return !_count;
  }
  
  _INLINE BOOL IsFull()
  {
    return (_count + 1 >= QUEUE_SIZE);
  }
};

// End of Queue class
// -------------------------------------------------------------------------
// #########################################################################


// #########################################################################
// -------------------------------------------------------------------------
// Serial Bluetooth class
// -------------------------------------------------------------------------
// #########################################################################
#define SERIAL_BUF_SIZE CMD_BUF_SIZE

uint8_t qWData[SERIAL_BUF_SIZE * QUEUE_SIZE];
Queue queue = Queue(qWData, SERIAL_BUF_SIZE);

class SerialBT
{

  uint8_t modePin;
  char btRBuf[SERIAL_BUF_SIZE];
  char btWBuf[SERIAL_BUF_SIZE];
  
  HardwareSerial *ptrSerial;
  HardwareSerial *ptrBTSerial;
  
  bool connected;
  bool mode;
  Queue *ptrQ;

private:
  
  _INLINE void queueWrite(const uint8_t *ptr, uint8_t len)
  {
    if(!ptrQ->IsFull())
    {
      ptrQ->Push(ptr);
    }
  }
  
  // Test bluetooth command
  bool testCmd()
  {
     String str;
     bool ret = false;
     
     setAtMode(true);
     
     // send AT command to bluetooth
     ptrBTSerial->println("AT");
     
     // read response from bluetooth
     if(readString(ptrBTSerial, str, 3) && str == "OK")
       ret = true;
       
     setAtMode(false);
     
     // send response to system when no response from bluetooth
     if(!ret)
       ptrSerial->println("No response from bluetooth device!");
     
     return ret;
  }
  
  // Set bluetooth mode
  void setAtMode(bool atMode)
  {
    mode = atMode;
    digitalWrite(modePin, atMode ? HIGH : LOW);
  }
  
public:
  SerialBT(HardwareSerial *srSys, HardwareSerial *srBT, const uint8_t mode_pin)
  {
    ptrSerial = srSys;
    ptrBTSerial = srBT;
    modePin = mode_pin;
    connected = false;
    ptrQ = &queue;
    
    pinMode(modePin, OUTPUT);
  }
  
  void sendData()
  {
    if(!ptrQ->IsEmpty())
    {
      uint8_t *ptr = (uint8_t *)ptrQ->Pop();
      
      if(connected)
        ptrBTSerial->write(ptr, SERIAL_BUF_SIZE);
      else
        ptrSerial->write(ptr, SERIAL_BUF_SIZE);
    }
  }
  
  // Set bluetooth speed for data transfer mode
  void setSpeed(uint32_t speed)
  {
    ptrSerial->begin(speed);
    ptrBTSerial->begin(speed);
  }
  
  // Change bluetooth to command mode
  void atModeOn()
  {
    println("Begin bluetooth command mode.");
    printf("Changing baudrate to %ld\r\n", DEFUALT_BT_SERIAL_SPEED);
    
    ptrSerial->end();
    ptrSerial->begin(DEFUALT_BT_SERIAL_SPEED);
    
    setAtMode(true);
  }
  
  // Change bluetooth to data transfer mode
  void atModeOff()
  {
    println("End bluetooth command mode.");
    printf("Changing baudrate to %ld\r\n", DEFUALT_SERIAL_SPEED);
    
    ptrSerial->end();
    ptrSerial->begin(DEFUALT_SERIAL_SPEED);
    
    setAtMode(false);
  }
  
  bool connect()
  {
    //connected = testCmd();
    
    return connected;
  }
  
  // Send command to bluetooth
  void sendAtCmd(String cmd)
  {
     if(mode)
       println(cmd);
  }
  
  // Print text with format via bluetooth/serial
  _INLINE void printf(const char *format, ...)
  {
    char buf[SERIAL_BUF_SIZE];
    memset(buf, 0, SERIAL_BUF_SIZE);
    
     va_list ap;
     va_start(ap, format);
     vsnprintf(buf, sizeof(buf), format, ap);
     queueWrite((const uint8_t *)buf, SERIAL_BUF_SIZE);
     va_end(ap);
  }
  
  // Print text with newline via bluetooth/serial
  _INLINE void println(String text)
  {
    char buf[SERIAL_BUF_SIZE];
    memset(buf, 0, SERIAL_BUF_SIZE);
    
    text.toCharArray(buf, SERIAL_BUF_SIZE);
    printf("%s\r\n", buf);
  }
  
  // Write raw data via bluetooth/serial
  _INLINE void write(const uint8_t *buf, const uint8_t len)
  {
    queueWrite(buf, len);
  }
  
  // Get available data from bluetooth
  _INLINE bool available()
  {
     return ptrBTSerial->available();
  }
  
  // Read data from bluetooth
  _INLINE char read()
  {
     return ptrBTSerial->read();
  }
  
  // Read string from bluetooth
  _INLINE bool readString(String &str, uint8_t retry = 1)
  {
    readString(ptrBTSerial, str, retry);
  }
  
  // Read string from specify serial
  _INLINE bool readString(HardwareSerial *serial, String &str, uint8_t retry = 1)
  {
    uint8_t idx = 0;
    memset(btRBuf, 0, sizeof(btRBuf));
    
    do
    {
      while(serial->available())
      {
        btRBuf[idx++] = serial->read();
      }
      
      if(idx == 0)
        delay(1);
    }while(--retry && idx);
    
    str = btRBuf;
    
    return (idx != 0);
  }
  
  
};

// declare bluetooth object
SerialBT gBT = SerialBT(&SysSerial, &BTSerial, BT_MODE_PIN);

// End of Serial Blutooth class
// -------------------------------------------------------------------------
// #########################################################################

// array of data items
PWMItem_t PWMItems[QUEUE_SIZE];
volatile uint64_t InjOffItems[QUEUE_SIZE];
Queue PWMQueue = Queue(PWMItems, sizeof(PWMItem_t));

volatile SysMon_t gSysMon;
volatile InjectorInfo_t gInjInfo[INJ_COUNT];

NoArgFuncPtr gTimerSCallback = NULL;
uint64_t gTimerSCallCount = 0;
uint8_t gTimerSTimeout = 0;

// #########################################################################
// -------------------------------------------------------------------------
// InjectorHandler class
// -------------------------------------------------------------------------
// #########################################################################

class InjectorHandler
{
#define TIMER3_CALLBACK_MAX 5

public:    
  static const uint8_t m_OutPins[INJ_COUNT];
  static const NoArgFuncPtr m_Callbacks[INJ_COUNT];
  static const uint8_t m_IntPins[INJ_COUNT];
  static const uint8_t m_InPins[INJ_COUNT];
  volatile uint32_t m_Timer3Counter;
  volatile NoArgFuncPtr m_Timer3CallbacksFunc[TIMER3_CALLBACK_MAX];
  volatile uint32_t m_Timer3CallbacksMask[TIMER3_CALLBACK_MAX];
  volatile size_t m_t3Count;
  
  volatile uint8_t m_RPMCheckNo;
  
public:
  InjectorHandler()
  {
  }
  
  void Init()
  {
    m_Timer3Counter = 0;
    
    m_t3Count = 0;
    m_Timer3CallbacksFunc[m_t3Count] = OnFuelAdjust;
    m_Timer3CallbacksMask[m_t3Count] = ((uint64_t)FUEL_ADJ_TIME_MS * 1000) / TIMER_CHK_PERIOD_US;
    ++m_t3Count;
    
    m_Timer3CallbacksFunc[m_t3Count] = OnSysMon;
    m_Timer3CallbacksMask[m_t3Count] = ((uint64_t)SYS_MON_PERIOD_MS * 1000) / TIMER_CHK_PERIOD_US;
    ++m_t3Count;
    
    m_Timer3CallbacksFunc[m_t3Count] = OnBlink;
    m_Timer3CallbacksMask[m_t3Count] = ((uint64_t)500000) / TIMER_CHK_PERIOD_US;
    ++m_t3Count;
    
    gSysMon.AdjustTimeMs = FUEL_ADJ_TIME_MS;
    
    // Startup
    gSysMon.EngineStarted = 0;
    gSysMon.EngineWarming = 0;
    
    // test led
    pinMode(led, OUTPUT);
    
    for(uint8_t i = 0; i < INJ_COUNT; ++i)
    {
      // initial input/output
      pinMode(m_InPins[i], INPUT);  // use external pullup
      pinMode(m_OutPins[i], OUTPUT);
      
      // initial interrupt
      attachInterrupt(m_IntPins[i], m_Callbacks[i], CHANGE);
    }
  }
  
  static void OnBlink()
  {
    ToggleLed();
  }
  
  // Timer3 callback functions
  // ----------------------------------
  static void OnFuelAdjust()
  {
    // Adjust fuel only on engine started
    if(gSysMon.EngineStarted)
    {
      nilSemSignal(&semFuelAdjust);
    }
  }
  
  static void OnSysMon()
  {
    if(gCurrentConfig->feature.mon)
    {
      internalCommand(CMD_MONITOR_INFO, (const void *)&gSysMon, sizeof(SysMon_t));
    }
    if(gCurrentConfig->feature.inj_mon)
    {
      internalCommand(CMD_INJECTOR_INFO, (const void *)&gInjInfo[0], sizeof(InjectorInfo_t));
    }
  }
  
  static void OnWarmingTimeout()
  {
    // disable and clear timer counter
    disableTimerS();
      
    gSysMon.EngineWarming = 0;
  }
  // ----------------------------------
  
  _INLINE void WaitUntil(const uint8_t inj_no, const uint64_t &timenow, const uint64_t &time_off)
  {
    if(gCurrentConfig->feature.en && (timenow + THRD_SLEEP_MIN_US < time_off))
    {
      InjOffItems[inj_no] = time_off;
      return;
    }
    
    // No wait time detected then force off
    InjOff(inj_no);
  }
  
  _INLINE void OnStatChanged(uint8_t inj_index)
  {
    static volatile uint64_t timenow;
    static PWMItem_t pwm;
    
    volatile InjectorInfo_t *ptrInj = &gInjInfo[inj_index];
    
    timenow = micros();
    ptrInj->Stat = digitalRead(m_InPins[inj_index]);
    
    if(ptrInj->Stat == INJ_ON_STAT)
    {
      InjOn(inj_index);
      
      // RPM & duty cycle calculation
      if(ptrInj->StartTime != 0 && timenow != ptrInj->StartTime)
      {    
        // Engine startup detected on 2nd cycle
        if(!gSysMon.EngineStarted)
        {
          gSysMon.EngineStartTime = timenow;
          gSysMon.EngineStarted = 1;
          gSysMon.EngineWarming = 1;
          enableTimerS(gCurrentConfig->warmup_time_s, OnWarmingTimeout);
        }
    
        ptrInj->CurrentRPM = 60000000L * gCurrentConfig->rev_per_pulse / (timenow - ptrInj->StartTime);
        ptrInj->CurrentDuty = (ptrInj->Period * 100 / (timenow - ptrInj->StartTime)) + 1;
        
        // calculate critical time
        if(ptrInj->CurrentRPM != 0)
          ptrInj->CriticalTimeUS = 600000L * FUEL_ADJ_MAX_DUTY / ptrInj->CurrentRPM;
      }
      
      ptrInj->StartTime = timenow;
    }
    else
    {
      pwm.inj_no = inj_index;
      pwm.period = (timenow - ptrInj->StartTime);
      pwm.start_time = ptrInj->StartTime;
      
      ptrInj->Period = pwm.period;
      
      if(PWMQueue.Push((const void *)&pwm))
        nilSemSignal(&semPWM);
    }
  }
  
  _INLINE void InjOn(uint8_t inj_index)
  {
    digitalWrite(m_OutPins[inj_index], HIGH);
  }
  
  _INLINE void InjOff(uint8_t inj_index)
  {
    digitalWrite(m_OutPins[inj_index], LOW);
  }
  
  _INLINE void OnTimer1Tick()
  {
    static uint8_t inj_no;
    static uint64_t timenow;

    timenow = micros();
    
    for(inj_no = 0; inj_no < INJ_COUNT; ++inj_no)
    {
      if(InjOffItems[inj_no] != 0 && timenow >= InjOffItems[inj_no])
      {
        InjOff(inj_no);
        InjOffItems[inj_no] = 0;
      }
    }
  }
  
  _INLINE void OnTimer3Tick()
  {
    static uint8_t idx;
    
    // increase counter
    ++m_Timer3Counter;
    
    for(idx = 0; idx < m_t3Count; ++idx)
    {
      if(m_Timer3Counter % m_Timer3CallbacksMask[idx] == 0)
      {
        m_Timer3CallbacksFunc[idx]();
      }
    }
  }
  
  static void ToggleLed()
  {
    led_stat = !led_stat;
    digitalWrite(led, led_stat);
  }
  
  void Start()
  {
    // Start Nil RTOS.
    nilSysBegin();
    
    // run forever to avoid function call
    while(true)
    {
      SysSerialEvent();
      BtSerial1Event();
      
      gBT.sendData();
    }
  }
};

static void _Inj1Changed();
static void _Inj2Changed();
static void _Inj3Changed();
static void _Inj4Changed();

const uint8_t InjectorHandler::m_IntPins[INJ_COUNT] = {0, 1, 4, 5};
const NoArgFuncPtr InjectorHandler::m_Callbacks[INJ_COUNT] = {_Inj1Changed, _Inj2Changed, _Inj3Changed, _Inj4Changed};
const uint8_t InjectorHandler::m_InPins[INJ_COUNT] = {2, 3, 19, 18};
const uint8_t InjectorHandler::m_OutPins[INJ_COUNT] = {8, 9, 10, 11};

static InjectorHandler injectors = InjectorHandler();

// End of InjectorHandler class
// -------------------------------------------------------------------------
// #########################################################################


// #########################################################################
// -------------------------------------------------------------------------
// RTOS section
// -------------------------------------------------------------------------
// #########################################################################

NIL_WORKING_AREA(waInjWaitTimeCal, 128); 
NIL_THREAD(InjWaitTimeCal,arg)
{
  const PWMItem_t *ptrPWM;
  volatile InjectorInfo_t *ptrInj;
  uint64_t time_off;
  uint64_t timenow;
  uint32_t period;
  
  while (true)
  {
    nilSemWait(&semPWM);
    
    ptrPWM = (const PWMItem_t *)PWMQueue.Pop();
    if(ptrPWM == NULL)
      continue;
    
    // Get injector infomation
    ptrInj = &gInjInfo[ptrPWM->inj_no];
    
    // calculate wait time
    period = (ptrPWM->period * (100 + gSysMon.FuelAdjust) / 100);
    
#if (CRITICAL_TIME_LIMIT == TRUE)
    if(period > ptrInj->CriticalTimeUS)
      period = ptrInj->CriticalTimeUS;
#endif

    time_off = ptrPWM->start_time + period;

    timenow = micros();
    injectors.WaitUntil(ptrPWM->inj_no, timenow, time_off);
  }
}

NIL_WORKING_AREA(waFuelAdjust, 128); 
NIL_THREAD(FuelAdjust,arg)
{
  uint8_t mapIndex;
  uint8_t rpmIndex;
  uint16_t mapVal;
  uint16_t prevRPM = 0;
  uint16_t prevMAP = 0;
  
  volatile InjectorInfo_t *ptrInj = &gInjInfo[0];  // adjust by first injector info
  
  while (true) 
  {        
    // wait next adjust request
    nilSemWait(&semFuelAdjust);
    
    if(gCurrentConfig->feature.en || gCurrentConfig->feature.mon)
    {      
      // adjust fuel by user
      gSysMon.FuelAdjust = gCurrentConfig->userAdj;
      
      if(gSysMon.EngineWarming)
      {
        if(ptrInj->CurrentRPM > WARMUP_RPM_LIMIT)
        {
          // Cancel warming when RPM more than limit
          InjectorHandler::OnWarmingTimeout();
        }
      }
      else
      {      
        // read MAP value from sensor in voltage
        mapVal = analogRead(MAP_SENSOR_PIN);
        mapVal = map(mapVal, 0, 1023, 0, 100);  // convert to load 0 - 100 %
        gSysMon.CurrentMAP = mapVal;
          
        // Process fuel adjust by MAP (load) acceleration change per second
        if(gCurrentConfig->feature.map_acc)
        {
          // Calculate acceleration
          if(prevMAP != 0)
          {
            // calculate MAP change per second
            gSysMon.CurrentMAPAcc = (gSysMon.CurrentMAP - prevMAP) * 1000 / gSysMon.AdjustTimeMs;
            
            // adjust on MAP change more than MAP step
            if(abs(gSysMon.CurrentMAPAcc) >= gCurrentConfig->map_adj_step)
            {            
              // adjust fuel by acceleration
              gSysMon.FuelAdjust += gCurrentConfig->mapAccAdj;
            }
          }
          prevMAP = gSysMon.CurrentMAP;
        }
          
        // Process fuel adjust by RPM acceleration change per second
        if(gCurrentConfig->feature.rpm_acc)
        {        
          // Calculate acceleration
          if(prevRPM != 0)
          {
            // calculate RPM change per sec
            ptrInj->CurrentRPMAcc = (ptrInj->CurrentRPM - prevRPM) * 1000 / gSysMon.AdjustTimeMs;
            
            // adjust on RPM change more than 100 RPM
            if(abs(ptrInj->CurrentRPMAcc) >= gCurrentConfig->rpm_adj_step)
            {            
              // adjust fuel by acceleration
              gSysMon.FuelAdjust += gCurrentConfig->rpmAccAdj;
            }
          }
          prevRPM = ptrInj->CurrentRPM;
        }
        
        // Process fuel adjust by RPM and MAP(load) mapping
        if(gCurrentConfig->feature.map)
        {
          // MAP index looking
          mapIndex = 0;
          if(gSysMon.CurrentMAP >= gCurrentConfig->map_start)
          {
            mapIndex = ((gSysMon.CurrentMAP - gCurrentConfig->map_start)) / gCurrentConfig->map_step;
            if(mapIndex >= gCurrentConfig->map_count )
              mapIndex = gCurrentConfig->map_count - 1;
          }
        
          // RPM index looking
          rpmIndex = 0;
          if(ptrInj->CurrentRPM >= gCurrentConfig->rpm_start)
          {
            rpmIndex = (((float)(ptrInj->CurrentRPM - gCurrentConfig->rpm_start) * 10) / gCurrentConfig->rpm_step + 5)/10;    
            if(rpmIndex < 0)
              rpmIndex = 0;
            else if(rpmIndex >= gCurrentConfig->rpm_count )
              rpmIndex = gCurrentConfig->rpm_count - 1;
          }
            
          // adjust fuel by mapping
          gSysMon.FuelAdjust += *(gCurrentMapping + (mapIndex * gCurrentConfig->map_count + rpmIndex));
        }
      }
      
      // Trim fuel adjustment between minimum and maximum
      if(gSysMon.FuelAdjust > FUEL_ADJ_MAX)
        gSysMon.FuelAdjust = FUEL_ADJ_MAX;
      else if(gSysMon.FuelAdjust < FUEL_ADJ_MIN)
        gSysMon.FuelAdjust = FUEL_ADJ_MIN;
        
      if(gSysMon.FuelAdjust + ptrInj->CurrentDuty > FUEL_ADJ_MAX_DUTY)
        gSysMon.FuelAdjust = FUEL_ADJ_MAX_DUTY - ptrInj->CurrentDuty;      
    }
  }
}

NIL_THREADS_TABLE_BEGIN()
NIL_THREADS_TABLE_ENTRY("InjWaitTimeCal", InjWaitTimeCal, NULL, waInjWaitTimeCal, sizeof(waInjWaitTimeCal))
NIL_THREADS_TABLE_ENTRY("FuelAdjust", FuelAdjust, NULL, waFuelAdjust, sizeof(waFuelAdjust))
NIL_THREADS_TABLE_END()

// -------------------------------------------------------------------------
// End of RTOS section
// -------------------------------------------------------------------------
// #########################################################################

// #########################################################################
// -------------------------------------------------------------------------
// Interrupt Services
// -------------------------------------------------------------------------
// #########################################################################

// Injector 1 changed interrupt
// -------------------------------------------------------------------------
static void _Inj1Changed()
{
  injectors.OnStatChanged(INJ_1);
}

// Injector 2 changed interrupt
// -------------------------------------------------------------------------
static void _Inj2Changed()
{
  injectors.OnStatChanged(INJ_2);
}

// Injector 3 changed interrupt
// -------------------------------------------------------------------------
static void _Inj3Changed()
{
  injectors.OnStatChanged(INJ_3);
}

// Injector 4 changed interrupt
// -------------------------------------------------------------------------
static void _Inj4Changed()
{
  injectors.OnStatChanged(INJ_4);
}

// Process on Timer1 interrupt
ISR(TIMER1_COMPA_vect)
{
  injectors.OnTimer1Tick();
}

// Process on Timer3 interrupt
ISR(TIMER3_COMPA_vect)
{
  injectors.OnTimer3Tick();
}

// Process on Timer4 interrupt
ISR(TIMER4_COMPA_vect)
{
  if(++gTimerSCallCount >= gTimerSTimeout)
  {
    // disable timer on timeout
    disableTimerS();
    
    if(gTimerSCallback != NULL)
      gTimerSCallback();
  }
}

// -------------------------------------------------------------------------
// End of Interrupt Services section
// -------------------------------------------------------------------------
// #########################################################################

// #########################################################################
// -------------------------------------------------------------------------
// Setup enveronment
// -------------------------------------------------------------------------
// #########################################################################

void programBTSerial(const char *name, const uint32_t &buadrate, const char *pswd)
{
#ifdef SERIAL_BT_DEV
  char serial_buf[32];
  
  pinMode(PROGRAM_MODE_PIN, OUTPUT);
  digitalWrite(PROGRAM_MODE_PIN, HIGH);
  
  // Change name of bluetooth device
  sprintf(serial_buf, "AT+NAME=\"%s\"\r\n", name);
  BTSerial.print(serial_buf);
  
  // Change name of bluetooth device
  sprintf(serial_buf, "AT+UART=%ld,0,0\r\n", buadrate);
  BTSerial.print(serial_buf);
  
  // Change password of bluetooth device
  sprintf(serial_buf, "AT+PSWD=\"%s\"\r\n", pswd);
  BTSerial.print(serial_buf);
  
  digitalWrite(PROGRAM_MODE_PIN, LOW);
  
  gSysMon.SerialSpeed = buadrate;
  EEPROM.write(EEPROM_SERIAL_ADDR, buadrate & 0xFF);
  EEPROM.write(EEPROM_SERIAL_ADDR, (buadrate >> 8) & 0xFF);
  EEPROM.write(EEPROM_SERIAL_ADDR, (buadrate >> 16) & 0xFF);
  
  BTSerial.begin(buadrate);
#endif
}

// 
// -------------------------------------------------------------------------
void initSerial() {  
  // Initial bluetooth buadrate
  gBT.setSpeed(DEFUALT_SERIAL_SPEED);
  gBT.connect();
}

// #########################################################################
// -------------------------------------------------------------------------
// EEPROM section
// -------------------------------------------------------------------------
// #########################################################################
void readEEPROM()
{
  uint16_t addr = 0;
  uint8_t *ptr = (uint8_t *)&gEEPROM;
  uint16_t size = sizeof(EEPROM_Data_t);
  
  while(addr < size)
    *ptr++ = EEPROM.read(addr++);
}

void writeEEPROM()
{
  uint16_t addr = 0;
  uint8_t *ptr = (uint8_t *)&gEEPROM;
  uint16_t size = sizeof(EEPROM_Data_t);
  
  while(addr < size)
     EEPROM.write(addr++, *ptr++);
}

void writeEEPROM(void *addr, uint16_t size)
{
  uint8_t *ptr = (uint8_t *)&gEEPROM;
  uint32_t offset = (uint32_t)&gEEPROM - (uint32_t)addr;
  
  while(size--)
  {
    EEPROM.write(offset, *(ptr + offset));
    ++offset;
  }
}

void initEEPROM()
{  
  // read EEPROM data into data structure
  readEEPROM();
  
  // Set current configuration
  if(gEEPROM.current >= MAX_CONFIG_COUNT)
    gEEPROM.current = (uint8_t)CFG_SYSTEM;    
  setActiveConfig(gEEPROM.current);
 
  // check defualt value
  // -----------------------
  uint8_t *ptr;
  uint16_t size;
  uint16_t addr;
  
  // features
  ptr = (uint8_t *)&gCurrentConfig->feature;
  size = sizeof(Feature_t);
  addr = DEFAULT_FEATURE_EN;
  while(size--)
  {
    if(*ptr == 0xFF)
      *ptr = addr & 0x01;
      
    ptr++;
    addr >>= 1;
  }

  // acc adjust
  if(gCurrentConfig->rpmAccAdj == 0xFF)
    gCurrentConfig->rpmAccAdj = DEFAULT_RPM_ACC_ADJ;
  if(gCurrentConfig->mapAccAdj == 0xFF)
    gCurrentConfig->mapAccAdj = DEFAULT_MAP_ACC_ADJ;
    
  gCurrentConfig->maxAdj = FUEL_ADJ_MAX;

  // load user adjust
  if(gCurrentConfig->userAdj == 0xFF)
    gCurrentConfig->userAdj = DEFAULT_USER_ADJ;

  // check RPM
  if(gCurrentConfig->rpm_count == 0xFF)
    gCurrentConfig->rpm_count = RPM_COUNT_MAX;
  if(gCurrentConfig->rpm_start == 0xFFFF)
    gCurrentConfig->rpm_start = RPM_START;
  if(gCurrentConfig->rpm_step == 0xFFFF)
    gCurrentConfig->rpm_step = RPM_STEP;
  if(gCurrentConfig->rpm_adj_step == 0xFFFF)
    gCurrentConfig->rpm_adj_step = RPM_ADJ_STEP;
    
  gCurrentConfig->rpm_end = gCurrentConfig->rpm_start + gCurrentConfig->rpm_step * gCurrentConfig->rpm_count;

  // check MAP
  if(gCurrentConfig->map_count == 0xFF)
    gCurrentConfig->map_count = MAP_COUNT_MAX;
  if(gCurrentConfig->map_start == 0xFF)
    gCurrentConfig->map_start = MAP_START;
  if(gCurrentConfig->map_step == 0xFF)
    gCurrentConfig->map_step = MAP_STEP;
  if(gCurrentConfig->map_adj_step == 0xFF)
    gCurrentConfig->map_adj_step = MAP_ADJ_STEP;
    
  gCurrentConfig->map_end = gCurrentConfig->map_start + gCurrentConfig->map_step * gCurrentConfig->map_count;

  // check engine
  if(gCurrentConfig->warmup_time_s == 0xFF)
    gCurrentConfig->warmup_time_s = WARMUP_TIME_S;
  if(gCurrentConfig->warmup_rpm == 0xFFFF)
    gCurrentConfig->warmup_rpm = WARMUP_RPM_LIMIT;
  if(gCurrentConfig->rev_per_pulse == 0xFF)
    gCurrentConfig->rev_per_pulse = REV_PER_PULSE;
    
  // fuel map
  ptr = gCurrentMapping;
  size = MAP_COUNT_MAX * RPM_COUNT_MAX;
  addr = 0;
  uint32_t ratio = (FUEL_ADJ_MAX - gCurrentConfig->userAdj) * 70 / gCurrentConfig->rpm_count;  // Ratio is 70% of adjust from different between maximum and manual adjust
  uint8_t x;
  uint8_t y;
  uint32_t aa = gCurrentConfig->rpm_count * gCurrentConfig->rpm_count;
  uint32_t bb = gCurrentConfig->map_count * gCurrentConfig->map_count;
  while(addr < size)
  {
    if(*ptr == 0xFF)
    {
      // calculate default adjust using ellipse equation x*x/a*a + y*y/b*b = 1
      x = addr % gCurrentConfig->rpm_count;
      y = addr / gCurrentConfig->rpm_count;
      
      *ptr = (x*x/aa + y*y/bb) * ratio / 100;
    }
      
    ptr++;
    ++addr;
  }
}
// End of EEPROM section
// -------------------------------------------------------------------------
// #########################################################################

// System setup
// -------------------------------------------------------------------------

void initTimers()
{
  // initial Timer1
  TCCR1A = 0;  // reset register
  TCCR1B = 0;  // reset register
  TCNT1 = 0;   // reset counter
  
  TCCR1B |= (1 << WGM12);  // Enable CTC Mode
  TCCR1B |= (1 << CS11);   // Set 8 prescaler (1 count = 0.5 us for 16MHz, 1 us for 8MHz)
  OCR1A = (INJ_OFF_CHK_TIME_US << 1) - 1;  // set compare interrupt every INJ_OFF_CHK_TIME_US us
  sbi(TIMSK1, OCIE1A);  // enable timer compare interrupt
  
  // initial Timer3
  TCCR3A = 0;  // reset register
  TCCR3B = 0;  // reset register
  TCNT3 = 0;   // reset counter
  
  TCCR3B |= (1 << WGM32);  // Enable CTC Mode
  TCCR3B |= (1 << CS31);   // Set 8 prescaler (1 count = 0.5 us for 16MHz, 1 us for 8MHz)
  OCR3A = (TIMER_CHK_PERIOD_US << 1) - 1;  // set compare interrupt every TIMER_CHK_PERIOD_US us
  sbi(TIMSK3, OCIE3A);  // enable timer compare interrupt
  
  // initial Timer4 - for multi purpose using
  TCCR4A = 0;  // reset register
  TCCR4B = 0;  // reset register
  TCNT4 = 0;   // reset counter
  
  TCCR4B |= (1 << WGM42);  // Enable CTC Mode
  TCCR4B |= (1 << CS42) | (1 << CS40);   // Set 1024 prescaler (1 count = 64 us for 16MHz)
  OCR4A = 15624;  // set compare interrupt in 1 s
}

void enableTimerS(uint8_t timeout, NoArgFuncPtr callback)
{
  gTimerSCallCount = 0;
  gTimerSCallback = callback;  // set callback function
  gTimerSTimeout = timeout;
  sbi(TIMSK4, OCIE4A);  // enable timer compare interrupt
}

void disableTimerS()
{
  cbi(TIMSK4, OCIE4A);  // disable timer compare interrupt
  gTimerSCallback = NULL;
}

void setup()
{ 
  cli();
  
  // load EEPROM data
  initEEPROM();
  
  // initial serial port
  initSerial();
  
  // initial Timers
  initTimers();
  
  // initial ADC
  setAdcFreq(FREQ_4MHz);
  
  // Initial injectors
  injectors.Init();
  
  // enable interrupt
  sei();
  
  // start, this process will run forever
  injectors.Start();
}
// -------------------------------------------------------------------------
// End of enveronment setup section
// -------------------------------------------------------------------------
// #########################################################################


// #########################################################################
// -------------------------------------------------------------------------
// Main loop
// -------------------------------------------------------------------------
// #########################################################################
void loop()
{
  // do nothing
}
// -------------------------------------------------------------------------
// End of main loop section
// -------------------------------------------------------------------------
// #########################################################################


// #########################################################################
// -------------------------------------------------------------------------
// Data communication between Arduino Mega 2560 and serial interface
// -------------------------------------------------------------------------
// #########################################################################

// Data package declaration
typedef union
{
  struct
  {
    uint8_t cmd;      // Command code
    uint8_t type;     // 0 for request, 1 for response
    uint8_t errCode;  // response error code
    uint8_t chkSum;   // package checksum
    uint8_t data[CMD_BUF_SIZE - CMD_HEADER_SIZE];  // data
  } Package;
  uint8_t  Buffer[CMD_BUF_SIZE];  // all package bytes
} DataPackage_t;

// Receive data package
volatile DataPackage_t cmdReceive;

// Response data package
volatile DataPackage_t cmdResponse;

// Serial buffer index
volatile uint8_t bufIndex = 0;

char sysBuf[CMD_BUF_SIZE];
uint8_t sysBufIndex = 0;

// Receive data event from bluetooth serial
// -------------------------------------------------------------------------
void BtSerial1Event() {
  while (BTSerial.available()) {
    // get the new byte:
    cmdReceive.Buffer[bufIndex] = BTSerial.read(); 
    
    if((bufIndex + 1 == CMD_BUF_SIZE) || cmdReceive.Buffer[bufIndex] == '\n')
    {
        // received CR LF then end package
        processCmd();
        bufIndex = 0;
    }
    else
    {
      ++bufIndex;
      if(bufIndex >= CMD_BUF_SIZE)
        bufIndex = 0;
    }
  }
}

// Receive data event from system serial
// -------------------------------------------------------------------------
void SysSerialEvent()
{
  while (SysSerial.available()) {
    // get the new byte:
    sysBuf[sysBufIndex] = SysSerial.read(); 
    
    if(sysBuf[sysBufIndex] == '\n')
    {
      if(sysBufIndex + 1 < CMD_BUF_SIZE)
        sysBuf[sysBufIndex] = '\0';
      memcpy((void *)cmdReceive.Buffer, sysBuf, sysBufIndex + 1);
      
      // received CR LF then end package
      processCmd();
      sysBufIndex = 0;
    }
    else
    {
      ++sysBufIndex;
      if(sysBufIndex >= CMD_BUF_SIZE)
        sysBufIndex = 0;
    }
  }
}

// Calculate package checksum
// params:
//    data = pointer to package buffer to calculate
//    count = package buffer size to calculate
// return:
//    checksum value
// -------------------------------------------------------------------------
uint8_t calChkSum(const uint8_t *data, size_t count)
{
  static const uint8_t ChkSumMask = 0x55;
  uint8_t sum = 0;
  
  while(count--)
    sum += *data++;
    
  sum += ChkSumMask;
  
  return sum;
}

// Check mapping index is in range
// params:
//    row = index of MAP
//    col = index of RPM
// return:
//    true = is in range
//    false = out of range
// -------------------------------------------------------------------------
bool isInMappingRange(int row, int col)
{
  return ((row >= 0 && row < gCurrentConfig->map_count) && (col >= 0 && col < gCurrentConfig->rpm_count));
}

// Send response package
// params:
//    errCode = package error code (default = ERR_NONE)
// ---------------------------------
void sendCmdResponse(const ErrCodes errCode = ERR_NONE)
{
  cmdResponse.Package.errCode = errCode;
  cmdResponse.Package.chkSum = calChkSum((const uint8_t *)cmdResponse.Buffer, CMD_BUF_SIZE);
  gBT.write((const uint8_t *)cmdResponse.Buffer, CMD_BUF_SIZE);
}

// Initial response package
// ---------------------------------
void initCmdResponse()
{  
  // clear package
  memset((void *)cmdResponse.Buffer, 0, CMD_BUF_SIZE);
  
  // copy command code
  cmdResponse.Package.cmd = cmdReceive.Package.cmd;
  
  // set command type to response
  cmdResponse.Package.type = 1;
  
  // clear checksum
  cmdResponse.Package.chkSum = 0;
  
  // clear error code
  cmdResponse.Package.errCode = ERR_NONE;
  
  // clear data
  memset((void *)cmdResponse.Package.data, 0, CMD_BUF_SIZE - CMD_HEADER_SIZE);
}

// Get next parameter from command string
// params:
//    cmdText = command string
//    param = return next paramter
//    first = first parameter flag
// return:
//    true = next parameter found
//    false = next parameter not found
// -------------------------------------------------------------------------
bool getNextParam(const String &cmdText, String &param, bool first = false)
{
  static const char Separator = ',';
  static int first_idx;
  static int last_idx;
  
  if(first)
    first_idx = 0;
    
  last_idx = cmdText.indexOf(Separator, first_idx + 1);
  if(last_idx == -1)
    last_idx = cmdText.length() - 1;
    
  if(first_idx < last_idx)
  {
    first_idx = last_idx;
    param = cmdText.substring(first_idx, last_idx);
    
    return true;
  }
  
  return false;
}

void internalCommand(CMDs cmd,const void *data, const uint8_t len)
{  
  // clear package
  memset((void *)cmdReceive.Buffer, 0, CMD_BUF_SIZE);
  
  cmdReceive.Package.cmd = (uint8_t)cmd;
  if(data != NULL && len != 0)
    memcpy((void *)cmdReceive.Package.data, data, len);
    
  cmdReceive.Package.chkSum = calChkSum((const uint8_t *)cmdReceive.Buffer, len + CMD_HEADER_SIZE);;
  
  processCmd();
}

// Process command received from serial interface
// -------------------------------------------------------------------------
void processCmd()
{
  uint8_t rowIndex;
  uint8_t colIndex;
  uint16_t address;
  ErrCodes errCode = ERR_NONE;
  volatile uint8_t *ptrReceiveData = cmdReceive.Package.data;
  volatile uint8_t *ptrResponseData = cmdResponse.Package.data;
  volatile uint8_t *ptrMapping;
  uint32_t count;
  uint8_t *ptr;
  
  if(cmdReceive.Package.cmd <= CMD_START || cmdReceive.Package.cmd >= CMD_END)
  {
    // process text commands
    String cmdText = String((char *)cmdReceive.Buffer);
    String param = "";
    
    // remove newline
    cmdText.replace("\n", "");
      
    // convert command text to update case
    cmdText.toUpperCase();
    
    if(param.substring(0, 2) == "AT")
    {
      // process blutooth AT Command
      gBT.sendAtCmd(cmdText);
    }
    else
    {
      if(getNextParam(cmdText, param, true))
      {
        String strCmd = param;
        
        if(strCmd == "ECHO")
        {
          getNextParam(cmdText, param);
          gBT.println(param);
        }
        else if(strCmd == "BT_MODE")
        {          
          if(getNextParam(cmdText, param))
          {
            if(param == "1")
              gBT.atModeOn();
            else
              gBT.atModeOff();
          }
        }
        else
          gBT.println("Command not found!");
      }
    }
    
    // end process command text
    return;
  }
  else
  {
    // check checksum before process command
    uint8_t tmp = cmdReceive.Package.chkSum;
    cmdReceive.Package.chkSum = 0;
    uint8_t chkSum = calChkSum((const uint8_t *)cmdReceive.Buffer, CMD_BUF_SIZE);
    if(chkSum != tmp)
    {
      // return checksum not match error
      errCode = ERR_CHK_SUM;
    }
  }
  
  if(errCode == ERR_NONE)
  {
    // initial response data package
    initCmdResponse();
    
    switch(cmdReceive.Package.cmd)
    {
      case CMD_MONITOR_INFO:
      case CMD_INJECTOR_INFO:
      case CMD_PRINT_INFO:
        memcpy((void *)ptrResponseData, (const void *)ptrReceiveData, CMD_BUF_SIZE - CMD_HEADER_SIZE);
      break;
      case CMD_READ_SYSMON:
        memcpy((void *)ptrResponseData, (const void *)&gSysMon, sizeof(SysMon_t));
      break;
      case CMD_READ_CONFIGS:
        ptr = (uint8_t *)gCurrentConfig;
        count = sizeof(Config_t);
        while(count--)
          *ptrResponseData++ = *ptr++;
      break;
      case CMD_WRITE_CONFIGS:
        ptr = (uint8_t *)gCurrentConfig;
        count = sizeof(Config_t);
        while(count--)
          *ptr++ = *ptrReceiveData++;
          
        writeEEPROM(gCurrentConfig, sizeof(Config_t));
      break;
      case CMD_READ_MAPPING:
        // read mapping buffer
        address = *((uint16_t *)ptrReceiveData);
        count = *(ptrReceiveData + 2);
        
        if(address + count < gCurrentConfig->map_count * gCurrentConfig->rpm_count)
        {
          *ptrResponseData++ = *ptrReceiveData++;
          *ptrResponseData++ = *ptrReceiveData++;
          *ptrResponseData++ = *ptrReceiveData++;
          
          ptr = gCurrentMapping + address;
          while(count--)
            *ptrResponseData++ = *ptr++;
        }
        else
          errCode = ERR_CMD_INDEX_OUT_OF_BOUNDS;
      break;
      case CMD_WRITE_MAPPING:      
        address = *((uint16_t *)ptrReceiveData);
        count = *(ptrReceiveData + 2);
        ptrReceiveData += 3;
        
        if(address + count < gCurrentConfig->map_count * gCurrentConfig->rpm_count)
        {
          ptr = gCurrentMapping + address;
          while(count--)
            *ptr++ = *ptrReceiveData++;
        }
        else
          errCode = ERR_CMD_INDEX_OUT_OF_BOUNDS;
      break;
      case CMD_WRITE_BT_CFG:
        programBTSerial("EIT Flex", 115200, "1234");
      break;
      default:
        errCode = ERR_CMD_NOT_FOUND;
      break;
    }
  }
  
  // send response
  sendCmdResponse(errCode);
}
// -------------------------------------------------------------------------
// End of data communication section
// -------------------------------------------------------------------------
// #########################################################################

