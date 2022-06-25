#pragma once

#include "Particle.h"

// SparkIntervalTimer library needs to be imported seperately
#if defined(_PARTICLE_BUILD_IDE_)
#  include "SparkIntervalTimer/SparkIntervalTimer.h"
#else
#  include "SparkIntervalTimer.h"
#endif

#define _MPSS_MAX_PORT        3     /* 最多支持3个软串口 */
#define _MPSS_BUFF_SIZE       64    // buffer size

typedef enum {
  itTxEmpty = 0,
  itTxFull = 1,
  itTxTimeout = 2,
  itRxByte = 4,
  itRxFull = 5
} itSoftUart_e;

class MultiSoftSerial : public Stream
{
private:
  int      _rxPin;
  int      _txPin;
    
  char _rxBuffer[_MPSS_BUFF_SIZE];
  volatile boolean	_rxEnable;
  volatile uint8_t _rxBitShift;
  volatile uint8_t _rxBitCounter;
  volatile uint8_t _rxHead;
  volatile uint8_t _rxTail;
  volatile uint8_t _rxCount;
	volatile uint8_t _rxTimingFlag;
	
  char _txBuffer[_MPSS_BUFF_SIZE];
  boolean	_txEnable;
  uint8_t _txBitShift;
  uint8_t _txBitCounter;
  uint8_t _txHead;
  uint8_t _txTail;
  uint8_t _txCount;

	uint8_t _txMoreData;
	uint8_t	_ITReg;

  void   resetPort(void);
  void   prepareGPIO(void);

public:
  // public methods
  MultiSoftSerial(const uint8_t id, const int rxPin, const int txPin);
  ~MultiSoftSerial();
  
  void begin(void);
  void end(void);
  uint8_t _id;
  boolean  _enabled;
  uint8_t _rxBitOffset;

  virtual int    availableForWrite(void);
  virtual int    available(void);
  virtual size_t write(uint8_t b);
  size_t         write(uint16_t b9); // nine-bit
  virtual size_t write(const uint8_t *buffer, const uint8_t size);
  boolean        waitUntilTxComplate(const uint32_t _ms = 50);

  virtual int    read(void);
  virtual int    peek(void);
  virtual void   flush(void);
  uint8_t readRxBuffer(uint8_t *_pBuf, uint8_t *_pLen);

  void abortTransmit(void);
  void abortReceive(void);

  using Print::write; // pull in write(str) and write(buf, size) from Print

  uint8_t enableRx(void);
  uint8_t disableRx(void);

  inline uint8_t readRxPin(void);
  void txProcess(void);
  void rxDataBitProcess(const uint8_t _bit);

  inline uint8_t getITStateAll() { return _ITReg; };
  inline uint8_t getITState(const itSoftUart_e _bit);
  inline void clearITState(const itSoftUart_e _bit);
  inline void setITState(const itSoftUart_e _bit);
  inline void transmitBit(const uint8_t _bit);
};

class MultiSoftSerialManager
{
private:  
  static MultiSoftSerial *m_ports[_MPSS_MAX_PORT];
  static IntervalTimer mpssTimer;
  static uint8_t m_portCnt;

  static uint32_t _usStartBit;
  static uint32_t _usBitLength;
  static uint8_t  _parity;
  static uint8_t  _dataBits;
  static uint8_t  _totalBits;

  static uint8_t scanRxPorts(void);

public:
  // public methods
  MultiSoftSerialManager();
  ~MultiSoftSerialManager();

  MultiSoftSerial *addPort(const int rxPin, const int txPin);
  void beginPorts(const uint32_t baud);
  void beginPorts(const uint32_t baud, const uint32_t config);
  void endPorts(void);

  // public only for easy access by interrupt handlers
  static inline void mpssTimerISR(void); // called by mpssTimer
};