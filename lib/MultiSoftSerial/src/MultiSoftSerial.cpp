#include "MultiSoftSerial.h"
#include "xliCommon.h"

#define SoftUartDataBits			    8
#define SoftUartSDBitLen			    (SoftUartDataBits + 1)		// 8 bit data + start bit
#define SoftUartSampleBits			  	(SoftUartDataBits + 2) 		// 8 bit data + start + stop bit
#define SoftUartStartBitSamples			2  		                    // check first 2 samples when detect start bit edge

#define S_UART_START_BIT			    0
#define S_UART_STOP_BIT				    1

typedef struct {
  uint32_t baudrate;
  uint16_t usStartBit;
  uint16_t usBitLength;
} BAUD_TIMING;

static const BAUD_TIMING btTable[] =
{
//    baud  �s/start  �s/bit 1/baudrate 
//                    shorter due to call latency
//  { 115200,     3,     9 }, // exact    8.68056�s
  {  57600,     9,    16 }, //         17.36111�s
  {  38400,    24,    24 }, //         26.04167�s
  {  31250,    31,    31 }, //         32.00000�s
  {  28800,    37,    33 }, //         34.72222�s
  {  19200,    61,    51 }, //         52.08333�s
  {  14400,    90,    68 }, //         69.44444�s
  {   9600,   140,   104 }, //        104.16667�s
  {   4800,   295,   208 }, //        208.33333�s
  {   2400,   610,   417 }, //        416.66667�s
  {   1200,  1230,   833 }, //        833.33333�s
  {    600,  2500,  1667 }, //       1666.66667�s
  {    300,  5000,  3333 }, //       3333.33333�s
  {      0,     0,     0 }  // end mark
};

// For timing division
static volatile uint8_t m_SUartTimer = 0;

MultiSoftSerial *MultiSoftSerialManager::m_ports[_MPSS_MAX_PORT] = {NULL};
IntervalTimer MultiSoftSerialManager::mpssTimer;
uint32_t MultiSoftSerialManager::_usStartBit    =          140; // start bit with odd lengths due to EXTI latency
uint32_t MultiSoftSerialManager::_usBitLength   =          104; // default 9600 baud = 104µs per bit
uint8_t  MultiSoftSerialManager::_parity        =         0x00; // default NONE
uint8_t  MultiSoftSerialManager::_dataBits      =            8; // default 8bit
uint8_t  MultiSoftSerialManager::_totalBits     =            9; // ignore start bit, 8data + 1stop
uint8_t  MultiSoftSerialManager::m_portCnt      =            0;

/* MultiSoftSerialManager ------------------------------------------------------------*/
MultiSoftSerialManager::MultiSoftSerialManager() {
}

MultiSoftSerialManager::~MultiSoftSerialManager() {
	endPorts();
	m_portCnt = 0;
	for(uint8_t i = 0; i < _MPSS_MAX_PORT; i++) m_ports[i] = NULL;
}

MultiSoftSerial *MultiSoftSerialManager::addPort(const int rxPin, const int txPin) {
	if(m_portCnt < _MPSS_MAX_PORT) {
		MultiSoftSerial *lv_port = new MultiSoftSerial(m_portCnt, rxPin, txPin);
		m_ports[m_portCnt++] = lv_port;
		return(lv_port);
	}
	// No resource
	return(NULL);
}

void MultiSoftSerialManager::beginPorts(const uint32_t baud) {
	beginPorts(baud, SERIAL_8N1);
}

void MultiSoftSerialManager::beginPorts(const uint32_t baud, const uint32_t config) {
	uint8_t i;
	if (config & SERIAL_DATA_BITS_9) {
		_dataBits = 9;
	}
	else if (config & SERIAL_DATA_BITS_7) {
		_dataBits = 7;
	} else {
		_dataBits = 8;
	}

	if (config & SERIAL_STOP_BITS_2) {
		_totalBits = _dataBits + 2;
	} else {
		_totalBits = _dataBits + 1;
	}

	if (config & SERIAL_PARITY) {
		_parity = (config & SERIAL_PARITY_ODD) ? 0x11 : 0x10;
		_totalBits++;
	} else {
		_parity = 0;
	}
  
  	for (i=0; btTable[i].baudrate > 0; i++) {
    	if (btTable[i].baudrate <= baud) {
#if (SYSTEM_VERSION >= 0x00060000)
      		if (btTable[i].baudrate != baud) {
        		SERIAL_LN("%lu not available! Selected rate %lu", baud, btTable[i].baudrate);
      		}
#endif
      		_usStartBit  = btTable[i].usStartBit;
      		_usBitLength = btTable[i].usBitLength;
      		break;
    	}
  	}

	/* 启动各端口 */
	for(i = 0; i < m_portCnt; i++) {
		if(m_ports[i]) m_ports[i]->begin();
	}

	/* 启动定时器 */
	// mpssTimerISR must call in interrupt every 0.2*(1/BR)
	// if BR=9600 then 0.2*(1/9600)=20.8333333 uS
	mpssTimer.begin(mpssTimerISR, _usBitLength / 5, uSec, TIMER6);
}

void MultiSoftSerialManager::endPorts() {
	/* 停止定时器 */
	mpssTimer.end();

	/* 停止各端口 */
	for(uint8_t i = 0; i < m_portCnt; i++) {
		if(m_ports[i]) m_ports[i]->end();
	}
}

// Capture RX and Get BitOffset
uint8_t MultiSoftSerialManager::scanRxPorts(void) {
	uint8_t _bitRow = 0x00, _bit;

	for(uint8_t i = 0; i < m_portCnt; i++) {
		// Read a bit from RX GPIO
		_bit = m_ports[i]->readRxPin();

		// Add all RX GPIO State to Buffer
		_bitRow |= ((_bit & 0x01) << i);
	}
	return _bitRow;
}

// mpssTimerISR must call in interrupt every 0.2*(1/BR)
// if BR=9600 then 0.2*(1/9600)=20.8333333 uS
void MultiSoftSerialManager::mpssTimerISR(void) {
	uint8_t i, _bitRow;

	// Capture RX and Get BitOffset
	_bitRow = scanRxPorts();

	for(i = 0; i < m_portCnt; i++) {
		// Receive Data at the middle data pulse position
		if(m_ports[i]->_rxBitOffset == m_SUartTimer) {
			m_ports[i]->rxDataBitProcess(((_bitRow >> i) & 0x01));
		}
	}

	// Sending always happens in the first time slot
	if(m_SUartTimer == 0) {
		// Transmit Data
		for(i = 0; i < m_portCnt; i++) {
			m_ports[i]->txProcess();
		}
	}

	// Timing process
	m_SUartTimer++;
	m_SUartTimer %= 5;
}

/* MultiSoftSerial ------------------------------------------------------------*/
MultiSoftSerial::MultiSoftSerial(const uint8_t id, const int rxPin, const int txPin) {
	_rxPin = rxPin;
	_txPin = txPin;
	_id = id;
	_enabled = false;
	memset(_rxBuffer, 0x00, sizeof(_rxBuffer));
	memset(_txBuffer, 0x00, sizeof(_txBuffer));
	resetPort();
}

MultiSoftSerial::~MultiSoftSerial() {
	end();
}

void MultiSoftSerial::resetPort(void) {
	_rxEnable = _txEnable = false;

	_rxBitShift = _rxBitCounter = 
	_txBitShift = _txBitCounter = 0;

	_rxTimingFlag = _rxBitOffset = 0;

	_rxTail = _rxHead = _rxCount = 
	_txTail = _txHead = _txCount = 0;

	_txMoreData = 0;
	_ITReg = 0;
}

void MultiSoftSerial::prepareGPIO(void) {
	pinMode(_rxPin, INPUT_PULLUP);
	pinMode(_txPin, OUTPUT);
	pinSetFast(_txPin);
}

void MultiSoftSerial::begin(void) {
	prepareGPIO();
	_enabled = true;
}

void MultiSoftSerial::end(void) {
	_enabled = false;
	abortTransmit();
	abortReceive();
}

// Return specific IT flag (0 or 1)
uint8_t MultiSoftSerial::getITState(const itSoftUart_e _bit) {
	return(BF_GET(_ITReg, _bit, 1));
}

// Clear specific IT flag
void MultiSoftSerial::clearITState(const itSoftUart_e _bit) {
	BF_SET(_ITReg, 0, _bit, 1);
}

// Set specific IT flag
void MultiSoftSerial::setITState(const itSoftUart_e _bit) {
	BF_SET(_ITReg, 1, _bit, 1);
}

// Send one bit to TX pin
void MultiSoftSerial::transmitBit(const uint8_t _bit) {
	if(_bit) pinSetFast(_txPin); else pinResetFast(_txPin);
}

// Enable Sofe-UART Receiving
uint8_t MultiSoftSerial::enableRx() {
	_rxEnable = 1;
	return 0;
}

// Disable Sofe-UART Receiving
uint8_t MultiSoftSerial::disableRx() {
	_rxEnable = 0;
	return 0;
}

uint8_t MultiSoftSerial::readRxPin(void) {
	// Read a bit from RX GPIO
	uint8_t _bit = pinReadFast(_rxPin);

	// Starting conditions
	if(!_rxBitCounter && !_rxTimingFlag && !_bit) {
		// Save RX Bit Offset
		// Calculate the middle position of data pulse
		_rxBitOffset = ((m_SUartTimer + SoftUartStartBitSamples) % 5);
		// Timing Offset is Set
		_rxTimingFlag = 1;
	}
	return _bit;
}

int MultiSoftSerial::availableForWrite(void) {
	return _txCount; 
}

// Return the size of received data in RX buffer
int MultiSoftSerial::available(void) {
	return _rxCount;
}

// Sofe-UART Transmit Data Process
void MultiSoftSerial::txProcess(void) {
	if(_txEnable) {
		if(_txBitCounter == 0) {
			// Start bit = 0
			_txMoreData = 1;
			_txBitShift = 0;
			transmitBit(S_UART_START_BIT);
			_txBitCounter++;
		} else if(_txBitCounter < SoftUartSDBitLen) {
			// Data bits
			transmitBit(((_txBuffer[_txHead])>>(_txBitShift))&0x01);
			_txBitCounter++;
			_txBitShift++;
		} else if(_txBitCounter == SoftUartSDBitLen) {
			// Stop bit = 1
			transmitBit(S_UART_STOP_BIT);

			// Completed one byte
			// Reset Bit Counter
			_txBitCounter = 0;

			// Ready to send next byte
			_txHead++;
			_txHead %= _MPSS_BUFF_SIZE;
			_txCount--;

			// Do we have more data to send?
			if(_txCount > 0) {
				// Continue sending
				_txMoreData = 1;
				_txEnable = 1;
			} else {
				// Finished
				_txMoreData = 0;
				_txEnable = 0;
				// Generate a Tx-buf empty interrupt
				setITState(itTxEmpty);
			}
		}
	}
}

// Sofe-UART Receive Data Process
void MultiSoftSerial::rxDataBitProcess(const uint8_t _bit) {
	if(_rxEnable) {
		if(_rxBitCounter == 0) {
			// Start bit must be 0
			if(_bit != S_UART_START_BIT) return;
			_rxBitShift = 0;
			_rxBitCounter++;
			_rxBuffer[_rxTail] = 0;
		} else if(_rxBitCounter < SoftUartSDBitLen) {
			// Data bits
			_rxBuffer[_rxTail] |= ((_bit & 0x01)<< _rxBitShift);
			_rxBitCounter++;
			_rxBitShift++;
		} else if(_rxBitCounter == SoftUartSDBitLen) {
			// Finished on receiving Stop bit
			_rxBitCounter = 0;
			_rxTimingFlag = 0;

			// Stop bit must be 1
			if(_bit == S_UART_STOP_BIT) {
				// Received successfully
				// Change RX Buffer Index
				if(_MPSS_BUFF_SIZE - _rxCount > 0) {
					_rxTail++;
					_rxTail %= _MPSS_BUFF_SIZE;
					_rxCount++;
					// Generate a byte received interrupt
					setITState(itRxByte);
				} else {
					// Otherwise, Rx-buffer is full.
					// Generate a Rx-buf-full interrupt
					setITState(itRxFull);
				}
			}
			// ERROR -> Overwrite data
		}
	}
}

// Wait Until Transmit Completed
// You do not usually need to use this function!
boolean MultiSoftSerial::waitUntilTxComplate(const uint32_t _ms) {
	for(uint32_t ms_start = millis(); _txMoreData; Particle.process()) {
		if (millis() - ms_start > _ms) {
			// Generate a Tx-timeout interrupt
			setITState(itTxTimeout);      
			return false;
		}
	}
	return true;
}

size_t MultiSoftSerial::write(uint8_t b) {
	// Enough space?
	if(_MPSS_BUFF_SIZE - _txCount == 0) {
		// Generate a Tx-buf full interrupt
		setITState(itTxFull);
		return 0;
	}

	// Prepare Tx-buffer
	uint8_t lv_ptr = _txTail;
  	_txBuffer[lv_ptr++] = b;
	lv_ptr %= _MPSS_BUFF_SIZE;

	_txTail = lv_ptr;
	_txCount++;
	_txMoreData = 1;
	_txEnable = true;

	return 1;
}

size_t MultiSoftSerial::write(uint16_t b9) {
	// 9 bit not yet implemented
	return write((uint8_t)b9);
}

// Copy Data into Tx-buffer and Start Sending
size_t MultiSoftSerial::write(const uint8_t *buffer, const uint8_t size) {
	// Busy? wait for up to 0.5 second 
  	if(!waitUntilTxComplate(500)) return 0;

	// Enough space?
	if(_MPSS_BUFF_SIZE - _txCount < size) {
		// Generate a Tx-buf full interrupt
		setITState(itTxFull);
		return 0;
	}

	// Prepare Tx-buffer
	uint8_t lv_ptr = _txTail;
	for(int8_t i = 0; i < size; i++) {
		_txBuffer[lv_ptr++] = buffer[i];
		lv_ptr %= _MPSS_BUFF_SIZE;
	}
	_txTail = lv_ptr;
	_txCount += (uint8_t)size;
	_txMoreData = 1;
	_txEnable = true;

	return (uint8_t)size;
}

// Popup one byte from Rx Buffer
int MultiSoftSerial::read(void) {
	uint8_t d;

	// Empty buffer?
	if(!available()) return -1;

	uint8_t lv_ptr = _rxHead;
	d = _rxBuffer[lv_ptr++];
	lv_ptr %= _MPSS_BUFF_SIZE;
	_rxHead = lv_ptr;
	_rxCount--;

	return d;
}

// Peek one byte from Rx Buffer
int MultiSoftSerial::peek(void) {
	// Empty buffer?
	if(!available()) return -1;
	return _rxBuffer[_rxHead];
}

void MultiSoftSerial::flush(void) {
	if(!waitUntilTxComplate(200)) abortTransmit();
}

// Move Received Data to given buffer
uint8_t MultiSoftSerial::readRxBuffer(uint8_t *_pBuf, uint8_t *_pLen) {
	// Length
	uint8_t i, lv_cnt = _rxCount;
	uint8_t lv_len = MIN_RETURN((*_pLen), lv_cnt);

	// Copy data
  	int lv_read;
	for(i = 0; i < lv_len; i++) {
    lv_read = read();
    if(lv_read < 0) break;
    _pBuf[i] = (uint8_t)lv_read;
	}
	// Return the exact data size
	*_pLen = i;
	return i;
}

// Abort sending operation and clear Tx-buffer
void MultiSoftSerial::abortTransmit() {
	if(_txEnable) {
		_txEnable = false;
		_txMoreData = 0;
		_txHead = 0;
		_txTail = 0;
		_txCount = 0;
		_txBitShift = 0;
		_txBitCounter = 0;
		clearITState(itTxFull);
		clearITState(itTxTimeout);
		// Generate a Tx-buf empty interrupt
		setITState(itTxEmpty);
	}
}

// Abort receiving operation and clear Rx-buffer
void MultiSoftSerial::abortReceive() {
	if(_rxEnable) {
		_rxEnable = false;
		_rxTimingFlag = 0;
		_rxBitOffset = 0;
		_rxHead = 0;
		_rxTail = 0;
		_rxCount = 0;
		_rxBitShift = 0;
		_rxBitCounter = 0;
		clearITState(itRxFull);
		clearITState(itRxByte);
	}
}
