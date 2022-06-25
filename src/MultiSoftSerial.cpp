/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "Particle.h"
#line 1 "d:/5.Works/codetest/MultiSoftSerial/src/MultiSoftSerial.ino"
/*
 * Project MultiSoftSerial
 * Description:
 * Author:
 * Date:
 */

#include "xliCommon.h"
#include "MultiSoftSerial.h"

//------------------------------------------------------------------
// Soft Serial Port
void setup();
void loop();
#line 13 "d:/5.Works/codetest/MultiSoftSerial/src/MultiSoftSerial.ino"
#define PIN_SS1_TXD	 	            D3
#define PIN_SS1_RXD	 	            D4

#define PIN_SS2_TXD	 	            A0
#define PIN_SS2_RXD	 	            A1

#define PIN_SS3_TXD	 	            A2
#define PIN_SS3_RXD	 	            A3

// Software Serial Port Manager
MultiSoftSerialManager m_mpssMgr;
MultiSoftSerial *m_pPort[3] = {NULL};

// setup() runs once, when the device is first turned on.
void setup() {
  // Put initialization like pinMode and begin functions here.
	// Open Serial Port
	TheSerial.begin(SERIALPORT_SPEED_DEFAULT);

	// Wait Serial connection so that we can see the starting information
	while(!TheSerial.available()) {
		if( Particle.connected() == true ) { Particle.process(); }
	}

  m_pPort[0] = m_mpssMgr.addPort(PIN_SS1_RXD, PIN_SS1_TXD);
  m_pPort[1] = m_mpssMgr.addPort(PIN_SS2_RXD, PIN_SS2_TXD);
  m_pPort[2] = m_mpssMgr.addPort(PIN_SS3_RXD, PIN_SS3_TXD);
 
	/* 启动全部软串口，
	   注意：addPort必须在这之前调用
	 */
	m_mpssMgr.beginPorts(SERIALPORT_SPEED_LOW);
  for(uint8_t i = 0; i < 3; i++) {
    if(m_pPort[i]) {
      m_pPort[i]->enableRx();
      SERIAL_LN("SoftSerial port: %d enabled: %d", m_pPort[i]->_id, m_pPort[i]->_enabled);
    } 
  }
}

// loop() runs over and over again, as quickly as it can execute.
void loop() {
  	// The core of your code will likely live here.
    static uint32_t _timer = 0;
    uint8_t _inByte;

    _timer++;
    for(uint8_t i = 0; i < 3; i++) {
      // Check incoming data
      if(m_pPort[i]) {
        while(m_pPort[i]->available()) {
          _inByte = m_pPort[i]->read();
          SERIAL("%c", _inByte);
        }

        if(_timer % 500 == i * 100) {
          if(m_pPort[i]->write((uint8_t)'+') == 0) {
            SERIAL_LN("\r\n SoftSerial port: %d ITState: 0x%02X", m_pPort[i]->_id, m_pPort[i]->getITStateAll());
          }
          m_pPort[i]->flush();
        }
      }
    }
    delay(10);
}