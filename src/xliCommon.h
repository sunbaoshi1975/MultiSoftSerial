//  xlCommon.h - Xlight common definitions header

#ifndef xliCommon_h
#define xliCommon_h

#ifndef MCU_TYPE_P1
#define MCU_TYPE_P1
#endif

#include "application.h"

// Here, common data types have been given alternative names through #define statements
// Common Data Type
#define BOOL                      boolean
#define UC                        uint8_t
#define US                        uint16_t
#define UL                        uint32_t
#define CHAR                      int8_t
#define SHORT                     int16_t
#define LONG                      int32_t

// Serial Port Speed
#define SERIALPORT_SPEED_LOW      9600
#define SERIALPORT_SPEED_14400    14400
#define SERIALPORT_SPEED_MID      19200
#define SERIALPORT_SPEED_57600    57600
#define SERIALPORT_SPEED_HIGH     115200
#define SERIALPORT_SPEED_DEFAULT  SERIALPORT_SPEED_HIGH

// Specify system serial port, could be Serial, USBSerial1, Serial1 or Seria2
#ifndef TheSerial
#define TheSerial       Serial
#endif

#define SERIAL          TheSerial.printf
#define SERIAL_LN       TheSerial.printlnf

//--------------------------------------------------
// Tools & Helpers
//--------------------------------------------------
#ifndef ABS_RETURN
#define ABS_RETURN(x,y)         	((x < y) ? (y-x) : (x-y))
#endif

#ifndef MIN_RETURN
#define MIN_RETURN(x,y)         	((x < y) ? (x) : (y))
#endif

#ifndef MAX_RETURN
#define MAX_RETURN(x,y)         	((x < y) ? (y) : (x))
#endif

#ifndef BIT
#define BIT(n)                    ( 1<<(n) )
#endif

#define BITTEST(var,pos)          (((var)>>(pos)) & 0x0001)
#define BITMASK(pos)              (0x0001 << (pos))
#define BITSET(var,pos)           ((var) | BITMASK(pos))
#define BITUNSET(var,pos)         ((var) & (~BITMASK(pos)))
#define _BV(x)                    (1<<(x))

// Create a bitmask of length len.
#define BIT_MASK(len)             ( BIT(len)-1 )
// Create a bitfield mask of length starting at bit 'start'.
#define BF_MASK(start, len)       ( BIT_MASK(len)<<(start) )

// Prepare a bitmask for insertion or combining.
#define BF_PREP(x, start, len)    ( ((x)&BIT_MASK(len)) << (start) )
// Extract a bitfield of length len starting at bit 'start' from y.
#define BF_GET(y, start, len)     ( ((y)>>(start)) & BIT_MASK(len) )
// Insert a new bitfield value x into y.
#define BF_SET(y, x, start, len)  ( y= ((y) &~ BF_MASK(start, len)) | BF_PREP(x, start, len) )

#endif /* xliCommon_h */
