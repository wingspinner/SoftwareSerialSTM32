/*
SoftSerialST32.cpp (based on NewSoftSerial.cpp) - 
Multi-instance software serial library for Arduino/Wiring
-- Compiles for STM32Arduino and AVR Arduino with no modifications
-- Hacks for 72MHz STM32F1 and timing calibrations by Ron Curry
   InSyte Technologies
-- Interrupt-driven receive and other improvements by ladyada
   (http://ladyada.net)
-- Tuning, circular buffer, derivation from class Print/Stream,
   multi-instance support, porting to 8MHz processors,
   various optimizations, PROGMEM delay tables, inverse logic and 
   direct port writing by Mikal Hart (http://www.arduiniana.org)
-- Pin change interrupt macros by Paul Stoffregen (http://www.pjrc.com)
-- 20MHz processor support by Garrett Mace (http://www.macetech.com)
-- ATmega1280/2560 support by Brett Hagman (http://www.roguerobotics.com/)

License
This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

The latest version of this library can always be found at
https://github.com/wingspinner
*/

#ifndef SoftSerialSTM32_h
#define SoftSerialSTM32_h

#include <inttypes.h>
#include <Stream.h>

/******************************************************************************
* Definitions
******************************************************************************/

#define _SS_MAX_RX_BUFF 64 // RX buffer size
#ifndef GCC_VERSION
#define GCC_VERSION (__GNUC__ * 10000 + __GNUC_MINOR__ * 100 + __GNUC_PATCHLEVEL__)
#endif
#define _SSSTM32_VERSION   1.1  // Library Version

class SoftSerialSTM32 : public Stream
{
private:
  // per object data
  uint8_t _transmitPin;
  uint8_t _receivePin;

  #ifndef __STM32F1__

  uint8_t _receiveBitMask;
  volatile uint8_t *_receivePortRegister;
  uint8_t _transmitBitMask;
  volatile uint8_t *_transmitPortRegister;
  volatile uint8_t *_pcint_maskreg;
  uint8_t _pcint_maskvalue;

  // Expressed as 4-cycle delays (must never be 0!)
  uint16_t _rx_delay_centering;
  uint16_t _rx_delay_intrabit;
  uint16_t _rx_delay_stopbit;
  uint16_t _tx_delay;

  #else
  
  // Expressed as 4-cycle delays (must never be 0!)
  uint32_t _rx_delay_centering;
  uint32_t _rx_delay_intrabit;
  uint32_t _rx_delay_stopbit;
  uint32_t _tx_delay;

  #endif
  
  uint16_t _buffer_overflow:1;
  uint16_t _inverse_logic:1;

  // static data
  static char _receive_buffer[_SS_MAX_RX_BUFF]; 
  static volatile uint8_t _receive_buffer_tail;
  static volatile uint8_t _receive_buffer_head;
  static SoftSerialSTM32 *active_object;

  // private methods
  void recv() __attribute__((__always_inline__));
  uint8_t rx_pin_read();
  void tx_pin_write(uint8_t pin_state) __attribute__((__always_inline__));
  void setTX(uint8_t transmitPin);
  void setRX(uint8_t receivePin);
  void setRxIntMsk(bool enable) __attribute__((__always_inline__));

  #ifndef __STM32F1__

  // Return num - sub, or 1 if the result would be < 1
  static uint16_t subtract_cap(uint16_t num, uint16_t sub);

  // private static method for timing
  static inline void tunedDelay(uint16_t delay);

  #else
  
  // private static method for timing
  static inline void tunedDelay(uint32_t delay);

  #endif

public:
  // public methods
  SoftSerialSTM32(uint8_t receivePin, uint8_t transmitPin, bool inverse_logic = false);
  ~SoftSerialSTM32();

  static int      library_version() { return _SSSTM32_VERSION; }  
  void            begin(long speed);
  bool            listen();
  void            end();
  bool            isListening() { return this == active_object; }
  bool            stopListening();
  bool            overflow() { bool ret = _buffer_overflow; if (ret) _buffer_overflow = false; return ret; }
  int             peek();

  virtual size_t  write(uint8_t byte);
  virtual int     read();
  virtual int     available();
  virtual void    flush();
  operator bool() { return true; }

  // for use during debug only
  uint16_t        getRXCentering() { return _rx_delay_centering; }
  void            setRXCentering(uint16_t rxDelay) { _rx_delay_centering = rxDelay; }
  uint16_t        getRXIntrabit() { return _rx_delay_intrabit; }
  void            setRXIntrabit(uint16_t rxDelay) { _rx_delay_intrabit = rxDelay; }
  uint16_t        getRXStopbit() { return _rx_delay_stopbit; }
  void            setRXStopbit(uint16_t rxDelay) { _rx_delay_stopbit = rxDelay; }
  uint16_t        getTXDelay() { return _tx_delay; }
  void            setTXDelay(uint16_t txDelay) { _tx_delay = txDelay; }
  
  
  using Print::write;

  // public only for easy access by interrupt handlers
  static inline void handle_interrupt() __attribute__((__always_inline__));
};

// Arduino 0012 workaround
#undef int
#undef char
#undef long
#undef byte
#undef float
#undef abs
#undef round

#endif

