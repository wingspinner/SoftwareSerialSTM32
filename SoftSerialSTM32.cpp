/**********************************************************************
SoftSerialSTM32.cpp (based on NewSoftSerial.cpp) - 
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

The latest version of this library can always be found at
https://github.com/wingspinner

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


************************************************************************/

// When set, _DEBUG co-opts pins 5 and 6 for debugging with an
// oscilloscope or logic analyzer.  Beware: it also slightly modifies
// the bit times, so don't rely on it too much at high baud rates
#define _DEBUG 0
#define _DEBUG_PIN1 5
#define _DEBUG_PIN2 6

// 
// Includes
// 
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <Arduino.h>
#include "SoftSerialSTM32.h"

#ifndef __STM32F1__

#include <util/delay_basic.h>

#endif


//
// Lookup table
//
typedef struct _DELAY_TABLE
{
  long baud;
  uint32_t rx_delay_centering;
  uint32_t rx_delay_intrabit;
  uint32_t rx_delay_stopbit;
  uint32_t tx_delay;
} DELAY_TABLE;


#if defined (__STM32F1__)

  #if F_CPU == 72000000
    static const DELAY_TABLE PROGMEM table[] = 
    {
      //  baud    rxcenter   rxintra    rxstop       tx
      { 115200,       6,         52,       15,        54,  },
      {  57600,      28,        112,       25,       117,  },
      {  38400,      60,        167,       25,       179,  },
      {  31250,      92,        223,       25,       223,  },
      {  28800,      91,        237,       25,       242,  },
      {  19200,     156,        352,       25,       367,  },
      {  14400,     232,        470,       25,       492,  },
      {   9600,     353,        718,       25,       741,  },
      {   4800,     808,       1476,       25,      1492,  },
      {   2400,    1516,       2992,       25,      2993,  },
      {   1200,    2872,       5993,       25,      5993,  },
      {    300,   11488,      23976,       25,     23976,  },
    };
    
    const int XMIT_START_ADJUSTMENT = 5;
  
  #else
  
    #error This version of SoftwareSerial supports only 72MHz STM32F1 processors or all speeds of ATMega processors
  
  #endif

#endif


//
// Statics
//
SoftSerialSTM32 *SoftSerialSTM32::active_object = 0;
char SoftSerialSTM32::_receive_buffer[_SS_MAX_RX_BUFF]; 
volatile uint8_t SoftSerialSTM32::_receive_buffer_tail = 0;
volatile uint8_t SoftSerialSTM32::_receive_buffer_head = 0;

//
// Debugging
//
// This function generates a brief pulse
// for debugging or measuring on an oscilloscope.
inline void DebugPulse(uint8_t pin, uint8_t count)
{

#if _DEBUG
  #if defined (__STM32F1__)

    // DebugPlus has ~ .4us impact on timing which affects
    // the timing of 38400 through 115200 baud requiring
    // adjustments to the delay table to compensate.
    while (count--)
    {
      digitalWrite(pin, 1);
      digitalWrite(pin, 0);
    }

  #else

    volatile uint8_t *pport = portOutputRegister(digitalPinToPort(pin));
  
    uint8_t val = *pport;
    while (count--)
    {
      *pport = val | digitalPinToBitMask(pin);
      *pport = val;
    }

  #endif
#endif

}


//
// Private methods
//


/* static */ 
#if defined (__STM32F1__)

// delay overhead is ~+1.487usec. one digit = ~137ns.
// Therefore, total delay is ~ (delay * 0.137us) + 1.487us
inline void SoftSerialSTM32::tunedDelay(uint32_t delay) { 
  static uint32_t i;
  for (i = 0; i < delay; i++) {
    asm volatile(
      "nop \n\t"
     ::);
  }
  
  #else

inline void SoftSerialSTM32::tunedDelay(uint16_t delay) { 
    _delay_loop_2(delay); // 16.375 us @ 16mhz

  #endif

}


// This function sets the current object as the "listening"
// one and returns true if it replaces another 
bool SoftSerialSTM32::listen()
{
  if (!_rx_delay_stopbit)
    return false;

  if (active_object != this)
  {
    if (active_object)
      active_object->stopListening();

    _buffer_overflow = false;
    _receive_buffer_head = _receive_buffer_tail = 0;
    active_object = this;

    #if defined (__STM32F1__)

    attachInterrupt(_receivePin, SoftSerialSTM32::handle_interrupt, FALLING);

    #else

      setRxIntMsk(true);

    #endif

    return true;
  }

  return false;
}


// Stop listening. Returns true if we were actually listening.
bool SoftSerialSTM32::stopListening()
{
  if (active_object == this)
  {

    #if defined (__STM32F1__)

      detachInterrupt(_receivePin);

    #else

      setRxIntMsk(false);

    #endif

    active_object = NULL;
    return true;
  }
  return false;
}


//
// The receive routine called by the interrupt handler
//
void SoftSerialSTM32::recv()
{

  #ifndef __STM32F1__
  #if GCC_VERSION < 40302
  // Work-around for avr-gcc 4.3.0 OSX version bug
  // Preserve the registers that the compiler misses
  // (courtesy of Arduino forum user *etracer*)
    asm volatile(
      "push r18 \n\t"
      "push r19 \n\t"
      "push r20 \n\t"
      "push r21 \n\t"
      "push r22 \n\t"
      "push r23 \n\t"
      "push r26 \n\t"
      "push r27 \n\t"
      ::);
  
  #endif 
  #endif 

  uint8_t d = 0;

  // If RX line is high, then we don't see any start bit
  // so interrupt is probably not for us
  if (_inverse_logic ? rx_pin_read() : !rx_pin_read())
  {
    // Disable further interrupts during reception, this prevents
    // triggering another interrupt directly after we return, which can
    // cause problems at higher baudrates.

    #if defined (__STM32F1__)

//      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    setRxIntMsk(false);

    #else

    setRxIntMsk(false);
        
    #endif

      // Wait approximately 1/2 of a bit width to "center" the sample
      tunedDelay(_rx_delay_centering);
      DebugPulse(_DEBUG_PIN2, 1);
  
      // Read each of the 8 bits
      for (uint8_t i=8; i > 0; --i)
      {
        tunedDelay(_rx_delay_intrabit);       
        DebugPulse(_DEBUG_PIN2, 1);
        
        d >>= 1;
        if (rx_pin_read())
          d |= 0x80;
      }
  
      if (_inverse_logic)
        d = ~d;
  
      // if buffer full, set the overflow flag and return
      uint8_t next = (_receive_buffer_tail + 1) % _SS_MAX_RX_BUFF;
      if (next != _receive_buffer_head)
      {
        // save new data in buffer: tail points to where byte goes
        _receive_buffer[_receive_buffer_tail] = d; // save new byte
        _receive_buffer_tail = next;
      } 
      else 
      {
        DebugPulse(_DEBUG_PIN1, 1);
        _buffer_overflow = true;
      }
  
      // skip the stop bit
      tunedDelay(_rx_delay_stopbit);
      DebugPulse(_DEBUG_PIN2, 1);

    // Re-enable interrupts when we're sure to be inside the stop bit
    #if defined (__STM32F1__)

//      }
    setRxIntMsk(true);

    #else

    setRxIntMsk(true);
        
    #endif

  }
  
#ifndef __STM32F1__
#if GCC_VERSION < 40302

// Work-around for avr-gcc 4.3.0 OSX version bug
// Restore the registers that the compiler misses
  asm volatile(
    "pop r27 \n\t"
    "pop r26 \n\t"
    "pop r23 \n\t"
    "pop r22 \n\t"
    "pop r21 \n\t"
    "pop r20 \n\t"
    "pop r19 \n\t"
    "pop r18 \n\t"
    ::);

#endif
#endif

}


uint8_t SoftSerialSTM32::rx_pin_read()
{
  #if defined (__STM32F1__)
    return digitalRead(_receivePin);
  #else
    return *_receivePortRegister & _receiveBitMask;
  #endif
}


//
// Interrupt handling
//

/* static */
inline void SoftSerialSTM32::handle_interrupt()
{
  if (active_object)
  {
    active_object->recv();
  }
}


#ifndef __STM32F1__

#if defined(PCINT0_vect)

ISR(PCINT0_vect)
{
  SoftSerialSTM32::handle_interrupt();
}
#endif

#if defined(PCINT1_vect)

ISR(PCINT1_vect, ISR_ALIASOF(PCINT0_vect));

#endif

#if defined(PCINT2_vect)

ISR(PCINT2_vect, ISR_ALIASOF(PCINT0_vect));

#endif

#if defined(PCINT3_vect)

ISR(PCINT3_vect, ISR_ALIASOF(PCINT0_vect));

#endif

#endif


//
// Constructor
//
SoftSerialSTM32::SoftSerialSTM32(uint8_t receivePin, uint8_t transmitPin, bool inverse_logic /* = false */) : 
  _rx_delay_centering(0),
  _rx_delay_intrabit(0),
  _rx_delay_stopbit(0),
  _tx_delay(0),
  _buffer_overflow(false),
  _inverse_logic(inverse_logic)
{
  setTX(transmitPin);
  setRX(receivePin);
}


//
// Destructor
//
SoftSerialSTM32::~SoftSerialSTM32()
{
  end();
}


void SoftSerialSTM32::setTX(uint8_t tx)
{
  // First write, then set output. If we do this the other way around,
  // the pin would be output low for a short while before switching to
  // output hihg. Now, it is input with pullup for a short while, which
  // is fine. With inverse logic, either order is fine.
  digitalWrite(tx, _inverse_logic ? LOW : HIGH);
  pinMode(tx, OUTPUT);
  _transmitPin = tx;

  #ifndef __STM32F1__

    _transmitBitMask = digitalPinToBitMask(tx);
    uint8_t port = digitalPinToPort(tx);
    _transmitPortRegister = portOutputRegister(port);

  #endif
}


void SoftSerialSTM32::setRX(uint8_t rx)
{
  pinMode(rx, INPUT);
  if (!_inverse_logic)
    digitalWrite(rx, HIGH);  // pullup for normal logic!
  _receivePin = rx;

  #if defined (__STM32F1__)

    attachInterrupt(_receivePin, SoftSerialSTM32::handle_interrupt, FALLING);

  #else

    _receiveBitMask = digitalPinToBitMask(rx);
    uint8_t port = digitalPinToPort(rx);
    _receivePortRegister = portInputRegister(port);

  #endif
}

#ifndef __STM32F1__

  uint16_t SoftSerialSTM32::subtract_cap(uint16_t num, uint16_t sub) {
    if (num > sub)
      return num - sub;
    else
      return 1;
  }

#endif


//
// Public methods
//

void SoftSerialSTM32::begin(long speed)
{
  _rx_delay_centering = _rx_delay_intrabit = _rx_delay_stopbit = _tx_delay = 0;

  // Precalculate the various delays, in number of 4-cycle delays

  #if defined (__STM32F1__)

    for (unsigned i=0; i<sizeof(table)/sizeof(table[0]); ++i)
    {
      long baud = table[i].baud;
      if (baud == speed)
      {
        _rx_delay_centering = table[i].rx_delay_centering;
        _rx_delay_intrabit = table[i].rx_delay_intrabit;
        _rx_delay_stopbit = table[i].rx_delay_stopbit;
        _tx_delay = table[i].tx_delay;
        break;
      }
    }

    attachInterrupt(_receivePin, SoftSerialSTM32::handle_interrupt, FALLING);

  #else

    uint16_t bit_delay = (F_CPU / speed) / 4;
  
  // 12 (gcc 4.8.2) or 13 (gcc 4.3.2) cycles from start bit to first bit,
  // 15 (gcc 4.8.2) or 16 (gcc 4.3.2) cycles between bits,
  // 12 (gcc 4.8.2) or 14 (gcc 4.3.2) cycles from last bit to stop bit
  // These are all close enough to just use 15 cycles, since the inter-bit
  // timings are the most critical (deviations stack 8 times)
  _tx_delay = subtract_cap(bit_delay, 15 / 4);

  // Only setup rx when we have a valid PCINT for this pin
  if (digitalPinToPCICR(_receivePin)) {

    #if GCC_VERSION > 40800

    // Timings counted from gcc 4.8.2 output. This works up to 115200 on
    // 16Mhz and 57600 on 8Mhz.
    //
    // When the start bit occurs, there are 3 or 4 cycles before the
    // interrupt flag is set, 4 cycles before the PC is set to the right
    // interrupt vector address and the old PC is pushed on the stack,
    // and then 75 cycles of instructions (including the RJMP in the
    // ISR vector table) until the first delay. After the delay, there
    // are 17 more cycles until the pin value is read (excluding the
    // delay in the loop).
    // We want to have a total delay of 1.5 bit time. Inside the loop,
    // we already wait for 1 bit time - 23 cycles, so here we wait for
    // 0.5 bit time - (71 + 18 - 22) cycles.
    _rx_delay_centering = subtract_cap(bit_delay / 2, (4 + 4 + 75 + 17 - 23) / 4);

    // There are 23 cycles in each loop iteration (excluding the delay)
    _rx_delay_intrabit = subtract_cap(bit_delay, 23 / 4);

    // There are 37 cycles from the last bit read to the start of
    // stopbit delay and 11 cycles from the delay until the interrupt
    // mask is enabled again (which _must_ happen during the stopbit).
    // This delay aims at 3/4 of a bit time, meaning the end of the
    // delay will be at 1/4th of the stopbit. This allows some extra
    // time for ISR cleanup, which makes 115200 baud at 16Mhz work more
    // reliably
    _rx_delay_stopbit = subtract_cap(bit_delay * 3 / 4, (37 + 11) / 4);

    #else // Timings counted from gcc 4.3.2 output

    // Note that this code is a _lot_ slower, mostly due to bad register
    // allocation choices of gcc. This works up to 57600 on 16Mhz and
    // 38400 on 8Mhz.
    _rx_delay_centering = subtract_cap(bit_delay / 2, (4 + 4 + 97 + 29 - 11) / 4);
    _rx_delay_intrabit = subtract_cap(bit_delay, 11 / 4);
    _rx_delay_stopbit = subtract_cap(bit_delay * 3 / 4, (44 + 17) / 4);

    #endif

      // Enable the PCINT for the entire port here, but never disable it
      // (others might also need it, so we disable the interrupt by using
      // the per-pin PCMSK register).
      *digitalPinToPCICR(_receivePin) |= _BV(digitalPinToPCICRbit(_receivePin));
      // Precalculate the pcint mask register and value, so setRxIntMask
      // can be used inside the ISR without costing too much time.
      _pcint_maskreg = digitalPinToPCMSK(_receivePin);
      _pcint_maskvalue = _BV(digitalPinToPCMSKbit(_receivePin));
    
    tunedDelay(_tx_delay); // if we were low this establishes the end
  
  }
  
  #endif // defined (__STM32F1__)

#if _DEBUG
  pinMode(_DEBUG_PIN1, OUTPUT);
  pinMode(_DEBUG_PIN2, OUTPUT);
#endif

  listen();
}


void SoftSerialSTM32::setRxIntMsk(bool enable)
{

  #if defined (__STM32F1__)

    if (enable)
      attachInterrupt(_receivePin, SoftSerialSTM32::handle_interrupt, FALLING);
    else
      detachInterrupt(_receivePin);

  #else

    if (enable)
      *_pcint_maskreg |= _pcint_maskvalue;
    else
      *_pcint_maskreg &= ~_pcint_maskvalue;

  #endif
  
}


void SoftSerialSTM32::end()
{
  stopListening();
}


// Read data from buffer
int SoftSerialSTM32::read()
{
  if (!isListening())
    return -1;

  // Empty buffer?
  if (_receive_buffer_head == _receive_buffer_tail)
    return -1;

  // Read from "head"
  uint8_t d = _receive_buffer[_receive_buffer_head]; // grab next byte
  _receive_buffer_head = (_receive_buffer_head + 1) % _SS_MAX_RX_BUFF;
  return d;
}


int SoftSerialSTM32::available()
{
  if (!isListening())
    return 0;

  return (_receive_buffer_tail + _SS_MAX_RX_BUFF - _receive_buffer_head) % _SS_MAX_RX_BUFF;
}


size_t SoftSerialSTM32::write(uint8_t b)
{
  if (_tx_delay == 0) {
    setWriteError();
    return 0;
  }

  // By declaring these as local variables, the compiler will put them
  // in registers _before_ disabling interrupts and entering the
  // critical timing sections below, which makes it a lot easier to
  // verify the cycle timings

  #if defined (__STM32F1__)
    bool inv = _inverse_logic;
    uint16_t delay = _tx_delay;
  #else  
    volatile uint8_t *reg = _transmitPortRegister;
    uint8_t reg_mask = _transmitBitMask;
    uint8_t inv_mask = ~_transmitBitMask;
    uint8_t oldSREG = SREG;
    bool inv = _inverse_logic;
    uint16_t delay = _tx_delay;
  #endif
  
  if (inv)
    b = ~b;

  #if defined (__STM32F1__)

  noInterrupts();

  #else

  cli();  // turn off interrupts for a clean txmit

  #endif
  
  // Write the start bit
  if (inv)
  #if defined (__STM32F1__)
    digitalWrite(_transmitPin, HIGH);
  else
    digitalWrite(_transmitPin, LOW);
  #else 
    *reg |= reg_mask;
  else
    *reg &= inv_mask;
  #endif

  tunedDelay(delay);

  // Write each of the 8 bits
  for (uint8_t i = 8; i > 0; --i)
  {
    if (b & 1) // choose bit
    #if defined (__STM32F1__)
      digitalWrite(_transmitPin, HIGH);
    else
      digitalWrite(_transmitPin, LOW);
    #else 
      *reg |= reg_mask;
    else
      *reg &= inv_mask;
    #endif

    tunedDelay(delay);
    b >>= 1;
  }

  // restore pin to natural state
  #if defined (__STM32F1__)

    if (inv)
      digitalWrite(_transmitPin, LOW);
    else
      digitalWrite(_transmitPin, HIGH);

    interrupts(); 
  
  #else 

    if (inv)
      *reg &= inv_mask;
    else
      *reg |= reg_mask;

    SREG = oldSREG; // turn interrupts back on

  #endif
    
  tunedDelay(_tx_delay);
  
  return 1;
}


void SoftSerialSTM32::flush()
{
  if (!isListening())
    return;

  #if defined (__STM32F1__)
    noInterrupts();
    _receive_buffer_head = _receive_buffer_tail = 0;
    interrupts();
  #else  
    uint8_t oldSREG = SREG;
    cli();
    _receive_buffer_head = _receive_buffer_tail = 0;
    SREG = oldSREG;
  #endif
}


int SoftSerialSTM32::peek()
{
  if (!isListening())
    return -1;

  // Empty buffer?
  if (_receive_buffer_head == _receive_buffer_tail)
    return -1;

  // Read from "head"
  return _receive_buffer[_receive_buffer_head];
}

