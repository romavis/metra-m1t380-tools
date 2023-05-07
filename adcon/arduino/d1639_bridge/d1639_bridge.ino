/**
 * Arduino Leonardo as a USB CDC -to- Metra M1T380 MHB8748 protocol converter
 * 
 * Copyright (c) 2023 Roman Dobrodii
 * 
 * Licensed under MIT License. See LICENSE for more details.
 * 
 * --------------------------------------------------------------------------
 * 
 * Side A - USB-CDC port (implemented by Arduino's Serial class)
 * Side B - custom 4-wire MHB8748 serial protocol 
 * 
 * 
 * Arduino Leonardo pinout:
 * 
 *  io4 - DATA_I  (PD4 - ICP1)
 *  io5 - RDY_I   (PC6)
 *  io6 - DATA_O  (PD7)
 *  io7 - RDY_O   (PE6)
 *  io13 - LED
 * 
 * DATA_I, DATA_O, RDY_I, RDY_O are connected directly to K1 connector on
 * the M1T380's D1639 board via a 30-50cm ribbon cable as follows.
 *  >> pin numbering of K1 socket follows the usual DIP16 scheme
 *  >> pins 1-8 are next to the PCB edge, pin 1 is the closest one to the
 *  >> mounting screw in the very corner of the PCB.
 * 
 *  Pin 1 - DATA_I (input to Arduino, output from D1639)
 *  Pin 2 - RDY_I  (input to Arduino, output from D1639)
 *  Pin 6 - DATA_O (output from Arduino, input to D1639)
 *  Pin 7 - RDY_O  (output from Arduino, input to D1639)
 *  Pin 9 - GND    (connect to Arduino GND)
 *  Pin 16 - +5 in (connect to Arduino +5V) 
 * 
 * 
 * WARNING:
 * As we use register-level programming to configure timer for Input
 * Capture and to access GPIO quickly, this code is **NOT** runnable
 * on other Arduino boards without modification!
 * 
 * It will work as-is **ONLY** on such Arduino Leonardo board:
 * https://www.arduino.cc/en/uploads/Main/arduino-leonardo-schematic_3b.pdf
 * 
 * Running it on Arduino Uno, even after modyfing the code, is **NOT**
 * recommended as it has no serial port flow control (which is available on
 * Leonardo thanks to USB-CDC), and MHB8748 really needs flow control as in
 * many cases it is not ready to receive data. No flow control on PC side
 * means very high chance of data loss with this code!
 * 
 * IOW, if you want to run this, consider using a Leonardo board. Seriously.
 */

#include <avr/io.h>
#include <avr/interrupt.h>

#define P_BIT_DATA_I    (4u)
#define P_BIT_RDY_I     (6u)
#define P_BIT_DATA_O    (7u)
#define P_BIT_RDY_O     (6u)

#define P_DDR_DATA_I    DDRD, P_BIT_DATA_I
#define P_DDR_RDY_I     DDRC, P_BIT_RDY_I
#define P_DDR_DATA_O    DDRD, P_BIT_DATA_O
#define P_DDR_RDY_O     DDRE, P_BIT_RDY_O

#define P_PIN_DATA_I    PIND, P_BIT_DATA_I
#define P_PIN_RDY_I     PINC, P_BIT_RDY_I

#define P_PORT_DATA_I   PORTD, P_BIT_DATA_I
#define P_PORT_RDY_I    PORTC, P_BIT_RDY_I
#define P_PORT_DATA_O   PORTD, P_BIT_DATA_O
#define P_PORT_RDY_O    PORTE, P_BIT_RDY_O

const uint8_t pin_led = 13;

/**
 * Timing constants
 */
#define T_BIT_US        (100u)
#define T_1BIT_US       (150u)

/**
 * Timer 1 timing conversions
 */
#define T1_OSC_DIV      (8u)
#define T1_TICKS(us)    ((u16) ((u32)F_CPU / 1000000 * (us) / T1_OSC_DIV))

const u16 t1_ticks_bit = T1_TICKS(T_BIT_US);
const u16 t1_ticks_1bit = T1_TICKS(T_1BIT_US);
const u16 t1_ticks_tx_dly = T1_TICKS(50);


/**
 * RAII IRQ lock implementation for critical sections
 */
class LockIRQ
{
 u8 _sreg;
public:
  LockIRQ() : _sreg(SREG)
  {
    cli();
  }
  ~LockIRQ()
  {
    SREG = _sreg;
  }
};

/**
 * Helper bit ops
 */
#define BITCLR_(v, n) \
  do { \
    (v) = (v) & ~(1u << (n)); \
  } while(0)

#define BITSET_(v, n) \
  do { \
    (v) = (v) | (1u << (n)); \
  } while(0)

#define BITWR_(v, n, x) \
  do { \
    (v) = ((v) & ~(1u << (n))) | (((x) ? 1 : 0) << (n)); \
  } while(0)

#define BITRD_(v, n) (((v) >> (n)) & 1)

#define BITCLR(...) BITCLR_(__VA_ARGS__)
#define BITSET(...) BITSET_(__VA_ARGS__)
#define BITWR(...) BITWR_(__VA_ARGS__)
#define BITRD(...) BITRD_(__VA_ARGS__)

#define BARRIER() do { asm volatile("":::"memory"); } while(0)

/**
 * Tx state machine
 */
enum {
  TX_IDLE,
  TX_WAIT_RDY,
  TX_XFER,
};

u8 tx_state = TX_IDLE;
u8 tx_busy;
u8 tx_data;
u8 tx_biti;

/**
 * Tx: FSM logic
 * 
 * Called from main(), this code is not time-critical
 */
void fsm_tx() {
  if (tx_state == TX_IDLE) {
    if (Serial.available()) {
      tx_state = TX_WAIT_RDY;
      tx_data = Serial.read();
      // set DATA_O = 0
      BITCLR(P_PORT_DATA_O);
    }
  } else if (tx_state == TX_WAIT_RDY) {
    if (BITRD(P_PIN_RDY_I) == 0) {
      tx_state = TX_XFER;
      // Kickoff timer-driven machinery in 200 us (empirical "safe" value)
      tx_biti = 8;
      tx_busy = 1;
      {
        LockIRQ lock_;
        OCR1A = TCNT1 + t1_ticks_tx_dly;
        BITSET(TIFR1, OCF1A);
        BARRIER();
        BITSET(TIMSK1, OCIE1A); 
      }
    }
  } else {
    if (!tx_busy) {
      // Timer finished its job, disabled its interrupt, set DATA_O to 1
      // we just return to IDLE
      tx_state = TX_IDLE;
    }
  }
}

/**
 * Tx: Timer 1 Compare A IRQ handler
 * 
 * Fires at scheduled intervals to write data bit value to DATA_O
 */
ISR(TIMER1_COMPA_vect) {
  /*
   * tx_biti:
   *  - 8 in the _beginning_ of a start bit
   *  - 7 in the _beginning_ of 7th bit (MSB)
   *  ...
   *  - 0 in the _beginning_ of 0th bit (LSB)
   *    and we still must keep on the line for 100us!
   *  - 255 after 0th bit transmission has been finished
   *  - 254 after guard bit transmission has been finished
   *  
   *  We transmit guard bit because otherwise, if some unexpected
   *  latency gets mixed in and last bit gets prolonged, the
   *  receiver (MHB8748) may treat our last bit as a beginning of
   *  a new frame. To avoid this, we explicitly transmit '1' - a
   *  guard bit - after LSB.
   */
  if (tx_biti == 254) {
    // end of transfer -> set DATA_O to 1, disable interrupt
    BITSET(P_PORT_DATA_O);
    BITCLR(TIMSK1, OCIE1A);
    BARRIER();
    tx_busy = 0;
  } else {
    if (tx_biti <= 7) {
      // transmit data bit
      BITWR(P_PORT_DATA_O, tx_data & 0x80);
      tx_data <<= 1;
    } else {
      // transmit start or guard bit
      BITSET(P_PORT_DATA_O);
    }
    // schedule next interrupt
    OCR1A += t1_ticks_bit;
    tx_biti--;
  }
}


/**
 * Rx state machine
 */
enum {
  RX_IDLE,
  RX_XFER,
};

u8 rx_state = RX_IDLE;
u8 rx_busy;
u8 rx_data;
u8 rx_biti;

/**
 * Rx: FSM logic
 * 
 * Called from main(), this code is not time-critical
 */
void fsm_rx() {
  if (rx_state == RX_IDLE) {
    if (BITRD(P_PIN_DATA_I) == 0) {
      rx_state = RX_XFER;
      rx_busy = 1;
      // Device has data for us -> enable DATA_I Input Capture IRQ
      {
        LockIRQ lock_;
        BITSET(TIFR1, ICF1);
        BARRIER();
        BITSET(TIMSK1, ICIE1);
        // Signal RDY and expect Input Capture interrupt
        BITCLR(P_PORT_RDY_O);
      }
    }
  } else {
    if (!rx_busy) {
      // Timer finished its job, disabled its interrupt, removed RDY,
      // so we send data to host and return to idle
      if (Serial.availableForWrite()) {
        BARRIER();
        Serial.write(rx_data);
        rx_state = RX_IDLE;
      }
    }
  }
}

/**
 * Rx - Timer 1 Input Capture handler
 * 
 * Fires when ICP1 (DATA_I) detects rising edge of a start bit
 */
ISR(TIMER1_CAPT_vect) {
  // Disable IC interrupt, clear RDY signal
  // schedule and enable OC-B interrupt
  BITCLR(TIMSK1, ICIE1);
  BITSET(P_PORT_RDY_O);
  OCR1B = ICR1 + t1_ticks_1bit;
  BARRIER();
  BITSET(TIFR1, OCF1B);
  BITSET(TIMSK1, OCIE1B);
  rx_biti = 7;
  rx_data = 0;
}

/**
 * Rx - Timer 1 Compare B handler
 * 
 * Fires at scheduled intervals to read data bit value from DATA_I
 */
ISR(TIMER1_COMPB_vect) {
  /*
   * NOTE: we can't just bail out after we've received last bit (LSB)
   * that is because LSB value will still be driven on the data line for
   * ~50us and that is enough to trigger our Rx state machine into
   * perceiving it as a beginning of a new frame. To avoid that, we
   * "receive" yet another bit after LSB and ignore it.
   */
  if (rx_biti == 255) {
    // end of transfer -> disable interrupt
    BITCLR(TIMSK1, OCIE1B);
    BARRIER();
    rx_busy = 0;
  } else {
    if (rx_biti <= 7) {
      // Read data bit
      rx_data = (rx_data << 1) | BITRD(P_PIN_DATA_I);
    }
    // schedule next interrupt
    OCR1B += t1_ticks_bit;
    rx_biti--;
  }
}

/*
 * setup()
 */
void setup() {
  // GPIO
  BITWR(P_DDR_DATA_O, 0);
  BITWR(P_DDR_RDY_O, 0);
  BITWR(P_DDR_DATA_I, 0);
  BITWR(P_DDR_RDY_I, 0);
  //
  BITWR(P_PORT_DATA_I, 1);
  BITWR(P_PORT_RDY_I, 1);
  BITWR(P_PORT_DATA_O, 1);
  BITWR(P_PORT_RDY_O, 1);
  //
  BITWR(P_DDR_DATA_O, 1);
  BITWR(P_DDR_RDY_O, 1);

  pinMode(pin_led, OUTPUT);

  // Timer1 - f=f_osc/8, Input Compare on posedge, counter starts
  TCCR1A = 0;
  TCCR1B = (1<<ICES1) | (1<<CS11);
  
  // Serial
  Serial.begin(115200);

  // Enable interrupts
  sei();
}

/**
 * main() loop
 */
void loop() {
  BARRIER();
  fsm_tx();
  fsm_rx();

  // Activate LED if either TX or RX is not IDLE
  digitalWrite(pin_led, rx_busy || tx_busy);
}
