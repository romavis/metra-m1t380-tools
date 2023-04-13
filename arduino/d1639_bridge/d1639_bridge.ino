// Tx channel
const int pin_data_o = A2;
const int pin_rdy_i = A1;
// Rx channel
const int pin_data_i = A0;
const int pin_rdy_o = A3;
// LED
const int pin_led = 13;

// Timing constants
#define T_BIT_US (100u)
#define T_1HBIT_US (150u)


void setup() {
  // Init GPIO
  pinMode(pin_data_o, OUTPUT);
  pinMode(pin_rdy_i, INPUT_PULLUP);
  pinMode(pin_data_i, INPUT_PULLUP);
  pinMode(pin_rdy_o, OUTPUT);
  pinMode(pin_led, OUTPUT);
  // Serial
  Serial.begin(115200);
}

//
// TX FSM
//

enum {
  FSM_TX_IDLE,
  FSM_TX_WAIT_RDY,
  FSM_TX_XFER,
};

uint8_t tx_state = FSM_TX_IDLE;
uint8_t tx_data;
uint8_t tx_biti;
uint32_t tx_tick;

void fsm_tx(uint32_t t) {
  // Next variables
  uint8_t next_state;
  uint8_t next_data;
  uint8_t next_biti;
  uint32_t next_tick;
  
  // 'Combinational' logic
  bool tock = ((int32_t)(t - tx_tick)) >= 0;
  bool rdy = (digitalRead(pin_rdy_i) == LOW);
  uint8_t state = tx_state;

  // State change
  next_state = state;
  if (state == FSM_TX_IDLE) {
    if (Serial.available()) {
      next_state = FSM_TX_WAIT_RDY;
    }
  } else if (state == FSM_TX_WAIT_RDY) {
    if (rdy) {
      next_state = FSM_TX_XFER;
    }
  } else if (state == FSM_TX_XFER) {
    if (tx_biti == 0 && tock) {
      next_state = FSM_TX_IDLE;
    }
  }

  bool state_change = (next_state != state);

  // Data
  next_data = tx_data;
  if (state == FSM_TX_IDLE && state_change) {
    next_data = Serial.read();
  }

  // Bit counter
  next_biti = tx_biti;
  if (state_change && next_state == FSM_TX_XFER) {
    next_biti = 8;
  } else if (state == FSM_TX_XFER && tock) {
    next_biti--;
  }

  // Timing
  if (state_change && next_state == FSM_TX_XFER) {
    next_tick = t + T_BIT_US;
  } else if (state == FSM_TX_XFER && tock) {
    next_tick += T_BIT_US;
  }

  // Save registered logic states
  tx_state = next_state;
  tx_data = next_data;
  tx_biti = next_biti;
  tx_tick = next_tick;
  
  // Compute combinatorial outputs: DATA_O
  if (tx_state == FSM_TX_IDLE) {
    digitalWrite(pin_data_o, HIGH);
  } else if (tx_state == FSM_TX_WAIT_RDY) {
    digitalWrite(pin_data_o, LOW);
  } else if (tx_state == FSM_TX_XFER) {
    bool v = HIGH;
    if (tx_biti <= 7) {
      v = (tx_data >> tx_biti) & 1;
    }
    digitalWrite(pin_data_o, v);
  }
}

//
// RX FSM
//

enum {
  FSM_RX_IDLE,
  FSM_RX_WAIT_START,
  FSM_RX_XFER,
};

uint8_t rx_state = FSM_RX_IDLE;
uint8_t rx_data;
uint8_t rx_biti;
uint32_t rx_tick;

void fsm_rx(uint32_t t) {
  // Next variables
  uint8_t next_state;
  uint8_t next_data;
  uint8_t next_biti;
  uint32_t next_tick;
  
  // 'Combinational' logic
  bool tock = ((int32_t)(t - rx_tick)) >= 0;
  bool di = digitalRead(pin_data_i);
  uint8_t state = rx_state;

  // State change
  next_state = state;
  if (state == FSM_RX_IDLE) {
    if (Serial.availableForWrite() && (di == LOW)) {
      next_state = FSM_RX_WAIT_START;
    }
  } else if (state == FSM_RX_WAIT_START) {
    if (di == HIGH) {
      next_state = FSM_RX_XFER;
    }
  } else if (state == FSM_RX_XFER) {
    // 1 extra bit interval delay before returning to IDLE
    if (rx_biti == 255 && tock) {
      next_state = FSM_RX_IDLE;
    }
  }

  bool state_change = (next_state != state);

  // Data
  next_data = rx_data;
  if (state == FSM_RX_XFER && tock) {
    next_data = (next_data << 1) | (di ? 1 : 0);
  }

  // Bit counter
  next_biti = rx_biti;
  if (state_change && next_state == FSM_RX_XFER) {
    next_biti = 7;
  } else if (state == FSM_RX_XFER && tock) {
    next_biti--;
  }

  // Timing
  if (state_change && next_state == FSM_RX_XFER) {
    next_tick = t + T_1HBIT_US;
  } else if (state == FSM_RX_XFER && tock) {
    next_tick += T_BIT_US;
  }

  // Send byte to host after transfer ends
  if (state_change && state == FSM_RX_XFER) {
    if(Serial.availableForWrite()) {
      Serial.write(rx_data);  // must not block
    } // else signal error?
  }

  // Save registered logic states
  rx_state = next_state;
  rx_data = next_data;
  rx_biti = next_biti;
  rx_tick = next_tick;
  
  // Compute combinatorial outputs: RDY_O
  if (rx_state == FSM_RX_WAIT_START) {
    digitalWrite(pin_rdy_o, LOW);
  } else {
    digitalWrite(pin_rdy_o, HIGH);
  }
}

bool ll;


void loop() {
//  uint32_t t = micros();

  fsm_tx(micros());
  fsm_rx(micros());

  { 
    static int i;
    i++;
    if (i >= 20000) {
      ll = !ll;
      i = 0;
    }
  }

  // Activate LED if either TX or RX is not IDLE  
//  digitalWrite(pin_led, (rx_state != FSM_RX_IDLE) || (tx_state != FSM_TX_IDLE));
  digitalWrite(pin_led, ll);
}
