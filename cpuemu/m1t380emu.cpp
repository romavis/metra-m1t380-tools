/**
 * @file m1t380emu.cpp
 * @author Roman Dobrodii
 *
 * Metra M1T380 system-level emulator, based on amazing z80 emulator library by
 * Ivan Kosarev: https://github.com/kosarev/z80
 *
 * @copyright Licensed under MIT license. See LICENSE for details
 */

// Change this to TRACE if you want to enable TRACE logs at compile time
// Beware, that will print A LOT of potentially useless information
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_DEBUG

#include <fmt/core.h>
#include <spdlog/pattern_formatter.h>
#include <spdlog/spdlog.h>

#include <array>
#include <cassert>
#include <cstdint>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <queue>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "argparse/argparse.hpp"
#include "z80.h"

using namespace std;
using namespace z80;

struct M1T380 {
  static constexpr size_t ROM_SIZE = 8192;
  static constexpr size_t RAM_SIZE = 1024;
  static constexpr size_t CALRAM_SIZE = 256;
  static constexpr size_t INT_PERIOD = 4096;  // cycles
  static constexpr double CYC_PER_SEC = 2048000;

  static int32_t sec2cyc(double sec) {
    sec *= CYC_PER_SEC;
    int32_t x = (int32_t)sec;
    if (abs((double)x - sec) > 1)
      throw domain_error(fmt::format(
          "ill-formed time value '{} sec', perhaps it is too large"));
    return x;
  }

  /**
   * @brief Class that implements wallclock time scale and allows scheduling
   * work asynchronously
   */
  class Clock {
   public:
    typedef void(CallbackFunction)();

    void advance(uint32_t cyc) { cycles_ += cyc; }

    uint32_t cyc() const { return cycles_; }

    double sec() const { return (double)cycles_ / CYC_PER_SEC; }

    void schedule(int32_t after_cyc, std::function<CallbackFunction> cb) {
      assert(after_cyc >= 0);
      if (after_cyc < 0)
        throw runtime_error(
            fmt::format("can not schedule work in the past (asked to schedule "
                        "in {} cycles)",
                        after_cyc));
      auto work = WorkItem{*this, cyc() + after_cyc, cb};
      SPDLOG_DEBUG("{}: scheduling work at {} cyc", cyc(), work.when_cyc);
      work_queue_.push(work);
    }

    void kick() {
      // Check if we should run any work items and run them
      while (work_queue_.size() &&
             (int32_t)(work_queue_.top().when_cyc - cyc()) <= 0) {
        auto work = work_queue_.top();
        work_queue_.pop();
        SPDLOG_DEBUG("{}: running work scheduled for {} cyc", cyc(),
                     work.when_cyc);
        work.cb();
      }
    }

   private:
    struct WorkItem {
      reference_wrapper<Clock> clock;
      uint32_t when_cyc;
      std::function<CallbackFunction> cb;

      int32_t cycles_till() const { return when_cyc - clock.get().cyc(); }

      // WorkItem that executes _later_ should be less than the _earlier_ one
      bool operator<(const WorkItem& o) const {
        return cycles_till() > o.cycles_till();
      }
    };

    uint32_t cycles_ = 0;
    priority_queue<WorkItem> work_queue_;
  };

  /**
   * @brief Display implementation
   */
  class Display {
   public:
    static constexpr unsigned NUM_COLUMNS = 10;
    static constexpr unsigned NUM_STATUS_LEDS = 20;
    static constexpr unsigned NUM_7SEG_SYMS = 8;

    Display(M1T380& sys) : sys_(sys) {}

    void set_seg(fast_u8 v) { seg_ = v; }

    void set_col(fast_u8 v) {
      col_ = v;
      // Since this is a dynamically-scanned display, we need to periodically
      // sample the address & segment buses, and update recorded value of
      // selected cell. We do this 500us after address bus has been updated by
      // 8080 - it is long enough to ensure that segment bus gets updated as
      // well and everything settles, and it is short enough to still be within
      // a single cell window of ~2ms (display cells are updated at a rate of
      // 500Hz, giving 50Hz refresh rate for the full display)
      sys_.clock_.schedule(sec2cyc(500e-6), [=]() { sample(); });
    }

    void print() {
      SPDLOG_INFO("--- 7-SEG DISPLAY ---");
      // Each symbol is printed 5 char wide X 3 lines tall
      string line[3];
      for (int s = 0; s < NUM_7SEG_SYMS; s++) {
        // Segment bits from LSB to MSB: 0 1 2 3 4 5 6 7
        //
        // For all symbols 0..7, connection of segments is following:
        //
        //   0
        // 5   1
        //   6
        // 4   2
        //   3  7
        //
        // (despite symbol 0 using VQE12 indicator while the rest is VQE14,
        // VQE12 is connected to be compatible/replacable by VQE14, so mapping
        // is the same!)
        bool seg[8];
        for (int b = 0; b < 8; b++) seg[b] = (contents_[s + 2] >> b) & 1;
        // For first symbol, only seg 1,2,6,7 are routed to 7seg indicator,
        // other bits are used for a few status LEDs so ignore them here
        if (s == 0) {
          seg[0] = seg[3] = seg[4] = seg[5] = false;
        }
        line[0] += fmt::format(" {}  ", seg[0] ? "__" : "  ");
        line[1] += fmt::format("{}{}{} ", seg[5] ? "|" : " ",
                               seg[6] ? "__" : "  ", seg[1] ? "|" : " ");
        line[2] +=
            fmt::format("{}{}{}{}", seg[4] ? "|" : " ", seg[3] ? "__" : "  ",
                        seg[2] ? "|" : " ", seg[7] ? "." : " ");
      }
      for (auto& l : line) SPDLOG_INFO(l);
      SPDLOG_INFO("");
      // Status LEDs
      SPDLOG_INFO("--- STATUS LEDS ---");
      for (const auto& m : m1t380_led_map_) {
        if ((contents_[m.sym] >> m.seg) & 1) {
          SPDLOG_INFO("  {}", m.name);
        }
      }
      SPDLOG_INFO("-------------------");
    }

   private:
    void sample() { contents_[col_ % NUM_COLUMNS] = seg_; }

    M1T380& sys_;
    fast_u8 seg_;
    fast_u8 col_;
    array<fast_u8, NUM_COLUMNS> contents_;

    /**
     * @brief Status LED map
     */
    struct StatusLed {
      int sym;
      int seg;
      string name;
    };

    const array<StatusLed, NUM_STATUS_LEDS> m1t380_led_map_{{
        {0, 0, "VOLTS"},  {0, 1, "AMPS"},  {0, 2, "OHMS"},     {0, 3, "AC"},
        {0, 4, "*A*"},    {0, 5, "ZERO"},  {0, 6, "COMP"},     {0, 7, "PROG"},
        {1, 0, "*mA*"},   {1, 1, "*Ohm*"}, {1, 2, "*mV*"},     {1, 3, "*CAL*"},
        {1, 4, "*kOhm*"}, {1, 5, "*V*"},   {1, 6, "*REMOTE*"}, {1, 7, "REP"},
        {2, 0, "FILTER"}, {2, 3, "FAST"},  {2, 4, "HI.RES"},   {2, 5, "AUTO"},
    }};
  };

  class Keys {
   public:
    static constexpr unsigned NUM_KEYS = 17;

    Keys(M1T380& sys) : sys_(sys) {}

    void set_col(fast_u8 v) {
      // Find keys mapped to this column
      selkeys_.fill(nullptr);
      for (Key& key : keys_) {
        if (key.col == v) {
          assert(key.row == 0 || key.row == 1);
          selkeys_[key.row] = &key;
        }
      }
    }

    fast_u8 get_rows() const {
      fast_u8 x = 0;
      if (selkeys_[0] && selkeys_[0]->pressed) x |= 1;
      if (selkeys_[1] && selkeys_[1]->pressed) x |= 2;
      // active LOW
      x = ~x;
      return x;
    }

    void set_key_state(const string& key_name, bool pressed) {
      for (Key& k : keys_) {
        if (k.name == key_name) {
          k.pressed = pressed;
          return;
        }
      }
      // Not found
      throw runtime_error(fmt::format("Unknown key: '{}'", key_name));
    }

   private:
    struct Key {
      unsigned col;
      unsigned row;
      string name;
      bool pressed{};
    };

    M1T380& sys_;
    array<Key*, 2> selkeys_{};

    array<Key, NUM_KEYS> keys_{{
        {0, 1, "VOLTS"},
        {0, 0, "AMPS"},
        {1, 1, "OHMS"},
        {1, 0, "AC"},
        {2, 1, "CAL"},
        {2, 0, "FILTER"},
        {3, 1, "FAST"},
        {3, 0, "HIRES"},
        {4, 1, "SAMPLE"},
        {4, 0, "REP"},
        {5, 1, "PROG"},
        {5, 0, "AUTO"},
        {6, 1, "UP"},
        {6, 0, "DOWN"},
        {7, 1, "ZERO"},
        {7, 0, "LOCAL"},
        {8, 1, "COMP"},
    }};
  };

  /**
   * @brief ADC communication channel implementation
   */
  class ADC {
   public:
    ADC(M1T380& sys) : sys_(sys) {}

    void set_data2(bool v) {
      data2_ = v;
      if (tx_state_ == 0 && !data2_) {
        // beginning of transmission from CPU to ADC
        tx_state_ = 1;
        rdy1_ = 0;
        // stop any ongoing rx...
        rx_state_ = 0;
        data1_ = 1;
        rx_queue_.clear();
      } else if (tx_state_ == 1 && data2_) {
        tx_state_ = 2;
        tx_byte_ = 0x00;
        rdy1_ = 1;
        sys_.clock_.schedule(sec2cyc(150e-6), [=]() { tx_read_bit(); });
      }
    }
    void set_rdy2(bool v) {
      rdy2_ = v;
      if (rx_state_ == 1 && !rdy2_) {
        // begin receive sequence - from ADC to CPU
        rx_state_ = 2;
        data1_ = 1;
        sys_.clock_.schedule(sec2cyc(110e-6), [=]() { rx_push_bit(); });
      }
    }
    bool get_data1() { return data1_; }
    bool get_rdy1() { return rdy1_; }

    void exec_cmd(fast_u8 cmd) {
      if (!(cmd & 0x80)) {
        // set_mode
        int mode = cmd & 0x1F;
        bool fil = cmd & 0x20;
        SPDLOG_INFO("ADC set_mode mode={} fil={}", mode, fil);
        mode_ = mode;
      } else if (!(cmd & 0x78)) {
        // run_meas
        int dur = 100;
        if (cmd & 1)
          dur = 1;
        else if (cmd & 2)
          dur = 10;

        // Simulate measurement
        auto adc_meas = get_measured_value();
        int16_t meas_tsx = (2000 * dur) >> 3;
        int32_t meas_tsn = (double)(2000 * dur) * adc_meas * 256 / 0x40000000;
        if (dur == 100) meas_tsn >>= 3;

        SPDLOG_INFO(
            "ADC run_meas dur={} mode={} - meas {:08x}, send "
            "TSx={:04x}, TSn={:06x}",
            dur, mode_, adc_meas, meas_tsx, meas_tsn);

        sys_.clock_.schedule(sec2cyc(dur * 25e-3), [=]() {
          rx_queue_.clear();
          rx_queue_.push_back((meas_tsn >> 0) & 0xFF);
          rx_queue_.push_back((meas_tsn >> 8) & 0xFF);
          rx_queue_.push_back((meas_tsn >> 16) & 0xFF);
          rx_queue_.push_back((meas_tsx >> 0) & 0xFF);
          rx_queue_.push_back((meas_tsx >> 8) & 0xFF);
          start_rx();
        });
      } else {
        // run_test
        int test = (cmd >> 3) & 0xF;
        SPDLOG_WARN("ADC run_test test={} - NOT IMPLEMENTED (do nothing)",
                    test);
      }
    }

    int32_t get_measured_value() {
      double vx = sim_in_;

      double r36 = 18e3;
      double r25 = 10e3;

      double adc_in_v;
      switch (mode_) {
        case 20:
          // AC/DC 0
          adc_in_v = 0;
          break;
        case 21:
          // AC/DC ref
          adc_in_v = 5 * 1.3;  // bogus..
          break;
        case 22:
          // AC/DC meas ref
          adc_in_v = -5;
          break;
        case 23:
          // DIV 0
          adc_in_v = 0;  // 2e-6;
          break;
        case 25:
          // DIV 0 (I)
          adc_in_v = 0;  // 1e-6;
          break;
        case 26:
        case 30:
          // ADC 0
          adc_in_v = 0;
          break;
        case 27:
        case 31:
          // ADC CAL+
          adc_in_v = sim_vcal_;
          break;
        case 28:
        case 29:
          // ADC CAL-
          adc_in_v = -sim_vcal_;
          break;
        default:
          if (adc_k_.count(mode_)) {
            adc_in_v = sim_in_ * adc_k_.at(mode_);
          } else {
            adc_in_v = 0;
          }
          break;
      }
      // add some ADC zero offset
      adc_in_v += sim_voff_;
      // ADC input is clipped to ~ -9.5..+9.5V
      adc_in_v = min(max(adc_in_v, -9.5), 9.5);
      // calculate scaled ADC result (0x40000000 corresponds to full-scale
      // range)
      double x = adc_in_v / sim_vref_ * r25 / r36 * 0x40000000LL;
      // Clip again (to handle screwed up Vref setting)
      x = min(max(x, (double)-0x60000000), (double)0x60000000);
      return x;
    }

    double sim_in_ = 5;

    /**
     * @note Voff and Vref are compensated out by calibration procedure
     */
    double sim_voff_ = 0;
    double sim_vref_ = 7.4;

    /**
     * @note In real life, Vcal should be rock stable, and its actual value
     * is taken into account when calculating CALRAM coefficients.
     * 
     * So you change Vcal - you also need to update CALRAM, otherwise measured
     * values will be (totally) off.
     */
    double sim_vcal_ = 8.4;

   private:
    void tx_read_bit() {
      tx_byte_ <<= 1;
      tx_byte_ |= data2_ & 1;
      tx_state_++;
      if (tx_state_ < 10) {
        sys_.clock_.schedule(sec2cyc(100e-6), [=]() { tx_read_bit(); });
      } else {
        tx_state_ = 0;
        SPDLOG_DEBUG("ADC TX 0x{:02x}", tx_byte_);
        exec_cmd(tx_byte_);
      }
    }

    void start_rx() {
      if (rx_queue_.empty()) return;
      rx_byte_ = rx_byte_s_ = rx_queue_.front();
      rx_queue_.pop_front();
      rx_state_ = 1;
      data1_ = 0;
    }

    void rx_push_bit() {
      data1_ = rx_byte_ & 0x80;
      rx_byte_ <<= 1;
      rx_byte_ |= 0x1;
      rx_state_ += 1;
      if (rx_state_ < 12) {
        sys_.clock_.schedule(sec2cyc(100e-6), [=]() { rx_push_bit(); });
      } else {
        (void)rx_byte_s_;
        SPDLOG_DEBUG("ADC RX 0x{:02x} - done", rx_byte_s_);
        rx_state_ = 0;
        start_rx();
      }
    }

    int tx_state_ = 0;
    fast_u8 tx_byte_ = 0;
    deque<fast_u8> rx_queue_;
    int rx_state_ = 0;
    fast_u8 rx_byte_ = 0;
    fast_u8 rx_byte_s_ = 0;
    bool data2_ = true;
    bool rdy2_ = true;
    bool data1_ = true;
    bool rdy1_ = true;
    int mode_ = 3;  // VDC 150V
    M1T380& sys_;

    /**
     * @brief Scaling factors for input circuitry
     *
     * Basically, for all modes except for calibration ones, ADC input voltage
     * is:
     *  V(ADC) = IN * K, where IN is the input value on H-L posts of the
     * multimeter (can be current, can be volts, can be ohms - formula is the
     * same)
     *
     * This dictionary specifies 'mode: K' mapping
     * 
     * @note In real life, K depends on the hardware component tolerance,
     * and will be different for each multimeter. That's why CALRAM is needed.
     */
    const map<int, double> adc_k_{{
        // IN is in Volts
        {0, -100. / 2},
        {1, -10. / 2},
        {2, -1. / 2},
        {3, -10. / 100 / 2},
        {4, -1. / 100 / 2},
        // IN is in Amps
        {5, -100. * 10 / 2},   // Rs=10
        {6, -100. * 0.1 / 2},  // Rs=0.1
        // IN is in Ohms
        {7, -5e-3 * -10.},
        {8, -0.5e-3 * -10.},
        {9, -0.5e-3 * -1.},
        {10, -50e-6 * -1.},
        {11, -5e-6 * -1.},
        {12, -0.5e-6 * -1.},
    }};
  };

  /**
   * @brief A primitive i8255 emulation (generic)
   */
  class I8255 {
   public:
    I8255() {}

    virtual fast_u8 input_port(int port, fast_u8 mask) { return 0xFF; }

    virtual void output_port(int port, fast_u8 mask, fast_u8 data) {}

    void write(fast_u16 addr, fast_u8 data) {
      switch (addr) {
        case 0:
        case 1:
        case 2:
          out_[addr] = data;
          output_port(addr, dir_[addr], dir_[addr] & out_[addr]);
          break;
        case 3:
          set_control(data);
          break;
        default:
          throw runtime_error(
              fmt::format("8255: write to invalid address {:04x}", addr));
      }
    }

    fast_u8 read(fast_u16 addr) {
      fast_u8 ret = 0xFF;
      switch (addr) {
        case 0:
        case 1:
        case 2: {
          auto v = input_port(addr, (~dir_[addr]) & 0xFF);
          v &= ~dir_[addr];
          v |= out_[addr] & dir_[addr];
          ret = v;
        } break;
        case 3:
          SPDLOG_WARN(
              "8255: read from control register not allowed, returns 0xFF");
          break;
        default:
          throw runtime_error(
              fmt::format("8255: read from invalid address {:04x}", addr));
      }
      return ret;
    }

   private:
    void set_control(fast_u8 data) {
      auto mode_set = (data >> 7) & 1;

      if (mode_set) {
        // MODE word
        auto dir_c_lo = data & 0x1;
        auto dir_b = (data >> 1) & 1;
        auto mode_b = (data >> 2) & 1;
        auto dir_c_hi = (data >> 3) & 1;
        auto dir_a = (data >> 4) & 1;
        auto mode_a = (data >> 5) & 0x3;

        if (mode_a || mode_b) {
          throw runtime_error(fmt::format(
              "8255: mode A={}, B={} not implemented", mode_a, mode_b));
        }

        dir_[0] = dir_a ? 0x00 : 0xFF;
        dir_[1] = dir_b ? 0x00 : 0xFF;
        dir_[2] = (dir_c_lo ? 0x00 : 0x0F) | (dir_c_hi ? 0x00 : 0xF0);

        SPDLOG_INFO(
            "8255: set directions (0 - in, 1 - out): A={:08b}, B={:08b}, "
            "C={:08b}",
            dir_[0], dir_[1], dir_[2]);

        // When mode is set, output registers are reset?
        for (int i = 0; i < 3; i++) {
          out_[i] = 0x00;
          output_port(i, dir_[i], out_[i] & dir_[i]);
        }

      } else {
        // Bit Set-Reset word
        throw runtime_error(fmt::format("8255: BSR not implemented"));
      }
    }

    // 0 - input, 1 - output
    fast_u8 dir_[3]{0x00, 0x00, 0x00};
    fast_u8 out_[3]{0x00, 0x00, 0x00};
  };

  /**
   * @brief 8255 PIO callbacks implementation
   */
  class PIO : public I8255 {
   public:
    PIO(M1T380& sys) : sys_(sys) {}

   private:
    fast_u8 input_port(int port, fast_u8 mask) override {
      fast_u8 n = 0;
      if (port == 2) {
        n = 0xF0;
        n |= (sys_.adc_.get_data1() & 1) << 2;
        n |= (sys_.adc_.get_rdy1() & 1) << 3;
        n |= sys_.keys_.get_rows() & 0x3;
      } else {
        n = 0xFF;  // doesnt matter anyway as those are output ports
      }

      SPDLOG_DEBUG("PIO IN port {} : {:08b}", (int)port, n);
      return n;
    }

    void output_port(int port, fast_u8 mask, fast_u8 data) override {
      SPDLOG_DEBUG("PIO OUT port {} : {:08b}", (int)port, data);
      if (port == 0) {
        sys_.display_.set_seg(data);
      } else if (port == 1) {
        sys_.keys_.set_col(data & 0xF);
        sys_.display_.set_col(data & 0xF);
        bool data2 = (~(data >> 4)) & 1;
        bool rdy2 = (~(data >> 5)) & 1;
        sys_.adc_.set_data2(data2);
        sys_.adc_.set_rdy2(rdy2);
      }
    }

    M1T380& sys_;
  };

  /**
   * @brief M1T380 CPU, system bus, and address decoding
   */
  class CPU : public i8080_cpu<CPU> {
   public:
    typedef i8080_cpu<CPU> base;

    CPU(M1T380& sys) : sys_(sys) {}

    fast_u8 on_read(fast_u16 addr) {
      auto cs = decode_cs(addr);
      fast_u8 ret = base::on_read(addr);
      switch (cs) {
        case CS::ROM:
          ret = sys_.rom_[addr & (ROM_SIZE - 1)];
          break;
        case CS::RAM:
          ret = sys_.ram_[addr & (RAM_SIZE - 1)];
          break;
        case CS::CALRAM:
          ret = 0xF0 | (sys_.calram_[addr & (CALRAM_SIZE - 1)] & 0x0F);
          SPDLOG_INFO("MEMR CALRAM {:02x}: {:02x}", addr, ret);
          break;
        case CS::PIO:
          ret = sys_.pio_.read(addr & 3);
          break;
        case CS::EXT_CSI:
          ret = 0xFF;
          break;
        default:
          SPDLOG_WARN("MEMR to unmapped address 0x{:04x}", addr);
          break;
      }
      SPDLOG_TRACE("MEMR 0x{:04x}:0x{:02x}", addr, ret);
      return ret;
    }

    void on_write(fast_u16 addr, fast_u8 n) {
      SPDLOG_TRACE("MEMW 0x{:04x}:0x{:02x}", addr, n);
      base::on_write(addr, n);
      auto cs = decode_cs(addr);
      switch (cs) {
        case CS::ROM:
          SPDLOG_WARN("MEMW to ROM 0x{:04x}: 0x{:02x}", addr, n);
          break;
        case CS::RAM:
          sys_.ram_[addr & (RAM_SIZE - 1)] = n;
          break;
        case CS::CALRAM:
          SPDLOG_INFO("MEMW CALRAM {:02x} : {:02x}", addr, n);
          sys_.calram_[addr & (CALRAM_SIZE - 1)] = n & 0x0F;
          break;
        case CS::PIO:
          sys_.pio_.write(addr & 3, n);
          break;
        default:
          SPDLOG_WARN("MEMW to unmapped address 0x{:04x}: 0x{:02x}", addr, n);
          break;
      }
    }

    fast_u8 on_input(fast_u16 port) {
      base::on_input(port);
      fast_u8 n = 0xFF;
      SPDLOG_DEBUG("IOR {:04x}:{:02x}", port, n);
      return n;
    }

    void on_output(fast_u16 port, fast_u8 n) {
      SPDLOG_DEBUG("IOW {:04x}:{:02x}", port, n);
      base::on_output(port, n);
      // In M1T380, IOW to any port simply resets IRQ pin
      sys_.irq_ = false;
    }

    void on_tick(unsigned t) {
      SPDLOG_TRACE("T {}", t);
      base::on_tick(t);
      // drive clock
      sys_.clock_.advance(t);
    }

   private:
    /**
     * @brief Decodable chip selects
     */
    enum struct CS {
      NONE,
      ROM,
      RAM,
      EXT_CSI,
      EXT_CSH,
      CALRAM,
      PIO,
    };

    /**
     * M1T380 CS decoder
     *
     * Memory map:
     * \code
     * A15 A14 A13 A12 A11 SEL
     *  0   0   0   0   0   ROM I6
     *  0   0   0   0   1   ROM I7
     *  0   0   0   1   0   ROM I8
     *  0   0   0   1   1   ROM I9
     *  0   0   1   0   0   EXT_CSI
     *  0   0   1   0   1   CALRAM
     *  0   0   1   1   0   RAM I10-I11
     *  0   0   1   1   1   EXT_CSH
     *  0   1   x   x   x   unmapped
     *  1   x   x   x   x   8255 PIO
     * \endcode
     */
    CS decode_cs(fast_u16 addr) {
      if ((addr & 0xC000) == 0) {
        // CS decoder
        switch ((addr & 0x3800) >> 11) {
          case 0:
          case 1:
          case 2:
          case 3:
            return CS::ROM;
            break;
          case 4:
            return CS::EXT_CSI;
            break;
          case 5:
            return CS::CALRAM;
            break;
          case 6:
            return CS::RAM;
            break;
          case 7:
            return CS::EXT_CSH;
            break;
          default:
            return CS::NONE;
            break;
        }
      } else if (addr & 0x8000) {
        return CS::PIO;
      } else {
        return CS::NONE;
      }
    }

    M1T380& sys_;
  };

  M1T380()
      : cpu_(*this), pio_(*this), display_(*this), keys_(*this), adc_(*this) {
    // Kickoff periodic interrupts
    irq_trigger();
  }

  /**
   * @brief Main function that steps CPU and whole system by one "step"
   */
  void step() {
    cpu_.on_step();
    if (irq_) {
      cpu_.on_handle_active_int();
    }
    clock_.kick();
  }

  /**
   * @brief Called by Clock every INT_PERIOD cycles
   */
  void irq_trigger() {
    clock_.schedule(INT_PERIOD, [=]() { irq_trigger(); });
    irq_ = true;
    SPDLOG_DEBUG("intdis {} iff {} iff1 {} iff2 {} im {}",
                 cpu_.on_is_int_disabled(), cpu_.on_get_iff(),
                 cpu_.on_get_iff1(), cpu_.on_get_iff2(),
                 cpu_.on_get_int_mode());
  }

  // Clock
  Clock clock_;
  // CPU
  CPU cpu_;
  // PIO
  PIO pio_;
  // Display
  Display display_;
  // Keys
  Keys keys_;
  // ADC
  ADC adc_;

  bool irq_ = false;  // whether INT pin is asserted

  array<fast_u8, ROM_SIZE> rom_{};
  array<fast_u8, RAM_SIZE> ram_{};
  array<fast_u8, CALRAM_SIZE> calram_{};
};

/***************************************************************************
 *
 * CUSTOM SPDLOG FLAG FORMATTER
 *
 ***************************************************************************/

class SpdFlagSimstate : public spdlog::custom_flag_formatter {
 public:
  SpdFlagSimstate(M1T380& sys) : sys_(sys) {}

  void format(const spdlog::details::log_msg&, const tm&,
              spdlog::memory_buf_t& dest) override {
    string some_txt =
        fmt::format("t {:7.3f} cyc {:10d} pc {:04x}", sys_.clock_.sec(),
                    sys_.clock_.cyc(), sys_.cpu_.on_get_pc());
    dest.append(some_txt.data(), some_txt.data() + some_txt.size());
  }

  unique_ptr<custom_flag_formatter> clone() const override {
    return spdlog::details::make_unique<SpdFlagSimstate>(sys_);
  }

 private:
  M1T380& sys_;
};

/***************************************************************************
 *
 * MAIN
 *
 ***************************************************************************/

/** Read file into string. Taken from https://stackoverflow.com/a/18816712 */
inline std::string slurp(const std::string& path) {
  std::ostringstream buf;
  std::ifstream inp(path.c_str());
  if (!inp) {
    throw runtime_error(fmt::format("Invalid file: '{}'", path));
  }
  buf << inp.rdbuf();
  return buf.str();
}

int main(int argc, char** argv) {
  argparse::ArgumentParser parser("m1t380emu");

  parser.add_argument("-v", "--verbose")
      .help(
          "increase output verbosity to DEBUG (or TRACE, if TRACE logs have "
          "been enabled at compile-time)")
      .default_value(false)
      .implicit_value(true);

  parser.add_argument("-r", "--rom")
      .help(fmt::format("path to ROM file, should be {} KiB max",
                        M1T380::ROM_SIZE / 1024))
      .required();

  parser.add_argument("-c", "--cal")
      .help(fmt::format(
          "path to CALRAM file, should be {} bytes max (Note: CALRAM is a "
          "4-bit memory, so only 4 LSBs of each byte are taken into account!)",
          M1T380::CALRAM_SIZE));

  parser.add_argument("-t", "--time")
      .help("number of seconds to run the emulated M1T380 for")
      .scan<'g', double>()
      .default_value(10.0);

  parser.add_argument("-d", "--disp")
      .help("number of seconds between display printouts")
      .scan<'g', double>()
      .default_value(0.5);

  parser.add_argument("--adc_vref")
      .help("Simualted ADC Vref in volts")
      .scan<'g', double>()
      .default_value(7.4);
  parser.add_argument("--adc_vcal")
      .help("Simualted ADC Vcal in volts")
      .scan<'g', double>()
      .default_value(8.4);
  parser.add_argument("--adc_voff")
      .help("Simulated ADC offset in volts")
      .scan<'g', double>()
      .default_value(0.);

  parser.add_argument("-a", "--action")
      .help(
          "Execute scripted action at a given time. Can be given multiple "
          "times")
      .metavar("TIME CMD ARGS..")
      .append()
      .default_value<vector<string>>({});

  // Parse args
  try {
    parser.parse_args(argc, argv);
  } catch (const std::runtime_error& err) {
    cerr << err.what() << endl;
    cerr << parser << endl;
    return 1;
  }

  // Set log level
  if (parser.get<bool>("-v") == true)
    spdlog::set_level(spdlog::level::level_enum(SPDLOG_ACTIVE_LEVEL));
  else
    spdlog::set_level(spdlog::level::info);

  // Set up the simulation
  M1T380 sys;

  // Set custom log formatter that will print simulation time and PC value
  {
    auto formatter = make_unique<spdlog::pattern_formatter>();
    formatter->add_flag<SpdFlagSimstate>('*', sys);
    formatter->set_pattern("[%^%L%$ %*] %v");
    spdlog::set_formatter(std::move(formatter));
  }

  // Load ROM image from file
  {
    auto path = parser.get<string>("--rom");
    auto data = slurp(path);
    SPDLOG_INFO("Read ROM from '{}': {} bytes", path, data.size());
    if (data.size() > sys.ROM_SIZE) {
      SPDLOG_WARN("File too big, will be truncated: {}, max allowed {}",
                  data.size(), sys.ROM_SIZE);
      data.resize(sys.ROM_SIZE);
    }
    for (size_t i = 0; i < min(sys.ROM_SIZE, data.size()); i++) {
      sys.rom_[i] = (uint8_t)data[i];
    }
  }

  // Load CALRAM image from file
  if (auto arg = parser.present<string>("--cal")) {
    auto path = *arg;
    auto data = slurp(path);
    SPDLOG_INFO("Read CALRAM from '{}': {} bytes", path, data.size());
    if (data.size() > sys.CALRAM_SIZE) {
      SPDLOG_WARN("File too big, will be truncated: {}, max allowed {}",
                  data.size(), sys.CALRAM_SIZE);
      data.resize(sys.CALRAM_SIZE);
    }
    for (size_t i = 0; i < min(sys.CALRAM_SIZE, data.size()); i++) {
      sys.calram_[i] = (uint8_t)data[i];
    }
  } else {
    SPDLOG_INFO("CALRAM not provided, using zero-initialized one");
  }

  // Set ADC params
  sys.adc_.sim_voff_ = parser.get<double>("--adc_voff");
  sys.adc_.sim_vcal_ = parser.get<double>("--adc_vcal");
  sys.adc_.sim_vref_ = parser.get<double>("--adc_vref");

  // Configure simulation killswitch
  bool run = true;
  sys.clock_.schedule(sys.sec2cyc(parser.get<double>("--time")), [&]() {
    SPDLOG_INFO("stopping simulation");
    run = false;
  });

  // Configure periodic display readouts
  auto disp_print_interval = sys.sec2cyc(parser.get<double>("--disp"));
  // NOTE: keep below `disp_print` variable alive! Demons will eat your computer
  // if it is RAII-cleaned up while lambda is still being scheduled.
  std::function<void()> disp_print = [&sys, &disp_print,
                                      disp_print_interval]() {
    SPDLOG_DEBUG("periodic display printout");
    sys.clock_.schedule(disp_print_interval, disp_print);
    sys.display_.print();
  };
  if (disp_print_interval > 0)
    sys.clock_.schedule(disp_print_interval, disp_print);

  // Configure scripted actions
  for (const string& action : parser.get<vector<string>>("--action")) {
    // Tokenize string
    regex reg("\\s+");
    sregex_token_iterator iter(action.begin(), action.end(), reg, -1);
    sregex_token_iterator end;
    deque<string> toks(iter, end);

    auto pop_tok = [&]() -> string {
      if (!toks.size()) throw runtime_error("Not enough arguments");
      auto r = toks.front();
      toks.pop_front();
      return r;
    };

    try {
      auto time = stod(pop_tok());
      auto cyc = sys.sec2cyc(time);
      auto cmd = pop_tok();
      if (cmd == "ping") {
        sys.clock_.schedule(cyc, [&sys]() { SPDLOG_INFO("ping!"); });
      } else if (cmd == "input") {
        auto vx = stod(pop_tok());
        sys.clock_.schedule(cyc, [&sys, vx]() {
          SPDLOG_INFO("setting measured value to {:f}", vx);
          sys.adc_.sim_in_ = vx;
        });
      } else if (cmd == "disp") {
        sys.clock_.schedule(cyc, [&sys]() {
          SPDLOG_INFO("scripted display printout");
          sys.display_.print();
        });
      } else if (cmd == "click") {
        // Key down, wait 0.2 sec, key up
        const double delay = 0.2;
        string key_name = pop_tok();

        sys.clock_.schedule(cyc, [&sys, key_name]() {
          SPDLOG_INFO("key '{}' down", key_name);
          sys.keys_.set_key_state(key_name, true);
        });
        sys.clock_.schedule(sys.sec2cyc(time + delay), [&sys, key_name]() {
          SPDLOG_INFO("key '{}' up", key_name);
          sys.keys_.set_key_state(key_name, false);
        });
      } else {
        throw runtime_error(fmt::format("Unknown command '{}'", cmd));
      }

      if (toks.size())
        throw runtime_error(fmt::format("Extraneous arguments given: '{}'",
                                        fmt::join(toks, " ")));
    } catch (const exception& e) {
      SPDLOG_ERROR(fmt::format("Error while parsing scripted action '{}': {}",
                               action, e.what()));
      return 1;
    }
  }

  // Simulate
  while (run) {
    sys.step();
  }
  // Print display at the end
  sys.display_.print();

  SPDLOG_INFO("Done.");
  return 0;
}