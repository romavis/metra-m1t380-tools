# Metra M1T380 CPUEMU

## What is this

This folder contains a system-level M1T380 emulator. It is based on Z80 library
made by Ivan Kosarev
[https://github.com/kosarev/z80](https://github.com/kosarev/z80), and it can
execute a real stock 8080 FW image dumped from M1T380 EPROMs.

To be of any use, we emulate some hardware bits:
- ROM
- RAM
- CALRAM
- 8255A PIO
- Display (7seg and LEDs)
- Low-level ADC communication
  [protocol](https://github.com/romavis/metra-m1t380-doc/blob/main/mhb8748/protocol.md)
  and ADC measurement/mode logic
- 500Hz periodic interrupt generator
- Buttons / keyboard **is not supported yet**
- etc.

## How to use

[Z80](https://github.com/kosarev/z80) and
[ArgParse](https://github.com/p-ranav/argparse) dependencies come pre-packaged,
since they're single-file header-only libraries. Besides them, to build this
you'll have to install following dependencies in your system:
- [spdlog](https://github.com/gabime/spdlog)
- [fmt](https://github.com/fmtlib/fmt) 
- CMake
- Some decent compiler with C++17 support

After that has been done, build it like any other CMake project:
```
$ mkdir build
$ cd build
$ cmake ..
$ cmake --build .

# The executable is called `m1t380` and is located in the build directory
```

To run:
```
$ ./build/m1t380 -r blob/M1T-380-v3.9_full8k_original.BIN
```

The output looks similar to this:
```
[2023-05-07 14:05:29.635] [info] [m1t380emu.cpp:562] 0019791935 0a65: MEMR CALRAM 2800: f0
[2023-05-07 14:05:29.635] [info] [m1t380emu.cpp:562] 0019791948 0a67: MEMR CALRAM 2801: f0
[2023-05-07 14:05:29.635] [info] [m1t380emu.cpp:562] 0019792013 0a65: MEMR CALRAM 2802: f0
[2023-05-07 14:05:29.635] [info] [m1t380emu.cpp:562] 0019792026 0a67: MEMR CALRAM 2803: f0
[2023-05-07 14:05:29.635] [info] [m1t380emu.cpp:562] 0019792091 0a65: MEMR CALRAM 2804: f0
[2023-05-07 14:05:29.635] [info] [m1t380emu.cpp:562] 0019792104 0a67: MEMR CALRAM 2805: f0
[2023-05-07 14:05:29.635] [info] [m1t380emu.cpp:562] 0019792169 0a65: MEMR CALRAM 2806: f0
[2023-05-07 14:05:29.635] [info] [m1t380emu.cpp:562] 0019792182 0a67: MEMR CALRAM 2807: f0
[2023-05-07 14:05:29.636] [info] [m1t380emu.cpp:249] 0019994318: ADC run_meas dur=10 mode=0 - meas 00000000, send TSx=09c4, TSn=000000
[2023-05-07 14:05:29.637] [info] [m1t380emu.cpp:881] 0020480004: stopping simulation
[2023-05-07 14:05:29.637] [info] [m1t380emu.cpp:122] --- 7-SEG DISPLAY ---
[2023-05-07 14:05:29.637] [info] [m1t380emu.cpp:153]                      __   __   __   __  
[2023-05-07 14:05:29.637] [info] [m1t380emu.cpp:153]                     |  | |  | |  | |  | 
[2023-05-07 14:05:29.637] [info] [m1t380emu.cpp:153]                     |__|.|__| |__| |__| 
[2023-05-07 14:05:29.637] [info] [m1t380emu.cpp:154] 
[2023-05-07 14:05:29.637] [info] [m1t380emu.cpp:156] --- STATUS LEDS ---
[2023-05-07 14:05:29.638] [info] [m1t380emu.cpp:159]   VOLTS
[2023-05-07 14:05:29.638] [info] [m1t380emu.cpp:159]   *mV*
[2023-05-07 14:05:29.638] [info] [m1t380emu.cpp:159]   REP
[2023-05-07 14:05:29.638] [info] [m1t380emu.cpp:159]   FILTER
[2023-05-07 14:05:29.638] [info] [m1t380emu.cpp:159]   AUTO
[2023-05-07 14:05:29.638] [info] [m1t380emu.cpp:162] -------------------
[2023-05-07 14:05:29.638] [info] [m1t380emu.cpp:901] Done.
```

By default it prints accesses to CALRAM, ADC command details, and will also warn
you if any unusual access occurs (such as attempting to write to ROM). Screen
state is also printed periodically and at the end of simulation.

Pass `-h / --help` to `m1t380emu` to see a list of available options:
- Passing CALRAM image
- Changing duration of simulation
- Increasing output verbosity
- Changing emulated input voltage
- etc.

## CALRAM

To make any use of M1T380, in real life and also in the simulation, it needs to
have valid calibration constants in CALRAM (256x4 bit battery-powered CMOS RAM
chip).

First of all, firmware checks the validity of CALRAM upon boot using a very
simple checksum algorithm. Surprisingly, initializing CALRAM to all-zeroes
passes the check, however, your multimeter will only measure zeroes in this case
:-)

To generate a practical CALRAM image, use *make_calram.py* script. It needs
Python 3.6+ and does not require any extra dependencies, so you can just run it
on most modern systems as-is:
```
$ ./make_calram.py calram.bin
Cn coefs:
MODE:             C,         Cn,     check(Cn)
   0: -1.6800000e+05, 0x92a41000, -1.6800000e+05
   1: -1.6800000e+05, 0x92a41000, -1.6800000e+05
   2: -1.6800000e+05, 0x92a41000, -1.6800000e+05
   3: -1.6800000e+05, 0x92a41000, -1.6800000e+05
   4: -1.6800000e+05, 0x92a41000, -1.6800000e+05
   5: 8.4000000e+00, 0x84066666, 8.3999996e+00
   6: 8.4000000e+00, 0x84066666, 8.3999996e+00
   7: 8.4000000e+00, 0x84066666, 8.3999996e+00
   8: 8.4000000e+00, 0x84066666, 8.3999996e+00
   9: 8.4000000e+00, 0x84066666, 8.3999996e+00
  10: 8.4000000e+00, 0x84066666, 8.3999996e+00
  11: 8.4000000e+00, 0x84066666, 8.3999996e+00
  12: 8.4000000e+00, 0x84066666, 8.3999996e+00
  13: 8.4000000e+00, 0x84066666, 8.3999996e+00
  14: 8.4000000e+00, 0x84066666, 8.3999996e+00
  15: 8.4000000e+00, 0x84066666, 8.3999996e+00
  16: 8.4000000e+00, 0x84066666, 8.3999996e+00
  17: 8.4000000e+00, 0x84066666, 8.3999996e+00
  18: 8.4000000e+00, 0x84066666, 8.3999996e+00
  19: 8.4000000e+00, 0x84066666, 8.3999996e+00
Done.
```

You can check the contents of the CALRAM:
```
hexdump -Cv calram.bin     
00000000  00 00 00 01 04 0a 02 09  00 00 00 01 04 0a 02 09  |................|
00000010  00 00 00 01 04 0a 02 09  00 00 00 01 04 0a 02 09  |................|
00000020  00 00 00 01 04 0a 02 09  06 06 06 06 06 00 04 08  |................|
00000030  06 06 06 06 06 00 04 08  06 06 06 06 06 00 04 08  |................|
00000040  06 06 06 06 06 00 04 08  06 06 06 06 06 00 04 08  |................|
00000050  06 06 06 06 06 00 04 08  06 06 06 06 06 00 04 08  |................|
00000060  06 06 06 06 06 00 04 08  06 06 06 06 06 00 04 08  |................|
00000070  06 06 06 06 06 00 04 08  06 06 06 06 06 00 04 08  |................|
00000080  06 06 06 06 06 00 04 08  06 06 06 06 06 00 04 08  |................|
00000090  06 06 06 06 06 00 04 08  06 06 06 06 06 00 04 08  |................|
000000a0  08 09 00 00 00 00 00 00  00 00 00 00 00 00 00 00  |................|
000000b0  00 00 00 00 00 00 00 00  00 00 00 00 00 00 00 00  |................|
000000c0  00 00 00 00 00 00 00 00  00 00 00 00 00 00 00 00  |................|
000000d0  00 00 00 00 00 00 00 00  00 00 00 00 00 00 00 00  |................|
000000e0  00 00 00 00 00 00 00 00  00 00 00 00 00 00 00 00  |................|
000000f0  00 00 00 00 00 00 00 00  00 00 00 00 00 00 00 00  |................|
00000100
```

It is worth noting that since CALRAM is a 4-bit memory, only 4 LSBs of each byte
are taken into account by emulator. 4 MSBs are simply ignored, *make_calram.py*
zeroes them.

You can load CALRAM image into emulator by passing `-c` option, and also set
input voltage by passing `-x` option:
```
$ ./build/m1t380 -r blob/M1T-380-v3.9_full8k_original.BIN -c calram.bin -x 12.3456
```

Since by default M1T380 initializes into *VOLTS DC* mode with autoranging, `-x`
specifies input volts setting. You should now see a correct measurement:
```
[2023-05-07 14:20:51.240] [info] [m1t380emu.cpp:122] --- 7-SEG DISPLAY ---
[2023-05-07 14:20:51.240] [info] [m1t380emu.cpp:153]                 __   __        __   __  
[2023-05-07 14:20:51.240] [info] [m1t380emu.cpp:153]              |  __|  __| |__| |__  |__  
[2023-05-07 14:20:51.240] [info] [m1t380emu.cpp:153]              | |__ . __|    |  __| |__| 
[2023-05-07 14:20:51.240] [info] [m1t380emu.cpp:154] 
[2023-05-07 14:20:51.240] [info] [m1t380emu.cpp:156] --- STATUS LEDS ---
[2023-05-07 14:20:51.240] [info] [m1t380emu.cpp:159]   VOLTS
[2023-05-07 14:20:51.240] [info] [m1t380emu.cpp:159]   *V*
[2023-05-07 14:20:51.240] [info] [m1t380emu.cpp:159]   REP
[2023-05-07 14:20:51.240] [info] [m1t380emu.cpp:159]   FILTER
[2023-05-07 14:20:51.240] [info] [m1t380emu.cpp:159]   AUTO
[2023-05-07 14:20:51.240] [info] [m1t380emu.cpp:162] -------------------
[2023-05-07 14:20:51.240] [info] [m1t380emu.cpp:902] Done.
```

For more details on how CALRAM contents are structured and calculated, refer to
`make_calram.py`.

In short, there is a single 32-bit floating-point calibration coefficient stored
in CALRAM per range (at least in VOLTS DC mode). The format of that 32-bit float
is not exactly *IEEE754 binary32*, but a pretty similar one.

The value of coefficient depends on:
- Display scaling factor for the selected mode (150mV range measures in 1uV
  units, 1500mV in 10uV units, etc.)
- Voltage of calibration source (CAL+/-)
- Analog input path scaling factor (input divider ratio, gain of input
  amplifier, etc.)
