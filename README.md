# Metra M1T380 Tools

This repository contains tools that I've developed when working and
reverse-engineering Metra M1T380 multimeter. If you have such a device and
you're thinking of hacking it, some of these may turn out to be useful.

## ADCON

[adcon](adcon/)

ADCON is a tool that talks to D1639/D1791 ADC board via a simple Arduino
Leonardo adapter. No extra hardware is needed except for Arduino, some wires and
connectors. It allows you to excersise low-level control over ADC, collect data
(e.g. when you want to analyze stability of ADC measurements over time or
temperature), and execute built-in factory tests which can be helpful during ADC
board repair.

## CPUEMU

[cpuemu](cpuemu/)

CPUEMU is a system-level M1T380 emulator that runs factory i8080 EPROM images,
and simulates minimal set of 8080 peripherals to make it useful. I wrote it to
analyze (trace) different aspects of factory 8080 FW, specifically to
reverse-engineer the layout of CALRAM memory.

Writing an emulator and tracing the execution is surely easier than fully
reverse-engineering 8 KiB of hand-written 8080 assembly!

# License

See [LICENSE](LICENSE)
