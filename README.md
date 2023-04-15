# Metra M1T380 ADCON

**ADCON** is a Python tool for communicating with the ADC board of Metra M1T380
multimeter from your PC.

It uses PySerial to talk to the Arduino board connected over USB. The Arduino
acts as an interface converter between Serial Port interface on the USB side and
M1T380-specific serial protocol on another side. It connects to the K1 connector
on D1639 ADC board - the one to which multimeter's own Intel 8080 CPU board is
normally connected.

This allows you to exercise full control over low-level functions implemented by
MHB8748 MCU:
- Control measurement & calibration modes via 'set_mode' command.
- Run ADC measurements via 'run_meas' command.
- Run built-in test & repair programs via 'run_test' command.
