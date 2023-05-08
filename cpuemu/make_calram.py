#!/usr/bin/env python3
import sys
import os
from math import frexp


def main():
    if len(sys.argv) != 2:
        print(f'Usage: {os.path.basename(sys.argv[0])} OUTPUT.BIN', file=sys.stderr)
        exit(1)
    out_bin = sys.argv[1]

    # Make 32-bit calram array
    # Format of CALRAM:
    #   32-bit coefficients:
    #       C0, C1, C2, C3, C4, ...
    #   Are written as bytes in little-endian order
    #       C0_0, C0_1, C0_2, C0_3, C1_0, C1_1, ..
    #   Which are then written as 4-bit nibbles in little-endian order
    #       C0_0L, C0_0H, C0_1L, C0_1H, C0_2L, C0_2H, C0_3L, C0_3H,
    #       C1_0L, C1_0H, C1_1L, C1_1H, C1_2L, C1_2H, C1_3L, C1_3H...
    #   Each Cx coefficient is a 32-bit floating-point value stored
    #   in custom format.
    #   Its bits are as follows:
    #       31..24  - shifted exponent ES
    #       23      - mantissa sign MS
    #       22..0   - mantissa magnitude MM
    #   The actual number X is:
    #       X = sign * 2**(ES - 128) * (0.5 + MM / 0x1000000)
    #   where
    #       sign = (MS==1) ? -1 : +1
    #
    # Each coefficient corresponds to its own mode:
    #   Cn is for mode `n` (C0 is for mode 0 - VDC 150mV, etc.)
    # Cn is used as a scaling factor to convert ADC measurement into
    # displayed value:
    #   D = Cn * (ADC(Vadc) / ADC(Vcal))
    #       where Vadc is the voltage on ADC input (chip I6 @ D1639)
    # For each mode and range, D is represented in different units:
    #   VDC 150mv   - 1uV
    #   VDC 1.5V    - 10uV
    #   VDC 15V     - 100uV
    #   VDC 150V    - 1mV
    #   IDC 1.5A    - 10uA
    #   ...
    #
    # Then:
    #   Cn = D * ADV(Vcal) / ADC(Vadc)
    #   Cn = D * Vcal / (X * k)
    #   Cn = Vcal / (k * (X / D))
    # Where:
    #   X - measured value (voltage, current, ohms, etc.)
    #   k - gain of input circuitry that converts X into Vadc:
    #       Vadc = X * k
    #   X / D - unit in which D is represented (1e-6 is 1uV/uA,
    #           1e-3 is 1mV/mA etc.) 
    
    # NOTE: in real life, Vcal is determined by reference voltage source
    # and will be different for each multimeter. That's why CALRAM is needed.
    vcal = 8.4

    mode_display_unit = {
        # VOLTS DC
        0: 1e-6,
        1: 1e-5,
        2: 1e-4,
        3: 1e-3,
        4: 1e-2,
        # AMPS DC
        5: 1e-7,
        6: 1e-5,
        # OHMS
        7: 1e-3,
        8: 1e-2,
        9: 1e-1,
        10: 1,
        11: 10,
        12: 100,
    }

    # NOTE: in real life, K depends on the hardware component tolerance,
    # and will be different for each multimeter. That's why CALRAM is needed.
    mode_k = {
        # VOLTS DC
        0: -100 / 2,
        1: -10 / 2,
        2: -1 / 2,
        3: -10 / 100 / 2,
        4: -1 / 100 / 2,
        # AMPS DC
        5: -100. * 10 / 2,
        6: -100. * 0.1 / 2,
        # OHMS
        7: -5e-3 * -10.,
        8: -0.5e-3 * -10.,
        9: -0.5e-3 * -1.,
        10: -50e-6 * -1.,
        11: -5e-6 * -1.,
        12: -0.5e-6 * -1.,
    }

    def to_mf32(c: float) -> int:
        m, e = frexp(c)
        cn = 0
        if m < 0:
            m = -m
            cn |= 1 << 23
        if m >= 0.5:
            assert m < 1
            m -= 0.5
            mm = int(m * 0x1000000 + 0.5)
            mm = min(max(mm, 0), 0x7FffFF)
            es = int(e + 128)
            es = min(max(es, 0), 0xFF)
            cn |= (es & 0xFF) << 24
            cn |= mm & 0x7FffFF
        return cn

    def from_mf32(cn: int) -> float:
        ms = (cn >> 23) & 1
        mm = cn & 0x7FffFF
        es = (cn >> 24) & 0xFF
        s = -1 if ms else 1
        x = s * 2**(es - 128) * (0.5 + mm / 0x1000000)
        return x

    print('Cn coefs:')
    print('MODE:             C,         Cn,     check(Cn)')
    cns = []
    for mode in range(20):
        unit = mode_display_unit.get(mode, 1)
        k = mode_k.get(mode, 1)
        c = vcal / (k * unit)
        cn = to_mf32(c)
        print(f'{mode:4d}: {c:10.7e}, 0x{cn:08x}, {from_mf32(cn):10.7e}')
        cns.append(cn)

    # Convert to bytes in little-endian order
    calb = bytearray()
    for cn in cns:
        calb.extend((
            (cn >> 0) & 0xFF,
            (cn >> 8) & 0xFF,
            (cn >> 16) & 0xFF,
            (cn >> 24) & 0xFF,
        ))
    # Append checksum byte
    s = sum(calb) & 0xFF
    calb.append((0x100 - s) & 0xFF)
    # Convert to nibbles
    caln = bytearray()
    for b in calb:
        caln.extend((
            b & 0x0F,
            (b >> 4) & 0x0F,
        ))
    # Zero-pad till end of CALRAM
    if len(caln) > 256:
        raise RuntimeError(f'CALRAM too long ({len(caln)} bytes)')
    caln.extend([0] * (256 - len(caln)))
    # Write to file
    with open(out_bin, 'wb') as f:
        f.write(caln)
    print('Done.')


if __name__ == '__main__':
    main()