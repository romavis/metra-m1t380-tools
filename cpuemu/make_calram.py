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
    #   Cn corresponds to mode=n
    # Cn is used as a scaling factor to convert ADC measurement into
    # displayed value:
    #   D = Cn * (ADC(Vx) / ADC(Vcal)) 
    # The system then places decimal point & sets units LEDs according
    # to the type of measurement performed. Units in which D should be
    # presented are determined according to range setting, e.g.:
    #   VDC 150mv   - 1uV
    #   VDC 1.5V    - 10uV
    #   VDC 15V     - 100uV
    #   VDC 150V    - 1mV
    #   etc.
    #
    # Then:
    #   Cn = D / ADC(Vx) * ADC(Vcal)
    #   Cn = D / (Vx * k) * Vcal
    #   Cn = Vcal * (D / Vx) / k
    # Where:
    #   D / Vx - establishes relation between D and Vx (sets D units)
    #   k - specifies gain from H-L input posts to ADC input 
    
    vcal = 8.4

    mode_d_over_vx = {
        0: 1e6,
        1: 1e5,
        2: 1e4,
        3: 1e3,
        4: 1e2
    }

    mode_adc_k = {
        0: -100 / 2,
        1: -10 / 2,
        2: -1 / 2,
        3: -10 / 100 / 2,
        4: -1 / 100 / 2
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
        dovx = mode_d_over_vx.get(mode, 1)
        k = mode_adc_k.get(mode, 1)
        c = dovx / k * vcal
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