from serial import Serial
import random
import logging
import sys
import struct
from typing import Tuple

logger = logging.getLogger('adcon')


class MetraD1639:
    MEAS_20MS = 0b001
    MEAS_200MS = 0b010
    MEAS_2000MS = 0b100

    TEST_COMM = 1

    TIMEOUT = 0.5
    TIMEOUT_CONV = 5

    def __init__(self, port: str):
        self.p = Serial(port)

    def xchg(self, tx: bytes = (), num_rx: int = 0, rx_timeout: float = -1):
        #logger.debug(f'xchg: write {len(tx)} bytes ({tx!r}), read {num_rx} bytes')
        self.p.write_timeout = self.TIMEOUT
        self.p.timeout = self.TIMEOUT
        if tx:
            self.p.write(tx)
        rx = ()
        if num_rx:
            if rx_timeout < 0:
                rx_timeout = self.TIMEOUT
            self.p.timeout = rx_timeout
            rx = self.p.read(num_rx)
            if len(rx) != num_rx:
                raise TimeoutError(f'Exchange timed out, read only {len(rx)} bytes, expected {num_rx}')
        return rx

    def set_mode(self, mode: int, fil: bool):
        logger.debug(f'set_mode: mode={mode}, fil={fil}')
        mode = int(mode)
        assert 0 <= mode <= 31
        cmd = mode & 0x1F
        if fil:
            cmd |= 1 << 5
        self.xchg(bytes((cmd,)))

    def run_test(self, test: int):
        logger.debug(f'run_test: test={test}')
        test = int(test)
        assert 1 <= test <= 15
        cmd = 1 << 7
        cmd |= (test & 0xF) << 3
        self.xchg(bytes((cmd,)))

    def run_meas(self, t1_mains_cycles: int) -> Tuple[int, int]:
        logger.debug(f'run_meas: T1 = {t1_mains_cycles} mains cycles...')
        if t1_mains_cycles == 1:
            dur = self.MEAS_20MS
            timeout = 0.5
        elif t1_mains_cycles == 10:
            dur = self.MEAS_200MS
            timeout = 0.8
        elif t1_mains_cycles == 100:
            dur = self.MEAS_2000MS
            timeout = 4.0
        else:
            raise ValueError(f'Unsupported T1 duration: {t1_mains_cycles!r}')
        cmd = 1 << 7
        cmd |= dur & 0x7
        rly = self.xchg(bytes((cmd,)), 5, rx_timeout=timeout)
        # Reply is 5 bytes: {x[0], x[1], x[2], t_sx[0], t_sx[1]}
        # x[2:0] is a 24-bit 2's-complement measurement result
        # t_sx[1:0] is a 16-bit unsigned T1 phase duration
        # both values are transmitted in little-endian order
        bytes_x = rly[0:3] + b'\0'
        bytes_t_sx = rly[3:5]
        x = struct.unpack('<I', bytes_x)[0]
        t_sx = struct.unpack('<H', bytes_t_sx)[0]
        # Fixup x sign, since it's a 24-bit 2's-complement
        if x >= 0x800000:
            x -= 0x1000000
        # If T1 duration was 100 cycles, x is divided by 8, so we revert that
        if dur == self.MEAS_2000MS:
            x *= 8
        # T_Sx value is expressed in 32-cycle units,
        # while X value is expressed in 4-cycle units.
        # We multiply T_Sx by 8 so it is aligned with X and is expressed
        # in 4-cycle units.
        t_sx *= 8
        logger.debug(f'run_meas: ...measured X={x}, T_Sx={t_sx}, X/T_Sx={x/t_sx}')
        return x, t_sx

    def test_comm(self):
        logger.info('test_comm: starting...')
        r = random.Random()
        r.seed(0)
        seq_tx = list(range(256))
        r.shuffle(seq_tx)
        seq_tx = seq_tx[:255]  # test_comm echoes only 255 bytes
        self.run_test(self.TEST_COMM)
        seq_rx = self.xchg(bytes(seq_tx), len(seq_tx))
        if len(seq_rx) != len(seq_tx) or tuple(seq_tx) != tuple(seq_rx):
            logger.error(f'test_comm: ...echo test failed!\n Tx: {seq_tx!r}\n Rx: {seq_rx!r}')
        else:
            logger.info(f'test_comm: ...test passed')


logging.basicConfig(stream=sys.stderr, level=logging.DEBUG)

c = MetraD1639('/dev/ttyACM0')

c.test_comm()

c.set_mode(3, False)

logging.getLogger().setLevel(logging.INFO)


def meas():
    for _ in range(4):
        x, t_sx = c.run_meas(10)
        logger.info(f'X={x:8d}, T_Sx={t_sx:7d}, X/T_Sx={x/t_sx:15f}')


logger.info('------------ CAL ADC ZERO ------------')
c.set_mode(26, False)
meas()
logger.info('------------ CAL ADC CAL+ ------------')
c.set_mode(27, False)
meas()
logger.info('------------ CAL ADC CAL- ------------')
c.set_mode(28, False)
meas()
logger.info('------------ OHMS 150R ------------')
c.set_mode(7, False)
meas()

