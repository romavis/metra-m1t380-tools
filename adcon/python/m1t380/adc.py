import serial
import random
import logging
import struct
import enum
from typing import Tuple, Union

logger = logging.getLogger(__name__)


@enum.unique
class Mode(enum.IntEnum):
    """
    List of mode indices for MHB8748's set_mode command
    """
    # Volts DC
    VDC_150MV = 0
    VDC_1500MV = 1
    VDC_15V = 2
    VDC_150V = 3
    VDC_1500V = 4
    # Amps DC
    IDC_15MA = 5
    IDC_1500MA = 6
    # Ohms
    OHMS_150R = 7
    OHMS_1500R = 8
    OHMS_15K = 9
    OHMS_150K = 10
    OHMS_1500K = 11
    OHMS_15MEG = 12
    # Volts AC
    VAC_150MV = 13
    VAC_1500MV = 14
    VAC_15V = 15
    VAC_150V = 16
    VAC_1500V = 17
    # Amps AC
    IAC_15MA = 18
    IAC_1500MA = 19
    # AC/DC calibration
    CAL_ACDC_ZERO = 20
    CAL_ACDC_REF = 21
    CAL_ACDC_MEAS_REF = 22
    # Input divider calibration
    CAL_IDIV_ZERO = 23
    CAL_IDIV_CALP = 24
    CAL_IDIV_IZERO = 25
    # ADC calibration
    CAL_ADC_ZERO = 26
    CAL_ADC_CALP = 27
    CAL_ADC_CALN = 28


@enum.unique
class Test(enum.IntEnum):
    TEST_COMM = 1
    TEST_MODE26_SNUL = 2
    TEST_SNP_SNN_SWITCHES = 3
    TEST_SN1P_SN1N_SWITCHES = 4
    TEST_COMPARATOR_K1 = 5
    TEST_COMPARATOR_K2 = 6
    TEST_CALP_CALN = 7


class AdcConnection:
    """
    Class that communicates with 8748 MCU on Metra M1T380's D1639 board.
    The whole pipeline looks like this:
      PC:
        AdcConnection -- PySerial -- ttyACM port -- ..
        .. USB -- Arduino board -- wire harness -- ..
        .. D1639 connector K1 -- optocouplers -- MHB8748

    This class only implements low-level protocol commands.
    """

    def __init__(self, port: str, skip_init: bool = False,
                 init_mode: Mode = Mode.VDC_150V):
        logger.info(f'AdcConnection init: port={port}, skip_init={skip_init}')
        self.p = serial.Serial(port, baudrate=115200)
        self.p.read_all()
        if not skip_init:
            logger.info(f'Setting mode to {init_mode}')
            # Send "set_mode" 256 times to get out of TEST_COMM if
            # it has been running
            for _ in range(256):
                self.set_mode(init_mode)
            # Check Tx & Rx pathways
            self.test_comm()
            # Measure ADC ZERO just to ensure everything works
            self.set_mode(Mode.CAL_ADC_ZERO)
            self.run_meas(1)
            # Go back to selected mode
            self.set_mode(init_mode)

    def _xchg(self, tx: bytes = (), num_rx: int = 0,
              tx_timeout: float = 0.5, rx_timeout: float = 0.5):
        self.p.write_timeout = tx_timeout
        self.p.timeout = rx_timeout
        if tx:
            try:
                self.p.write(tx)
            except serial.SerialTimeoutException as e:
                raise TimeoutError(f'Write timed out (timeout={tx_timeout}s). '
                                   f'Check your HW.') from e
        rx = ()
        if num_rx:
            rx = self.p.read(num_rx)
            if len(rx) != num_rx:
                raise TimeoutError(f'Read timed out '
                                   f'(timeout={rx_timeout}s), read {len(rx)} '
                                   f'bytes, expected {num_rx}. Check your HW.')
        return rx

    def set_mode(self, mode: Union[Mode, int], filter: bool = False):
        """
        Issue 'set_mode' command to MHB8748

        :param mode: See :py:class:`Mode`.
        :param filter: Enable or disable noise filter.
        """
        logger.debug(f'set_mode: mode={mode}, filter={filter}')
        mode = int(mode)
        assert 0 <= mode <= 31
        cmd = mode & 0x1F
        if filter:
            cmd |= 1 << 5
        self._xchg(bytes((cmd,)))

    def run_test(self, test: Union[Test, int]):
        """
        Issue 'run_test' command to MHB8748

        :param test: See :py:class:`Test`.
        """
        logger.debug(f'run_test: test={test}')
        test = int(test)
        assert 1 <= test <= 15
        cmd = 1 << 7
        cmd |= (test & 0xF) << 3
        self._xchg(bytes((cmd,)))

    def run_meas(self, t1_mains_cycles: int) -> Tuple[int, int]:
        """
        Issue 'run_meas' command to MHB8748, and collect measurement result.

        :param t1_mains_cycles: Duration of T1 integration phase in number
            of mains cycles: 1, 10 or 100 (that's 20, 200 or 2000ms @ 50 Hz).
        :return: tuple (T_Snd, T_Sx)
        """
        logger.debug(f'run_meas: T1 = {t1_mains_cycles} mains cycles...')
        dur_bits = {
            1: 0b001,
            10: 0b010,
            100: 0b100
        }
        dur_timeouts = {
            1: 0.5,
            10: 0.5,
            100: 3.0,
        }
        try:
            dur = dur_bits[t1_mains_cycles]
            rx_timeout = dur_timeouts[t1_mains_cycles]
        except KeyError:
            raise ValueError(f'Unsupported duration: {t1_mains_cycles} cycles')
        cmd = 1 << 7
        cmd |= dur & 0x7
        rly = self._xchg(bytes((cmd,)), 5, rx_timeout=rx_timeout)
        # Reply is 5 bytes: {t_snd[0], t_snd[1], t_snd[2], t_sx[0], t_sx[1]}
        # t_snd[2:0] is a 24-bit 2's-complement measurement result
        #   T_Snd = (256*(T_Sn-) + (T_Sn1-)) - (256*(T_Sn+) + (T_Sn1+))
        #   T_Sn-, T_Sn+ - times during which Sn-, Sn+ were closed
        #   T_Sn1-, T_Sn1+ - times during which Sn1-, Sn1+ were closed
        #       (expressed in 10us units)
        # t_sx[1:0] is a 16-bit unsigned T1 phase duration
        #   T_Sx - time during which Sx was closed
        #       (expressed in 80us units)
        # both values are transmitted in little-endian order
        bytes_x = rly[0:3] + b'\0'
        bytes_t_sx = rly[3:5]
        t_snd = struct.unpack('<I', bytes_x)[0]
        t_sx = struct.unpack('<H', bytes_t_sx)[0]
        # Fixup T_Snd sign, since it's a 24-bit 2's-complement
        if t_snd >= 0x800000:
            t_snd -= 0x1000000
        # If T1 duration was 100 cycles, T_Snd is sent divided by 8 to fit into
        # 24 bits. We revert that here.
        if t1_mains_cycles == 100:
            t_snd *= 8
        # Express T_Sx in 10us units, so it aligns with T_Snd.
        t_sx *= 8
        logger.debug(f'run_meas: converted result T_Snd={t_snd}, T_Sx={t_sx}')
        return t_snd, t_sx

    def test_comm(self):
        """
        Enters TEST_COMM, tests protocol loopback and returns MHB8748 to the
        previous state. This can be executed at any time, it does not change
        MREG or anything.
        """
        logger.info('test_comm: starting...')
        r = random.Random()
        r.seed(0)
        seq_tx = list(range(256))
        r.shuffle(seq_tx)
        seq_tx = seq_tx[:255]  # test_comm echoes only 255 bytes
        self.run_test(Test.TEST_COMM)
        seq_rx = self._xchg(bytes(seq_tx), len(seq_tx))
        if len(seq_rx) != len(seq_tx) or tuple(seq_tx) != tuple(seq_rx):
            raise RuntimeError(f'Echo test failed.\n '
                               f'Tx: {seq_tx!r}\n Rx: {seq_rx!r}')
        logger.info(f'test_comm: passed')


def test():
    logging.basicConfig(level=logging.DEBUG)
    AdcConnection('/dev/ttyACM0')


if __name__ == '__main__':
    test()
