import logging
import time

from .adc import AdcConnection, Mode

logger = logging.getLogger(__name__)


class CalibrationConsts:
    def __init__(self, adc_vref: float = 7.4):
        self.adc_dix_over_in = 0
        self.adc_din_over_in = 0
        # This is just a "sane" default value, it will give you 20-30%
        # measurement error. The only way to improve it is to calibrate
        # it using a reference voltage source.
        # 18K (R36) * Vref / 10K (R25) / 256 (In1/In)
        self.adc_rin = 18e3 * adc_vref / 10e3 / 256

    def __str__(self) -> str:
        d = {
            'adc_rin': self.adc_rin,
            'adc_dix_over_in': self.adc_dix_over_in,
            'adc_din_over_in': self.adc_din_over_in
        }
        s = ', '.join(f'{k}={v}' for k, v in d.items())
        return f'({s})'


class Measurement:
    def __init__(self, adc: AdcConnection, calibration: CalibrationConsts = None):
        if calibration is None:
            calibration = CalibrationConsts()
        self.adc = adc
        self.cal = calibration

    def _measure_ab_avg(self, dur: int, nmeas: int = 1, dly: float = 0.1):
        assert nmeas > 0
        la = []
        lb = []
        for _ in range(nmeas):
            time.sleep(dly)
            t_snd, t_sx = self.adc.run_meas(dur)
            a = t_snd / t_sx
            b = abs(a)
            la.append(a)
            lb.append(b)
        a = sum(la) / len(la)
        b = sum(lb) / len(lb)
        logger.debug(f'Measured series a:{la}, b:{lb}. Avg: a={a}, b={b}')
        return a, b

    def calibrate_adc_offsets(self, dur: int = 10, nmeas: int = 5,
                              delay: float = 0.1):
        """
        Switches ADC board into ADC_CAL_* modes and calibrates:
            - Ix current offset
            - In+/In- current asymmetry

        :param dur: Measurement duration (1,10,100 cycles)
        :param nmeas: Number of measurements for averaging
        :param delay: Delay between measurements for voltages to settle
        """
        # CAL ZERO
        self.adc.set_mode(Mode.CAL_ADC_ZERO, False)
        a0, b0 = self._measure_ab_avg(dur, nmeas, delay)
        # CAL+
        self.adc.set_mode(Mode.CAL_ADC_CALP, False)
        a1, b1 = self._measure_ab_avg(dur, nmeas, delay)
        if a1 <= 0:
            raise RuntimeError("CAL+ measurement is negative. Check your HW.")
        # CAL-
        self.adc.set_mode(Mode.CAL_ADC_CALN, False)
        a2, b2 = self._measure_ab_avg(dur, nmeas, delay)
        if a2 >= 0:
            raise RuntimeError("CAL- measurement is positive. Check your HW.")
        # Switch Sn+ switches current In+, switch Sn- switches current In-
        # Those currents are defined as follows:
        #   In+ =  In + delta(In)
        #   In- = -In + delta(In)
        # Where delta(In) accounts for asymmetry of currents
        #
        # Calculate adc_din_to_in = delta(In) / In
        self.cal.adc_din_over_in = (a1 + a2 - 2 * a0) / (b1 + b2 - 2 * b0)
        # Calculate delta(Ix) / In - input offset current
        self.cal.adc_dix_over_in = a0 - b0 * self.cal.adc_din_over_in
        logger.info(f'Updated calibration: {self.cal}')

    def get_adc_granularity(self, dur: int, mains_freq: float = 50) -> float:
        """
        Returns ADC scale granularity (in volts) for specified
        T1 conversion phase duration.

        NOTE: this depends on calibration parameters and is anyway an
        approximate value as the actual T1 phase duration is always determined
        by the mains frequency.

        :param dur: Duration of T1 phase
        :param mains_freq: Mains frequency in Hz
        :return: Granularity (in volts)
        """
        assert mains_freq > 0
        assert dur in (1, 10, 100)
        g = self.cal.adc_rin / (dur / mains_freq / 10e-6)
        return g

    def adc_measure(self, dur: int = 10) -> float:
        """
        Runs ADC conversion of specified duration and returns voltage
        measured at the _ADC input_ (pin 3 of I6, board D1639).

        :param dur: Duration of T1 phase
        :return: Voltage in volts
        """
        t_snd, t_sx = self.adc.run_meas(dur)
        a = t_snd / t_sx
        b = abs(a)
        volt = self.cal.adc_rin * (a - b * self.cal.adc_din_over_in - self.cal.adc_dix_over_in)
        return volt
