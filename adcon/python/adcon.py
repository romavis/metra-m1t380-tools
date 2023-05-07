import logging
import sys
import argparse
import time
from m1t380 import AdcConnection, Mode, Measurement

logger = logging.getLogger(__name__)


def main():
    logging.basicConfig(stream=sys.stderr, level=logging.INFO)

    parser = argparse.ArgumentParser()
    parser.add_argument(
        '-p', '--port',
        type=str,
        required=True,
        help='Serial port to use (e.g. /dev/ttyACM0)'
    )

    args = parser.parse_args()

    adc = AdcConnection(args.port)
    meas = Measurement(adc)

    meas.calibrate_adc_offsets()

    # Report ADC granularity
    logger.info('ADC granularity:')
    for dur in (1, 10, 100):
        logger.info(f' dur={dur:3d}: {meas.get_adc_granularity(dur):10.7f} V')
    # Measure
    logger.info('----- MEASURE ADC CAL ZERO -----')
    adc.set_mode(Mode.CAL_ADC_ZERO, False)
    for _ in range (6):
        time.sleep(0.2)
        v = meas.adc_measure(10)
        logger.info(f'{v:10.7f} V')
    logger.info('----- MEASURE ADC CAL+ -----')
    adc.set_mode(Mode.CAL_ADC_CALP, False)
    for _ in range (6):
        time.sleep(0.1)
        v = meas.adc_measure(10)
        logger.info(f'{v:10.7f} V')
    logger.info('----- MEASURE ADC CAL- -----')
    adc.set_mode(Mode.CAL_ADC_CALN, False)
    for _ in range (6):
        time.sleep(0.1)
        v = meas.adc_measure(10)
        logger.info(f'{v:10.7f} V')

    logger.info('----- MEASURE VDC, RANGE 150V -----')
    while True:
        adc.set_mode(Mode.VDC_150V, True)
        for _ in range(20):
            # Mode 3 - ADC input is:
            #   0.5 (I5 MUX) * -10 (input amp) * Vin * 0.010092 (input div)
            # Of course, above should be calibrated via measurement.
            k = 0.5 * -10 * 0.010092
            v = meas.adc_measure(10) / k
            logger.info(f'{v:10.4f} V')
            time.sleep(0.05)
        # calibrate ADC
        meas.calibrate_adc_offsets(dur=10, nmeas=1)


main()
