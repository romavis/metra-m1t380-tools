from serial import Serial


class Comm:
    TIMEOUT = 0.5
    TIMEOUT_CONV = 5

    def __init__(self, port: str):
        self.p = Serial(port, timeout=self.TIMEOUT, write_timeout=self.TIMEOUT)

    def xchg(self, tx: bytes, num_rx: int = 0, rx_timeout: float = -1):
        print(f'write {tx!r}, read {num_rx} bytes')
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
        assert 0 <= mode <= 31
        cmd = (1 if fil else 0) << 5
        cmd |= mode & 0x1F
        cmd &= 0xFF
        self.xchg(bytes((cmd,)))

    def test_comm(self):
        self.xchg(bytes((0b10001000,)))


import time

c = Comm('/dev/ttyACM0')

c.test_comm()
# c.set_mode(20, False)
# mode = 0
# while True:
#     mode = mode + 1
#     mode = mode & 0x1F
#     print(f'set_mode {mode}')
#     c.set_mode(mode, False)
#     time.sleep(0.2)
