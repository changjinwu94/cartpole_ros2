import re
import time
import serial


class Motor:
    """One unaddressed MC2.x ASCII controller; one outstanding request only."""
    def __init__(self, port, baud, timeout):
        self.io = serial.Serial(port, baud, timeout=timeout, write_timeout=timeout,
                                exclusive=True)
        try:
            self.io.reset_input_buffer()
            # Normalize old echo/answer settings while disabled. No EEPROM writes.
            self.raw('DI')
            self.raw('ANSW2')
            time.sleep(0.1)
            self.io.reset_input_buffer()
            self.command('DI')
        except Exception:
            self.emergency_zero()
            self.io.close()
            raise

    def raw(self, command):
        self.io.write((command + '\r').encode('ascii'))

    def reply(self):
        raw = self.io.read_until(b'\n', 128)
        if not raw.endswith(b'\n'):
            raise TimeoutError('motor response incomplete or missing')
        return raw.decode('ascii').strip()

    def command(self, command):
        self.raw(command)
        if self.reply() != 'OK':
            raise RuntimeError('motor rejected ' + command)

    def number(self, command):
        self.raw(command)
        value = self.reply()
        if not re.fullmatch(r'[+-]?\d+', value):
            raise ValueError('unexpected motor response: ' + value)
        return int(value)

    def enable(self):
        self.command('CONTMOD')
        self.command('SOR0')
        self.command('V0')
        self.command('EN')

    def velocity(self, rpm):
        self.command('V' + str(int(round(rpm))))

    def emergency_zero(self):
        # Best-effort independent writes even if responses are lost/desynchronized.
        for command in ('V0',):
            try:
                self.raw(command)
            except Exception:
                pass

    def stop(self, timeout=1.0, stationary_rpm=5.0):
        """Confirm low speed AND bounded position spread over >=60ms.

        Position tolerance is 2 motor counts (~2.75um with current geometry).
        Feedback timestamps bracket host queries, not drive capture times.
        This remains blocking; DI is withheld if confirmation fails.
        """
        self.stop_trace = []
        deadline = time.monotonic() + timeout
        window = []
        def event(kind, **values):
            self.stop_trace.append(dict(kind=kind, host_monotonic_ns=time.monotonic_ns(), **values))
        try:
            event('zero_request')
            self.command('V0')
            event('zero_ack')
            while time.monotonic() < deadline:
                before = time.monotonic_ns()
                pos = self.number('POS')
                pos_after = time.monotonic_ns()
                rpm = self.number('GN')
                after = time.monotonic_ns()
                self.stop_trace.append(dict(kind='feedback', before_ns=before,
                    position_reply_ns=pos_after, after_ns=after,
                    position_counts=pos, motor_rpm=rpm))
                if time.monotonic() >= deadline:
                    break
                if abs(rpm) > stationary_rpm:
                    window = []
                else:
                    window.append((after, pos))
                    # Remove old extremes until the entire candidate window
                    # stays within tolerance; slow accumulated drift cannot
                    # pass by remaining below a per-sample delta threshold.
                    while window and max(p for _, p in window) - min(p for _, p in window) > 2:
                        window.pop(0)
                    if (len(window) >= 3 and
                            window[-1][0] - window[0][0] >= 60_000_000):
                        event('disable_request')
                        self.command('DI')
                        event('disable_ack')
                        self.stop_trace.append(dict(kind='result', confirmed=True,
                            host_monotonic_ns=time.monotonic_ns()))
                        return True
                time.sleep(0.005)
        except Exception as exc:
            event('error', message=str(exc))
        self.emergency_zero()
        event('result', confirmed=False)
        return False

    def close(self):
        self.stop()
        self.io.close()
