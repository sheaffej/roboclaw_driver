import threading

from . roboclaw import Roboclaw

ERRORS = {
    # 0x0000: "Normal",
    0x0001: "M1 over current warning",
    0x0002: "M2 over current warning",
    0x0004: "Emergency Stop warning",
    0x0008: "Temperature1 error",
    0x0010: "Temperature2 error",
    0x0020: "Main batt voltage high error",
    0x0040: "Logic batt voltage high error",
    0x0080: "Logic batt voltage low error",
    0x0100: "M1 driver fault error",
    0x0200: "M2 driver fault error",
    0x0400: "Main batt voltage high warning",
    0x0800: "Main batt voltage low warning",
    0x1000: "Temperature1 warning",
    0x2000: "Temperature2 warning",
    # 0x4000: "M1 home",
    # 0x8000: "M2 home"
}


class RoboclawStats:
    """Holds point-in-time stats values about the motors read from a Roboclaw controller.

    Stats:
        m1_enc_val: Motor 1 encoder value (>= 0)
        m2_enc_val: Motor 2 encoder value (>= 0)
        m1_enc_qpps: Motor 1 encoder speed in QPPS (+/-)
        m2_enc_qpps: Motor 2 encoder speed in QPPS (+/-)
        error_messages: List of error messages occuring during stats reading

    Notes:
        Quadrature encoders have a range of 0 to 4,294,967,295
    """
    def __init__(self):
        self.m1_enc_val = None
        self.m2_enc_val = None
        self.m1_enc_qpps = None
        self.m2_enc_qpps = None
        self.error_messages = []

    def __str__(self):
        return "[M1 enc: {}, qpps: {}]  [M2 enc: {}, qpps: {}]".format(
                self.m1_enc_val, self.m1_enc_qpps,
                self.m2_enc_val, self.m2_enc_qpps
        )


class RoboclawDiagnostics:
    """Holds point-in-time diagnostic values read from the Roboclaw controller.

    Diagnostics:
        m1_current: Motor 1 current (amps)
        m2_current: Motor 2 current (amps)
        temp: Roboclaw controller temperature (C)
        main_battery_v: Voltage of the main battery (V)
        error_messages: List of error messages occuring during stats reading
    """
    def __init__(self):
        self.m1_current = None
        self.m2_current = None
        self.temp1 = None
        self.temp2 = None
        self.main_battery_v = None
        self.logic_battery_v = None
        self.error_messages = []


class RoboclawControl:
    def __init__(self, roboclaw, address=0x80):
        """
        Parameters:
            :param Roboclaw roboclaw: The Roboclaw object from the Python driver library
            :param int address: The serial addresss of the Roboclaw controller
        """
        self._roboclaw = roboclaw
        self._address = address
        self._serial_lock = threading.RLock()

        if isinstance(roboclaw, Roboclaw):
            self._initialize()

    @property
    def roboclaw(self):
        return self._roboclaw

    def _initialize(self):
        # Connect and initialize the Roboclaw controller over serial
        with self._serial_lock:
            try:
                self._roboclaw.Open()
            except Exception:
                # Pass the exception up so it can be logged by the ROS logging facilities
                # Also, if this fais we really can't continue
                raise

            self.stop()
            self._roboclaw.ResetEncoders(self._address)

    def stop(self):
        """Stop Roboclaw.

        Returns: True if the command succeeded, False if failed
        :rtype: bool
        """
        with self._serial_lock:
            # return self.SpeedAccelDistanceM1M2(
            #     accel=0, speed1=0, distance1=0, speed2=0, distance2=0, reset_buffer=1
            # )
            return self.driveM1M2qpps(0, 0, 0)

    def driveM1M2qpps(self, m1_qpps, m2_qpps, max_secs):
        with self._serial_lock:
            return self._roboclaw.SpeedAccelDistanceM1M2(
                self._address,
                accel=max(abs(m1_qpps), abs(m2_qpps)) * 2,
                speed1=m1_qpps, distance1=abs(m1_qpps * max_secs),
                speed2=m2_qpps, distance2=abs(m2_qpps * max_secs),
                buffer=1
            )

    def read_stats(self):
        """Read and return the monitorinng values of the Roboclaw

        Returns: RoboclawStats object containing the current values of the stats:
        :rtype: RoboclawStats
        """
        stats = RoboclawStats()

        # Read encoder value
        with self._serial_lock:
            try:
                response = self._roboclaw.ReadEncM1(self._address)
                if response[0]:
                    stats.m1_enc_val = response[1]
                else:
                    raise ValueError("Enoder1 read failed CRC or number of retries")
            except ValueError as e:
                stats.error_messages.append("Encoder1 value ValueError: {}".format(e.message))
                raise

            try:
                response = self._roboclaw.ReadEncM2(self._address)
                if response[0]:
                    stats.m2_enc_val = response[1]
                else:
                    raise ValueError("Enoder2 read failed CRC or number of retries")
            except ValueError:
                stats.error_messages.append("Encoder2 value ValueError: {}".format(e.message))

            # Read encoder speed
            try:
                response = self._roboclaw.ReadSpeedM1(self._address)
                if response[0]:
                    stats.m1_enc_qpps = response[1]
                else:
                    raise ValueError("Enoder1 speed read failed CRC or number of retries")
            except ValueError as e:
                stats.error_messages.append("Encoder1 speed ValueError: {}".format(e.message))

            try:
                response = self._roboclaw.ReadSpeedM2(self._address)
                if response[0]:
                    stats.m2_enc_qpps = response[1]
                else:
                    raise ValueError("Enoder2 read failed CRC or number of retries")
            except ValueError:
                stats.error_messages.append("Encoder2 speed ValueError: {}".format(e.message))

        return stats

    def read_diag(self):
        """Read and return the diagnostic values of the Roboclaw

        Returns: RoboclawDiagnostics object containing the current values of the diagnostics
        :rtype: RoboclawDiagnostics
        """
        diag = RoboclawDiagnostics()

        with self._serial_lock:
            # Read motor current
            try:
                success, cur1, cur2 = self._roboclaw.ReadCurrents(self._address)
                diag.m1_current = cur1 / 100.0
                diag.m2_current = cur2 / 100.0
            except ValueError as e:
                diag.error_messages.append("Motor currents ValueError: {}".format(e.message))

            # Read Roboclaw temperature
            try:
                success, temp = self._roboclaw.ReadTemp(self._address)
                diag.temp1 = temp / 10.0
            except ValueError as e:
                diag.error_messages.append("Temperature 1 ValueError: {}".format(e.message))

            try:
                success, temp = self._roboclaw.ReadTemp2(self._address)
                diag.temp2 = temp / 10.0
            except ValueError as e:
                diag.error_messages.append("Temperature 2 ValueError: {}".format(e.message))

            # Read main battery voltage
            try:
                success, volts = self._roboclaw.ReadMainBatteryVoltage(self._address)
                diag.main_battery_v = volts / 10.0
            except ValueError as e:
                diag.error_messages.append("Main battery voltage ValueError: {}".format(e.message))

            # Read logic battery voltage
            try:
                success, volts = self._roboclaw.ReadLogicBatteryVoltage(self._address)
                diag.logic_battery_v = volts / 10.0
            except ValueError as e:
                diag.error_messages.append(
                    "Logic battery voltage ValueError: {}".format(e.message))

            # Read errors
            try:
                success, errors = self._roboclaw.ReadError(self._address)
            except ValueError as e:
                diag.error_messages.append("Read status/errors ValueError: {}".format(e.message))

        if errors:
            for err in ERRORS.keys():
                if (errors & err):
                    diag.error_messages.append(ERRORS[err])

        return diag
