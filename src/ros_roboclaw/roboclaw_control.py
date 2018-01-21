from . roboclaw import Roboclaw


class RoboclawStats:
    """Holds point-in-time stats values read from a Roboclaw controller

    Stats:
        error_messages: List of error messages occuring during stats reading
        m1_enc_val: Motor 1 encoder value (>= 0)
        m2_enc_val: Motor 2 encoder value (>= 0)
        m1_enc_qpps: Motor 1 encoder speed in QPPS (+/-)
        m2_enc_qpps: Motor 2 encoder speed in QPPS (+/-)
        m1_current: Motor 1 current (amps)
        m2_current: Motor 2 current (amps)
        temp: Roboclaw controller temperature (C)
        main_battery_v: Voltage of the main battery (V)

    Notes:
        Quadrature encoders have a range of 0 to 4,294,967,295
    """
    def __init__(self):
        self.error_messages = []
        self.m1_enc_val = None
        self.m2_enc_val = None
        self.m1_enc_qpps = None
        self.m2_enc_qpps = None
        self.m1_current = None
        self.m2_current = None
        self.temp = None
        self.main_battery_v = None


class RoboclawControl:
    def __init__(self, roboclaw, address=0x80):
        """
        Parameters:
            roboclaw: The Roboclaw object from the Python driver library
                :type roboclaw: Roboclaw
            address: The serial addresss of the Roboclaw controller
                :type address: int
        """
        self._roboclaw = roboclaw
        self._address = address
        if isinstance(roboclaw, Roboclaw):
            self._initialize()

    def _initialize(self):
        # Connect and initialize the Roboclaw controller over serial
        try:
            self._roboclaw.Open()
        except Exception as e:
            raise IOError(
                "Error connecting Roboclaw port: {}".format(e.message)
            )
        self.forward(0.0)
        self.left(0.0)
        self._roboclaw.ResetEncoders(self._address)

    def stop(self):
        """Stop Roboclaw.

        Returns: True if the command succeeded, False if failed
            :rtype: bool
        """
        return (self.forward(0.0) and self.left(0.0))

    def forward(self, speed_pct):
        """Drive Roboclaw forward using a % of full speed.

        Parameters:
        speed_pct: % speed in the forward direction (i.e. 0.5 for 50%)
            :type speed_pct: float

        Returns: True if the command succeeded, False if failed
            :rtype: bool
        """
        val = int(127 * speed_pct)  # 0 is full stop & 127 is full speed
        return self._roboclaw.ForwardMixed(self._address, val)

    def reverse(self, speed_pct):
        """Drive Roboclaw in revers using a % of full speed.

        Parameters:
        speed_pct: % speed in the reverse direction (i.e. 0.5 for 50%)
            :type speed_pct: float

        Returns: True if the command succeeded, False if failed
            :rtype: bool
        """
        val = int(127 * speed_pct)  # 0 is full stop & 127 is full speed
        return self._roboclaw.BackwardMixed(self._address, val)

    def right(self, rot_speed_pct):
        """Turn Roboclaw right using % of full speed turn.

        Parameters:
        rot_speed_pct: % speed rotation to the right (i.e. 0.5 for 50%)
            :type rot_speed_pct: float

        Returns: True if the command succeeded, False if failed
            :rtype: bool
        """
        val = int(127 * rot_speed_pct)  # 0 is full stop & 127 is full speed
        return self._roboclaw.TurnRightMixed(self._address, val)

    def left(self, rot_speed_pct):
        """Turn Roboclaw left using a % of full speed turn.

        Parameters:
        rot_speed_pct: % speed rotation to the left (i.e. 0.5 for 50%)
            :type rot_speed_pct: float

        Returns: True if the command succeeded, False if failed
            :rtype: bool
        """
        val = int(127 * rot_speed_pct)  # 0 is full stop & 127 is full speed
        return self._roboclaw.TurnLeftMixed(self._address, val)

    def read_stats(self):
        """Read and return the monitorinng values of the Roboclaw

        Returns:
            RoboclawStats object containing the current values of the stats:
                :rtype: RoboclawStats

        """
        stats = RoboclawStats()

        # Read encoder value
        try:
            success, stats.m1_enc_val, crc1 = self._roboclaw.ReadEncM1(self._address)
        except ValueError as e:
            stats.error_messages.append("Encoder1 value ValueError: {}".format(e.message))

        try:
            success, stats.m2_enc_val, crc2 = self._roboclaw.ReadEncM2(self._address)
        except ValueError:
            stats.error_messages.append("Encoder2 value ValueError: {}".format(e.message))

        # Read encoder speed
        try:
            success, stats.m1_enc_qpps, crc1 = self._roboclaw.ReadM1Speed(self._address)
        except ValueError as e:
            stats.error_messages.append("Encoder1 speed ValueError: {}".format(e.message))

        try:
            success, stats.m2_enc_qpps, crc2 = self._roboclaw.ReadM2Speed(self._address)
        except ValueError:
            stats.error_messages.append("Encoder2 speed ValueError: {}".format(e.message))

        # Read motor current
        try:
            stats.m1_current, stats.m2_current, crc1 = self._roboclaw.ReadCurrents(self._address)
        except ValueError as e:
            stats.error_messages.append("Motor currents ValueError: {}".format(e.message))

        # Read Roboclaw temperature
        try:
            success, stats.temp = self._roboclaw.ReadTemp(self._address)
        except ValueError as e:
            stats.error_messages.append("Temperature ValueError: {}".format(e.message))

        # Read main battery voltage
        try:
            success, main_battery_v = self._roboclaw.ReadMainBatteryVoltage(self._address)
        except Exception as e:
            stats.error_messages.append("Main battery voltage ValueError: {}".format(e.message))

        return stats
