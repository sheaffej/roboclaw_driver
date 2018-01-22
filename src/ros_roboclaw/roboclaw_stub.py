class RoboclawStub:

    def __init__(self, comport, rate, timeout=0.01, retries=3):
        # Parameters sent to the real Roboclaw
        self.comport = comport
        self.rate = rate
        self.timeout = timeout
        self._trystimeout = retries

        # Testing stub state
        self._m1_enc_val = 0
        self._m1_enc_qpps = 0
        self._m1_current = 0.0  # Amps/100

        self._m2_enc_val = 0
        self._m2_enc_qpps = 0
        self._m2_current = 0.0  # Amps/100

        self._temp = 30.0  # Celsius
        self._main_bat_voltage = 7.4  # Volts

        # These mimic the return values of the roboclaw.py library's read & write functions
        self._read_success = 1  # Read success: 1 = success, 0 = failure
        self._write_success = True  # Write success: True/False

        self._max_qpps = 3500

    #
    # Methods that match the roboclaw.py libary's functions
    #
    def ForwardMixed(self, address, val):
        self._m1_enc_qpps = int(self._max_qpps * self._byte_to_pctdecimal(val))
        self._m2_enc_qpps = int(self._max_qpps * self._byte_to_pctdecimal(val))

        self._m1_enc_val += self._m1_enc_qpps * 1
        self._m2_enc_val += self._m2_enc_qpps * 1
        return self._write_success

    def BackwardMixed(self, address, val):
        return self._write_success

    def TurnRightMixed(self, address, val):
        return self._write_success

    def TurnLeftMixed(self, address, val):
        return self._write_success

    def ReadEncM1(self, address):
        return (self._read_success, self._m1_enc_val)

    def ReadEncM2(self, address):
        return (self._read_success, self._m2_enc_val)

    def ReadM1Speed(self, address):
        return (self._read_success, self._m1_enc_qpps)

    def ReadM2Speed(self, address):
        return (self._read_success, self._m2_enc_qpps)

    def ReadCurrents(self, address):
        return (self._read_success, self._m1_current, self._m2_current)

    def ReadTemp(self, address):
        return (self._read_success, self._temp)

    def ReadMainBatteryVoltage(self, address):
        return (self._read_success, self._main_bat_voltage)

    #
    # Testing methods to set stub's behavior
    #
    def SetEncoderValues(self, m1_val, m2_val):
        """Sets the testing Motor1 and Motor2 encoder count values.

        Parameters: unsigned 32-bit int values for M1 and M2
        :type m1_val: int
        :type m2_val: int
        """
        self._m1_enc_val = m1_val
        self._m2_enc_val = m2_val

    def SetEncoderSpeeds(self, m1_qpps, m2_qpps):
        """Sets the testing Motor1 and Motor2 encoder QPPS values.

        Parameters: Signed 32-bit int values for M1 and M2
        :type m1_qpps: int
        :type m2_qpps: int
        """
        self._m1_enc_qpps = m1_qpps
        self._m2_enc_qpps = m2_qpps

    def SetMotorCurrents(self, m1_amps, m2_amps):
        """Sets the testing Motor1 and Motor2 current values in Amps.

        Parameters: unsigned flat values for M1 and M2
        :type m1_amps: float
        :type m2_amps: float
        """
        self._m1_current = m1_amps/100.0
        self._m2_current = m2_amps/100.0

    def SetTemp(self, temp):
        """Sets the testing Roboclaw controller temp1.

        Parameter: Unsigned float value in Celsius
        :type temp: float
        """
        self._temp = temp

    def SetMainBatteryVoltage(self, volts):
        """Sets the testing main battery voltage as seen by Roboclaw controller.

        Parameter: Signed float value in Volts
        :type temp: float
        """
        self._main_bat_voltage = volts

    def SetReadSuccess(self, success):
        """Sets the stub's read function's return value.
        True (1) = success, False(0) = failure

        Parameter: success
        :type success: bool
        """
        if success:
            self._read_success = 1
        else:
            self._read_success = 0

    def SetWriteSuccess(self, success):
        """Sets the stub's write function's return value.
        True = success, False = failure

        Parameter: success
        :type success: bool
        """
        self._write_success = success

    def _byte_to_pctdecimal(self, byteval):
        return byteval / 127.0
