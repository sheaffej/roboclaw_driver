import threading
from datetime import datetime

MAX_MOTOR_CURRENT = 2.1  # Amps


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

        self._m2_enc_val = 0
        self._m2_enc_qpps = 0

        # Diagnostic state
        self._m1_current = 0.0  # Amps/100
        self._m2_current = 0.0  # Amps/100
        self._temp1 = 30.0 * 10  # Celsius
        self._temp2 = 0.0  # Celsius
        self._main_bat_voltage = 7.4 * 10  # Volts
        self._logic_bat_voltage = 5.0 * 10  # Volts
        self._error_bitmap = 0x0000

        # These mimic the return values of the roboclaw.py library's read & write functions
        self._read_success = 1  # Read success: 1 = success, 0 = failure
        self._write_success = True  # Write success: True/False

        self._max_qpps = 3500

        # Speed Command variables
        self._m1_target_qpps = 0
        self._m2_target_qpps = 0
        self._m1_target_dist = 0
        self._m2_target_dist = 0
        self._target_accel = 0

        # Tracks how far into the commanded distance has been traveled
        # So we can stop once we've traveled the full commanded distance
        self._m1_actual_dist = 0
        self._m2_actual_dist = 0

        self._state_lock = threading.RLock()
        self._last_sim_update = datetime.now()

    def _simulate(self, sim_secs=None):
        """Run by _sim_loop to calculate the next state values.
        This logic is a separate from the _sim_loop so it can easily be called by a unittest.
        """
        with self._state_lock:

            if not sim_secs:
                nowtime = datetime.now()
                sim_secs = (nowtime - self._last_sim_update).total_seconds()
                self._last_sim_update = nowtime

            if self._m1_actual_dist >= self._m1_target_dist:
                self._m1_target_qpps = 0

            if self._m2_actual_dist >= self._m2_target_dist:
                self._m2_target_qpps = 0

            self._m1_enc_qpps = self._m1_target_qpps
            newdist = self._m1_target_qpps * sim_secs
            self._m1_enc_val += newdist
            self._m1_actual_dist += abs(newdist)
            pct = abs(float(self._m1_enc_qpps) / float(self._max_qpps))
            self._m1_current = MAX_MOTOR_CURRENT * pct

            self._m2_enc_qpps = self._m2_target_qpps
            newdist = self._m2_target_qpps * sim_secs
            self._m2_enc_val += newdist
            self._m2_actual_dist += abs(newdist)
            pct = abs(float(self._m2_enc_qpps) / float(self._max_qpps))
            self._m2_current = MAX_MOTOR_CURRENT * pct

    #
    # Methods that match the roboclaw.py libary's functions
    #
    def SpeedAccelDistanceM1M2(self, address, accel, speed1, distance1, speed2, distance2, buffer):
        with self._state_lock:
            self._simulate()
            self._target_accel = accel
            self._m1_target_qpps = speed1
            self._m2_target_qpps = speed2
            self._m1_target_dist = distance1
            self._m2_target_dist = distance2
            self._m1_actual_dist = 0
            self._m2_actual_dist = 0
        return self._write_success

    def ReadEncM1(self, address):
        with self._state_lock:
            self._simulate()
            return (self._read_success, self._m1_enc_val)

    def ReadEncM2(self, address):
        with self._state_lock:
            return (self._read_success, self._m2_enc_val)

    def ReadSpeedM1(self, address):
        with self._state_lock:
            return (self._read_success, self._m1_enc_qpps)

    def ReadSpeedM2(self, address):
        with self._state_lock:
            return (self._read_success, self._m2_enc_qpps)

    def ReadCurrents(self, address):
        with self._state_lock:
            return (self._read_success, self._m1_current, self._m2_current)

    def ReadTemp(self, address):
        with self._state_lock:
            return (self._read_success, self._temp1)

    def ReadTemp2(self, address):
        with self._state_lock:
            return (self._read_success, self._temp2)

    def ReadMainBatteryVoltage(self, address):
        with self._state_lock:
            return (self._read_success, self._main_bat_voltage)

    def ReadLogicBatteryVoltage(self, address):
        with self._state_lock:
            return (self._read_success, self._logic_bat_voltage)

    def ReadError(self, address):
        with self._state_lock:
            return (self._read_success, self._error_bitmap)
