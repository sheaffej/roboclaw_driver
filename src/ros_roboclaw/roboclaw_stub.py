class RoboclawStub:

    def __init__(self, comport, rate, timeout=0.01, retries=3):
        # self.comport = comport
        # self.rate = rate
        # self.timeout = timeout
        # self._trystimeout = retries
        # self._crc = 0
        pass

    def ForwardMixed(self, address, val):
        return self._write1(True)

    def BackwardMixed(self, address, val):
        return self._write1(True)

    def TurnRightMixed(self, address, val):
        return self._write1(True)

    def TurnLeftMixed(self, address, val):
        return self._write1(True)

    def _write1(self, ok):
        """Simulates a 1 byte write to the Roboclaw

        Parameters:
        :type ok: bool
            Whether the write should be successful or not

        Returns:
            True if the write response indicates checksum was OK
            False if not
        """
        return ok
