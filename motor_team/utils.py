import numpy as np

class Utils:
    def pos2rad(self, value):
        pos = self.protectOverflow(value)
        rad = (2 * np.pi / 4096) * pos

        return rad

    def rad2pos(self, rad):
        pos = rad * 4096 / (2 * np.pi)

        return pos

    def protectOverflow(self, value):
        # value가 int 타입인지 확인
        if isinstance(value, int):
            if value & 0x80000000:
                value -= 0x100000000
        return value

    # 0.229: read 로 받아들인 값 1눈금 당 0.229 rpm 이라는 뜻
    def rpm2pps(self, value):
        rpm = self.protectOverflow(value)
        rps = rpm * 0.229 / 60  # Convert RPM to RPS
        pos_per_sec = rps * 4096  # Convert RPS to pos/sec

        return pos_per_sec

    def rpm2radps(self, value):
        rpm = self.protectOverflow(value)
        rps = rpm * 0.229 / 60  # Convert RPM to RPS
        rad_per_sec = rps * 2 * np.pi  # Convert RPS to rad/sec

        return rad_per_sec

    def pps2rpm(self, pps):
        rps = pps / 4096
        rpm = rps * 60 / 0.229

        return rpm

    # 1digit 당 2.69mA 이고, 들어 오는 값이 A 임.
    def amp2digit(self, ampere):
        digit = ampere / 0.00269

        return int(digit)

    def sgn(self, s):
        if s > 0:
            return 1
        elif s < 0:
            return -1
        else:
            return 0

    def sat(self, s, threshold=1):
        if s > threshold:
            return 1
        elif s < -threshold:
            return -1
        else:
            return s / threshold
