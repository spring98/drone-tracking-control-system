
class Controller:
    def __init__(self, max_time, time_slice, utils):
        # 현재값, 에러값
        self.present_rad = 0
        self.present_radps = 0
        self.integral_rad_error = 0

        # 토크 상수 N*m / A
        self.Kt = 1.84

        self.time_slice = time_slice
        self.dt = max_time / self.time_slice
        self.utils = utils

    def execute_dynamics(self, motor, dynamics,
                         desired_rad, desired_radps, desired_radps2, desired_pos2, desired_vel2, desired_acc2):
        pass


