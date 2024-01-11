class Control:
    def __init__(self, max_time, time_slice, utils):
        # 게인
        # self.Kpp = 15  # 위치 비례
        # self.Kpi = 0  # 위치 적분
        # self.Kpd = 0  # 위치 미분
        # self.Kvp = 5  # 속도 비례
        # self.Kvi = 2  # 속도 적분
        # self.Kvd = 0.2  # 속도 미분

        self.Kpp = 8  # 위치 비례
        self.Kpi = 10  # 위치 적분
        self.Kpd = 3  # 위치 미분

        self.Kvp = 1.5  # 속도 비례
        self.Kvi = 10  # 속도 적분
        self.Kvd = 0.01  # 속도 미분

        # 현재값, 에러값
        self.present_rad = 0
        self.integral_rad_error = 0
        self.derivative_rad_error = 0
        self.last_rad_error = 0

        self.present_radps = 0
        self.integral_radps_error = 0
        self.derivative_radps_error = 0
        self.last_radps_error = 0

        # 토크 상수 N*m / A
        self.Kt = 1.84

        self.time_slice = time_slice
        self.dt = max_time/self.time_slice

        self.utils = utils

    def execute_dynamics(self, motor, dynamics, desired_rad, desired_radps, desired_radps2, desired_pos2, desired_vel2, desired_acc2):
        # 위치 및 속도 오류 계산
        rad_error = desired_rad - self.present_rad
        radps_error = desired_radps - self.present_radps

        # 위치 적분 오류 업데이트
        self.integral_rad_error += rad_error * self.dt

        # 속도 적분 오류 업데이트
        self.integral_radps_error += radps_error * self.dt

        # 위치 미분 오류 업데이트
        self.derivative_rad_error = (rad_error - self.last_rad_error) / self.dt

        # 속도 미분 오류 업데이트
        self.derivative_radps_error = (radps_error - self.last_radps_error) / self.dt

        # 피드포워드 가속도 항 추가
        rad_signal = self.Kpp * rad_error + self.Kpi * self.integral_rad_error + self.Kpd * self.derivative_rad_error
        radps_signal = self.Kvp * radps_error + self.Kvi * self.integral_radps_error + self.Kvd * self.derivative_radps_error
        print(f'위치 >> P: {self.Kpp * rad_error:.3f}, I: {self.Kpi * self.integral_rad_error:.3f}, D: {self.Kpd * self.derivative_rad_error:.3f}')
        print(f'속도 >> P: {self.Kvp * radps_error:.3f}, I: {self.Kvi * self.integral_radps_error:.3f}, D: {self.Kvd * self.derivative_radps_error:.3f}')

        tau_apostrophe = desired_radps2 + rad_signal + radps_signal
        # tau_apostrophe = rad_signal + radps_signal

        # # 동역학 모델을 사용하여 토크 계산
        mass_tau1, mass_tau2 = dynamics.calcMassTorque_numerical(th1=self.present_rad, th2=0, ddth1=tau_apostrophe, ddth2=0)
        coriolis_tau1, coriolis_tau2 = dynamics.calcCoriolisGravityTorque_numerical(th1=self.present_rad, th2=0, dth1=self.present_radps, dth2=0)

        # 최종 토크 신호 계산
        tau1 = mass_tau1 + coriolis_tau1
        tau2 = (mass_tau2 + coriolis_tau2) * 30

        # 최종 토크를 전류량으로 변환 >> 단위는 암페어
        current1 = tau1 / self.Kt
        # print(f'Radian 에러: {rad_error:.3f}', end=',\t\t')
        # print(f'Radian/s 에러: {radps_error:.3f}', end=',\t\t')

        self.printSignal(control_signal=tau_apostrophe, pos_signal=rad_signal, vel_signal=radps_signal, acc_signal=desired_radps2)

        print(f'전류량: {current1 * 1000:.3f}mA', end='\t\t')
        print(f'전류 디지털: {self.utils.amp2digit(current1)}')

        # 모터로 토크 신호 전송, 현재 상태 업데이트
        self.present_rad, self.present_radps = motor.sendCUR(current1)

        # 이전 오류 업데이트
        self.last_rad_error = rad_error
        self.last_radps_error = radps_error

        return self.present_rad, self.present_radps

    def printSignal(self, control_signal, pos_signal, vel_signal, acc_signal):
        denominator = control_signal
        if denominator != 0:
            pos_share = float(pos_signal / denominator) * 100
            vel_share = float(vel_signal / denominator) * 100
            acc_share = float(acc_signal / denominator) * 100
        else:
            # 적절한 대체 처리
            pos_share = 0  # 또는 다른 적절한 값
            vel_share = 0  # 또는 다른 적절한 값
            acc_share = 0  # 또는 다른 적절한 값

        print(f'[ID:1] '
              f'rad: {pos_signal:7.3f}({pos_share:7.3f}%),\t\t'
              f'radps: {vel_signal:7.3f}({vel_share:7.3f}%),\t'
              f'radps2: {acc_signal:7.3f}({acc_share:7.3f}%)', end='\t\t')
