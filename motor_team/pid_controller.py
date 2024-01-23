import time

class PIDController:
    def __init__(self, motor, dynamics, max_time, time_slice, utils):
        self.Ku = 1133
        self.Tu = 0.1425

        # PD 게인 (906, 0, 16)
        # self.Kp = 0.8 * self.Ku
        # self.Ki = 0
        # self.Kd = self.Kp * self.Tu / 8

        # PID 게인 (679, 9541, 12)
        # self.Kp = 0.6 * self.Ku
        # self.Ki = 2 * self.Kp / self.Tu
        # self.Kd = self.Kp * self.Tu / 8

        # 게인
        self.Kp = 906
        self.Ki = 0
        self.Kd = 36

        # 현재값, 에러값
        self.th_1 = 0
        self.dth_1 = 0
        self.integral_rad_error = 0

        # 토크 상수 N*m / A
        self.Kt = 1.84

        self.motor = motor
        self.dynamics = dynamics

        self.time_slice = time_slice
        self.dt = max_time/self.time_slice
        self.utils = utils

    def execute_dynamics(self, th_d1, dth_d1, ddth_d1, th_d2, dth_d2, ddth_d2):
        # 위치 및 속도 오류 계산
        rad_error = th_d1 - self.th_1
        radps_error = dth_d1 - self.dth_1

        # 위치 적분 오류 업데이트
        self.integral_rad_error += rad_error * self.dt

        # 각도에러, 각속도에러 제어값 계산
        signal = self.Kp * rad_error + self.Ki * self.integral_rad_error + self.Kd * radps_error

        # 질량 매트릭스에 들어갈 tau_apostrophe 계산
        tau_apostrophe = ddth_d1 + signal

        # 동역학 모델을 사용하여 토크 계산
        mass_tau1, mass_tau2 = self.dynamics.calcMassTorque_numerical(th1=self.th_1, th2=0, ddth1=tau_apostrophe, ddth2=0)
        coriolis_tau1, coriolis_tau2 = self.dynamics.calcCoriolisGravityTorque_numerical(th1=self.th_1, th2=0, dth1=self.dth_1, dth2=0)

        # 최종 토크 신호 계산
        tau1 = mass_tau1 + coriolis_tau1
        tau2 = mass_tau2 + coriolis_tau2

        # 최종 토크를 전류량으로 변환 >> 단위는 암페어
        current1 = tau1 / self.Kt

        # 데이터 출력
        print(f'전류량: {current1 * 1000:7.3f}mA\t\t'
              f'전류 디지털 (max: 1193): {self.utils.amp2digit(current1)}')

        # 모터로 토크 신호 전송, 현재 상태 업데이트
        self.th_1, self.dth_1 = self.motor.sendCUR(current1)

        # 모터1의 각도, 각속도 피드백
        return self.th_1, self.dth_1
