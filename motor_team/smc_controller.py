import time

class SMCController:
    def __init__(self, motor, dynamics, max_time, time_slice, utils):
        # smc 상수
        self.C1 = 10
        self.K1 = 0.01

        # 현재값, 에러값
        self.th_1 = 0
        self.dth_1 = 0

        # 토크 상수 N*m / A
        self.Kt = 1.84

        self.motor = motor
        self.dynamics = dynamics

        self.time_slice = time_slice
        self.dt = max_time/self.time_slice
        self.utils = utils

    def execute_dynamics(self, th_d1, dth_d1, ddth_d1, th_d2, dth_d2, ddth_d2):
        # 위치 및 속도 오류 계산
        e1 = th_d1 - self.th_1
        e2 = dth_d1 - self.dth_1

        # 슬라이딩 라인
        s1 = e1 * self.C1 + e2

        # 중간 계산
        m2A_square = self.dynamics.calcM2A_square_numerical(self.th_1, 0)
        m2AB = self.dynamics.calcM2AB_dth1_dth2_numerical(self.th_1, 0, self.dth_1, 0)

        # 최종 토크 신호 계산
        tau1 = m2A_square * (self.C1 * e2 + ddth_d1) + 2 * m2AB + self.K1 * self.utils.ksgn(s1)
        # tau2 = mass_tau2 + coriolis_tau2

        print(f'ddth_d1: {ddth_d1}, m2A_square: {m2A_square},m2AB: {m2AB}, s1: {s1}, tau1: {tau1}')

        # 최종 토크를 전류량으로 변환 >> 단위는 암페어
        current1 = tau1 / self.Kt

        # 데이터 출력
        print(f'전류량: {current1 * 1000:7.3f}mA\t\t'
              f'전류 디지털 (max: 1193): {self.utils.amp2digit(current1)}')

        # 모터로 토크 신호 전송, 현재 상태 업데이트
        self.th_1, self.dth_1 = self.motor.sendCUR(current1)

        # 모터1의 각도, 각속도 피드백
        return self.th_1, self.dth_1, e1, e2
