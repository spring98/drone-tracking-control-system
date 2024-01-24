import time

class SMCController:
    def __init__(self, motor, dynamics, max_time, time_slice, utils):
        # smc 상수
        # self.C1 = 6
        # self.K1 = 0.05
        self.C1 = 10
        self.K1 = 0.15
        self.C2 = 10
        self.K2 = 0.25

        # 현재값, 에러값
        self.th_1 = 0
        self.dth_1 = 0
        self.th_2 = 0
        self.dth_2 = 0

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

        e3 = th_d2 - self.th_2
        e4 = dth_d2 - self.dth_2

        # 슬라이딩 라인
        s1 = e1 * self.C1 + e2
        s2 = e3 * self.C2 + e4

        # motor1 중간 계산
        m2A_square = self.dynamics.calcM2A_square(self.th_1, self.th_2)
        m2AB_dth1_dth2 = self.dynamics.calcM2AB_dth1_dth2(self.th_1, self.th_2, self.dth_1, self.dth_2)
        m2A_square = self.dynamics.calcM2A_square(self.th_1, self.th_2)
        m2AB_dth1_dth2 = self.dynamics.calcM2AB_dth1_dth2(self.th_1, self.th_2, self.dth_1, self.dth_2)

        # motor2 중간 계산
        m2C = self.dynamics.calcM2C()
        m2AB_dth1_square = self.dynamics.calcM2AB_dth1_square(self.th_1, self.th_2, self.dth_1, self.dth_2)
        m2Ag = self.dynamics.calcM2Ag(self.th_1, self.th_2)

        # 최종 토크 신호 계산
        tau1 = m2A_square * (e2 * self.C1 + ddth_d1) + 2 * m2AB_dth1_dth2 + self.K1 * self.utils.sat(s1, 1)
        tau2 = m2C * (e4 * self.C2 + ddth_d2) - m2AB_dth1_square - m2Ag + self.K2 * self.utils.sat(s2, 2.5)

        # print(f'A: {m2A_square * (self.C1 * e2 + ddth_d1):7.3f}, B: {2 * m2AB_dth1_dth2:7.3f}, C: {self.K1 * self.utils.sat(s1, 5):7.3f}, '
        #       f's1: {s1:7.3f}, e1: {e1:7.3f}, e2: {e2:7.3f}, tau1: {tau1:7.3f}')
        print(f'A: {m2C * (e4 * self.C2 + ddth_d2):7.3f}, B1: {- m2AB_dth1_square:7.3f}, B2: {- m2Ag:7.3f}, '
              f'C: {self.K2 * self.utils.sat(s2, 5):7.3f}, 합: {tau2:7.3f}')

        # 최종 토크를 전류량으로 변환 >> 단위는 암페어
        # current1 = tau1 / self.Kt
        current2 = tau2 / self.Kt

        # 데이터 출력
        # print(f'전류량: {current1 * 1000:7.3f}mA\t\t'
        #       f'전류 디지털 (max: 1193): {self.utils.amp2digit(current1)}')

        # 모터로 토크 신호 전송, 현재 상태 업데이트
        # self.th_1, self.dth_1 = self.motor.sendCUR(current1)
        self.th_2, self.dth_2 = self.motor.sendCUR(current2)

        # 모터1의 각도, 각속도 피드백
        # return self.th_1, self.dth_1, e1, e2, s1, self.C1
        return self.th_2, self.dth_2, e3, e4, s2, self.C2
