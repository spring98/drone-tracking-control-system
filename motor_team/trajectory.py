import numpy as np
import matplotlib.pyplot as plt
from motor_team.utils import Utils

class Trajectory:
    def __init__(self, start_pos, end_pos, start_velocity, utils, max_time=0.5, time_slice=1000):
        # 초기 각도
        self.start_pos = start_pos
        # 도착 각도
        self.end_pos = end_pos
        # 초기 속도
        self.start_velocity = start_velocity
        # 도달 하기 까지 걸리는 시간 설정
        self.max_time = max_time

        # 최대 속도 (위의 값들로 자동 으로 유도)
        self.max_velocity = 4 / (3 * self.max_time) * (self.end_pos - self.start_pos) - (1 / 6 * self.start_velocity)

        # 시간을 몇 등분 으로 나눌지 결정
        self.time_division = time_slice

        # utils 객체 주입
        self.utils = utils

    # # 가속도 함수 정의
    def angular_acceleration(self, t):
        if t < self.max_time / 4:
            return (4 * self.max_velocity - 4 * self.start_velocity) / self.max_time
        elif t < 3 * self.max_time / 4:
            return 0
        elif t <= self.max_time:
            return -4 * self.max_velocity / self.max_time
        else:
            return 0  # 시간이 max_time을 초과하면 가속도는 0으로 가정합니다.

    # 각속도 함수 정의
    def angular_velocity(self, t):
        if t < self.max_time / 4:
            return ((4 * self.max_velocity - 4 * self.start_velocity) / self.max_time) * t + self.start_velocity
        elif t < 3 * self.max_time / 4:
            return self.max_velocity
        else:
            return self.max_velocity - (4 * self.max_velocity / self.max_time) * (t - 3 * self.max_time / 4)

    # 각도 함수 정의 (적분을 통해)
    def angular_position(self):
        theta = self.start_pos  # 초기 각도로 시작
        dt = self.max_time / self.time_division  # 시간 증분값 (적분 정밀도 조절용)
        angular_positions = []  # 시간에 따른 각도 저장용 리스트

        for ti in np.linspace(0, self.max_time, self.time_division):
            theta += self.angular_velocity(ti) * dt
            angular_positions.append(theta)

        return np.array(angular_positions)

    # # 각도 함수 정의 --> 지글러 니콜스 용
    # def angular_position(self):
    #     theta = 500
    #     angular_positions = []  # 시간에 따른 각도 저장용 리스트
    #
    #     for _ in np.linspace(0, self.max_time, self.time_division):
    #         angular_positions.append(theta)
    #
    #     return np.array(angular_positions)
    #
    # # 가속도 함수 정의 --> 지글러 니콜스 용
    # def angular_acceleration(self, t):
    #     return 0
    #
    # # 각속도 함수 정의 --> 지글러 니콜스 용
    # def angular_velocity(self, t):
    #    return 0

    def plot(self):
        # 각속도 그래프 플롯
        angular_velocity_values = [self.angular_velocity(t) for t in np.linspace(0, self.max_time, self.time_division)]
        plt.figure(figsize=(10, 5))
        plt.subplot(2, 1, 2)
        plt.plot(np.linspace(0, self.max_time, self.time_division), angular_velocity_values, label='position/s')
        plt.xlabel('time')
        plt.legend()

        # 각도 그래프 플롯
        angular_position_values = self.angular_position()
        plt.subplot(2, 1, 1)
        plt.plot(np.linspace(0, self.max_time, self.time_division), angular_position_values, label='position')
        plt.xlabel('time')
        plt.legend()

        plt.tight_layout()
        plt.show()

    def plot_data(self, datas, label):
        # 범용 그래프 플롯
        plt.figure(figsize=(10, 5))
        plt.plot(np.linspace(0, self.max_time, self.time_division), datas, label=label)
        plt.xlabel('time')
        plt.legend()

        plt.tight_layout()
        plt.show()

    def plot_reference_real(self, poses, veles):
        # 각도 그래프 플롯
        angular_position_values = self.angular_position()
        plt.subplot(2, 1, 1)
        plt.plot(np.linspace(0, self.max_time, self.time_division), angular_position_values, label='reference-position')
        plt.plot(np.linspace(0, self.max_time, len(poses)), poses, label='present-position')
        plt.xlabel('time')
        plt.legend()

        # 각속도 그래프 플롯
        angular_velocity_values = [self.angular_velocity(t) for t in np.linspace(0, self.max_time, self.time_division)]
        plt.subplot(2, 1, 2)
        plt.plot(np.linspace(0, self.max_time, self.time_division), angular_velocity_values, label='reference-position/s')
        plt.plot(np.linspace(0, self.max_time, len(veles)), veles, label='present-position/s')
        plt.xlabel('time')
        plt.legend()

        plt.tight_layout()
        plt.show()

    def phase_portrait(self, x_list, y_list, C1):
        plt.figure(figsize=(8, 6))

        # 전체 데이터를 파란색으로 표시
        plt.plot(x_list, y_list, label='Phase Portrait', color='b')

        # 처음 10개 데이터만 빨간색으로 표시
        # min 함수는 리스트가 10개 미만일 경우를 고려
        plt.plot(x_list[:min(100, len(x_list))], y_list[:min(100, len(y_list))], color='r', linestyle=':')

        # x1의 값들에 대해 s = 0을 만족하는 x2(e2)의 값을 계산
        x1_vals = np.linspace(min(x_list), max(x_list), len(x_list))
        x2_vals = -C1 * x1_vals
        plt.plot(x1_vals, x2_vals, 'k--', label='s = 0')  # 'k--'는 검은색 점선을 의미

        plt.xlabel('e1')
        plt.ylabel('e2')
        plt.title('Phase Portrait')
        plt.legend()
        plt.grid(True)
        plt.show()

    def execute(self):
        velocities = [self.angular_velocity(t) for t in np.linspace(0, self.max_time, self.time_division)]
        poses = self.angular_position()
        accelerations = [self.angular_acceleration(t) for t in np.linspace(0, self.max_time, self.time_division)]

        return poses, velocities, accelerations

    def execute_rad(self):
        velocities = [self.utils.pos2rad(self.angular_velocity(t)) for t in np.linspace(0, self.max_time, self.time_division)]
        poses = [self.utils.pos2rad(pos) for pos in self.angular_position()]
        accelerations = [self.utils.pos2rad(self.angular_acceleration(t)) for t in np.linspace(0, self.max_time, self.time_division)]

        return poses, velocities, accelerations


if __name__ == "__main__":
    trajectory = Trajectory(start_pos=0, end_pos=1500, start_velocity=0, max_time=0.5, time_slice=1000, utils=Utils())
    # pos_list, vel_list = trajectory.execute()
    trajectory.plot()
