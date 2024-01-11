import numpy as np
import matplotlib.pyplot as plt

class Trajectory:
    def __init__(self, start_pos, end_pos, start_velocity, max_time=0.5, time_slice=1000):
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

    # 가속도 함수 정의
    def angular_acceleration(self, t):
        if t < self.max_time / 4:
            return (4 * self.max_velocity - 4 * self.start_velocity) / self.max_time
        elif t < 3 * self.max_time / 4:
            return 0
        elif t <= self.max_time:
            return -4 * self.max_velocity / self.max_time
        else:
            return 0  # 시간이 max_time을 초과하면 가속도는 0으로 가정합니다.

    # # 각속도 함수 정의
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

    def plot_reference_real(self, poses, veles):
        # 각도 그래프 플롯
        angular_position_values = self.angular_position()
        plt.subplot(2, 1, 1)
        plt.plot(np.linspace(0, self.max_time, self.time_division), angular_position_values, label='position')
        plt.plot(np.linspace(0, self.max_time, len(poses)), poses, label='real_position')
        plt.xlabel('time')
        plt.legend()

        # 각속도 그래프 플롯
        angular_velocity_values = [self.angular_velocity(t) for t in np.linspace(0, self.max_time, self.time_division)]
        plt.subplot(2, 1, 2)
        plt.plot(np.linspace(0, self.max_time, self.time_division), angular_velocity_values, label='position/s')
        plt.plot(np.linspace(0, self.max_time, len(veles)), veles, label='real_position/s')
        plt.xlabel('time')
        plt.legend()

        plt.tight_layout()
        plt.show()

    def phase_portrait(self):
        velocities = [self.angular_velocity(t) for t in np.linspace(0, self.max_time, self.time_division)]
        poses = self.angular_position()

        plt.figure(figsize=(8, 6))
        plt.plot(poses, velocities, label='Phase Portrait', color='b')
        plt.xlabel('Angular Position (theta)')
        plt.ylabel('Angular Velocity (w)')
        plt.title('Phase Portrait')
        plt.legend()
        plt.grid(True)
        plt.show()

    def execute(self):
        velocities = [self.angular_velocity(t) for t in np.linspace(0, self.max_time, self.time_division)]
        poses = self.angular_position()
        accelerations = [self.angular_acceleration(t) for t in np.linspace(0, self.max_time, self.time_division)]

        return poses, velocities, accelerations

    def execute_rad(self, utils):
        velocities = [utils.pos2rad(self.angular_velocity(t)) for t in np.linspace(0, self.max_time, self.time_division)]
        poses = [utils.pos2rad(pos) for pos in self.angular_position()]
        accelerations = [utils.pos2rad(self.angular_acceleration(t)) for t in np.linspace(0, self.max_time, self.time_division)]

        return poses, velocities, accelerations


if __name__ == "__main__":
    trajectory = Trajectory(start_pos=0, end_pos=1500, start_velocity=0, max_time=0.5, time_slice=1000)
    # pos_list, vel_list = trajectory.execute()
    trajectory.plot()
