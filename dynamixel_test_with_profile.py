from motor_team.interface_profile_Impl import DynamixelInterfaceWithProfile
from motor_team.trajectory import Trajectory
from motor_team.dynamics import Dynamics
from motor_team.utils import Utils
from motor_team.control import DynamixelControl
import signal
import time

# pos 2000 profile acc: 2729, profile vel: 341

if __name__ == "__main__":
    start_pos, end_pos = 0, 1000
    # start_pos, end_pos = 1000, 0

    # 필요한 객체 생성
    utils = Utils()
    motor = DynamixelInterfaceWithProfile(util=utils)

    # 필요한 경로 생성
    trajectory1 = Trajectory(start_pos=start_pos, end_pos=end_pos, start_velocity=0, max_time=1, time_slice=1000)
    # _, veles, acces = trajectory1.execute()
    # profile_acc = int(utils.pps2rpm(max([abs(acc) for acc in acces])))
    # profile_vel = int(utils.pps2rpm(max([abs(vel) for vel in veles])))

    trajectory1.execute()
    # profile_acc = 125
    # profile_vel = 500
    profile_acc = 250
    profile_vel = 1000

    print(f'profile acc: {profile_acc}, profile vel: {profile_vel}')

    # 모터 중지를 위한 인터럽트 시그널 함수
    signal.signal(signal.SIGINT, motor.signal_handler)

    # HOME 으로 이동
    # motor.setHome()

    time.sleep(1)
    start = time.time()

    # 모터 제어
    poses, veles = motor.sendCBP(position=end_pos, profile_acc=profile_acc, profile_vel=profile_vel)

    print()
    end = time.time()

    print(f'Elapse: {end-start} seconds')

    motor.disableTorque()
    # # motor_team.plot(desired_positions1, desired_velocities1, desired_positions2, desired_velocities2)
    # motor_team.fakePlot(desired_positions1, desired_velocities1, desired_positions2, desired_velocities2)

    # print(poses)
    # print(veles)

    trajectory1.plot_with_profile(poses=poses, veles=veles)
    # trajectory1.

