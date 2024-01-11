from motor_team.interface import Interface
from motor_team.trajectory import Trajectory
from motor_team.dynamics import Dynamics
from motor_team.utils import Utils
from motor_team.control import Control
import signal
import time

if __name__ == "__main__":
    # motor time slice
    time_slice, max_time = 15, 0.5 # 15Hz 로 계산::0.5초
    # time_slice, max_time = 31, 1 # 31Hz 로 계산::1초
    # time_slice, max_time = 62, 2 # 62Hz 로 계산::2초
    # time_slice, max_time = 124, 4 # 124Hz 로 계산::4초
    # time_slice, max_time = 156, 5 # 156Hz 로 계산::5초
    # time_slice, max_time = 625, 20 # 625Hz 로 계산::20초

    # 필요한 객체 생성
    utils = Utils()

    motor = Interface(utils=utils)
    trajectory1 = Trajectory(start_pos=0, end_pos=1000, start_velocity=0, max_time=max_time, time_slice=time_slice)
    trajectory2 = Trajectory(start_pos=0, end_pos=500, start_velocity=0, max_time=max_time, time_slice=time_slice)

    dynamics = Dynamics()
    control = Control(max_time=max_time, time_slice=time_slice, utils=utils)

    # 모터 중지를 위한 인터럽트 시그널 함수
    signal.signal(signal.SIGINT, motor.signal_handler)

    # 필요한 경로 생성
    print('MAKE TRAJECTORY')
    desired_rad_list, desired_radps_list, desired_radps2_list = trajectory1.execute_rad(utils=utils)
    desired_positions2, desired_velocities2, desired_accelerations2 = trajectory2.execute_rad(utils=utils)

    print('GO HOME')
    motor.setHome()
    time.sleep(1)

    print('CONTROL START')
    present_rad_list = []
    present_radps_list = []
    present_radps2_list = []

    start = time.time()
    for i in range(time_slice):
        rad, radps, radps2 = control.execute_dynamics(motor=motor, dynamics=dynamics,
                                              desired_rad=desired_rad_list[i], desired_radps=desired_radps_list[i], desired_radps2=desired_radps2_list[i],
                                              desired_pos2=desired_positions2[i], desired_vel2=desired_velocities2[i], desired_acc2=desired_accelerations2[i])

        present_rad_list.append(rad)
        present_radps_list.append(radps)
        present_radps2_list.append(radps2)

    end = time.time()
    print(f'Elapse: {end-start} seconds')

    motor.disableTorque()

    trajectory1.plot_reference_real(
        poses=[utils.rad2pos(rad) for rad in present_rad_list],
        veles=[utils.rad2pos(radps) for radps in present_radps_list]
    )
    # trajectory1.plot_reference_real(poses=present_rad_list, veles=present_radps_list)
