from motor_team.interface import Interface
from motor_team.trajectory import Trajectory
from motor_team.dynamics import Dynamics
from motor_team.utils import Utils
from motor_team.control import Control
import signal
import time

if __name__ == "__main__":
    torque_control_flag = True
    # torque_control_flag = False

    end_pos = 1000
    # end_pos = 1500

    # motor time slice
    time_slice, max_time = 160, 0.5  # 15Hz 로 계산::0.5초
    
    # 필요한 객체 생성
    utils = Utils()

    motor = Interface(utils=utils)
    trajectory1 = Trajectory(start_pos=0, end_pos=end_pos, start_velocity=0, max_time=max_time, time_slice=time_slice)
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

    start = time.time()

    if torque_control_flag:
        present_rad_list = []
        present_radps_list = []
        for i in range(time_slice):
            rad, radps = control.execute_dynamics(
                motor=motor, dynamics=dynamics,
                desired_rad=desired_rad_list[i], desired_radps=desired_radps_list[i], desired_radps2=desired_radps2_list[i],
                desired_pos2=desired_positions2[i], desired_vel2=desired_velocities2[i], desired_acc2=desired_accelerations2[i]
            )

            present_rad_list.append(rad)
            present_radps_list.append(radps)

        print()
        print(f'Elapse: {time.time() - start} seconds')
        motor.disableTorque()

        trajectory1.plot_reference_real(
            poses=[utils.rad2pos(rad) for rad in present_rad_list],
            veles=[utils.rad2pos(radps) for radps in present_radps_list]
        )

    else:
        profile_acc = 125
        profile_vel = 500
        # profile_acc = 250
        # profile_vel = 1000

        # 모터 제어
        present_rad_list, present_radps_list = motor.sendCBP(position=end_pos, profile_acc=profile_acc, profile_vel=profile_vel)

        print()
        print(f'Elapse: {time.time() - start} seconds')
        motor.disableTorque()

        trajectory1.plot_reference_real(present_rad_list, present_radps_list)
