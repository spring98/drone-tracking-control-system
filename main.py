from motor_team.interface import Interface
from motor_team.smc_controller import SMCController
from motor_team.trajectory import Trajectory
from motor_team.dynamics import Dynamics
from motor_team.utils import Utils
from motor_team.pid_controller import PIDController
import signal
import time

if __name__ == "__main__":
    # end_pos = 500
    end_pos = 1000

    times = 1

    # motor time slice (175Hz::0.5s)
    time_slice = 175 * times
    max_time = 0.5 * times

    # 필요한 객체 생성
    utils = Utils()
    dynamics = Dynamics()

    motor = Interface(utils=utils)
    trajectory1 = Trajectory(start_pos=0, end_pos=end_pos, start_velocity=0, max_time=max_time, time_slice=time_slice, utils=utils)
    trajectory2 = Trajectory(start_pos=0, end_pos=500, start_velocity=0, max_time=max_time, time_slice=time_slice, utils=utils)

    # control = PIDController(motor=motor, dynamics=dynamics, max_time=max_time, time_slice=time_slice, utils=utils)
    control = SMCController(motor=motor, dynamics=dynamics, max_time=max_time, time_slice=time_slice, utils=utils)

    # 모터 중지를 위한 인터럽트 시그널 함수
    signal.signal(signal.SIGINT, motor.signal_handler)

    # 필요한 경로 생성
    print('MAKE TRAJECTORY')
    desired_rad_list, desired_radps_list, desired_radps2_list = trajectory1.execute_rad()
    desired_positions2, desired_velocities2, desired_accelerations2 = trajectory2.execute_rad()

    print('GO HOME')
    motor.setHome()
    time.sleep(1)

    print('CONTROL START')

    start = time.time()

    th_list = []
    dth_list = []

    e1_list = []
    e2_list = []
    s_list = []
    for i in range(time_slice):
        th, dth, e1, e2, s = control.execute_dynamics(
            th_d1=desired_rad_list[i], dth_d1=desired_radps_list[i], ddth_d1=desired_radps2_list[i],
            th_d2=desired_positions2[i], dth_d2=desired_velocities2[i], ddth_d2=desired_accelerations2[i]
        )

        th_list.append(th)
        dth_list.append(dth)

        e1_list.append(e1)
        e2_list.append(e2)
        s_list.append(s)

    print()
    print(f'Elapse: {time.time() - start} seconds')
    motor.disableTorque()

    # plot
    trajectory1.plot_reference_real(
        poses=[utils.rad2pos(rad) for rad in th_list],
        veles=[utils.rad2pos(radps) for radps in dth_list]
    )

    trajectory1.plot_data(s_list, 's')
    trajectory1.phase_portrait(e1_list, e2_list)
