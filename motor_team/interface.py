from dynamixel_sdk import *


# noinspection,DuplicatedCode
class Interface:
    def __init__(self, utils):
        # 환경 변수
        self.DEVICE_NAME = '/dev/tty.usbserial-FT66WBIV' # Linux: "/dev/ttyUSB*"
        self.PROTOCOL_VERSION = 2.0

        # 모터 아이디
        self.DXL_ID1 = 4

        # 모터 프로퍼티 주소
        self.ADDR_TORQUE_ENABLE = 64
        self.ADDR_GOAL_CURRENT = 102
        self.ADDR_GOAL_POSITION = 116
        self.ADDR_GOAL_VELOCITY = 104
        self.ADDR_PRESENT_POSITION = 132
        self.ADDR_PRESENT_VELOCITY = 128

        self.ADDR_OPERATING_MODE = 11

        # 모터 프로퍼티 데이터
        # self.BAUD_RATE = 57600
        self.BAUD_RATE = 3000000
        self.TORQUE_ENABLE = 1
        self.TORQUE_DISABLE = 0

        # 모터 컨트롤 상수
        self.DXL_MOVING_STATUS_THRESHOLD = 3  # Dynamixel moving status threshold

        # 모터 컨트롤 객체
        self.PORT_HANDLER = None
        self.PACKET_HANDLER = None

        # 모터 컨트롤 객체 초기화
        self.initializeHandler()
        self.enableTorque()

        # 모터 테스트 변수
        self.running = True

        # 유틸
        self.utils = utils

    def __del__(self):
        self.disableTorque()
        self.PORT_HANDLER.closePort()

    def initializeHandler(self):
        self.PORT_HANDLER = PortHandler(self.DEVICE_NAME)
        self.PACKET_HANDLER = PacketHandler(self.PROTOCOL_VERSION)

        # Open port
        if self.PORT_HANDLER.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Program terminate...")
            quit()

        # Set port baudrate
        if self.PORT_HANDLER.setBaudRate(self.BAUD_RATE):
            print("Succeeded to change the baud rate")
        else:
            print("Failed to change the baud rate")
            print("Program terminate...")
            quit()

    def enableTorque(self):
        # Enable Dynamixel Torque
        result, error = self.PACKET_HANDLER.write1ByteTxRx(self.PORT_HANDLER, self.DXL_ID1, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
        self.printLog(result, error)

    def disableTorque(self):
        # Disable Dynamixel Torque
        result, error = self.PACKET_HANDLER.write1ByteTxRx(self.PORT_HANDLER, self.DXL_ID1, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
        self.printLog(result, error)

    def signal_handler(self, sig, frame):
        print(f'You pressed Ctrl+C! {sig} {frame}')
        self.running = False

    def printLog(self, result, error):
        if result != COMM_SUCCESS:
            print("%s" % self.PACKET_HANDLER.getTxRxResult(result))
        elif error != 0:
            print("%s" % self.PACKET_HANDLER.getRxPacketError(error))

    def readData(self):
        # Read CURRENT POSITION
        ID1_CURRENT_POSITION, _, _ = self.PACKET_HANDLER.read4ByteTxRx(self.PORT_HANDLER, self.DXL_ID1, self.ADDR_PRESENT_POSITION)

        # Read CURRENT VELOCITY
        ID1_CURRENT_VELOCITY, _, _ = self.PACKET_HANDLER.read4ByteTxRx(self.PORT_HANDLER, self.DXL_ID1, self.ADDR_PRESENT_VELOCITY)

        ID1_CURRENT_POSITION = self.utils.pos2rad(ID1_CURRENT_POSITION)
        ID1_CURRENT_VELOCITY = self.utils.rpm2radps(ID1_CURRENT_VELOCITY)

        return ID1_CURRENT_POSITION, ID1_CURRENT_VELOCITY

    def setHome(self):
        # 위치 제어 모드로 변경
        self.disableTorque()
        self.PACKET_HANDLER.write2ByteTxRx(self.PORT_HANDLER, self.DXL_ID1, self.ADDR_OPERATING_MODE, 1)
        self.enableTorque()

        Kp = 1  # 비례 게인
        Ki = 0.02  # 적분 게인
        Kd = 0.1  # 미분 게인

        # PID 오차 초기화
        error_sum1 = 0
        last_error1 = 0

        while True:
            # 현재 위치 읽기
            ID1_PRESENT_POSITION, _, _ = self.PACKET_HANDLER.read4ByteTxRx(self.PORT_HANDLER, self.DXL_ID1, self.ADDR_PRESENT_POSITION)

            # 오버플로우 방지
            ID1_PRESENT_POSITION = self.utils.protectOverflow(ID1_PRESENT_POSITION)

            # 오차 계산
            error1 = 0 - ID1_PRESENT_POSITION

            # 오차 누적
            error_sum1 += error1

            # 미분 계산
            delta_error1 = error1 - last_error1

            # PID 제어
            goal = Kp * error1 + Ki * error_sum1 + Kd * delta_error1
            print(f'Kp: {Kp * error1:.3f}, Ki: {Ki * error_sum1:.3f}, Kd: {Kd * delta_error1:.3f}, 합: {goal:.3f}')

            goal = int(goal * 0.1)

            # 모터 제어
            self.PACKET_HANDLER.write4ByteTxRx(self.PORT_HANDLER, self.DXL_ID1, self.ADDR_GOAL_VELOCITY, goal)

            # 이전 오차 업데이트
            last_error1 = error1

            # 출력
            print("[ID:%03d] PRESENT_POS:%03d  GOAL_POS:%03d" % (self.DXL_ID1, ID1_PRESENT_POSITION, goal), end='\t\t')

            # 종료 조건
            if abs(error1) < self.DXL_MOVING_STATUS_THRESHOLD:
                print('break...')
                # 토크 해제
                self.disableTorque()

                # 위치 제어 모드로 변경
                self.PACKET_HANDLER.write2ByteTxRx(self.PORT_HANDLER, self.DXL_ID1, self.ADDR_OPERATING_MODE, 0)

                # 토크 인가
                self.enableTorque()
                break

            # 사용자 종료
            if not self.running:
                quit()

    # # Send Current
    # def sendCUR(self, goal_current):
    #     # 사용자 종료
    #     if not self.running:
    #         quit()
    #
    #     print(f'전류 디지털(-1193 ~ 1193): {self.utils.amp2digit(goal_current)}')
    #
    #     # Write GOAL PWM
    #     self.PACKET_HANDLER.write2ByteTxRx(self.PORT_HANDLER, self.DXL_ID1, self.ADDR_GOAL_CURRENT, self.utils.amp2digit(goal_current))
    #
    #     return self.readData()

    def sendCUR(self, goal_current):
        # 사용자 종료 확인
        if not self.running:
            quit()

        # GroupSyncWrite instance for goal current
        groupSyncWriteCurrent = GroupSyncWrite(self.PORT_HANDLER, self.PACKET_HANDLER, self.ADDR_GOAL_CURRENT, 2)

        # Allocate goal current value into byte array
        param_goal_current = [DXL_LOBYTE(self.utils.amp2digit(goal_current)),
                              DXL_HIBYTE(self.utils.amp2digit(goal_current))]

        # Add goal current value to the Syncwrite storage
        groupSyncWriteCurrent.addParam(self.DXL_ID1, param_goal_current)

        # Syncwrite goal current
        groupSyncWriteCurrent.txPacket()

        # Clear syncwrite parameter storage
        groupSyncWriteCurrent.clearParam()

        # GroupSyncRead instance for present position and velocity
        groupSyncReadPosition = GroupSyncRead(self.PORT_HANDLER, self.PACKET_HANDLER, self.ADDR_PRESENT_POSITION, 4)
        groupSyncReadVelocity = GroupSyncRead(self.PORT_HANDLER, self.PACKET_HANDLER, self.ADDR_PRESENT_VELOCITY, 4)

        # Add parameter storage for present position and velocity
        groupSyncReadPosition.addParam(self.DXL_ID1)
        groupSyncReadVelocity.addParam(self.DXL_ID1)

        # 여기서 가장 시간이 오래 걸리는 중 (0.029s)
        # Syncread present position and velocity
        groupSyncReadPosition.txRxPacket()
        groupSyncReadVelocity.txRxPacket()

        # Get present position and velocity
        dxl1_present_position = groupSyncReadPosition.getData(self.DXL_ID1, self.ADDR_PRESENT_POSITION, 4)
        dxl1_present_velocity = groupSyncReadVelocity.getData(self.DXL_ID1, self.ADDR_PRESENT_VELOCITY, 4)

        # Clear syncread parameter storage
        groupSyncReadPosition.clearParam()
        groupSyncReadVelocity.clearParam()

        return self.utils.pos2rad(dxl1_present_position), self.utils.rpm2radps(dxl1_present_velocity)

