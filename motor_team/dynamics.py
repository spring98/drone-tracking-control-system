import sympy as sp

class Dynamics:
    def __init__(self):
        # 심볼들을 정의합니다.
        self.theta1, self.theta2, self.dtheta1, self.dtheta2, self.ddtheta1, self.ddtheta2 \
            = sp.symbols('theta1 theta2 dtheta1 dtheta2 ddtheta1 ddtheta2')
        self.m2, self.g, self.l2, self.l3 = sp.symbols('m2 g l2 l3')
        self.tau1, self.tau2 = sp.symbols('tau1 tau2')
        self.mass_tau1, self.mass_tau2 = sp.symbols('mass_tau1 mass_tau2')
        self.coriolis_tau1, self.coriolis_tau2 = sp.symbols('coriolis_tau1 coriolis_tau2')

        self.A = self.l2 * sp.sin(self.theta2) - self.l3 * sp.cos(self.theta2)
        self.B = self.l3 * sp.sin(self.theta2) + self.l2 * sp.cos(self.theta2)

        # 질량 매트릭스
        self.M = sp.Matrix([
            [self.m2 * self.A ** 2, 0],
            [0, self.m2 * (self.l2 ** 2 + self.l3 ** 2)]
        ])

        # 코리올리스와 원심력 매트릭스
        self.V = sp.Matrix([
            [2 * self.m2 * self.A * self.B * self.dtheta1 * self.dtheta2],
            [-self.m2 * self.A * self.B * self.dtheta1 ** 2]
        ])

        # 중력 매트릭스
        self.G = sp.Matrix([
            [0],
            [-self.m2 * self.A * self.g]
        ])

        # 토크 벡터
        self.tau = sp.Matrix([self.tau1, self.tau2])

        # 실제 값 정의
        self.m2_value = 0.5
        self.l2_value = 0.1
        self.l3_value = 0.15
        self.g_value = 9.81

    def calcCoriolisGravityTorque(self, th1, th2, dth1, dth2):
        equations = self.V + self.G

        solutions = {
            self.coriolis_tau1: equations[0],
            self.coriolis_tau2: equations[1]
        }

        # 변수에 대한 값 정의
        values = {
            self.m2: self.m2_value,  # 질량 m2
            self.l2: self.l2_value,  # 길이 l2
            self.l3: self.l3_value,  # 길이 l3
            self.g: self.g_value,  # 중력 가속도

            self.theta1: th1,  # 각도 theta1
            self.theta2: th2,  # 각도 theta2
            self.dtheta1: dth1,  # 각속도 dtheta1
            self.dtheta2: dth2,  # 각속도 dtheta2
        }

        # solutions에 값 대입
        solutions_with_values = {key: value.subs(values) for key, value in solutions.items()}

        # print(solutions_with_values)
        return solutions_with_values.values()

    # Coriolis와 중력 토크를 수치적으로 계산하는 함수
    def calcCoriolisGravityTorque_numerical(self, th1, th2, dth1, dth2):
        # 심볼릭 표현을 수치 함수로 변환
        coriolis_gravity_func = sp.lambdify(
            (self.theta1, self.theta2, self.dtheta1, self.dtheta2, self.m2, self.l2, self.l3, self.g),
            self.V + self.G,
            modules='numpy'
        )
        # 함수에 값을 대입하여 수치 계산
        coriolis_gravity_torque = coriolis_gravity_func(th1, th2, dth1, dth2, self.m2_value, self.l2_value, self.l3_value, self.g_value)
        return coriolis_gravity_torque[0][0], coriolis_gravity_torque[1][0]

    def calcMassTorque(self, th1, th2, ddth1, ddth2):
        equations = self.M * sp.Matrix([[self.ddtheta1], [self.ddtheta2]])

        solutions = {
            self.mass_tau1: equations[0],
            self.mass_tau2: equations[1]
        }

        # 변수에 대한 값 정의
        values = {
            self.m2: self.m2_value,  # 질량 m2
            self.l2: self.l2_value,  # 길이 l2
            self.l3: self.l3_value,  # 길이 l3
            self.g: self.g_value,  # 중력 가속도

            self.theta1: th1,  # 각도 theta1
            self.theta2: th2,  # 각도 theta2
            self.ddtheta1: ddth1,  # 각도 theta1
            self.ddtheta2: ddth2,  # 각도 theta2
        }

        # solutions에 값 대입
        solutions_with_values = {key: value.subs(values) for key, value in solutions.items()}

        # print(solutions_with_values)
        return solutions_with_values.values()

    # Mass 토크를 수치적으로 계산하는 함수
    def calcMassTorque_numerical(self, th1, th2, ddth1, ddth2):
        # 심볼릭 표현을 수치 함수로 변환
        mass_torque_func = sp.lambdify(
            (self.theta1, self.theta2, self.ddtheta1, self.ddtheta2, self.m2, self.l2, self.l3),
            self.M * sp.Matrix([[self.ddtheta1], [self.ddtheta2]]),
            modules='numpy'
        )
        # 함수에 값을 대입하여 수치 계산
        mass_torque = mass_torque_func(th1, th2, ddth1, ddth2, self.m2_value, self.l2_value, self.l3_value)
        return mass_torque[0][0], mass_torque[1][0],
    #
    # def calcAcc(self):
    #     # 로봇 운동 방정식 M*ddtheta + V + G = tau
    #     # 여기서는 조인트 가속도 ddtheta1과 ddtheta2에 대해 풀고자 합니다.
    #     equations = self.M * sp.Matrix([[self.ddtheta1], [self.ddtheta2]]) + self.V + self.G - self.tau
    #
    #     # ddtheta1과 ddtheta2에 대해 풉니다.
    #     solutions = sp.solve(equations, (self.ddtheta1, self.ddtheta2))
    #
    #     print(solutions)
    #     # 변수에 대한 값 정의
    #     values = {
    #         self.m2: 1.5,  # 질량 m2
    #         self.A: 0.5,   # 길이 A
    #         self.B: 0.3,   # 길이 B
    #         self.g: 9.81,  # 중력 가속도
    #         self.l2: 1.0,  # 길이 l2
    #         self.l3: 0.5,  # 길이 l3
    #         self.dtheta1: 0.1, # 각속도 dtheta1
    #         self.dtheta2: 0.2, # 각속도 dtheta2
    #         self.tau1: 0.05, # 토크 tau1
    #         self.tau2: 0.02 # 토크 tau2
    #     }
    #
    #     # solutions에 값 대입
    #     solutions_with_values = {key: value.subs(values) for key, value in solutions.items()}
    #
    #     print(solutions_with_values)
    #
    #
    # def calcTorque(self):
    #     equations = self.M * sp.Matrix([[self.ddtheta1], [self.ddtheta2]]) + self.V + self.G
    #
    #     solutions = {
    #         self.tau1: equations[0],
    #         self.tau2: equations[1]
    #     }
    #
    #     # print(solutions)
    #
    #     # 변수에 대한 값 정의
    #     values = {
    #         self.m2: 0.5,  # 질량 m2
    #         self.l2: 0.1,  # 길이 l2
    #         self.l3: 0.15,  # 길이 l3
    #         self.g: 9.81,  # 중력 가속도
    #
    #         self.theta1: 0.1,  # 각도 theta1
    #         self.theta2: 0.2,  # 각도 theta2
    #         self.dtheta1: 0.1,  # 각속도 dtheta1
    #         self.dtheta2: 0.2,  # 각속도 dtheta2
    #         self.ddtheta1: 0.1,  # 각가속도 ddtheta1
    #         self.ddtheta2: 0.2,  # 각가속도 ddtheta2
    #     }
    #
    #     # solutions에 값 대입
    #     solutions_with_values = {key: value.subs(values) for key, value in solutions.items()}
    #
    #     # print(solutions_with_values)
    #     return solutions_with_values.values()
    #
    # # 수치적으로 평가하기 위한 함수 생성
    # def calcTorque_numerical(self):
    #     torque_func = sp.lambdify((self.theta1, self.theta2, self.dtheta1, self.dtheta2,
    #                                self.ddtheta1, self.ddtheta2, self.m2, self.l2, self.l3, self.g),
    #                               self.M * sp.Matrix([[self.ddtheta1], [self.ddtheta2]]) + self.V + self.G,
    #                               modules='numpy')
    #
    #     # 값을 넣어서 평가
    #     values = (0.1, 0.2, 0.1, 0.2, 0.1, 0.2, 0.5, 0.1, 0.15, 9.81)
    #     torque_values = torque_func(*values)
    #
    #     return torque_values
