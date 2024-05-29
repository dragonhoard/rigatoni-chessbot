import math
import signal
import time
from collections import deque
from collections.abc import Sequence

import numpy as np
from dynamixel_utils.dc_motor_model import DCMotorModel
from dynamixel_utils.motor import (
    get_mx28_control_table_json_path,
    group_sync_write,
    group_sync_read,
)
from dynio.dynamixel_controller import DynamixelIO, DynamixelMotor
import dynio.dynamixel_controller as dxl
from matplotlib import pyplot as plt
from numpy.typing import NDArray

import dynamixel_utils

class PIDPositionController:
    """
    This class manages a PID Position Controller
    """

    def __init__(
        self,
        serial_port_name: str,
        dynamixel_ids: tuple[int, int],
        K_P: NDArray[np.double],
        K_I: NDArray[np.double],
        K_D: NDArray[np.double],
        q_initial_deg: Sequence[float],
        q_desired_deg: Sequence[float],
    ):
        # ------------------------------------------------------------------------------
        # Controller Related Variables
        # ------------------------------------------------------------------------------
        self.q_initial = np.asarray(q_initial_deg, dtype=np.double)
        self.q_desired = np.asarray(q_desired_deg, dtype=np.double)

        self.K_P = np.asarray(K_P, dtype=np.double)
        self.K_I = np.asarray(K_I, dtype=np.double)
        self.K_D = np.asarray(K_D, dtype=np.double)

        self.control_freq_Hz = 30.0
        self.control_period_s = 1 / self.control_freq_Hz
        self.loop_manager = dynamixel_utils.FixedFrequencyLoopManager(
            period_ns=round(self.control_period_s * 1e9)
        )
        self.should_continue = True

        self.joint_position_history = deque()
        self.timestamp = 0.0
        self.time_stamps = deque()
        self.dt = self.loop_manager.period_s

        self.error = np.zeros(2,)
        self.error_integral = np.zeros(2,)
        self.error_derivative = np.zeros(2,)
        self.error_window = deque(maxlen=2)
        self.num_convergence_samples = round(0.5 / self.dt)
        self.convergence_window = deque(maxlen=self.num_convergence_samples)

        # ------------------------------------------------------------------------------
        # Motor Communication Related Variables
        # ------------------------------------------------------------------------------
        self.dxl_io = DynamixelIO(device_name=str(serial_port_name), baud_rate=57_600)
        self.dynamixel_ids = dynamixel_ids

        motor1 = DynamixelMotor(
            dynamixel_ids[0],
            self.dxl_io,
            json_file=get_mx28_control_table_json_path(),
            protocol=2,
        )
        motor2 = DynamixelMotor(
            dynamixel_ids[1],
            self.dxl_io,
            json_file=get_mx28_control_table_json_path(),
            protocol=2,
        )
        self.motors = [motor1, motor2]

        # ------------------------------------------------------------------------------
        # DC Motor Modeling
        # ------------------------------------------------------------------------------
        pwm_limit_data_address, pwm_limit_data_len = self.motors[0].CONTROL_TABLE[
            "PWM_Limit"
        ]
        pwm_limits = group_sync_read(
            self.dxl_io, self.dynamixel_ids, pwm_limit_data_address, pwm_limit_data_len
        )
        self.pwm_limits = np.asarray(
            [pwm_limits[dynamixel_ids[0]], pwm_limits[dynamixel_ids[1]]]
        )

        velocity_limit_data_address, velocity_limit_data_len = self.motors[
            0
        ].CONTROL_TABLE["PWM_Limit"]
        velocity_limits = group_sync_read(
            self.dxl_io,
            self.dynamixel_ids,
            velocity_limit_data_address,
            velocity_limit_data_len,
        )
        self.velocity_limits = np.asarray(
            [velocity_limits[dynamixel_ids[0]], velocity_limits[dynamixel_ids[1]]]
        )

        # def go_to_home_configuration(self):
        #     self.should_continue = True
        #     # Put motor in "home" position
        #     for motor in self.motors:
        #         motor.torque_disable()
        #         motor.set_extended_position_mode()
        #         motor.torque_enable()
        #
        #     # Move to home position (self.q_initial)
        #     goal_position_data_address, goal_position_data_len = self.motors[
        #         0
        #     ].CONTROL_TABLE["Goal_Position"]
        #     group_sync_write(
        #         self.dxl_io,
        #         {
        #             motor.dxl_id: self.convert_deg_to_position_tick(motor, q)
        #             for motor, q in zip(self.motors, self.q_initial)
        #         },
        #         goal_position_data_address,
        #         goal_position_data_len,
        #     )
        #
        #     time.sleep(1)
        #     self.motors[0].torque_disable()
        #     self.motors[1].torque_disable()
        #     # Set PWM Mode (i.e. voltage control)
        #     operating_mode_data_address, operating_mode_data_len = self.motors[
        #         0
        #     ].CONTROL_TABLE["Operating_Mode"]
        #     group_sync_write(
        #         self.dxl_io,
        #         {dxl_id: 16 for dxl_id in self.dynamixel_ids},
        #         operating_mode_data_address,
        #         operating_mode_data_len,
        #     )
        #     self.motors[0].torque_enable()
        #     self.motors[1].torque_enable()

        # This model is based on the DC motor model learned in class, it allows us to
        # convert the torque control action u into something we can actually send to the
        # MX28-AR dynamixel motors (pwm commands).
        self.motor_model = DCMotorModel(
            self.control_period_s, pwm_limits=self.pwm_limits
        )

        # ------------------------------------------------------------------------------
        # Clean Up / Exit Handler Code
        # ------------------------------------------------------------------------------
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

    def start(self):
        self.motors[0].torque_enable()
        self.motors[1].torque_enable()
        start_time = time.time()
        while self.should_continue:
            # Step 1 - Get Feedback
            q_actual = self.read_joint_positions_deg()

            self.joint_position_history.append(q_actual)  # Save for plotting
            self.time_stamps.append(time.time() - start_time)  # Save for plotting

            # Step 2 - Calculate Command
            self.error = self.q_desired - q_actual

            self.error_window.appendleft(self.error)
            self.convergence_window.append(self.error)
            self.error_integral += self.error * self.dt

            # only start derivative error at the second time step
            if len(self.error_window) == 2:
                self.error_derivative = (
                    (self.error_window[-1] - self.error_window[0]) / 2 / self.dt
                )

            if len(self.convergence_window) == self.num_convergence_samples:
                if np.mean(np.abs(self.convergence_window)) < 0.5:
                    for motor in self.motors:
                        motor.torque_disable()
                    return

            pwm_command = list()
            for i in range(2):
                pwm_command[i] = (
                    self.K_P[i, i] * self.error
                    + self.K_I[i, i] * self.error_integral
                    + self.K_D[i, i] * self.error_derivative
                )
            for i in range(len(self.motors)):
                pwm_lim = self.pwm_limits[i]
                pwm_command[i] = round(max(min(pwm_command[i], pwm_lim), -self.pwm_limits[i]))

            # Step 3 - Send Command
            for motor in self.motors:
                motor.write_control_table("Goal_PWM", pwm_command)

            # print(
            #     self.convert_encoder_tick_to_deg(
            #         self.motor.read_control_table("Present_Position")
            #     )
            # )

            self.loop_manager.sleep()
            self.timestamp += self.dt

        for motor in self.motors:
            motor.torque_disable()

    def stop(self):
        self.should_continue = False
        time.sleep(self.loop_manager.period_s)

    def convert_encoder_tick_to_deg(self, encoder_tick: int):
        deg = np.zeros(2,)
        for i in range(len(self.motors)):
            deg[i] = (encoder_tick - self.motors[i].min_position)/((self.motors[i].max_position + 1) - self.motors[i].min_position)*(self.motors[i].max_angle)
        return deg

    def signal_handler(self, *_):
        self.stop()
        for motor in self.motors:
            motor.torque_disable()
        exit(-1)

    def read_joint_positions_deg(self) -> NDArray[np.double]:
        # Assumes motors are the same model (i.e. MX-28AR)
        (
            present_position_control_table_address,
            present_position_data_len,
        ) = self.motors[0].CONTROL_TABLE["Present_Position"]

        joint_position_ticks_by_dxl_id = group_sync_read(
            self.dxl_io,
            self.dynamixel_ids,
            present_position_control_table_address,
            present_position_data_len,
        )

        joint_positions = np.empty((2,), dtype=np.double)
        for i, motor in enumerate(self.motors):
            joint_positions[i] = self.convert_position_tick_to_deg(
                motor, joint_position_ticks_by_dxl_id[motor.dxl_id]
            )

        return joint_positions

    def read_joint_velocities_deg_per_s(self):
        # Assumes motors are the same model (i.e. MX-28AR)
        (
            present_velocity_control_table_address,
            present_velocity_data_len,
        ) = self.motors[0].CONTROL_TABLE["Present_Velocity"]

        joint_velocity_ticks_by_dxl_id = group_sync_read(
            self.dxl_io,
            self.dynamixel_ids,
            present_velocity_control_table_address,
            present_velocity_data_len,
        )

        joint_velocities = np.empty((2,), dtype=np.double)
        for i, motor in enumerate(self.motors):
            joint_velocities[i] = self.convert_velocity_tick_to_deg_per_s(
                joint_velocity_ticks_by_dxl_id[motor.dxl_id]
            )

        ################################################################################
        return joint_velocities

    @staticmethod
    def convert_velocity_tick_to_deg_per_s(velocity_tick: int):
        velocity_tick = np.asarray(velocity_tick).astype(np.int16)
        if velocity_tick > 65536:
            velocity_tick = -np.invert(velocity_tick)

        rpm = float(velocity_tick) * 0.229
        return 6 * rpm

    @staticmethod
    def convert_deg_to_position_tick(motor: DynamixelMotor, angle_deg: float):
        return round(
            angle_deg / motor.max_angle * (motor.max_position + 1 - motor.min_position)
            + motor.min_position
        )

if __name__ == "__main__":
    q_initial = [30, 30] # idk what this is supposed to look like
    q_desired = [30, 100]

    dynamixel_ids = (1, 2)
    serial_port_name = "/dev/tty.usbserial-FT3FSMAZ"

    K_P = np.array([[0.4, 0], [0, 0.2]])
    K_I = np.array([[0, 0], [0, 0]])
    K_D = np.array([[0, 0], [0, 0]])


    controller = PIDPositionController(
        serial_port_name=serial_port_name,
        K_P=K_P,
        K_I=K_I,
        K_D=K_D,
        dynamixel_ids=dynamixel_ids,
        q_initial_deg=q_initial,
        q_desired_deg=q_desired,
    )

    controller.start()

    # fig = plt.figure(1)
    # ax = fig.add_subplot(111)
    # ax.axhline(90.0, ls="--", color="red")
    # ax.plot(controller.timestamps, controller.position_history, color="black")
    # plt.show()
