# Code adapted from mini-lab 1 and 2 scripts written by Cole Ten as
# provided by the UCLA MAE C263C class

# editors: Julia, Jane
# last edit: 6/5/2024 11:51 by Jane

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

from dynamixel_utils import FixedFrequencyLoopManager

import dynamixel_utils
from RigatoniTrajectoryGenerator import TrajectoryHolder

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
        feedforward_gain: NDArray[np.double],
    ):
        # ------------------------------------------------------------------------------
        # Controller Related Variables
        # ------------------------------------------------------------------------------

        self.K_P = np.asarray(K_P, dtype=np.double)
        self.K_I = np.asarray(K_I, dtype=np.double)
        self.K_D = np.asarray(K_D, dtype=np.double)
        self.feedforward_gain = np.asarray(feedforward_gain, dtype=np.double)

        self.control_freq_Hz = 60.0
        self.control_period_s = 1 / self.control_freq_Hz
        self.loop_manager = FixedFrequencyLoopManager(
            period_ns=round(self.control_period_s * 1e9)
        )
        self.timeout = 2
        self.threshold = .5
        self.should_continue = True

        self.stiction_comp = np.array([0, 0])
        self.pwm_help = np.array([10, 10])


        self.timestamp = 0.0
        self.time_stamps = deque()
        self.dt = self.control_period_s
        self.joint_position_history = deque()
        self.joint_velocity_history = deque()
        self.position_error_history = deque()

        self.error = np.zeros(2,)
        self.error_integral = np.zeros(2,)
        self.velocity_error = np.zeros(2, )
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

    def first_enable(self):
        self.motors[0].torque_disable()
        self.motors[1].torque_disable()
        #self.motors[0].set_extended_position_mode()
        #self.motors[1].set_extended_position_mode()

        # Set PWM Mode (i.e. voltage control)
        operating_mode_data_address, operating_mode_data_len = self.motors[
            0
        ].CONTROL_TABLE["Operating_Mode"]
        group_sync_write(
            self.dxl_io,
            {dxl_id: 16 for dxl_id in self.dynamixel_ids},
            operating_mode_data_address,
            operating_mode_data_len,
        )

        self.motors[0].torque_enable()
        self.motors[1].torque_enable()

    def start(self, traj: TrajectoryHolder):
        self.should_continue = True
        self.error_integral = 0
        start_time = time.time()

        while self.should_continue:
            self.time_stamps.append(time.time() - start_time)  # Save for plotting
            #print("Time: " + str(time.time()-start_time))
            # Step 1 - Get Feedback
            q_actual = self.read_joint_positions_deg()
            qd_actual = self.read_joint_velocities_deg_per_s()


            # Step 2 - Calculate Command
            print("Current Position: " + str(q_actual))
            print("Goal Position " + str(traj.get_q(time.time()-start_time)))
            q_des, qd_des, qdd_des = traj.get_q_qd_qdd((time.time()-start_time))
            self.error = q_des - q_actual # make sure the indexing is right
            self.velocity_error = qd_des - qd_actual

            self.position_error_history.append(self.error)
            #self.error_window.appendleft(self.error)
            #self.convergence_window.append(self.error)
            self.error_integral += self.error * self.dt

            # # only start derivative error at the second time step
            # if len(self.error_window) == 2:
            #     self.velocity_error = (
            #         (self.error_window[-1] - self.error_window[0]) / self.dt
            #     )

            pwm_commands = (
                    self.K_P @ self.error
                    + self.K_I @ self.error_integral
                    + self.K_D @ self.velocity_error
                    + self.feedforward_gain @ qdd_des
            )
            print(qdd_des[0])

            pwm_command = pwm_commands.tolist()
            for i in range(len(self.motors)):
                pwm_lim = self.pwm_limits[i]
                if qd_actual[i] < 2 and abs(pwm_command[i]) > self.pwm_help[i]:
                    print("add" + str(i+1))
                    pwm_command[i] += self.stiction_comp[i]*np.sign(pwm_command[i])
                pwm_command[i] = round(max(min(pwm_command[i], pwm_lim), -pwm_lim))

                # send command
                print('PWM' + str(pwm_command[i]))
                self.motors[i].write_control_table("Goal_PWM", pwm_command[i])

            print("Velocity" + str(qd_actual))
            self.joint_position_history.append(q_actual)  # Save for plotting
            self.joint_velocity_history.append(qd_actual)
            print("Position Error: " + str(self.error))
            self.should_continue = (time.time()-start_time < traj.get_end_time() or  # trajectory not done
                                    ((not np.all(abs(self.error) < self.threshold) or not np.all(abs(self.velocity_error) < 1)) and time.time()-start_time < traj.get_end_time() + self.timeout)) # trajectory done, error is too large and have not timed out
            self.loop_manager.sleep()


    def stop(self):
        self.should_continue = False
        time.sleep(self.loop_manager.control_period_s)

    def convert_encoder_tick_to_deg(self, encoder_tick: int, motor_id: int):
        # deg = np.zeros(2,)
        deg = (encoder_tick - self.motors[motor_id].min_position)/((self.motors[motor_id].max_position + 1) - self.motors[motor_id].min_position)*(self.motors[motor_id].max_angle)
        return deg-360 # altered so zero position is one turn in at 180 so hopefully range of dynamixel is -540 to +large angle in multiturn

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
            joint_positions[i] = self.convert_encoder_tick_to_deg(
                joint_position_ticks_by_dxl_id[motor.dxl_id], i
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
