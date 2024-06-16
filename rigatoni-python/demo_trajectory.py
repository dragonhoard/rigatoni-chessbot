from enum import Enum
import time
import serial
import numpy as np
import matplotlib.pyplot as plt

from RigatoniGame import RigatoniGame, Color, Interactor
import RigatoniParameterClasses as par
from RigatoniTrajectoryGenerator import create_path_points, generate_time_trajectory, EE_Operation, TrajectoryHolder, generate_time_trajectory_joint_space
from PID import PIDPositionController
from InverseDynamics import InverseDynamicsController

from dynamixel_utils.dc_motor_model import DCMotorModel
from dynio.dynamixel_controller import DynamixelIO, DynamixelMotor
from dynamixel_utils.motor import (
    get_mx28_control_table_json_path,
    group_sync_write,
    group_sync_read,
)

# open serial port for gripper communication via arduino, uncomment when using gripper
ardSerialInst = serial.Serial()
portVar = "COM7"
ardSerialInst.baudrate = 9600
ardSerialInst.port = portVar
ardSerialInst.open()
time.sleep(2)

ee_dynamixel_id = 2
# test controllers
dynamixel_ids = (4, 5)
serial_port_name = "COM5"

def close_gripper():
    global ardSerialInst
    print("Closing")
    command = "CLOSE"
    ardSerialInst.write(command.encode('utf-8'))
    time.sleep(1)
    return None
def open_gripper():
    global ardSerialInst
    print("Opening")
    command = "OPEN"
    ardSerialInst.write(command.encode('utf-8'))
    time.sleep(1)
    return None

# arm parameters
l1 = 0.165  # m, link 1 length
l2 = 0.165  # m, link 2 length
rest_position = [-0.15, 0]

# board parameters
L = 0.280  # m, outer board side length
l = 0.256  # m, playing area side length
h = 0.0425  # m, m, offset from side of board to robot base

# trajectory settings
sampling_rate = 0.005
ee_speed = 0.2
ee_acceleration = 2

# wrap all parameters
trajSet = par.TrajectorySettings(sampling_rate, ee_speed, ee_acceleration)
armRigatoni = par.Arm(l1, l2, rest_position)
boardRigatoni = par.Board(L, l, h)

K_P = np.array([[65, 0], [0, 55]])
K_I = np.array([[28.5, 0], [0, 21]])
K_D = np.array([[-3, 0], [0, -2]])
velocity_feedforward = np.array([[1.5, 0], [0, .75]])
acceleration_feedforward = np.array([[1.75, 0], [0, 0.8]])

controller = PIDPositionController(
    serial_port_name=serial_port_name,
    K_P=K_P,
    K_I=K_I,
    K_D=K_D,
    velocity_feedforward=velocity_feedforward,
    acceleration_feedforward=acceleration_feedforward,
    dynamixel_ids=dynamixel_ids,
    ee_dynamixel_id=ee_dynamixel_id,
)

# K_P = np.array([[4.25, 0], [0, 8]])
# K_D = np.array([[.009, 0.0], [0, 0.15]])
#
# controller = InverseDynamicsController(
#     serial_port_name=serial_port_name,
#     dynamixel_ids=dynamixel_ids,
#     K_P=K_P,
#     K_D=K_D,
#     ee_dynamixel_id=ee_dynamixel_id,
# )

# do the demo

arm_move = 'c3f6'  # 'a1a5'
path_points, point_action, piece_associated = create_path_points(arm_move, 0, 0, armRigatoni, boardRigatoni)

curr_joint_pos = np.deg2rad(controller.read_joint_positions_deg())
curr_task_pos = armRigatoni.FK(curr_joint_pos)
path_points[:,0] = curr_task_pos.flatten()
path_points[:,-1] = curr_task_pos.flatten()

controller.first_enable()

down_pos = 540
up_pos = 1700
kind_of_down = 800

controller.ee_motor.write_control_table("Goal_Position", up_pos)
time.sleep(.5)
# for each pathpoint
for i in range(len(path_points[1,:])):
#   perform EE instruction at pathpoint
    match point_action[i]:
        case EE_Operation.NOTHING:
            print('ee does nothing')
            pass
        case EE_Operation.PICK_UP:
            print('pick up')
            controller.ee_motor.torque_enable()
            controller.ee_motor.write_control_table("Goal_Position", down_pos)
            time.sleep(.1)
            close_gripper()
            time.sleep(.5)
            controller.ee_motor.write_control_table("Goal_Position", up_pos)
        case EE_Operation.PUT_DOWN:
            print('put down')
            controller.ee_motor.torque_enable()
            controller.ee_motor.write_control_table("Goal_Position", down_pos)
            time.sleep(.1)
            open_gripper()
            time.sleep(.5)
            controller.ee_motor.write_control_table("Goal_Position", up_pos)
        case EE_Operation.LOWER:
            print('lower')
            #controller.ee_motor.torque_enable()
            #controller.ee_motor.write_control_table("Goal_Position", kind_of_down)

    if not i == len(path_points[1,:]) - 1: # if not last point
        print('generate trajectory')
        #   generate trajectory to next pathpoint, task space line
        tvec, tq, tg = generate_time_trajectory(path_points[:,i], path_points[:,i+1], armRigatoni, trajSet)
        q = np.rad2deg(tq.q)
        qd = np.rad2deg(tq.qd)
        qdd = np.rad2deg(tq.qdd)

        traj = TrajectoryHolder(tvec, q, qd, qdd)

        # # generate trajectory joint space
        # tvec, tq = generate_time_trajectory_joint_space(armRigatoni.IK(path_points[:, i]),
        #                                                 armRigatoni.IK(path_points[:, i + 1]), armRigatoni, trajSet)
        # q = np.rad2deg(tq.q)
        # qd = np.rad2deg(tq.qd)
        # qdd = np.rad2deg(tq.qdd)
        # traj = TrajectoryHolder(tvec, q, qd, qdd)

        #   feed trajectory to chosen controller

        controller.motors[0].torque_enable()
        controller.motors[1].torque_enable()
        controller.start(traj)
        controller.motors[0].torque_disable()
        controller.motors[1].torque_disable()
        controller.ee_motor.torque_disable()

fig = plt.figure()
ax1 = plt.subplot(211)
ax1.plot(controller.time_stamps, np.asarray(controller.joint_position_history)[:, 0], label='Theta 1',
         ls='solid', color='r')
ax1.plot(controller.time_stamps, np.asarray(controller.goal_position_history)[:, 0], label='Theta 1 Desired', ls='dashed', color='r')
ax1.set_ylabel('Position (deg)')
ax1.legend()
ax1.set_title('Position in Joint Space')
ax2 = plt.subplot(212)
ax2.plot(controller.time_stamps, np.asarray(controller.joint_position_history)[:, 1], label='Theta 2',
         ls='solid', color='b')
ax2.plot(controller.time_stamps, np.asarray(controller.goal_position_history)[:, 1], label='Theta 2 Desired', ls='dashed', color='b')
ax2.set_ylabel('Position (deg)')
ax2.set_xlabel('Time (s)')
ax2.legend()

fig_error = plt.figure()
ax = plt.subplot(111)
ax.plot(controller.time_stamps, np.asarray(controller.position_error_history)[:, 0], label='Theta 1',
        ls='solid', color='r')
ax.plot(controller.time_stamps, np.asarray(controller.position_error_history)[:, 1], label='Theta 2',
        ls='solid', color='b')
ax.axhline(0, color='black')
ax.set_ylabel('Position Error (deg)')
ax.set_xlabel('Time (s)')
ax.set_title('Position Error in Joint Space')
ax.legend()

# get task space traj
x = np.zeros([2, len(controller.time_stamps)])
x_des = np.zeros([2, len(controller.time_stamps)])
for i in range(len(controller.time_stamps)):
    x[:, i:i + 1] = armRigatoni.FK(np.deg2rad(np.array(controller.joint_position_history)[i, :]))
    x_des[:, i:i + 1] = armRigatoni.FK(np.deg2rad(np.array(controller.goal_position_history)[i, :]))

fig_task_space = plt.figure()
ax = plt.subplot(111)
ax.plot(x[0, :], x[1, :], label='Actual', ls='solid', color='r')
ax.plot(x_des[0, :], x_des[1, :], label='Desired', ls='dashed', color='k')
ax.set_ylabel('Y Position (m)')
ax.set_xlabel('X Position (m)')
ax.set_title('Trajectory in Task Space')
ax.legend()

plt.show()

fig.savefig('fig1.png')
fig_error.savefig('fig3.png')
fig_task_space.savefig('fig4.png')