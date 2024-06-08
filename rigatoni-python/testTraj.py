from enum import Enum
import time
import serial
import numpy as np
import matplotlib.pyplot as plt

from RigatoniGame import RigatoniGame, Color, Interactor
import RigatoniParameterClasses as par
from RigatoniTrajectoryGenerator import create_path_points, generate_time_trajectory, EE_Operation, TrajectoryHolder
from PID import PIDPositionController
from InverseDynamics import InverseDynamicsController

# arm parameters
l1 = 0.165  # m, link 1 length
l2 = 0.165  # m, link 2 length
rest_position = [0.1, 0]

# board parameters
L = 0.280  # m, outer board side length
l = 0.256  # m, playing area side length
h = 0.052  # m, m, offset from side of board to robot base

# trajectory settings
sampling_rate = 0.005
ee_speed = 0.1
ee_acceleration = 2

# wrap all parameters
trajSet = par.TrajectorySettings(sampling_rate, ee_speed, ee_acceleration)
armRigatoni = par.Arm(l1, l2, rest_position)
boardRigatoni = par.Board(L, l, h)

traj_choice = 0 # 0 for board based, 1 for point to point in task space, 2 for manual joint space points
match traj_choice:
    case 0:
        # generate a fake trajectory
        arm_move = 'f6a7' #'a1a5'
        path_points, point_action, piece_associated = create_path_points(arm_move, 0, 0, armRigatoni, boardRigatoni)
        print(path_points)
        i = 1 # which trajectory
        tvec, tq, tg = generate_time_trajectory(path_points[:,i], path_points[:,i+1], armRigatoni, trajSet)

        q = np.rad2deg(tq.q)
        qd = np.rad2deg(tq.qd)
        qdd = np.rad2deg(tq.qdd)

        test_traj = TrajectoryHolder(tvec, q, qd, qdd)
        first_traj = TrajectoryHolder([2], np.array([q[0,:]]), np.array([[0, 0]]), np.array([[0, 0]]))
    case 1:
        point_1 = np.array([-.1, .2])
        point_2 = np.array([.1, .2])
        tvec, tq, tg = generate_time_trajectory(point_1, point_2, armRigatoni, trajSet)

        q = np.rad2deg(tq.q)
        qd = np.rad2deg(tq.qd)
        qdd = np.rad2deg(tq.qdd)
        print(qdd)
        test_traj = TrajectoryHolder(tvec, q, qd, qdd)
        first_traj = TrajectoryHolder([2], np.array([q[0,:]]), np.array([[0, 0]]), np.array([[0, 0]]))
    case 2:
        # constant goal
        #offset = np.array([-90 + 9.49, -12.83])
        tvec = [1]
        q = np.array([[90, 0]])
        test_traj = TrajectoryHolder(tvec, q, np.array([[0, 0]]), np.array([[0, 0]]))

# test controllers
dynamixel_ids = (4, 5)
serial_port_name = "COM5"

K_P = np.array([[30, 0], [0, 25]])
K_I = np.array([[0, 0], [0, 0]])
K_D = np.array([[0, 0], [0, 0]])
feedforward_gain = 0*np.array([[10000, 0], [0, 175]])

controller = PIDPositionController(
    serial_port_name=serial_port_name,
    K_P=K_P,
    K_I=K_I,
    K_D=K_D,
    feedforward_gain=feedforward_gain,
    dynamixel_ids=dynamixel_ids,
    # q_initial_deg=q_initial,
    # q_desired_deg=q_desired,
)

# K_P = np.array([[0.5, 0], [0, 0.5]])
# K_D = np.array([[0, 0.0], [0, 0.0]])
#
# controller = InverseDynamicsController(
#     serial_port_name=serial_port_name,
#     dynamixel_ids=dynamixel_ids,
#     K_P=K_P,
#     K_D=K_D,
# )

controller.first_enable()

controller.motors[0].torque_enable()
controller.motors[1].torque_enable()

controller.start(first_traj)
controller.start(test_traj)

controller.motors[0].torque_disable()
controller.motors[1].torque_disable()

fig = plt.figure()
ax1 = plt.subplot(211)
ax1.plot(controller.time_stamps, np.asarray(controller.joint_position_history)[:,0], label='Theta 1', ls='solid', color='r')
ax1.plot(tvec, q[:,0], label='Theta 1 Desired', ls='dashed', color='r')
ax1.set_ylabel('Position (deg)')
ax1.legend()
ax2 = plt.subplot(212)
ax2.plot(controller.time_stamps, np.asarray(controller.joint_position_history)[:,1], label='Theta 2', ls='solid', color='b')
ax2.plot(tvec, q[:,1], label='Theta 2 Desired', ls='dashed', color='b')
ax2.set_ylabel('Position (deg)')
ax2.set_xlabel('Time (s)')
ax2.legend()

fig_vel = plt.figure()
ax1vel = plt.subplot(211)
ax1vel.plot(controller.time_stamps, np.asarray(controller.joint_velocity_history)[:,0], label='Theta 1', ls='solid', color='r')
ax1vel.plot(tvec, qd[:,0], label='Theta 1 Desired', ls='dashed', color='r')
ax1vel.set_ylabel('Velocity (deg/s)')
ax1vel.set_ylim([-5, 5])
ax1vel.legend
ax2vel = plt.subplot(212)
ax2vel.plot(controller.time_stamps, np.asarray(controller.joint_velocity_history)[:,1], label='Theta 2', ls='solid', color='b')
ax2vel.plot(tvec, qd[:,1], label='Theta 2 Desired', ls='dashed', color='b')
ax2vel.set_ylabel('Velocity (deg/s)')
ax2vel.set_xlabel('Time (s)')
ax2vel.set_ylim([-5, 5])
ax2vel.legend()

fig_error = plt.figure()
ax = plt.subplot(111)
ax.plot(controller.time_stamps, np.asarray(controller.position_error_history)[:,0], label='Theta 1', ls='solid', color='r')
ax.plot(controller.time_stamps, np.asarray(controller.position_error_history)[:,1], label='Theta 2', ls='solid', color='b')
ax.axhline(0, color='black')
ax.set_ylabel('Position Error (deg)')
ax.set_xlabel('Time (s)')
ax.legend()

plt.show()


#data = np.column_stack((np.asarray(controller.time_stamps), np.asarray(controller.joint_position_history)))
#np.savetxt('position.csv', data, delimiter=',')

