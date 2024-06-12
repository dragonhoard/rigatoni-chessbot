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

# arm parameters
l1 = 0.165  # m, link 1 length
l2 = 0.165  # m, link 2 length
rest_position = [0.1, 0]

# board parameters
L = 0.280  # m, outer board side length
l = 0.256  # m, playing area side length
h = 0.0425  # m, m, offset from side of board to robot base

# trajectory settings
sampling_rate = 0.005
ee_speed = .2
ee_acceleration = 2

# wrap all parameters
trajSet = par.TrajectorySettings(sampling_rate, ee_speed, ee_acceleration)
armRigatoni = par.Arm(l1, l2, rest_position)
boardRigatoni = par.Board(L, l, h)

# test controllers
dynamixel_ids = (4, 5)
serial_port_name = "COM5"

# K_P = np.array([[85.3, 0], [0, 68]])
# K_I = np.array([[0, 0], [0, 0.5]])
# K_D = np.array([[1.3, 0], [0, 1.16]])
# velocity_feedforward = 0*np.array([[1.5, 0], [0, 0.75]])
# acceleration_feedforward = 0*np.array([[1.75, 0], [0, 0.8]])
#
# controller = PIDPositionController(
#     serial_port_name=serial_port_name,
#     K_P=K_P,
#     K_I=K_I,
#     K_D=K_D,
#     velocity_feedforward=velocity_feedforward,
#     acceleration_feedforward=acceleration_feedforward,
#     dynamixel_ids=dynamixel_ids,
#     ee_dynamixel_id=2,
# )

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
    ee_dynamixel_id=2,
)


# K_P = np.array([[2.5, 0], [0, 4.7]])
# K_D = np.array([[0.035, 0.0], [0, 0.05]])
#
# controller = InverseDynamicsController(
#     serial_port_name=serial_port_name,
#     dynamixel_ids=dynamixel_ids,
#     K_P=K_P,
#     K_D=K_D,
#     ee_dynamixel_id=2,
# )

traj_choice = 0 # 0 for board based, 1 for point to point in task space, 2 for manual joint space points
match traj_choice:
    case 0:
        # generate a fake trajectory
        #arm_move = 'e5a8' #'a1a5'
        #arm_move = 'a8h8'  # 'a1a5'
        #arm_move = 'h8h1'  # 'a1a5'
        #arm_move = 'h1a1'  # 'a1a5'
        #arm_move = 'a1a8'  # 'a1a5'
        #arm_move = 'a8e5'  # 'a1a5'
        arm_move = 'c3f6'  # 'a1a5'
        path_points, point_action, piece_associated = create_path_points(arm_move, 0, 0, armRigatoni, boardRigatoni)
        print(path_points)
        i = 1 # which trajectory
        tvec, tq, tg = generate_time_trajectory(path_points[:,i], path_points[:,i+1], armRigatoni, trajSet)

        q = np.rad2deg(tq.q)
        qd = np.rad2deg(tq.qd)
        qdd = np.rad2deg(tq.qdd)

        test_traj = TrajectoryHolder(tvec, q, qd, qdd)

        curr_joint_pos = np.deg2rad(controller.read_joint_positions_deg())
        curr_task_pos = armRigatoni.FK(curr_joint_pos)

        tvecf, tqf, tgf = generate_time_trajectory(curr_task_pos, path_points[:, i], armRigatoni, trajSet)
        qf = np.rad2deg(tqf.q)
        qdf = np.rad2deg(tqf.qd)
        qddf = np.rad2deg(tqf.qdd)
        first_traj = TrajectoryHolder(tvecf, qf, qdf, qddf)
    case 1:
        point_1 = np.array([-.1, .2])
        point_2 = np.array([.1, .2])
        tvec, tq, tg = generate_time_trajectory(point_1, point_2, armRigatoni, trajSet)

        q = np.rad2deg(tq.q)
        qd = np.rad2deg(tq.qd)
        qdd = np.rad2deg(tq.qdd)
        test_traj = TrajectoryHolder(tvec, q, qd, qdd)
        first_traj = TrajectoryHolder([2], np.array([q[0,:]]), np.array([[0, 0]]), np.array([[0, 0]]))
    case 2:
        # constant goal
        #offset = np.array([-90 + 9.49, -12.83])
        tvec = [1]
        q = np.array([[90, 0]])
        test_traj = TrajectoryHolder(tvec, q, np.array([[0, 0]]), np.array([[0, 0]]))
    case 3:
        # generate a fake trajectory
        # arm_move = 'e5a8' #'a1a5'
        # arm_move = 'a8h8'  # 'a1a5'
        # arm_move = 'h8h1'  # 'a1a5'
        # arm_move = 'h1a1'  # 'a1a5'
        # arm_move = 'a1a8'  # 'a1a5'
        # arm_move = 'a8e5'  # 'a1a5'
        arm_move = 'a7f6'  # 'a1a5'
        path_points, point_action, piece_associated = create_path_points(arm_move, 0, 0, armRigatoni, boardRigatoni)
        print(path_points)
        i = 1  # which trajectory
        tvec, tq = generate_time_trajectory_joint_space(armRigatoni.IK(path_points[:, i]), armRigatoni.IK(path_points[:, i+1]), armRigatoni, trajSet)

        q = np.rad2deg(tq.q)
        qd = np.rad2deg(tq.qd)
        qdd = np.rad2deg(tq.qdd)

        test_traj = TrajectoryHolder(tvec, q, qd, qdd)

        curr_joint_pos = np.deg2rad(controller.read_joint_positions_deg())

        tvecf, tqf = generate_time_trajectory_joint_space(curr_joint_pos, armRigatoni.IK(path_points[:, i]), armRigatoni, trajSet)
        qf = np.rad2deg(tqf.q)
        qdf = np.rad2deg(tqf.qd)
        qddf = np.rad2deg(tqf.qdd)
        first_traj = TrajectoryHolder(tvecf, qf, qdf, qddf)

controller.first_enable()

controller.motors[0].torque_enable()
controller.motors[1].torque_enable()

# controller.motors[0].torque_disable()
# controller.motors[1].torque_disable()

controller.start(first_traj)
controller.reset_history()
controller.start(test_traj)

controller.motors[0].torque_disable()
controller.motors[1].torque_disable()

fig = plt.figure()
ax1 = plt.subplot(211)
ax1.plot(controller.time_stamps, np.asarray(controller.joint_position_history)[:,0], label='Theta 1', ls='solid', color='r')
ax1.plot(tvec, q[:,0], label='Theta 1 Desired', ls='dashed', color='r')
ax1.set_ylabel('Position (deg)')
ax1.legend()
ax1.set_title('Position in Joint Space')
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
ax1vel.set_title('Velocity in Joint Space')
ax2vel = plt.subplot(212)
ax2vel.plot(controller.time_stamps, np.asarray(controller.joint_velocity_history)[:,1], label='Theta 2', ls='solid', color='b')
ax2vel.plot(tvec, qd[:,1], label='Theta 2 Desired', ls='dashed', color='b')
ax2vel.set_ylabel('Velocity (deg/s)')
ax2vel.set_xlabel('Time (s)')
ax2vel.legend()

fig_error = plt.figure()
ax = plt.subplot(111)
ax.plot(controller.time_stamps, np.asarray(controller.position_error_history)[:,0], label='Theta 1', ls='solid', color='r')
ax.plot(controller.time_stamps, np.asarray(controller.position_error_history)[:,1], label='Theta 2', ls='solid', color='b')
ax.axhline(0, color='black')
ax.set_ylabel('Position Error (deg)')
ax.set_xlabel('Time (s)')
ax.set_title('Position Error in Joint Space')
ax.legend()

# get task space traj
x = np.zeros([2, len(controller.time_stamps)])
for i in range(len(controller.time_stamps)):
    x[:, i:i+1] = armRigatoni.FK(np.deg2rad(np.array(controller.joint_position_history)[i,:]))
fig_task_space = plt.figure()
ax = plt.subplot(111)
ax.plot(x[0,:], x[1,:], label='Actual', ls='solid', color='r')
ax.plot(tg.q[:,0], tg.q[:,1], label='Desired', ls='solid', color='b')
ax.set_ylabel('Y Position (m)')
ax.set_xlabel('X Position (m)')
ax.set_title('Trajectory in Task Space')
ax.legend()

# # get task space traj
# fig_pwm = plt.figure()
# ax = plt.subplot(111)
# ax.plot(controller.time_stamps, np.asarray(controller.pwm_history)[:,0], label='Joint 1', ls='solid', color='r')
# ax.plot(controller.time_stamps, np.asarray(controller.pwm_history)[:,1], label='Joint 2', ls='solid', color='b')
# ax.set_ylabel('PWM')
# ax.set_xlabel('Time (s)')
# ax.set_title('PWM History')
# ax.legend()

plt.show()

fig.savefig('fig1.png')
fig_vel.savefig('fig2.png')
fig_error.savefig('fig3.png')
fig_task_space.savefig('fig4.png')


#data = np.column_stack((np.asarray(controller.time_stamps), np.asarray(controller.joint_position_history)))
#np.savetxt('position.csv', data, delimiter=',')

