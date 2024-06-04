from enum import Enum
import time
import serial
import numpy as np
import matplotlib.pyplot as plt

from RigatoniGame import RigatoniGame, Color, Interactor
import RigatoniParameterClasses as par
from RigatoniTrajectoryGenerator import create_path_points, generate_time_trajectory, EE_Operation, TrajectoryHolder
from PID import PIDPositionController

# arm parameters
l1 = 0.165  # m, link 1 length
l2 = 0.165  # m, link 2 length
rest_position = [0.1, 0]

# board parameters
L = 0.280  # m, outer board side length
l = 0.256  # m, playing area side length
h = 0.052  # m, m, offset from side of board to robot base

# trajectory settings
sampling_rate = 0.001
ee_speed = 0.25
ee_acceleration = 3

# wrap all parameters
trajSet = par.TrajectorySettings(sampling_rate, ee_speed, ee_acceleration)
armRigatoni = par.Arm(l1, l2, rest_position)
boardRigatoni = par.Board(L, l, h)

# generate a fake trajectory
arm_move = 'a1a3' #'a1a5'
path_points, point_action, piece_associated = create_path_points(arm_move, 0, 0, armRigatoni, boardRigatoni)
print(path_points)
i = 1 # which trajectory
tvec, tq, tg = generate_time_trajectory(path_points[:,i], path_points[:,i+1], armRigatoni, trajSet)

offset = np.array([-90+9.49, -12.83])

q = np.rad2deg(tq.q)
q[:,0] = q[:,0] + 180 + offset[0]
q[:,1] = -q[:,1] + 180 + offset[1]
qd = np.rad2deg(tq.qd)
qdd = np.rad2deg(tq.qdd)

print(q)

test_traj = TrajectoryHolder(tvec, q, qd, qdd)

test_traj = TrajectoryHolder([0], [[0+180, 0+180]] + offset, [0, 0], [0, 0])

# test controllers
dynamixel_ids = (4, 5)
serial_port_name = "COM5"

K_P = np.array([[15, 0], [0, 12]])
K_I = np.array([[2, 0], [0, 2]])
K_D = np.array([[.1, 0], [0, .1]])

controller = PIDPositionController(
    serial_port_name=serial_port_name,
    K_P=K_P,
    K_I=K_I,
    K_D=K_D,
    dynamixel_ids=dynamixel_ids,
    # q_initial_deg=q_initial,
    # q_desired_deg=q_desired,
)

controller.first_enable()

controller.motors[0].torque_enable()
controller.motors[1].torque_enable()

controller.start(test_traj)

controller.motors[0].torque_disable()
controller.motors[1].torque_disable()

fig = plt.figure()
ax = plt.subplot(111)
ax.plot(controller.time_stamps, np.asarray(controller.joint_position_history)[:,0] - 90 - offset[0], label='Theta 1')
ax.plot(controller.time_stamps, -(np.asarray(controller.joint_position_history)[:,1] - 180 - offset[1]), label='Theta 2')
plt.legend()
plt.show()

data = np.column_stack((np.asarray(controller.time_stamps), np.asarray(controller.joint_position_history)))
np.savetxt('position.csv', data, delimiter=',')

