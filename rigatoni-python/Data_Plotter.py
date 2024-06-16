import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import string as str
import os

# assign directory
directory = 'data_to_plot_together'

# create plots
# position plot
fig_pos, axs_pos = plt.subplots(2)
# error plot
fig_err, axs_err = plt.subplots(2)
# pwm plot
fig_pwm, axs_pwm = plt.subplots(2)
# task space pot
fig_task, ax_task = plt.subplots(1)

# iterate over files in
# that directory
for filename in os.scandir(directory):
    if filename.is_file():
        print(filename.path)
        # read comment as legend label
        with open(filename) as f:
            # 0: controller
            # 1: arm move
            # 2: joint or task space generated
            # 3: custom label
            label = f.readlines()[3].replace("# ", "").strip("\n")
        print(label)
        # read data
        data = pd.read_csv(filename, dtype=float, skiprows=4)
        print(data)

        axs_pos[0].plot(data["t"], data["q1"], label=label)
        axs_pos[1].plot(data["t"], data["q2"], label=label)
        axs_err[0].plot(data["t"], data["q1err"], label=label)
        axs_err[1].plot(data["t"], data["q2err"], label=label)
        axs_pwm[0].plot(data["t"], data["pwm1"], label=label)
        axs_pwm[1].plot(data["t"], data["pwm2"], label=label)
        ax_task.plot(data["x"], data["y"], label=label)

axs_pos[0].plot(data["t"], data["q1_des"], label="Desired", ls='dashed', color="black")
axs_pos[1].plot(data["t"], data["q2_des"], label="Desired", ls='dashed', color="black")
axs_err[0].axhline(0, ls='dashed', color="black")
axs_err[1].axhline(0, ls='dashed', color="black")
axs_err[0].axhline(3, label='+/-3 deg', ls='dashed', color="red")
axs_err[1].axhline(3, ls='dashed', color="red")
axs_err[0].axhline(-3, ls='dashed', color="red")
axs_err[1].axhline(-3, label='+/-3 deg', ls='dashed', color="red")
ax_task.plot(data["x_des"], data["y_des"], label="Desired", ls='dashed', color="black")

axs_pos[1].set_xlabel("Time (s)")
axs_pos[0].set_ylabel("Joint 1 Position (deg)")
axs_pos[1].set_ylabel("Joint 2 Position (deg)")
axs_pos[0].set_title("Joint Positions")

axs_err[1].set_xlabel("Time (s)")
axs_err[0].set_ylabel("Joint 1 Error (deg)")
axs_err[1].set_ylabel("Joint 2 Error (deg)")
axs_err[0].set_title("Joint Position Error")

axs_pwm[1].set_xlabel("Time (s)")
axs_pwm[0].set_ylabel("Joint 1 PWM")
axs_pwm[1].set_ylabel("Joint 2 PWM")
axs_pwm[0].set_title("Joint PWM Command")

ax_task.set_xlabel("X (m)")
ax_task.set_ylabel("Y (m)")
ax_task.set_title("Task Space Trajectory")

lr = 'lower right'
ur = 'upper right'
small_font_size = 8
font_size = 10
axs_pos[0].legend(loc=lr, fontsize=small_font_size)
axs_pos[1].legend(loc=ur, fontsize=small_font_size)
axs_err[0].legend(loc=ur, fontsize=small_font_size)
axs_err[1].legend(loc=lr, fontsize=small_font_size)
axs_pwm[0].legend(loc=lr, fontsize=font_size)
axs_pwm[1].legend(loc=ur, fontsize=font_size)
ax_task.legend(loc=lr, fontsize=font_size)

plt.show()

fig_pos.savefig("fig_pos.png")
fig_err.savefig("fig_err.png")
fig_pwm.savefig("fig_pwm.png")
fig_task.savefig("fig_task.png")
