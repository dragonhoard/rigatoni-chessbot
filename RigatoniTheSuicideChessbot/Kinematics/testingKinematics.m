%% Testing Kinematics
clc
clear
close all

rigatoni_params
board_params

th1 = pi/9
th2 = pi/2

figure;
ax = axes;
seeBoard2D(boardParam, ax)
hold(ax, "on")

seeRigatoni2D([th1 th2], ax, rigParam)
xlim((l1+l2)*[-1 1])
ylim((l1+l2)*[-1 1])

posEE = rigatoni_2DFK([th1, th2], rigParam)


q1 = rigatoni_2DIK(posEE, rigParam)
q2 = rigatoni_2DIK(posEE, rigParam, 1)

hold on
%seeRigatoni2D(q1, ax, rigParam, 'ro--')
%seeRigatoni2D(q2, ax, rigParam, 'bo--')
xlim((l1+l2)*[-1 1])
ylim((l1+l2)*[-1 1])
axis equal