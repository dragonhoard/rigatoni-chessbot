%% Simulation Runner
clc
clear

%{
% Open Loop Plant
mdl = "Simulation";
open_system(mdl);
out = sim(mdl,'StartTime','0','StopTime','100','FixedStep','0.1');

simulationMotionVisualizer
%}

%% q trajectory
rigatoni_params
board_params
trajectory_params

restPos = [0.050; 0];
player = 1; % player moving
botside = 0; % side that bot is on
move = 'a2E5k';
[pathPoints, pointAction] = pathPointGenerator(move, player, botside, restPos, boardParam);
[tvec, q_des, qd_des, qdd_des, x, xd, xdd] = timeTrajectory(pathPoints(:,1), pathPoints(:,2), trajParam, rigParam);

%inputTraj.time = tvec;
q_des_ts = timeseries(q_des, tvec);
qd_des_ts = timeseries(qd_des, tvec);
qdd_des_ts = timeseries(qdd_des, tvec);

q0 = [0 transpose(q_des(:,1))];

% gains
Kp = [1500, 0;
    0, 750];
Kd = [10, 0;
    0, 10];

mdl = "inverseDynamics";
%open_system(mdl);
out = sim(mdl,'StartTime','0','StopTime',string(tvec(end)),'FixedStep','0.0001');

simulationMotionVisualizer

figure
plot(tvec, rad2deg(q_des), 'r--')
hold on
plot(out.tout, rad2deg(squeeze(out.q)))
xlabel('Time (s)')
ylabel('Angle (deg)')
legend('\theta_1 desired', '\theta_2 desired', '\theta_1', '\theta_2')
title('Joint Trajectory with Inverse Dynamics')


% error
t_interp = linspace(min(out.tout), max(out.tout), length(tvec));
q_interp = transpose(interp1(out.tout, transpose(squeeze(out.q)), t_interp, 'linear'));

figure
plot(tvec, rad2deg(q_des - q_interp))
hold on
yline(0)
xlabel('Time (s)')
ylabel('Angle Error (deg)')
legend('\theta_1', '\theta_2', 'Zero Error')
title('Joint Error with Inverse Dynamics')