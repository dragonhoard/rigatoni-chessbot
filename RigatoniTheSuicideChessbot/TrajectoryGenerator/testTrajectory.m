%% Trajectory Generator Checks
clc
clear
close all

rigatoni_params
board_params
trajectory_params

restPos = [0.050; 0];
player = 1; % player moving
botside = 0; % side that bot is on
move = 'a2E5k';
[pathPoints, pointAction] = pathPointGenerator(move, player, botside, restPos, boardParam);

figure;
ax = axes;

for i = 1:(length(pointAction)-1)
    [tvec, q_des, qd_des, qdd_des, x, xd, xdd] = timeTrajectory(pathPoints(:,i), pathPoints(:,i+1), trajParam, rigParam);
    for j = 1:3:length(tvec)
        seeBoard2D(boardParam, ax, 'k')
        hold on
        seeRigatoni2D(q_des(:,j), ax, rigParam, 'b')
        hold off
        axis equal
        pause(0.0001)
    end
end
% 
% figure;
% ax = axes;
% seeBoard2D(boardParam, ax)
% hold(ax, "on")
% 
% plot(pathPoints(1,:), pathPoints(2,:), 'r*-')
% text(pathPoints(1,1:floor(length(pointAction)/2)), pathPoints(2,1:floor(length(pointAction)/2)), string(1:floor(length(pointAction)/2)), "Color", 'r', 'VerticalAlignment','bottom');
% text(pathPoints(1,floor(length(pointAction)/2)+1:length(pointAction)), pathPoints(2,floor(length(pointAction)/2)+1:length(pointAction)), string(floor(length(pointAction)/2)+1:length(pointAction)), "Color", 'r', 'VerticalAlignment','top');