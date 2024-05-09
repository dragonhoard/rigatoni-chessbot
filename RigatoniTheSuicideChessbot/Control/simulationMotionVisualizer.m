%% Simulation Visualizer
% use after running simulink to visalize robot motion
% simulation must output
    % tout, nx1,  time
    % q, 2x1xn, position

close all

rigatoni_params

t = out.tout;
q = out.q;

figure
ax = axes;
for i = 1:length(t)
    %hold on
    seeRigatoni2D(q(:,1,i), ax, rigParam, 'ro-')
    xlim((l1+l2)*[-1 1])
    ylim((l1+l2)*[-1 1])
    axis equal
    pause(0.01)
    hold off
end
