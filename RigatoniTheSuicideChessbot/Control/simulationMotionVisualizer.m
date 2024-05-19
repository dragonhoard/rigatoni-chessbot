%% Simulation Visualizer
% use after running simulink to visalize robot motion
% simulation must output
    % tout, nx1,  time
    % q, 2x1xn, position

close all

rigatoni_params

t = out.tout;
q = out.q;

writerObj = VideoWriter('sampleTrajectory.mp4');
writerObj.FrameRate = 5;
open(writerObj);

tsofar = [];
possofar = [];

figure
ax = axes;
for i = 1:length(t)
    pos = rigatoni_2DFK(q(:,1,i), rigParam);
    possofar = [possofar; pos];
    %hold on
    seeBoard2D(boardParam, ax, 'k')
    seeRigatoni2D(q(:,1,i), ax, rigParam, 'ro-')
    hold on
    plot(possofar(:,1), possofar(:,2), 'b--')
    xlim((l1+l2)*[-1 1])
    ylim((l1+l2)*[-1 1])
    axis equal
    writeVideo(writerObj, getframe(gcf));
    pause(0.001)
    hold off
end

close(writerObj);
