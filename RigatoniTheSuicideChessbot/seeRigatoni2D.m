%% Plot Rigatoni in 2D
function [] = seeRigatoni2D(q, ax, rigParam, style)
% q, [th1 th2]
% ax, axes of figure (after opening figure, get axes with axName = axes)

if nargin < 4
    style = 'ko-';
end

l1 = rigParam(1);
l2 = rigParam(2);
th1 = q(1);
th2 = q(2);

% calculate coordinates of each joint
jt1 = [0, 0]; 
jt2 = l1*[cos(th1), sin(th1)];
jt3 = jt2 + l2*[cos(th1+th2), sin(th1+th2)];
pts = [jt1; jt2; jt3];

% plot
plot(ax, pts(:,1), pts(:,2), style)
