function [tvec, q_des, qd_des, qdd_des, x, xd, xdd] = timeTrajectory(pointA, pointB, trajParam, rigParam)
% INPUT
%   pointA = [x; y], 2 x 1, starting point in task space
%   pointB = [x; y], 2 x 1, ending point in taskspace
%   trajParam = [sampRate maxVel maxAccel 
% path points are constructed with the plan that EE will come to rest at
% each point
% OUTPUT
%   joint space trajectory

% for each pair of points
% determine time needed based on maxEEvel and maxEEaccel

[sampRate, EESpeed, EEAccel, buffer] = getTrajParam(trajParam);

% create task space trajectory usmmming linear parabolic blend
distance = norm(pointA - pointB);

% estimate how long parabolic blend should take
tb = EESpeed/EEAccel; % time to get up to max speed
distcovered = tb^2*EEAccel; % distance already covered during acceleration parts
lineart = (distance-distcovered)/EESpeed; % time to cover remaining distance
t = 2*tb + max(0,lineart) + buffer;
V = EESpeed + 1;

while V > EESpeed
    t = sampRate*ceil(t/sampRate); % rather take slightly longer with exact sampling rate coinciding
    tvec = 0:sampRate:t;
    [x, xd, xdd] = lspb(pointA(1), pointB(1), tvec);
    [y, yd, ydd] = lspb(pointA(2), pointB(2), tvec);
    t = t + .01;
    V = max(sqrt(xd.^2+yd.^2));
end

% convert to joint space
q_des = zeros(2, length(x));
qd_des = zeros(2, length(x));
for i = 1:length(x)
    q_des(:,i) = rigatoni_2DIK([x(i) y(i)], rigParam);
    qd_des(:,i) = rigatoni_Jacobian(q_des(:,i), rigParam)\[xd(i); yd(i)];
end
qdd_des = [diff([qd_des [0;0]])];

x = transpose([x, y]);
xd = transpose([xd, yd]);
xdd = transpose([xdd, ydd]);

    
