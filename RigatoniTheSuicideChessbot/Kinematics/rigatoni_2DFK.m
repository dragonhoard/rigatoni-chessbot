%% 2D Forward Kinematics of Rigatoni
function posEE = rigatoni_2DFK(q, rigParam)
% input
%   q = [th1, th2], joint position (radians)
%   geoParam = [l1, l2], link lengths

l1 = rigParam(1);
l2 = rigParam(2);
th1 = q(1);
th2 = q(2);

xe = l1*cos(th1) + l2*cos(th1 + th2);
ye = l1*sin(th1) + l2*sin(th1 + th2);
posEE = [xe, ye];

end