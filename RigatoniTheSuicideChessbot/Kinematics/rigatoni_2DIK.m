%% 2D Inverse Kinematics of Rigatoni
function q = rigatoni_2DIK(posEE, rigParam, choice)
% input
%   posEE = [xe, ye], end effector position
%   choice = 0 or 1, whether to use negative or positive sin for th2
%   rigParam = [l1, l2, ..., tiny], link lengths

% default choice
if nargin < 3
   choice = 0;
end

l1 = rigParam(1);
l2 = rigParam(2);
xe = posEE(1);
ye = posEE(2);

% th2
c2 = computeNeglect((xe^2 + ye^2 - l1^2 - l2^2)/(2*l1*l2), rigParam(end));
s2 = computeNeglect(sqrt(1 - c2^2), rigParam(end));
if choice == 1
    s2 = -s2;
end

th2 = atan2(s2, c2);

% th1
a = l1 + l2*c2;
b = l2*s2;

th1 = atan2(a*ye - xe*b, xe*a + b*ye);

q = [th1, th2];
end
