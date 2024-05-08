%% Parameter File for Rigatoni

%% Diagram 
%   o                   o
%   |                   |
%   |                   |
%   |                   | th1
%   o                   o - - -
%   |                  /
%   ^ Y               /
%   |                / th1
%   o-> X           o - - -

% calculation assumptions
tiny = 1e-6;

% kinematic geometry
l1 = 0.165; % m
l2 = 0.165; % m

rigParam = [l1, l2, tiny];

% mechanical parameters (get from solidworks and dynamixel site)
