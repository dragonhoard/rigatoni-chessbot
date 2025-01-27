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

%% calculation assumptions
tiny = 1e-6;

%% kinematic geometry
l1 = 0.165; % m
l2 = 0.165; % m

rigParam = [l1, l2, tiny];

%% mechanical parameters (get from solidworks and dynamixel site)

%
%use once solidworks available
% masses
m1 = .309534869; % kg, mass of link 1 + motor 2
m2_1 = .3088; % kg, mass of link 2
m2_2 = 0; % kg, mass of end effector (may eventually be lumped into m2)
m2 = m2_1 + m2_2;

% center of masses
P1c1 = [ 82.499963e-3;... % m, location for center of mass of link 1 + motor 2, wrt frame {1}
        0;...
        207.529513e-3];
P2c2_1 = [117.23e-3;... % m, location for center of mass of link 2, wrt frame {2}
        0;...
        132.964286e-3];
P2c2_2 = [0;... % m, location for center of mass of end effector, wrt frame {2}
        0;...
        0];
P2c2 = (m2_1*P2c2_1 + m2_2*P2c2_2)/m2;

% inertias
    % from solidworks or other approximation
R = [1 0 0; 0 1 0; 0 0 1];
Ic1 = 1e-9* [ 167019.272612 206.604918        75.662525;...    % Ix, kgm^2, inertia of link 1 + motor 2 about CoM
            206.604918     1481241.899881    -0.000004 ;...             % Iy, kgm^2,
             75.662525   -0.000004               1472493.387525];      % Iz, kgm^2 (only this one matters)
Ic1 = R*Ic1*transpose(R);
Ic2_1 = 1e-9*   [  271207.175463   -995.035173     -173145.637280;...  % Ix, kgm^2, inertia of link 2 about CoM
                  -995.035173   1034757.519275     -404.044358;...    % Iy, kgm^2,
                  -173145.637280       -404.044358         949382.955276];      % Iz, kgm^2 (only this one matters)
Ic2_1= R*Ic2_1*transpose(R);
IcEE = [ 0 0 0;...   % Ix, kgm^2, inertia of end effector about CoM
         0 0 0;...    % Iy, kgm^2,
         0 0 0];      % Iz, kgm^2 (only this one matters)
    % parallel axis thm
r1 = P2c2_1-P2c2; % m, position vector from m2 CoM to m2_1 CoM
r2 = P2c2_2-P2c2; % m, position vector from m2 CoM to m2_2 CoM
Id = diag([1 1 1]);
Ic2 = Ic2_1 + m2_1*(Id*norm(r1)^2 - r1*r1') + ... % inertia of link 2 and EE, 
       IcEE + m2_2*(Id*norm(r2)^2 - r2*r2');    %  combined with parallel axis thm, 
                                                %  about combined CoM
%}

%{
% temporary mass parameter estimates
link_m = 0.200; % kg, link is .200 kg, uniform thin rod
act_m = 0.077; % kg, motor is point mass 0.077kg https://emanual.robotis.com/docs/en/dxl/mx/mx-28-2/#drawings
ee_m = .150 ; % kg, random EE estimate

m1 = link_m + act_m; % kg, mass of link 1 + motor 2
m2_1 = link_m; % kg, mass of link 2
m2_2 = ee_m; % kg, mass of end effector (may eventually be lumped into m2)
m2 = m2_1 + m2_2;

% center of masses
P1c1 = [ (link_m*l1/2 + act_m*l1)/m1;... % m, location for center of mass of link 1 + motor 2, wrt frame {1}
        0;...
        0];
P2c2_1 = [l2/2;... % m, location for center of mass of link 2, wrt frame {2}
        0;...
        0];
P2c2_2 = [l2;... % m, location for center of mass of end effector, wrt frame {2}
        0;...
        0];
P2c2 = (m2_1*P2c2_1 + m2_2*P2c2_2)/m2;

% inertias
    % from solidworks or other approximation
Ic1 = [ 0 0 0;...    % Ix, kgm^2, inertia of link 1 + motor 2 about CoM (ignoring estimated motor mass)
        0 m1*l1^2/12 0;...    % Iy, kgm^2,
        0 0 m1*l1^2/12];      % Iz, kgm^2 (only this one matters)
Ic2_1 = [ 0 0 0;...  % Ix, kgm^2, inertia of link 2 about CoM
          0 m2*l2^2/12 0;...    % Iy, kgm^2,
          0 0 m2*l2^2/12];      % Iz, kgm^2 (only this one matters)
IcEE = [ 0 0 0;...   % Ix, kgm^2, inertia of end effector about CoM
         0 0 0;...    % Iy, kgm^2,
         0 0 0];      % Iz, kgm^2 (only this one matters)
    % parallel axis thm
r1 = P2c2_1-P2c2; % m, position vector from m2 CoM to m2_1 CoM
r2 = P2c2_2-P2c2; % m, position vector from m2 CoM to m2_2 CoM
Id = diag([1 1 1]);
Ic2 = Ic2_1 + m2_1*(Id*norm(r1)^2 - r1*r1') + ... % inertia of link 2 and EE, 
       IcEE + m2_2*(Id*norm(r2)^2 - r2*r2');    %  combined with parallel axis thm, 
                                                %  about combined CoM
%}