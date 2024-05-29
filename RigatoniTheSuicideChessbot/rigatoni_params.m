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
m1 = .297; % kg, mass of link 1 + motor 2
m2_1 = .207; % kg, mass of link 2
m2_2 = 0; % kg, mass of end effector (may eventually be lumped into m2)
m2 = m2_1 + m2_2;

% center of masses
P1c1 = [ 82.4096e-3;... % m, location for center of mass of link 1 + motor 2, wrt frame {1}
        0;...
        32.0935e-3];
P2c2_1 = [122.4726e-3;... % m, location for center of mass of link 2, wrt frame {2}
        0;...
        -35.6748e-3];
P2c2_2 = [2;... % m, location for center of mass of end effector, wrt frame {2}
        0;...
        0];
P2c2 = (m2_1*P2c2_1 + m2_2*P2c2_2)/m2;

% inertias
    % from solidworks or other approximation
R = [1 0 0; 0 0 -1; 0 1 0];
Ic1 = 1e-9* [ 157037.4032 873.4683        -206.5921;...    % Ix, kgm^2, inertia of link 1 + motor 2 about CoM
            873.4683     1439280.4785    0 ;...             % Iy, kgm^2,
             -206.5920   0               1447428.5685];      % Iz, kgm^2 (only this one matters)
Ic1 = R*Ic1*transpose(R);
Ic2_1 = 1e-9*   [ 139180.4325   -68968.2968     -48.2631;...  % Ix, kgm^2, inertia of link 2 about CoM
                  -68968.2968   763510.3717     19.8545;...    % Iy, kgm^2,
                  -48.2631      19.8545         774703.8465];      % Iz, kgm^2 (only this one matters)
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