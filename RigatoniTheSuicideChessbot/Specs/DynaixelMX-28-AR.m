%% Dynamixel MX-28-AR Specs
% https://www.robotis.us/dynamixel-mx-28ar/
% https://emanual.robotis.com/docs/en/dxl/mx/mx-28/

% PID control

% given parameters
mDyn28 = 0.077; % kg, mass of motor
KrDyn28 = 193; % gear ratio

stallTorqueDyn28 = 2.50; % Nm, at 12V, 1.4A
stallCurrentDyn28 = 1.4; % A
noLoadSpeed = 55; % rpm, at 12V
noLoadCurrent = 0.12; % A
voltageDyn28; 12 ; % V, recommended voltage, min 10, max 14.8

resolutionDyn28 = 0.0879; % deg/pulse
backlashDyn28 = 0.33; % deg, backlash

% assume
% Fv negligible from high gear ratio
