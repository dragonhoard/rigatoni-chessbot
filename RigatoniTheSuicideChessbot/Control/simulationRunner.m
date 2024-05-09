%% Simulation Runner
clc
clear

mdl = "Simulation";
open_system(mdl);
out = sim(mdl,'StartTime','0','StopTime','100','FixedStep','0.1');

simulationMotionVisualizer