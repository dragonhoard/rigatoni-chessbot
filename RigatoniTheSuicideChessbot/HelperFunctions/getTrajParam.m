function [sampRate, maxEESpeed, maxEEAccel, buffer] = getTrajParam(trajParam)
sampRate = trajParam(1);
maxEESpeed = trajParam(2);
maxEEAccel = trajParam(3);
buffer = trajParam(4);
end