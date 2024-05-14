%% Ellipsoid exploration
clear
close all
clc

rigatoni_params
board_params


figure;
ax = axes;
seeBoard2D(boardParam, ax)

% generate every square position and grave position
gvpt = [L/2+gv h+L/2];
A1x = -(l/2 - s/2);
A1y = h + b + s/2;

x = A1x + s*linspace(0,7,8);
y = A1y + s*linspace(0,7,8);
conditions = zeros(8,8);
maxscalings = zeros(8,8);
minscalings = zeros(8,8);

scale = .01;
for i = 1:length(x)
    for j = 1:length(y)
        pos = [x(i), y(j)];
        q = rigatoni_2DIK(pos, rigParam);
        J = rigatoni_Jacobian(q, rigParam);
        plot_ellipse(inv(J*transpose(J)), pos, scale)
        hold on
        %seeRigatoni2D(q, ax, rigParam, '-')
        axis equal
        %hold on
        eigvects = eig(J*transpose(J));
        scalings = sqrt(eigvects);
        conditions(i,j) = max(scalings)/min(scalings);
        maxscalings(i,j) = max(scalings);
        minscalings(i,j) = min(scalings);
    end
end
xlim(.6*L*[-1, 1])
ylim([-h L+2*h])


% adapted from https://stackoverflow.com/questions/15915625/plotting-an-ellipse-in-matlab-given-in-matrix-form
function plot_ellipse(E, center, scale)
 % plots an ellipse of the form xEx = 1
 R = chol(E);
 t = linspace(0, 2*pi, 100); % or any high number to make curve smooth
 z = [cos(t); sin(t)];
 ellipse = inv(R) * z;
 plot(scale*ellipse(1,:) + center(1), scale*ellipse(2,:) + center(2))
end 