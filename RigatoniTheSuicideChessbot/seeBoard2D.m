%% Plot Chessboard in 2D
function [] = seeBoard2D(boardParam, ax, color)
% boardParam = [L, l, b, s, gv];
% ax, axes of figure (after opening figure, get axes with axName = axes)

if nargin < 3
    color = 'k';
end

L = boardParam(1);
l = boardParam(2);
b = boardParam(3);
s = boardParam(4);
h = boardParam(5);
gv = boardParam(6);

% plot base joint
plot(ax, 0, 0, 'o', "Color", color)
hold(ax, "on")
% plot outer border
pts = [-L/2, h;
       -L/2, h+L;
        L/2, h+L;
        L/2, h;
       -L/2, h];
plot(ax, pts(:,1), pts(:,2), '-', "Color", color)

% plot horizontal lines
for i = 0:8
    xpts = [-l/2 l/2];
    ypts = [h+b h+b] + i*s;
    plot(ax, xpts, ypts, '-', "Color", color)
end

% plot vertical lines
for i = -4:4
    xpts = i*s*[1 1];
    ypts = [h+b h+b+l];
    plot(ax, xpts, ypts, '-', "Color", color)
end
% plot grave spots
gvpt = [L/2+gv h+L/2];
plot(ax, gvpt(1), gvpt(2), '*', "Color", color)
plot(ax, -gvpt(1), gvpt(2), '*', "Color", color)


