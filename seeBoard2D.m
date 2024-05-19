%% Plot Chessboard in 2D
function [] = seeBoard2D(boardParam, ax, color)
% boardParam = [L, l, b, s, gv];
% ax, axes of figure (after opening figure, get axes with axName = axes)

if nargin < 3
    color = 'k';
end

[L, l, b, s, h, gv] = getBoardParam(boardParam);

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

% plot row labels
xpts = linspace(-(l/2-s/2), (l/2-s/2), 8);
ypts = h - s/2 + zeros(1,8);
text(ax, xpts, ypts, ["A" "B" "C" "D" "E" "F" "G" "H"])

% plot col labels
xpts = -(L/2+s/2) + zeros(1,8);
ypts = h + b + linspace(s/2, l-s/2, 8);
text(ax, xpts, ypts, string(1:1:8))


