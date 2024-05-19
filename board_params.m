%% Board Parameters

% conversion
in2mm = 25.4;

% board dimensions
L = 0.280; % m, outer board side length
l = 0.256; % m, playing area side length
b = (L-l)/2; % m, border width
s = l/8; % m, playing square side length

% additional dimensions
h = 0.052; % m, offset from side of board to robot base
gv = 1.5*s; % m, using 1.5 square lengths to graveyard spot

% robot base placed at center of one of the playing sides

boardParam = [L, l, b, s, h, gv];

%{ 
Medium Board
L = 15; % in, outer board side length
l = 13+7/8; % in, playing area side length
%}
