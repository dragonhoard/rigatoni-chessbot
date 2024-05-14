%% Gravespot Position
function [prepos, pos] = getGravespot(gravespot, botside, boardParam)
% INPUT
%   gravespot - string describing desired designating gravespot 
%       0 if white or 1 if black
%   botside - 0 if bot is on white side (chessboard is right side up), 1 if
%       bot is on black side (chessboard is upside down)
%   boardParam = [L, l, b, s, h, gv] as specified in board_param file
% OUTPUT
%   prepos = xy position of pre-grave drop spot (so robot can push piece
%   off)
%   pos = xy poition of gravespot
% NO ERROR CHECKS
% own gravespot is on right hand side from perspective of player

% get needed board values
[L, ~, ~, s, h, gv] = getBoardParam(boardParam);

flip = gravespot + botside; % 1 if want opponents gravespot
% white grave from black side (0,1)
% black grave from white side (1,0)


% calculate spot
x = (L/2 + gv)*(-1)^flip; % either positive for own side or negative for opponents side
y = L/2 + h; % same from either side

pos = [x; y];
prepos = [x-s*(-1)^flip; y]; % predrop spot is one square width in from gravespot

end