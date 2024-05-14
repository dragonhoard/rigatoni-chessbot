%% Return Position of Chessboard
function pos = getSquarePosition(spot, botside, boardParam)
% INPUT
%   spot - string describing desired position, specified with chess notation
%       (i.e. E4)
%   botside - 0 if bot is on white side (chessboard is right side up), 1 if
%       bot is on black side (chessboard is upside down)
%   boardParam = [L, l, b, s, h, gv] as specified in board_param file
% OUTPUT
%   pos = xy position of spot
% NO ERROR CHECKS (assumes spot is given in the proper format and is only 2
% characters; will correct for upper/lower case)

     [~, l, b, s, h, ~] = getBoardParam(boardParam);

    function row = letter2num(letter)
        % returns number corresponding to letter (A = 1, B = 2, etc.)
        letter = upper(letter);
        row = double(letter) - 64;
    end

    % get row/column
    col = letter2num(spot(1));
    row = str2double(spot(2));

    % alter row and columns values if bot is black side
    if botside == 1
        row = 9 - row;
        col = 9 - col;
    end

    % calculate position of middle of A1 square (11)
    A1x = -(l/2 - s/2);
    A1y = h + b + s/2;

    x = A1x + (col-1)*s;
    y = A1y + (row-1)*s;

    pos = [x; y];

end

% Assumed Chess Setup
%  BLACK SIDE
%   A B C D E F G H
% 8
% 7
% 6
% 5
% 4
% 3
% 2 P P P P P P P P
% 1 R K B X Q B K R
%   A B C D E F G H
%
%  WHITE SIDE
% (lower left square is black)

