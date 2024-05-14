%% Pathpoint Generator
function [pathPoints, pointAction] = pathPointGenerator(move, player, botside, restPos, boardParam)
% INPUT
%   move = letter-number combo specifying starting space and ending space, if a kill is being
%       performed, add k (technically any char) at the end (i.e. E4E5k or
%       E4E5)
%   player = player making move, 0 for white, 1 for black
%   botside = side of board that bot is on, 0 for white, 1 for black
%   restPos = [x, y]; % (x, y), rest position of bot, move to controller/trajectory setting file later

% OUTPUT
%   pathPoints = [x; y], 2 x n, where n is number of path points
%   pointAction = 1 x n, contains instruction for EE action
% path points are constructed with the plan that EE will come to rest at
% each point

% specify starting space and ending space, if a kill is being
% performed, add k (technically any char) at the end

%% Error Checks

%% Path Points

% start at rest position
pathPoints = restPos;
pointAction = 0; % EE action position associated with
    % nothing (0)
    % lower, pick up, raise (1)
    % lower, put down, raise (2) (not dependent on where EE is at the
        % moment)
    % partial lower (3) (specifially for grave drops)

% parse move
if length(move) == 5
    % kill instruction
    kill = true;
else
    kill = false;
end
startSquare = getSquarePosition(move(1:2), botside, boardParam);
endSquare = getSquarePosition(move(3:4), botside, boardParam);

% if kill, there is extra trajectory
if kill
    [pregrave, gravespot] = getGravespot(abs(player-1), botside, boardParam);
    % go to end (kill) square
        % OPER 1
    % go to grave pre-drop position
        % OPER 3
    % go to grave spot
        % OPER 2
    pathPoints = [pathPoints, endSquare, pregrave, gravespot];
    pointAction = [pointAction, 1, 3, 2];
end

% execute main action of moving piece
% go to start square
    % OPER 1
% go to end square
    % OPER 2
% return to rest position
    % no oper, 0
    pathPoints = [pathPoints, startSquare, endSquare, restPos];
    pointAction = [pointAction, 1, 2, 0];


