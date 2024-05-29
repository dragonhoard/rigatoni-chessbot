from enum import Enum

class ChessPiece(Enum):
    PAWN = 1
    KNIGHT = 2
    BISHOP = 3
    ROOK = 4
    QUEEN = 5
    KING = 6
                #   null    PAWN    KNIGHT  BISHOP  ROOK    QUEEN   KING
clearance_height = [0,      1,      2,      3,      4,      5,      6]  # clearance height in m
grab_height      = [0,      1,      2,      3,      4,      5,      6]  # clearance height in m
lower_height     = [0,      1,      2,      3,      4,      5,      6]  # clearance height in m
close_diameter   = [0,      1,      2,      3,      4,      5,      6]  # clearance height in m

