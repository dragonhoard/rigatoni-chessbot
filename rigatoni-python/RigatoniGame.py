# Code written for final project of UCLA MAEC263C course
# editor: Jane
# last edit: 5/28/2024 11:51 by Jane

import chess
import chess.variant
import chess.svg
import chess.engine

from enum import Enum, IntEnum
class Color(IntEnum):
    WHITE = 0
    BLACK = 1
class Interactor(Enum):
    USER = 0
    BOT = 1

class RigatoniGame():
    # white_info and black_info are 2 entry tuples
    #   move generated by: 0 - user, 1 - bot
    #   move physically executed by: 0 - user, 1 - rigatoni
    # bot_side is 0 or 1 indicating which side of chessboard rigatoni is located
    #   0 - white
    #   1 - black

    def __init__(self, white_info, black_info, bot_side, armfun, engine_path):
        self._board = chess.variant.SuicideBoard()
        self._game_over = False

        self._turn = 0  # white starts

        self._deciders = [white_info[0], black_info[0]]
        self._executors = [white_info[1], black_info[1]]

        self._engine = chess.engine.SimpleEngine.popen_uci(engine_path)  # using fairy stockfish
        self._send_robot_move = armfun

        self._run_game()

    def _run_game(self):
        print('Let''s start the Game')
        while not self._game_over:
            print(self._board)

            # get the move
            if self._deciders[self._turn] == Interactor.USER:  # user decides
                # prompt from console
                while True:
                    try:
                        user_input = input(Color(self._turn).name + ' MOVE: ')
                        if user_input == 'help':
                            print(self._board.legal_moves)
                            continue
                        else:
                            move = chess.Move.from_uci(user_input)
                            if not move in self._board.legal_moves:
                                # check that the valid uci move can be done on this board
                                print('Good format, not a legal move, try again')
                                continue
                    except:  # letting chess library's from_uci throw errors
                        print('Not a legal move, try again')
                        # better try again... Return to the start of the loop
                        continue
                    else:
                        # successful parsing of move
                        break
            elif self._deciders[self._turn] == Interactor.BOT:
                # call the bot (assuming bot gives valid move, no error check)
                result = self._engine.play(self._board, chess.engine.Limit(time=0.1))
                move = result.move
                print(Color(self._turn).name + ' MOVE, Bot chosen: ' + str(move))
            else:
                raise Exception("Invalid decider interactor.")

            # make the move on the material plane
            if self._executors[self._turn] == Interactor.USER:
                print('Go ahead and make the move')
            elif self._executors[self._turn] == Interactor.BOT:
                arm_move, ee_info = self.format_to_robot_command(move)


                self._send_robot_move(arm_move, ee_info, self._turn)
            else:
                raise Exception("Invalid mover interactor.")

            # make the move on the virtual board
            self._board.push_san(str(move))
            self._turn ^= 1 # change player, 0 to 1, 1 to 0
            end_condition = [self._board.is_variant_end(),
                                   self._board.is_variant_win(),
                                   self._board.is_variant_loss(),
                                   self._board.is_variant_draw(),
                                   self._board.is_stalemate()]
            self._game_over = any(end_condition)
        print('Game Over')
        if end_condition[1]:
            print('Winner: ' + Color(self._turn).name)
        elif end_condition[2]:
            print('Winner: ' + Color(self._turn^1).name)
        elif end_condition[3] or end_condition[4]:
            print('Stalemate')
        else:
            print('Somehow the game is over')

    def format_to_robot_command(self, move):
        # piece at to_square
        to_square_piece = self._board.piece_at(move.to_square)
        from_square_piece = self._board.piece_at(move.from_square)

        if to_square_piece == None:  # no piece to be killed
            ee_info = [to_square_piece]
            arm_move = str(move)
        else:
            ee_info = [to_square_piece, from_square_piece]
            arm_move = str(move) + 'k'

        # arm_move = uci format + indication of kill ('e1e2k' or 'a1e2')
        # ee_move = piece to be moved and killed piece ('12' for pawn kill knight, or '1' for pawn move)
        return arm_move, ee_info

