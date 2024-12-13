#!/usr/bin/env python3
#-*- coding:utf-8 -*-


#//BUSHRA ASHFAQUE CS-19011//

from copy import deepcopy
import time
import math
import cv2
from move_coord import image2coordinate

import os
import rospy
from std_msgs.msg import String


ansi_black = "\u001b[30m"
ansi_red = "\u001b[31m"
ansi_green = "\u001b[32m"
ansi_yellow = "\u001b[33m"
ansi_blue = "\u001b[34m"
ansi_magenta = "\u001b[35m"
ansi_cyan = "\u001b[36m"
ansi_white = "\u001b[37m"
ansi_reset = "\u001b[0m"
com = 0


class Node:
    def __init__(self, board, move=None, parent=None, value=None):
        self.board = board
        self.value = value
        self.move = move
        self.parent = parent

    def get_children(self, maximizing_player, mandatory_jumping):
        current_state = deepcopy(self.board)
        available_moves = []
        children_states = []
        big_letter = ""
        queen_row = 0
        if maximizing_player is True:
            available_moves = Checkers.find_available_moves(current_state, mandatory_jumping)
            big_letter = "C"
            queen_row = 7
        else:
            available_moves = Checkers.find_player_available_moves(current_state, mandatory_jumping)
            big_letter = "B"
            queen_row = 0
        for i in range(len(available_moves)):
            old_i = available_moves[i][0]
            old_j = available_moves[i][1]
            new_i = available_moves[i][2]
            new_j = available_moves[i][3]
            state = deepcopy(current_state)
            Checkers.make_a_move(state, old_i, old_j, new_i, new_j, big_letter, queen_row)
            children_states.append(Node(state, [old_i, old_j, new_i, new_j]))
        return children_states

    def set_value(self, value):
        self.value = value

    def get_value(self):
        return self.value

    def get_board(self):
        return self.board

    def get_parent(self):
        return self.parent

    def set_parent(self, parent):
        self.parent = parent


class Checkers:
    def __init__(self):
        # # ROS Publisher 초기화
        rospy.init_node('checkers_move_publisher', anonymous=True)
        self.task_status = False
        self.move_pub = rospy.Publisher('/checkers/move', String, queue_size=10)
        self.read_robot_state = rospy.Subscriber('/task_status', String, self.callback)

        self.matrix = [[], [], [], [], [], [], [], []]
        self.current_turn = True
        self.computer_pieces = 12
        self.player_pieces = 12
        self.available_moves = []
        self.mandatory_jumping = False

        self.firststart = True

        for row in self.matrix:
            for i in range(8):
                row.append("---")
        self.position_computer()
        self.position_player()

    def position_computer(self):
        for i in range(3):
            for j in range(8):
                if (i + j) % 2 == 1:
                    self.matrix[i][j] = ("c" + str(i) + str(j))

    def position_player(self):
        for i in range(5, 8, 1):
            for j in range(8):
                if (i + j) % 2 == 1:
                    self.matrix[i][j] = ("b" + str(i) + str(j))

    def cap_video(self):
        while True:
            # video capture            
            self.cap = cv2.VideoCapture(2)
            self.success, self.frame = self.cap.read()
            if self.success:
                print("successfully captured image")
                self.cap.release()
                break

    def print_matrix(self):
        i = 0
        print()
        for row in self.matrix:
            print(i, end="  |")
            i += 1
            for elem in row:
                print(elem, end=" ")
            print()
        print()
        for j in range(8):
            if j == 0:
                j = "     0"
            print(j, end="   ")
        print("\n")


    def delete_file(file_path):
        try:
            if os.path.exists(file_path):
                os.remove(file_path)
        except Exception as e:
            print(f"Error occur when erasing file: {e}")

    #----------------------------------------- get message from the robot move file (mc.py) ----------------------------------#
    def callback(self, msg):
        self.task_status = True
        print(f"Message received: {msg.data}")
        # 받은 후 처리하고, 다시 False로 초기화
        time.sleep(1)  # 잠시 대기 (필요한 처리 추가 가능)
        self.task_status = False

    # def check_status(self):
    #     if self.task_status:
    #         print("Task is currently in progress...")
    #     else:
    #         print("No task in progress.")

    #------------------------------------------------------------------------------------------------------------------------#

    #----------------------------------- capture 된 돌 좌표 없애기 --------------------------------------------------#
    def remove_coordinate_from_file(self, file_path, target_coordinate):

        target_str = f"({target_coordinate[0]}, {target_coordinate[1]})"  # 좌표를 문자열로 변환
        lines_to_keep = []  # 삭제되지 않은 라인을 저장할 리스트

        # 파일을 읽고, 각 라인을 확인하여 좌표가 포함되어 있지 않으면 lines_to_keep에 추가
        with open(file_path, 'r', encoding='utf-8') as file:
            for line in file:
                if target_str not in line:
                    lines_to_keep.append(line)

        # 수정된 내용을 파일에 다시 저장
        with open(file_path, 'w', encoding='utf-8') as file:
            file.writelines(lines_to_keep)
        print(f"좌표 {target_coordinate}가 파일에서 삭제되었습니다.")

    def check_coordinate_and_remove(self, file_path, target_coordinate):

        target_str = f"({target_coordinate[0]}, {target_coordinate[1]})"  # 좌표를 문자열로 변환

        # 파일에서 좌표가 포함되어 있는지 확인
        with open(file_path, 'r', encoding='utf-8') as file:
            for line in file:
                if target_str in line:
                    self.remove_coordinate_from_file(file_path, target_coordinate)
                    return True  # 좌표가 발견되면 True 반환

        return False  # 좌표가 발견되지 않으면 False 반환
    #----------------------------------------------------------------------------------------------------------------#

    def get_player_input(self):
        
        available_moves = Checkers.find_player_available_moves(self.matrix, self.mandatory_jumping)
        if len(available_moves) == 0:
            if self.computer_pieces > self.player_pieces:
                print(
                    ansi_red + "You have no moves left, and you have fewer pieces than the computer.YOU LOSE!" + ansi_reset)
                self.delete_file("/home/gun/catkin_ws/src/ur_python/src/coord_check/bs_coord.txt")
                exit()
            else:
                print(ansi_yellow + "You have no available moves.\nGAME ENDED!" + ansi_reset)
                self.delete_file("/home/gun/catkin_ws/src/ur_python/src/coord_check/bs_coord.txt")
                exit()
        self.player_pieces = 0
        self.computer_pieces = 0
        while True:
    # ------------------ Modified image to coordinate input from user --------------------- #
            # wait until the move "press Enter"

            while True:

                # place = input("press enter after your stone placement")

                # if not place:
                #     print("stone place completed")
                #     break

                if self.firststart:
                    print("Its your turn make a move")
                    t = 8
                    print("Waiting for 8 seconds...")
                    for i in range(t, 0, -1):
                        print(f"{i}s left", end = "\r")
                        time.sleep(1)  # 5초 기다림

                    self.firststart = False
                    break


                if self.task_status:
                    t = 8
                    print("Waiting for 8 seconds...")
                    for i in range(t, 0, -1):
                        print(f"{i}s left", end = "\r")
                        time.sleep(1)  # 5초 기다림
                    # w = 2
                    # for i in range(w, 0, -1):
                    #     print(f"Your turn is up put your hand away from the board {i} sec", "\r")
                    #     time.sleep(1)
                    break

            print("Capturing Video")
            self.cap_video()

    #-------------------------- Figuring out the captured stone of the user & erasing form the coordinate text-----------------------------#
            x = 0
            y = 0
            if com:
                print (com[0],com[1],com[2], com[3])
                y1 = com[0]
                y2 = com[2]
                x1 = com[1]
                x2 = com[3]

                if abs(x2 - x1) >= 2:
                    if (x2-x1) > 0 and (y2-y1) > 0:
                        x = x2 -1
                        y = y2 -1
                    elif (x2-x1) > 0 and (y2-y1) < 0:
                        x = x2 -1
                        y = y2 +1
                    elif (x2-x1) < 0 and (y2-y1) > 0:
                        x = x2 +1
                        y = y2 -1
                    elif (x2-x1)< 0 and (y2-y1) < 0:
                        x = x2 +1
                        y = y2 +1

                    target_coordinate = (y, x)
                    self.check_coordinate_and_remove("/home/gun/catkin_ws/src/ur_python/src/coord_check/bs_coord.txt", target_coordinate)
    #--------------------------------------------------------------------------------------------------------------------------------------#

    #-------------------------------------------------- image to coordinate ---------------------------------------------------------------#
            coord1_set, coord2_set = image2coordinate(self.frame)

            # coord1과 coord2가 비어있는지 확인
            if not coord1_set or not coord2_set:
                print("Error: coord1 or coord2 is empty!")
                
            else:
                # set에서 튜플을 추출한 뒤 문자열로 변환
                coord1 = f"{next(iter(coord1_set))[0]},{next(iter(coord1_set))[1]}"
                coord2 = f"{next(iter(coord2_set))[0]},{next(iter(coord2_set))[1]}"

                # 결과 출력
                print(f"Mock input coord1: {coord1}")
                print(f"Mock input coord2: {coord2}")
    # --------------------------------------------------------------------------------------------------------------------------------------#
                
            if coord1 == "":
                print(ansi_cyan + "Game ended!" + ansi_reset)
                self.delete_file("/home/gun/catkin_ws/src/ur_python/src/coord_check/bs_coord.txt")
                exit()
            elif coord1 == "s":
                print(ansi_cyan + "You surrendered.\nCoward." + ansi_reset)
                self.delete_file("/home/gun/catkin_ws/src/ur_python/src/coord_check/bs_coord.txt")
                exit()

            if coord2 == "":
                print(ansi_cyan + "Game ended!" + ansi_reset)
                self.delete_file("/home/gun/catkin_ws/src/ur_python/src/coord_check/bs_coord.txt")
                exit()
            elif coord2 == "s":
                print(ansi_cyan + "You surrendered.\nCoward." + ansi_reset)
                self.delete_file("/home/gun/catkin_ws/src/ur_python/src/coord_check/bs_coord.txt")
                exit()
            old = coord1.split(",")
            new = coord2.split(",")

            if len(old) != 2 or len(new) != 2:
                print(ansi_red + "Ilegal input" + ansi_reset)
            else:
                old_i = old[0]
                old_j = old[1]
                new_i = new[0]
                new_j = new[1]
                if not old_i.isdigit() or not old_j.isdigit() or not new_i.isdigit() or not new_j.isdigit():
                    print(ansi_red + "Ilegal input" + ansi_reset)
                else:
                    move = [int(old_i), int(old_j), int(new_i), int(new_j)]
                    if move not in available_moves:
                        print(ansi_red + "Ilegal move!" + ansi_reset)
                    else:
                        Checkers.make_a_move(self.matrix, int(old_i), int(old_j), int(new_i), int(new_j), "B", 0)
                        for m in range(8):
                            for n in range(8):
                                if self.matrix[m][n][0] == "c" or self.matrix[m][n][0] == "C":
                                    self.computer_pieces += 1
                                elif self.matrix[m][n][0] == "b" or self.matrix[m][n][0] == "B":
                                    self.player_pieces += 1
                        print(self.matrix)

                        break

    @staticmethod
    def find_available_moves(board, mandatory_jumping):
        available_moves = []
        available_jumps = []
        for m in range(8):
            for n in range(8):
                if board[m][n][0] == "c":
                    if Checkers.check_moves(board, m, n, m + 1, n + 1):
                        available_moves.append([m, n, m + 1, n + 1])
                    if Checkers.check_moves(board, m, n, m + 1, n - 1):
                        available_moves.append([m, n, m + 1, n - 1])
                    if Checkers.check_jumps(board, m, n, m + 1, n - 1, m + 2, n - 2):
                        available_jumps.append([m, n, m + 2, n - 2])
                    if Checkers.check_jumps(board, m, n, m + 1, n + 1, m + 2, n + 2):
                        available_jumps.append([m, n, m + 2, n + 2])
                elif board[m][n][0] == "C":
                    if Checkers.check_moves(board, m, n, m + 1, n + 1):
                        available_moves.append([m, n, m + 1, n + 1])
                    if Checkers.check_moves(board, m, n, m + 1, n - 1):
                        available_moves.append([m, n, m + 1, n - 1])
                    if Checkers.check_moves(board, m, n, m - 1, n - 1):
                        available_moves.append([m, n, m - 1, n - 1])
                    if Checkers.check_moves(board, m, n, m - 1, n + 1):
                        available_moves.append([m, n, m - 1, n + 1])
                    if Checkers.check_jumps(board, m, n, m + 1, n - 1, m + 2, n - 2):
                        available_jumps.append([m, n, m + 2, n - 2])
                    if Checkers.check_jumps(board, m, n, m - 1, n - 1, m - 2, n - 2):
                        available_jumps.append([m, n, m - 2, n - 2])
                    if Checkers.check_jumps(board, m, n, m - 1, n + 1, m - 2, n + 2):
                        available_jumps.append([m, n, m - 2, n + 2])
                    if Checkers.check_jumps(board, m, n, m + 1, n + 1, m + 2, n + 2):
                        available_jumps.append([m, n, m + 2, n + 2])
        if mandatory_jumping is False:
            available_jumps.extend(available_moves)
            return available_jumps
        elif mandatory_jumping is True:
            if len(available_jumps) == 0:
                return available_moves
            else:
                return available_jumps

    @staticmethod
    def check_jumps(board, old_i, old_j, via_i, via_j, new_i, new_j):
        if new_i > 7 or new_i < 0:
            return False
        if new_j > 7 or new_j < 0:
            return False
        if board[via_i][via_j] == "---":
            return False
        if board[via_i][via_j][0] == "C" or board[via_i][via_j][0] == "c":
            return False
        if board[new_i][new_j] != "---":
            return False
        if board[old_i][old_j] == "---":
            return False
        if board[old_i][old_j][0] == "b" or board[old_i][old_j][0] == "B":
            return False
        return True

    @staticmethod
    def check_moves(board, old_i, old_j, new_i, new_j):

        if new_i > 7 or new_i < 0:
            return False
        if new_j > 7 or new_j < 0:
            return False
        if board[old_i][old_j] == "---":
            return False
        if board[new_i][new_j] != "---":
            return False
        if board[old_i][old_j][0] == "b" or board[old_i][old_j][0] == "B":
            return False
        if board[new_i][new_j] == "---":
            return True

    @staticmethod
    def calculate_heuristics(board):
        result = 0
        mine = 0
        opp = 0
        for i in range(8):
            for j in range(8):
                if board[i][j][0] == "c" or board[i][j][0] == "C":
                    mine += 1

                    if board[i][j][0] == "c":
                        result += 5
                    if board[i][j][0] == "C":
                        result += 10
                    if i == 0 or j == 0 or i == 7 or j == 7:
                        result += 7
                    if i + 1 > 7 or j - 1 < 0 or i - 1 < 0 or j + 1 > 7:
                        continue
                    if (board[i + 1][j - 1][0] == "b" or board[i + 1][j - 1][0] == "B") and board[i - 1][
                        j + 1] == "---":
                        result -= 3
                    if (board[i + 1][j + 1][0] == "b" or board[i + 1][j + 1] == "B") and board[i - 1][j - 1] == "---":
                        result -= 3
                    if board[i - 1][j - 1][0] == "B" and board[i + 1][j + 1] == "---":
                        result -= 3

                    if board[i - 1][j + 1][0] == "B" and board[i + 1][j - 1] == "---":
                        result -= 3
                    if i + 2 > 7 or i - 2 < 0:
                        continue
                    if (board[i + 1][j - 1][0] == "B" or board[i + 1][j - 1][0] == "b") and board[i + 2][
                        j - 2] == "---":
                        result += 6
                    if i + 2 > 7 or j + 2 > 7:
                        continue
                    if (board[i + 1][j + 1][0] == "B" or board[i + 1][j + 1][0] == "b") and board[i + 2][
                        j + 2] == "---":
                        result += 6

                elif board[i][j][0] == "b" or board[i][j][0] == "B":
                    opp += 1

        return result + (mine - opp) * 1000

    @staticmethod
    def find_player_available_moves(board, mandatory_jumping):
        available_moves = []
        available_jumps = []
        for m in range(8):
            for n in range(8):
                if board[m][n][0] == "b":
                    if Checkers.check_player_moves(board, m, n, m - 1, n - 1):
                        available_moves.append([m, n, m - 1, n - 1])
                    if Checkers.check_player_moves(board, m, n, m - 1, n + 1):
                        available_moves.append([m, n, m - 1, n + 1])
                    if Checkers.check_player_jumps(board, m, n, m - 1, n - 1, m - 2, n - 2):
                        available_jumps.append([m, n, m - 2, n - 2])
                    if Checkers.check_player_jumps(board, m, n, m - 1, n + 1, m - 2, n + 2):
                        available_jumps.append([m, n, m - 2, n + 2])
                elif board[m][n][0] == "B":
                    if Checkers.check_player_moves(board, m, n, m - 1, n - 1):
                        available_moves.append([m, n, m - 1, n - 1])
                    if Checkers.check_player_moves(board, m, n, m - 1, n + 1):
                        available_moves.append([m, n, m - 1, n + 1])
                    if Checkers.check_player_jumps(board, m, n, m - 1, n - 1, m - 2, n - 2):
                        available_jumps.append([m, n, m - 2, n - 2])
                    if Checkers.check_player_jumps(board, m, n, m - 1, n + 1, m - 2, n + 2):
                        available_jumps.append([m, n, m - 2, n + 2])
                    if Checkers.check_player_moves(board, m, n, m + 1, n - 1):
                        available_moves.append([m, n, m + 1, n - 1])
                    if Checkers.check_player_jumps(board, m, n, m + 1, n - 1, m + 2, n - 2):
                        available_jumps.append([m, n, m + 2, n - 2])
                    if Checkers.check_player_moves(board, m, n, m + 1, n + 1):
                        available_moves.append([m, n, m + 1, n + 1])
                    if Checkers.check_player_jumps(board, m, n, m + 1, n + 1, m + 2, n + 2):
                        available_jumps.append([m, n, m + 2, n + 2])
        if mandatory_jumping is False:
            available_jumps.extend(available_moves)
            return available_jumps
        elif mandatory_jumping is True:
            if len(available_jumps) == 0:
                return available_moves
            else:
                return available_jumps

    @staticmethod
    def check_player_moves(board, old_i, old_j, new_i, new_j):
        if new_i > 7 or new_i < 0:
            return False
        if new_j > 7 or new_j < 0:
            return False
        if board[old_i][old_j] == "---":
            return False
        if board[new_i][new_j] != "---":
            return False
        if board[old_i][old_j][0] == "c" or board[old_i][old_j][0] == "C":
            return False
        if board[new_i][new_j] == "---":
            return True

    @staticmethod
    def check_player_jumps(board, old_i, old_j, via_i, via_j, new_i, new_j):
        if new_i > 7 or new_i < 0:
            return False
        if new_j > 7 or new_j < 0:
            return False
        if board[via_i][via_j] == "---":
            return False
        if board[via_i][via_j][0] == "B" or board[via_i][via_j][0] == "b":
            return False
        if board[new_i][new_j] != "---":
            return False
        if board[old_i][old_j] == "---":
            return False
        if board[old_i][old_j][0] == "c" or board[old_i][old_j][0] == "C":
            return False
        return True

    def evaluate_states(self):
        global com 

        t1 = time.time()
        current_state = Node(deepcopy(self.matrix))

        first_computer_moves = current_state.get_children(True, self.mandatory_jumping)
        if len(first_computer_moves) == 0:
            if self.player_pieces > self.computer_pieces:
                print(
                    ansi_yellow + "Computer has no available moves left, and you have more pieces left.\nYOU WIN!" + ansi_reset)
                self.delete_file("/home/gun/catkin_ws/src/ur_python/src/coord_check/bs_coord.txt")
                exit()
            else:
                print(ansi_yellow + "Computer has no available moves left.\nGAME ENDED!" + ansi_reset)
                self.delete_file("/home/gun/catkin_ws/src/ur_python/src/coord_check/bs_coord.txt")
                exit()
        dict = {}
        for i in range(len(first_computer_moves)):
            child = first_computer_moves[i]
            value = Checkers.minimax(child.get_board(), 4, -math.inf, math.inf, False, self.mandatory_jumping)
            dict[value] = child

        if len(dict.keys()) == 0:
            print(ansi_green + "Computer has cornered itself.\nYOU WIN!" + ansi_reset)
            self.delete_file("/home/gun/catkin_ws/src/ur_python/src/coord_check/bs_coord.txt")
            exit()
        new_board = dict[max(dict)].get_board()
        move = dict[max(dict)].move
        self.matrix = new_board
        t2 = time.time()
        diff = t2 - t1

        # # ROS 메시지로 move를 Publish
        move_msg = f"{move[0]},{move[1]}->{move[2]},{move[3]}"
        self.move_pub.publish(move_msg)  # 메시지 발행
        print(f"Published move: {move_msg}")

        # save computer coordinate 
        com = move


        print("Computer has moved (" + str(move[0]) + "," + str(move[1]) + ") to (" + str(move[2]) + "," + str(
            move[3]) + ").")
        print("It took him " + str(diff) + " seconds.")

    @staticmethod
    def minimax(board, depth, alpha, beta, maximizing_player, mandatory_jumping):
        if depth == 0:
            return Checkers.calculate_heuristics(board)
        current_state = Node(deepcopy(board))
        if maximizing_player is True:
            max_eval = -math.inf
            for child in current_state.get_children(True, mandatory_jumping):
                ev = Checkers.minimax(child.get_board(), depth - 1, alpha, beta, False, mandatory_jumping)
                max_eval = max(max_eval, ev)
                alpha = max(alpha, ev)
                if beta <= alpha:
                    break
            current_state.set_value(max_eval)
            return max_eval
        else:
            min_eval = math.inf
            for child in current_state.get_children(False, mandatory_jumping):
                ev = Checkers.minimax(child.get_board(), depth - 1, alpha, beta, True, mandatory_jumping)
                min_eval = min(min_eval, ev)
                beta = min(beta, ev)
                if beta <= alpha:
                    break
            current_state.set_value(min_eval)
            return min_eval

    @staticmethod
    def make_a_move(board, old_i, old_j, new_i, new_j, big_letter, queen_row):
        letter = board[old_i][old_j][0]
        i_difference = old_i - new_i
        j_difference = old_j - new_j
        if i_difference == -2 and j_difference == 2:
            board[old_i + 1][old_j - 1] = "---"

        elif i_difference == 2 and j_difference == 2:
            board[old_i - 1][old_j - 1] = "---"

        elif i_difference == 2 and j_difference == -2:
            board[old_i - 1][old_j + 1] = "---"

        elif i_difference == -2 and j_difference == -2:
            board[old_i + 1][old_j + 1] = "---"

        if new_i == queen_row:
            letter = big_letter
        board[old_i][old_j] = "---"
        board[new_i][new_j] = letter + str(new_i) + str(new_j)

    def play(self):
        print(ansi_cyan + "##### WELCOME TO CHECKERS ####" + ansi_reset)
        print("\nSome basic rules:")
        print("1.You enter the coordinates in the form i,j.")
        print("2.You can quit the game at any time by pressing enter.")
        print("3.You can surrender at any time by pressing 's'.")
        print("Now that you've familiarized yourself with the rules, enjoy!")
        while True:
            answer = input("\nFirst, we need to know, is jumping mandatory?[Y/n]: ")
            # answer = "n"
            if answer == "Y":
                self.mandatory_jumping = True
                break
            elif answer == "n":
                self.mandatory_jumping = False
                break
            else:
                print(ansi_red + "Invalid option!" + ansi_reset)
        while True:
            self.print_matrix()
            if self.current_turn is True:
                print(ansi_cyan + "\nPlayer's turn." + ansi_reset)
                self.get_player_input()
            else:
                print(ansi_cyan + "Computer's turn." + ansi_reset)
                print("Thinking...")
                self.evaluate_states()
            if self.player_pieces == 0:
                self.print_matrix()
                print(ansi_red + "You have no pieces left.\nYOU LOSE!" + ansi_reset)
                self.delete_file("/home/gun/catkin_ws/src/ur_python/src/coord_check/bs_coord.txt")
                exit()
            elif self.computer_pieces == 0:
                self.print_matrix()
                print(ansi_green + "Computer has no pieces left.\nYOU WIN!" + ansi_reset)
                self.delete_file("/home/gun/catkin_ws/src/ur_python/src/coord_check/bs_coord.txt")
                exit()
            # elif self.computer_pieces - self.player_pieces == 7:
            #     wish = input("You have 7 pieces fewer than your opponent.Do you want to surrender?")
            #     if wish == "" or wish == "yes":
            #         print(ansi_cyan + "Coward." + ansi_reset)
            #         self.delete_file("/home/gun/catkin_ws/src/ur_python/src/coord_check/bs_coord.txt")
            #         exit()
            self.current_turn = not self.current_turn


if __name__ == '__main__':
    checkers = Checkers()
    checkers.play()
