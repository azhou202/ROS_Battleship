#!/usr/bin/env python

import rospy
from Player import Player
from ros_battleship.srv import ShotValue, ShotValueResponse, CreateShotValue, CreateShotValueResponse, UpdateBoardValue, UpdateBoardValueResponse

def play_game():
    """
    Sets up necessary game values and plays game until a winner is determined
    """

    # create counter for taking turns, 1 = player 1, 2 = player 2
    turn = 1

    # start game

    while(rospy.get_param('/player1/score', 0) < 17 and rospy.get_param('/player2/score', 0) < 17): # checks for a winner
        if turn == 1:
            print("PLAYER ONE'S TURN")

            # p1 create shot
            rospy.wait_for_service('/player1/create_shot')
            try:
                create_shot_caller = rospy.ServiceProxy('/player1/create_shot', CreateShotValue)
                target = create_shot_caller()
            
                # unpack shot
                x = target.result[0]
                y = target.result[1]
            except rospy.ServiceException as e:
                pass 

            # fire p1's created shot
            rospy.wait_for_service('/player2/process_shot')
            try:
                fire_shot_caller = rospy.ServiceProxy('/player2/process_shot', ShotValue)
                result = fire_shot_caller(x, y)
            except rospy.ServiceException as e:
                pass

            # update p1 board
            rospy.wait_for_service('/player1/update_board')
            try:
                update_board_caller = rospy.ServiceProxy('/player1/update_board', UpdateBoardValue)
                update = update_board_caller(result.response, target.result)
            except rospy.ServiceException as e:
                pass

            # switch to player 2's turn
            turn = 2

        else: # player 2's turn
            print("PLAYER TWO'S TURN")

            # p2 create shot
            rospy.wait_for_service('/player2/create_shot')
            try:
                create_shot_caller = rospy.ServiceProxy('/player2/create_shot', CreateShotValue)
                target = create_shot_caller()

                # unpack shot
                x = target.result[0]
                y = target.result[1]
            except rospy.ServiceException as e:
                pass

            # fire p2's created shot
            rospy.wait_for_service('/player1/process_shot')
            try:
                fire_shot_caller = rospy.ServiceProxy('/player1/process_shot', ShotValue)
                result = fire_shot_caller(x, y)
            except rospy.ServiceException as e:
                pass

            # update p2 board
            rospy.wait_for_service('/player2/update_board')
            try:
                update_board_caller = rospy.ServiceProxy('/player2/update_board', UpdateBoardValue)
                update = update_board_caller(result.response, target.result)
            except rospy.ServiceException as e:
                pass

            turn = 1
        
        rospy.sleep(0.25)

    winner = None
    if rospy.get_param('/player1/score') == 17:
        winner = 'player_1'
    else:
        winner = 'player_2'
    print("GAME OVER, " + winner.upper() + " WINS!")
    

if __name__ == '__main__':
    play_game()
