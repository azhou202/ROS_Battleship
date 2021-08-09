#!/usr/bin/env python

from random import randint
from board_construction_utils import place_ships
import rospy
import sys
from ros_battleship.srv import ShotValue, ShotValueResponse, CreateShotValue, CreateShotValueResponse, UpdateBoardValue, UpdateBoardValueResponse


class Player:
    """
    A class that represents a player of the game

    Attributes
    ----------
    target_board: 2D array
        2D array that stores the hits/misses of this player
    home_board: Board
        2D array that stores the ships of this player
    name: str
        player name used for ROS communication

    Methods
    -------
    create_shot()
        Randomly generates x and y coordinates to fire a shot to
    update_board()
        Updates target board with the result of the fired shot
    process_shot()
        Reads the coordinate from an opponent shot.
        Determines if the opponent hit or missed a ship.
        Sends back a response of hit (1) or miss (0)    
    """


    def __init__(self):
        """
        Parameters
        ----------
        name: str
            Name of the player used for ROS communication
        """

        # debug
        print("CREATING INSTANCE OF PLAYER ")

        # create boards
        self.target_board = [[99 for i in range(8)] for j in range(8)]
        self.home_board = [[99 for i in range(8)] for j in range(8)]
        place_ships(self.home_board)

        # counter that keeps track of ships hit
        self.name = rospy.get_namespace()
        rospy.set_param(self.name + 'score', 0) # when this number reaches 16 the player wins 
                        # 17 is chosen because that is the total area of all the ships

        self.create_shot_service = rospy.Service('create_shot', CreateShotValue, self.create_shot) 
        self.process_shot_service = rospy.Service('process_shot', ShotValue, self.process_shot)
        self.update_board_service = rospy.Service('update_board', UpdateBoardValue, self.update_board)
        

    def create_shot(self, req): # a callback
        """
         Creates a random shot and ensures it hasn't been chosen before. Returns this value as an array.

         Parameters
         ----------
         req: CreateShotValue
            None

         Return
         ------
         CreateShotValueResponse
            int8[] that contains the x and y coordinates of the chosen shot
        """

        # make list of legal guesses
        possibilities = list()
        for x in range(8):
            for y in range(8):
                if self.target_board[x][y] == 99:
                    possibilities.append([x, y])
       
        # generate target coordinate
        coordinates = possibilities[randint(0, len(possibilities)-1)]
        x, y, = coordinates[0], coordinates[1]
    
        return CreateShotValueResponse([x,y])


    def update_board(self, req): # a callback
        """
         Updates board

         Parameters
         ----------
         req: UpdateBoardValue 
            either a 0 or 1 int, 0 means miss, 1 means a hit
         
         Return
         ------
            None
        """

        x = req.target[0]
        y = req.target[1]
        
        if req.result == 0:
            self.target_board[x][y] = 0
        else:
            self.target_board[x][y] = 1
            score = rospy.get_param(self.name + 'score') + 1
            rospy.set_param(self.name + 'score', score) 
        

    def process_shot(self, req): # a callback
        """
         Read in the opponent's shot and return hit or miss (1 or 0)

         Parameters
         ----------
         req: ShotValue
            contains two integers that represents the x and y coordinates of the target shot.
        
         Return
         ------
        ShotValueResponse
            int8 value where 1 means hit and 0 means miss
        """

        if self.home_board[req.x][req.y] != 99:
            return ShotValueResponse(1)
        else:
            return ShotValueResponse(0)
        

def main():
        # Start node
        rospy.init_node('player_manager')

        p = Player()

        while not rospy.is_shutdown():

            rospy.spin()
        
        print('Exiting...')


if __name__ == '__main__':
    main()