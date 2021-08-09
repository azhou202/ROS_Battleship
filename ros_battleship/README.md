# ROS Battleship
ROS Battleship is a version of the popular game Battleship written in ROS and Python. The game is a simple proof-of-concept that only allows the computer to play itself.
## Installation
Clone the repo onto your local machine. Make sure that you are running Ubuntu 18.04 and have ROS/Python installed.
## Usage
In terminal, in the /ros_battleship directory:
```python
roslaunch ros_battleship start_game.launch
```
## Program Structure
Central to the game are Player objects. Each Player contains a board that contains their ships and a board that contains the hits and misses of the shots they fired. Additionally, each player keeps track of their currrent score.   
The Players interact with each other through the game.py file. This file operates as a state machine that allows the Players to take turns sending and receving shots. 

General Structure  
![General_Structure]  
(https://ibb.co/ZV4CHbv)  
Player Object Structure  
![Player_Object_Structure]  
(https://ibb.co/KN9ZFSz)   
Gameplay  
![Gameplay]  
(https://ibb.co/TczQbCY)