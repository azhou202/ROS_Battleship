from random import randint

def place_ships(board):
    """
    Populates cells on the board with ships. 
    Each space that a ship takes up on the board is denoted with a 1.
    Empty spaces are 99.

    Parameters
    ----------
    board: Board
        The Board object that the ships should be placed on
    
    Return
    ------
    None
    """

    ships = [2, 3, 3, 4, 5] # lenghts of the ships

    for l in ships:
        # generate a starting coordinate 
        options = find_legal_locations(board, l)
        coordinates = options[randint(0, len(options)-1)]
        x, y, = coordinates[0], coordinates[1]
        
        # place the boat once legal start point found
        place_ship(board, l, x, y)

        print("ship placed: " + str(l))


def place_ship(board, length, x, y):
    """
    Places a ship on the board.

    Parameters
    ----------
    board: Board
        The Board object that holds the ships
    length: int
        Length of the ship
    x: int
        Starting x coordinate of the ship
    y: int
        Starting y coordinate of the ship
    
    Return
    ------
    None
    """

    # find a legal path for ship placement
    paths = path_checker(board, length, x, y)
    direction = paths[randint(0, len(paths)-1)]
    
    # place ship
    for i in range(length):
        if direction == 'R':
            board[x][y+i] = 1
        elif direction == 'L':
            board[x][y-i] = 1
        elif direction == 'D':
            board[x+i][y] = 1
        elif direction == 'U':
            board[x-i][y] = 1


def find_legal_locations(board, length):
    """
    Creates a list of all legal start coordinates. 
    A start coordinate is legal when a ship can be placed starting from there and fit on the board and not overlap others

    Parameters
    ----------
    board: Board
        The Board object that holds the ships
    length: int
        Length of ship
    
    Return
    ------
    possibilities: list
        A list that contains the legal coordinates where a ship can be placed
    """

    possibilities = list()
    for x in range(8):
        for y in range(8):
            if path_checker(board, length, x, y):
                possibilities.append([x, y])

    return possibilities


def path_checker(board, length, x, y):
    """
    Returns a list of possible ship placement paths.

    Parameters
    ----------
    board: Board
        The Board object that holds the ships
    length: int
        Length of the ship
    x: int
        Starting x coordinate of the ship
    y: int
        Starting y coordinate of the ship
    
    Return
    ------
    paths: list
        A list that contains the legal directions that a ship can be placed in.
    """

    paths = ['R', 'L', 'D', 'U']

    for i in range(length):
        if y+i > 7 or board[x][y+i] != 99: #RIGHT
            if paths.count('R') != 0:
                paths.remove('R') 
        if y-i < 0 or board[x][y-i] != 99: #LEFT
            if paths.count('L') != 0:
                paths.remove('L')
        if x+i > 7 or board[x+i][y] != 99: #DOWN
            if paths.count('D') != 0:
                paths.remove('D')
        if x-i < 0 or board[x-i][y] != 99: #UP
            if paths.count('U') != 0:
                paths.remove('U')
    
    return paths