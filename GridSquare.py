class GridSquare:
    ''' 
    x = coordinate x
    y = coordinate y
    type = one of the following options
    > 0 : unknown
    > 1 : traversed square
    > 2 : heat source
    > 3 : mag source
    > 4 : end
    > 5 : origin
    '''
    def __init__(x, y, type):
        self.x = x
        self.y = y
        self.type = type