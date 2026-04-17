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
    def __init__(self, x, y, typo, value=-1):
        self.x = x
        self.y = y
        self.typo = typo
        self.value = value