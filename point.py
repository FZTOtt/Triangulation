class Point:
    def __init__(self, x=None, y=None, dict=None) -> None:
        if dict is not None:
            self.x = dict[0].get('x')
            self.y = dict[0].get('y')
        else:
            self.x = x
            self.y = y

    def __eq__(self, other: object) -> bool:
        if isinstance(other, Point):
            return self.x == other.x and self.y == other.y
        return False

    def __hash__(self):
        return hash((self.x, self.y))

    def __repr__(self):
        return f"Point({self.x}, {self.y})"
    
    def distance(self, p):
        '''
        Returns the distance between self and p
        '''
        return math.sqrt((p.x - self.x)**2 + (p.y - self.y)**2)
        
    def plot_point(self):
        '''
        Plots x,y coordinates of self
        '''
        plt.plot([self.x], [self.y], 'ro')