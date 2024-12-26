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