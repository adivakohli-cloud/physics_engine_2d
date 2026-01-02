import math

class Vec2:
    __slots__=("x","y")

    def __init__(self,x=0.0,y=0.0):
        self.x=float(x)
        self.y=float(y)

    def __add__(self,other):
        return Vec2(self.x+other.x,self.y+other.y)

    def __sub__(self,other):
        return Vec2(self.x-other.x,self.y-other.y)

    def __mul__(self,scalar):
        return Vec2(self.x*scalar,self.y*scalar)

    __rmul__=__mul__

    def __neg__(self):
        return Vec2(-self.x, -self.y)

    def perp(self):
        return Vec2(-self.y,self.x)

    def length(self):
        return math.hypot(self.x, self.y)

    def normalized(self):
        l=self.length()
        return self*(1/l) if l>1e-8 else Vec2()

    def dot(self, other):
        return self.x * other.x + self.y * other.y
