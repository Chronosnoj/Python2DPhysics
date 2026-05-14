import math
from dataclasses import dataclass

@dataclass
class Vec2:
    x: float
    y: float

    def __add__(self, other): return Vec2(self.x + other.x, self.y + other.y)
    def __sub__(self, other): return Vec2(self.x - other.x, self.y - other.y)
    def __mul__(self, value: float): return Vec2(self.x * value, self.y * value)
    def __rmul__(self, other): return self * other
    def __truediv__(self, value: float): return Vec2(self.x / value, self.y / value)
    def __neg__(self): return Vec2(-self.x, -self.y)

    def lengthSquared(self): return (self.x * self.x + self.y * self.y)
    def length(self): return math.sqrt(self.x * self.x + self.y * self.y)
    def dot(self, other): return self.x * other.x + self.y * other.y
    def rotate(self, angle: float): return Vec2(math.cos(angle) * self.x - math.sin(angle) * self.y, math.sin(angle) * self.x + math.cos(angle) * self.y)
    
    def normalize(self):
        sqLength = self.x * self.x + self.y * self.y
        length = math.sqrt(sqLength)

        return Vec2(self.x / length, self.y / length)
    
    def getNormalToVector(self): 
        norm = Vec2(-self.y, self.x)
        return norm.normalize()
