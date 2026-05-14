from dataclasses import dataclass, field

from body import PhysObj
from vector2 import Vec2

@dataclass
class Box(PhysObj):
    halfWidth : float = 0.0
    halfHeight : float = 0.0

    def GetSupport(self, direction: Vec2) -> Vec2:

        support = Vec2(self.halfWidth, self.halfHeight)
        if direction.x < 0:
            support.x = -self.halfWidth
        if direction.y < 0:
            support.y = -self.halfHeight

        return support
    
    def GetMoment(self) -> float:
        return self.mass * (pow((self.halfHeight * 2),2) + pow((self.halfWidth * 2), 2)) / 12
