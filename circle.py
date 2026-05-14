from dataclasses import dataclass, field

from body import PhysObj
from vector2 import Vec2

@dataclass
class Circle(PhysObj):
    radius: float = 0.0

    def GetSupport(self, direction: Vec2) -> Vec2:

        return direction * self.radius;
        
    def GetMoment(self) -> float:
        return .5 * self.mass * self.radius * self.radius