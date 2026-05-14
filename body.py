from dataclasses import dataclass, field

from vector2 import Vec2

@dataclass
class PhysObj:
    position: Vec2 = field(default_factory=lambda: Vec2(0, 0))
    rotation: float = 0.0

    velocity: Vec2 = field(default_factory=lambda: Vec2(0, 0))
    angVelocity: float = 0.0

    mass: float = 1.0
    momentofInertia: float = 1.0
    restitution: float = .5
    frictionCoeff: float = .5

    def __post_init__(self):
        self.momentofInertia = self.GetMoment()

    def GetSupport(self, direction: Vec2) -> Vec2:
        support = Vec2(0,0)

        return support
    
    def GetMoment(self) -> float:
        return self.momentofInertia
