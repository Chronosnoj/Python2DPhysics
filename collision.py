from dataclasses import dataclass, field
import math

from vector2 import Vec2
from body import PhysObj
from circle import Circle
from box import Box

@dataclass
class CollisionResult:
    colliding: bool = False
    overlap: float = 0.0
    normal: Vec2 = field(default_factory=lambda: Vec2(0,0))
    contactPoints: list[Vec2] = field(default_factory=lambda: [Vec2(0,0)])

@dataclass 
class CollisionPair:
    obj1: PhysObj
    obj2: PhysObj
    result: CollisionResult

#jumptable for quick access to collision functions
jump_table = {}

#register function that allows the jump table to work
#*types allow any number of types to be included
def register(*types):
    def decorator(func):
        jump_table[types] = func
        return func
    return decorator

@register(Circle, Circle)
def CircleCircle(cir1: Circle, cir2: Circle) -> CollisionResult:

    result = CollisionResult()
    
    normVec = cir2.position - cir1.position;
    dist = normVec.length();

    radSum = cir1.radius + cir2.radius;
    if dist < radSum:
        result.colliding = True
        result.normal = normVec / dist
        result.overlap = (radSum - dist)
        result.contactPoints[0] = cir1.position + result.normal * cir1.radius

    return result


aabbNormVecs = [Vec2(0,1), Vec2(-1,0), Vec2(0,-1), Vec2(1,0)]

@register(Box, Circle)
def BoxCircle(box: Box, cir: Circle) -> CollisionResult:

    result = CollisionResult()

    #move and rotate circle into box's frame
    newCirPos = cir.position - box.position
    newCirPos = newCirPos.rotate(-box.rotation)
    
    minOverlap = math.inf
    edgeNorm = Vec2(0,0)
    #use normal with support point from box and support point from cir
    for norm in aabbNormVecs:
        boxSupport = box.GetSupport(norm)

        cirSupport = newCirPos + cir.radius * -norm
        cirProj = cirSupport.dot(norm)
        boxProj = boxSupport.dot(norm)
        overlap = boxProj - cirProj
        if overlap < minOverlap:
            minOverlap = overlap
            edgeNorm = norm

    if minOverlap < 0:
        return result

    closestPoint = Vec2(max(-box.halfWidth, min(box.halfWidth, newCirPos.x)),
    max(-box.halfHeight, min(box.halfHeight, newCirPos.y)))
    
    trueDir = newCirPos - closestPoint
    trueDist = trueDir.length()
    if trueDist > cir.radius:
        return result

    result.colliding = True
    if trueDist == 0:
        result.normal = edgeNorm.rotate(box.rotation)
    else:
        result.normal = (trueDir / trueDist).rotate(box.rotation)
    
    result.overlap = cir.radius - trueDist
    result.contactPoints[0] = closestPoint.rotate(box.rotation) + box.position

    return result

@register(Circle, Box)
def CircleBox(cir: Circle, box: Box) -> CollisionResult:
    result = BoxCircle(box, cir)
    result.normal = -result.normal
    return result

epsilon = .0001

#returns true if the point is within or within some episilon of the box
def BoxPoint(box: Box, point: Vec2, epsilonVal: float = 0.0) -> bool:

    #rotate point into box frame
    newPointPos = point - box.position
    newPointPos = newPointPos.rotate(-box.rotation)

    #get closest point on box
    closestPoint = Vec2(max(-box.halfWidth, min(box.halfWidth, newPointPos.x)),
    max(-box.halfHeight, min(box.halfHeight, newPointPos.y)))

    #check distance between point and closest point, if within episilon, return true
    distVec = newPointPos - closestPoint
    dist = distVec.length()

    if dist < epsilonVal:
        return True

    return False

@register(Box, Box)
def BoxBox(box1: Box, box2: Box) -> CollisionResult:
    result = CollisionResult()

    minOverlap = math.inf
    edgeNorm = Vec2(0,0)
    closestPoint1 = Vec2(0,0)
    closestPoint2 = Vec2(0,0)

    #test against box1
    newBox2Pos = box2.position - box1.position

    for norm in aabbNormVecs:
        boxSupport1 = box1.GetSupport(norm)

        boxAdjustedNorm = norm.rotate(box1.rotation)
        boxAdjustedNorm = boxAdjustedNorm.rotate(-box2.rotation)
        boxSupport2 = box2.GetSupport(-boxAdjustedNorm)
        boxSupport2 = boxSupport2.rotate(box2.rotation) + newBox2Pos
        boxSupport2 = boxSupport2.rotate(-box1.rotation)

        boxProj1 = boxSupport1.dot(norm)
        boxProj2 = boxSupport2.dot(norm)

        overlap = boxProj1 - boxProj2
        if overlap < minOverlap:
            minOverlap = overlap
            edgeNorm = norm.rotate(box1.rotation)
            closestPoint1 = boxSupport1.rotate(box1.rotation) + box1.position
            closestPoint2 = boxSupport2.rotate(box1.rotation) + box1.position

    #test against box2
    newBox1Pos = box1.position - box2.position

    for norm in aabbNormVecs:
        boxSupport2 = box2.GetSupport(norm)

        boxAdjustedNorm = norm.rotate(box2.rotation)
        boxAdjustedNorm = boxAdjustedNorm.rotate(-box1.rotation)
        boxSupport1 = box1.GetSupport(-boxAdjustedNorm)
        boxSupport1 = boxSupport1.rotate(box1.rotation) + newBox1Pos
        boxSupport1 = boxSupport1.rotate(-box2.rotation)

        boxProj1 = boxSupport1.dot(norm)
        boxProj2 = boxSupport2.dot(norm)

        overlap = boxProj2 - boxProj1
        if overlap < minOverlap:
            minOverlap = overlap
            edgeNorm = -norm.rotate(box2.rotation) #keep same box1 to box2 normal for resolution
            closestPoint1 = boxSupport1.rotate(box2.rotation) + box2.position
            closestPoint2 = boxSupport2.rotate(box2.rotation) + box2.position

    if minOverlap < 0:
        return result

    result.colliding = True
    result.normal = edgeNorm
    result.overlap = minOverlap
    
    #clip contact points if they are not touching the other box
    point1 = False
    point2 = False

    if BoxPoint(box1, closestPoint1, epsilon) and BoxPoint(box2, closestPoint1, epsilon):
        point1 = True

    if BoxPoint(box1, closestPoint2, epsilon) and BoxPoint(box2, closestPoint2, epsilon):
        point2 = True

    if point1:
        result.contactPoints[0] = closestPoint1
        if point2:
            result.contactPoints.append(closestPoint2)
    elif point2:
        result.contactPoints[0] = closestPoint2

    return result

def Collide(physobj1: PhysObj, physobj2: PhysObj) -> CollisionResult:
    return jump_table[type(physobj1), type(physobj2)](physobj1, physobj2);

