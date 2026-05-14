from vector2 import Vec2
from collision import CollisionPair

def Resolution(pair: CollisionPair):

    #grab data from CollisionPair
    obj1 = pair.obj1
    obj2 = pair.obj2
    collision = pair.result

    #determine the greater of the two resitutions
    resitution = obj1.restitution if obj1.restitution > obj2.restitution else obj2.restitution

    #determine coeff of friction to use
    coeffFriction = obj1.frictionCoeff if obj1.frictionCoeff > obj2.frictionCoeff else obj2.frictionCoeff

    for contact in collision.contactPoints:

        #determine the relative velocity
        repulsionVec1 = contact - obj1.position
        repulsionVec2 = contact - obj2.position

        angVeloc2 = Vec2(-obj2.angVelocity * repulsionVec2.y, obj2.angVelocity * repulsionVec2.x)
        angVeloc1 = Vec2(-obj1.angVelocity * repulsionVec1.y, obj1.angVelocity * repulsionVec1.x)

        relativeVelocity = obj2.velocity - obj1.velocity + angVeloc2 - angVeloc1

        #contact velocity between to the two objects
        contactVelocity = relativeVelocity.dot(collision.normal)
        if contactVelocity >= 0:
            continue
        
        #calculate impulse
        repulsionCrossNorm1 = repulsionVec1.x * collision.normal.y - repulsionVec1.y * collision.normal.x
        repulsionCrossNorm2 = repulsionVec2.x * collision.normal.y - repulsionVec2.y * collision.normal.x

        inverseMassSum = 1 / obj1.mass + 1 / obj2.mass + pow(repulsionCrossNorm1,2) / obj1.momentofInertia + pow(repulsionCrossNorm2,2) / obj2.momentofInertia

        jacobian = -(1.0 + resitution) * contactVelocity / inverseMassSum

        impulse = jacobian * collision.normal / len(collision.contactPoints)

        #apply velocity changes
        obj1.velocity -= impulse / obj1.mass
        obj2.velocity += impulse / obj2.mass

        obj1.angVelocity -= (repulsionVec1.x * impulse.y - repulsionVec1.y * impulse.x) / obj1.momentofInertia;
        obj2.angVelocity += (repulsionVec2.x * impulse.y - repulsionVec2.y * impulse.x) / obj2.momentofInertia;

        #friction
        #need to recalculate relative and contact velocities
        angVeloc2 = Vec2(-obj2.angVelocity * repulsionVec2.y, obj2.angVelocity * repulsionVec2.x)
        angVeloc1 = Vec2(-obj1.angVelocity * repulsionVec1.y, obj1.angVelocity * repulsionVec1.x)

        relativeVelocity = obj2.velocity - obj1.velocity + angVeloc2 - angVeloc1
        contactVelocity = relativeVelocity.dot(collision.normal)

        #calculate tangent
        tangentVeloc = relativeVelocity - collision.normal * contactVelocity
        if tangentVeloc.lengthSquared() <= 0:
            continue

        tangent = tangentVeloc.normalize()
        frictionContactvelocity = relativeVelocity.dot(tangent)

        #friction impulse
        tangentCrossNorm1 = repulsionVec1.x * tangent.y - repulsionVec1.y * tangent.x
        tangentCrossNorm2 = repulsionVec2.x * tangent.y - repulsionVec2.y * tangent.x

        frictionInverseMassSum =  1 / obj1.mass + 1 / obj2.mass + pow(tangentCrossNorm1,2) / obj1.momentofInertia + pow(tangentCrossNorm2,2) / obj2.momentofInertia
        
        jFriction = -frictionContactvelocity / frictionInverseMassSum
        jFriction = min(jacobian * coeffFriction, max(jFriction, -jacobian * coeffFriction))

        frictionImpulse = jFriction * tangent / len(collision.contactPoints)

        #apply friction changes
        obj1.velocity -= frictionImpulse / obj1.mass
        obj2.velocity += frictionImpulse / obj2.mass

        obj1.angVelocity -= (repulsionVec1.x * frictionImpulse.y - repulsionVec1.y * frictionImpulse.x) / obj1.momentofInertia;
        obj2.angVelocity += (repulsionVec2.x * frictionImpulse.y - repulsionVec2.y * frictionImpulse.x) / obj2.momentofInertia;
    
