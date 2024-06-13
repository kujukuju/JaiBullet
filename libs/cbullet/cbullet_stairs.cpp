
#include "cbullet_stairs.h"
#include "cbullet.h"
#include <assert.h>
#include <stdint.h>
#include <iostream>
#include "btBulletCollisionCommon.h"
#include "btBulletDynamicsCommon.h"
#include "BulletDynamics/ConstraintSolver/btContactConstraint.h"
#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolverMt.h"
#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorldMt.h"
#include "BulletDynamics/Character/btKinematicCharacterController.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcherMt.h"
#include "BulletCollision/CollisionDispatch/btGhostObject.h"

bool canCollide(const btCollisionObject* body0, const btCollisionObject* body1) {
    return (body0->getBroadphaseHandle()->m_collisionFilterGroup & body1->getBroadphaseHandle()->m_collisionFilterMask) &&
           (body1->getBroadphaseHandle()->m_collisionFilterGroup & body0->getBroadphaseHandle()->m_collisionFilterMask);
}

// turns out you can't reliably do this in bullet without a lot of effort
// because bullet convex sweep test will not return true information about the sweep intersection
// but just the approximated EPA result

// it also returns slightly incorrect triangles because it is an approximated intersection time
// so we can't even take the triangle data and perform math ourselves

// options going forward

// 1.
// I can maybe still make bullet work
// by checking the local point, getting the local triangle
// and resolving all collisions myself (called from jai?)
// this will likely lead to new collisions because it's not returning
// the correct triangles in the first place
// so I will have to continue solving the triangle collisions until I run out of vertical space, or an invalid normal is found
// this seems like a horrible plan

// 2.
// when a new collision is detected, so I think when the manifold is being dispatched
// call into jai to perform my own gjk directional function that will return the correct normal and t value
// this would depend on the collision dispatcher including all intersecting triangles in either a manifold or a contact point
// I don't think I can rely on bullet to do this because it heuristically will probably remove triangles that are less deep in the mesh
// and if I try to resolve in a specific direction these triangles would be relevant
// so it seems like itll be unreliable, but theres a chance itll work if I have all the triangles I need

// 3. 
// when I'm dispatching all manifolds I could get all unique objects I'm colliding with
// and I could get all triangle overlaps regardless of the actual manifold or contact point
// I could then use my own gjk direction thing and solve these triangles myself, and potentially discard the manifolds
// I think this would work but it's basically repeating a step in the narrow phase physic algorithm and I think
// it would be pretty bad performance
// seems like a lot of work for not much but since it would work this could be a fallback option

// 4.
// switch to physx
// I would probably end up treating the character as a dynamic rigid body
// and I would implement a physx character controller that's kinematic
// every frame I would get the dynamic character position and velocity
// and I would update the kinematic character to move in the same step for this frame
// if the kinematic character collides with a step, resolve it by moving the character up
// collisionFlags & PxControllerCollisionFlag::eCOLLISION_DOWN I think
// if the character gets moved up, teleport / set the position of the dynamic rigid body the new location
// maintain all original rigid body forces otherwise
//
// I need to be sure you can mark specific rigid bodies as continuous collision detection in physx
//
// this seems like maybe the best solution........

class btKinematicClosestConvexResultCallback : public btCollisionWorld::ClosestConvexResultCallback {
public:
	btKinematicClosestConvexResultCallback(btCollisionObject* me, const btVector3& up, float minSlopeDot)
		: btCollisionWorld::ClosestConvexResultCallback(btVector3(0.0, 0.0, 0.0), btVector3(0.0, 0.0, 0.0)), m_me(me), m_up(up), m_minSlopeDot(minSlopeDot) {}

	virtual float addSingleResult(btCollisionWorld::LocalConvexResult& convexResult, bool normalInWorldSpace) {
		if (convexResult.m_hitCollisionObject == m_me) {
			return 1.0;
        }

		if (!convexResult.m_hitCollisionObject->hasContactResponse()) {
            std::cout << "Skipping sweep because no contact response" << std::endl;
			return 1.0;
        }

        if (!canCollide(m_me, convexResult.m_hitCollisionObject)) {
            std::cout << "Skipping sweep because cannot collide" << std::endl;
            return 1.0;
        }

		btVector3 hitNormalWorld;
		if (normalInWorldSpace) {
			hitNormalWorld = convexResult.m_hitNormalLocal;
		} else {
			/// need to transform normal into worldspace
			hitNormalWorld = convexResult.m_hitCollisionObject->getWorldTransform().getBasis() * convexResult.m_hitNormalLocal;
		}

		float dotUp = m_up.dot(hitNormalWorld);
		if (dotUp < m_minSlopeDot) {
            std::cout << "Found a invalid slope dot product of " << dotUp << " ... " << hitNormalWorld.x() << ", " << hitNormalWorld.y() << ", " << hitNormalWorld.z() << std::endl;
			return 1.0;
		}

        std::cout << "Found maybe correct surface " << dotUp << " " << hitNormalWorld.x() << ", " << hitNormalWorld.y() << ", " << hitNormalWorld.z() << " " << m_up.x() << ", " << m_up.y() << ", " << m_up.z() << std::endl;

		return ClosestConvexResultCallback::addSingleResult(convexResult, normalInWorldSpace);
	}

protected:
	btCollisionObject* m_me;
	const btVector3 m_up;
	float m_minSlopeDot;
};

CbtStairCollisionDispatcher::CbtStairCollisionDispatcher(btCollisionConfiguration* collisionConfiguration, btDiscreteDynamicsWorld* collisionWorld, CbtGetStairHeight getStairHeight)
    : btCollisionDispatcher(collisionConfiguration), m_collisionWorld(collisionWorld), m_getStairHeight(getStairHeight) {}

CbtStairCollisionDispatcher::~CbtStairCollisionDispatcher() {}



// overwrite the default btCollisionDispatcher
// overwrite dispatchAllCollisionPairs
// get the getOverlappingPairArray result
// go through all overlapping pairs
// these are pairs of collision objects
// get the manifold from the dispatcher

// if it contains a rigid body that is allowed to stair step
// perform a convex sweep from step height down to current height
// if a valid surface was found
// manipulate the existing manifold contact points to be +/- your resolve direction (surface normal) and resolve distance

// otherwise if its a normal body collision call into defaultNearCallback



// one remaining problem is that the all overlaps / collision manifolds get found before dispatchAllCollisionPairs happens
// if you find a valid stair step, the correct thing to do is discard all remaining manifolds and
// look for any new collision overlap / manifolds and resolve all of those afterwards


btPersistentManifold* CbtStairCollisionDispatcher::getNewManifold(const btCollisionObject* b0, const btCollisionObject* b1) {
    btCollisionObject* dynamic = nullptr;
    btCollisionShape* shape = nullptr;
    float stairHeight = 0;
    if (b0 && b1 && b0->getInternalType() == btCollisionObject::CO_RIGID_BODY && !b0->isStaticOrKinematicObject() && b1->isStaticOrKinematicObject()) {
        dynamic = const_cast<btCollisionObject*>(b0);
        stairHeight = m_getStairHeight((CbtBodyHandle) dynamic);
        shape = dynamic->getCollisionShape();
    } else if (b0 && b1 && b1->getInternalType() == btCollisionObject::CO_RIGID_BODY && !b1->isStaticOrKinematicObject() && b0->isStaticOrKinematicObject()) {
        dynamic = const_cast<btCollisionObject*>(b1);
        stairHeight = m_getStairHeight((CbtBodyHandle) dynamic);
        shape = dynamic->getCollisionShape();
    }

    if (dynamic && dynamic->hasContactResponse() && stairHeight > 0 && shape && shape->isConvex()) {
        const btConvexShape* convexShape = static_cast<const btConvexShape*>(shape);

        const btVector3& gravity = m_collisionWorld->getGravity();
        const btVector3 up = -gravity.normalized();
        const float minDot = 0.7;

        btTransform end = dynamic->getWorldTransform();
        end.setOrigin(end.getOrigin() - up * stairHeight);
        btTransform start = dynamic->getWorldTransform();
        start.setOrigin(start.getOrigin() + up * stairHeight);

        std::cout << "Trying to sweep from (new) " << start.getOrigin().x() << ", " << start.getOrigin().y() << ", " << start.getOrigin().z() << " to " << end.getOrigin().x() << ", " << end.getOrigin().y() << ", " << end.getOrigin().z() << std::endl;
        std::cout << "Allowed penetration is " << m_collisionWorld->getDispatchInfo().m_allowedCcdPenetration << std::endl;

        btKinematicClosestConvexResultCallback callback(dynamic, up, minDot);
        callback.m_collisionFilterGroup = dynamic->getBroadphaseHandle()->m_collisionFilterGroup;
        callback.m_collisionFilterMask = dynamic->getBroadphaseHandle()->m_collisionFilterMask;

        m_collisionWorld->convexSweepTest(convexShape, start, end, callback, m_collisionWorld->getDispatchInfo().m_allowedCcdPenetration);

        if (callback.hasHit()) {
            float t = callback.m_closestHitFraction;
            const btVector3 stairOffset = up * stairHeight * (0.5 - t);

            if (t < 0.5) {
                std::cout << "Found stair: " << stairOffset.x() << ", " << stairOffset.y() << ", " << stairOffset.z() << " ... " << b0 << " " << b1 << std::endl;
            } else {
                std::cout << "Found not stair: " << stairOffset.x() << ", " << stairOffset.y() << ", " << stairOffset.z() << " ... " << b0 << " " << b1 << std::endl;
            }
        } else {
            std::cout << "Has a collision pair without a hit..." << std::endl;
        }
    } else {

    }

    return btCollisionDispatcher::getNewManifold(b0, b1);
}


void CbtStairCollisionDispatcher::dispatchAllCollisionPairs(btOverlappingPairCache* pairCache, const btDispatcherInfo& dispatchInfo, btDispatcher* dispatcher) {
    btBroadphasePairArray& overlappingPairArray = pairCache->getOverlappingPairArray();

    int i;
	for (i = 0; i < overlappingPairArray.size();) {
		btBroadphasePair& pair = overlappingPairArray[i];

        btCollisionObject* obj0 = static_cast<btCollisionObject*>(pair.m_pProxy0->m_clientObject);
        btCollisionObject* obj1 = static_cast<btCollisionObject*>(pair.m_pProxy1->m_clientObject);

        btCollisionObject* dynamic = nullptr;
        btCollisionShape* shape = nullptr;
        float stairHeight = 0;
        if (obj0 && obj1 && obj0->getInternalType() == btCollisionObject::CO_RIGID_BODY && !obj0->isStaticOrKinematicObject() && obj1->isStaticOrKinematicObject()) {
            dynamic = obj0;
            stairHeight = m_getStairHeight((CbtBodyHandle) dynamic);
            shape = dynamic->getCollisionShape();
        } else if (obj0 && obj1 && obj1->getInternalType() == btCollisionObject::CO_RIGID_BODY && !obj1->isStaticOrKinematicObject() && obj0->isStaticOrKinematicObject()) {
            dynamic = obj1;
            stairHeight = m_getStairHeight((CbtBodyHandle) dynamic);
            shape = dynamic->getCollisionShape();
        }

//         if (dynamic) {
//             (this->getNearCallback())(pair, *this, dispatchInfo);
//             i += 1;
// //            pairCache->removeOverlappingPair(pair.m_pProxy0, pair.m_pProxy1, dispatcher);

//         } else {
//             (this->getNearCallback())(pair, *this, dispatchInfo);
//             i += 1;
//         }

        if (dynamic && dynamic->hasContactResponse() && stairHeight > 0 && shape && shape->isConvex()) {
            btConvexShape* convexShape = static_cast<btConvexShape*>(shape);

            const btVector3& gravity = m_collisionWorld->getGravity();
            const btVector3 up = -gravity.normalized();
            const float minDot = 0.7;

            btTransform end = dynamic->getWorldTransform();
            end.setOrigin(end.getOrigin() - up * stairHeight);
            btTransform start = dynamic->getWorldTransform();
            start.setOrigin(start.getOrigin() + up * stairHeight);

            std::cout << "Trying to sweep from (dispatch) " << start.getOrigin().x() << ", " << start.getOrigin().y() << ", " << start.getOrigin().z() << " to " << end.getOrigin().x() << ", " << end.getOrigin().y() << ", " << end.getOrigin().z() << std::endl;
            // std::cout << "Allowed penetration is " << m_collisionWorld->getDispatchInfo().m_allowedCcdPenetration << std::endl;

            btKinematicClosestConvexResultCallback callback(dynamic, up, minDot);
            callback.m_collisionFilterGroup = dynamic->getBroadphaseHandle()->m_collisionFilterGroup;
	        callback.m_collisionFilterMask = dynamic->getBroadphaseHandle()->m_collisionFilterMask;

            m_collisionWorld->convexSweepTest(convexShape, start, end, callback, m_collisionWorld->getDispatchInfo().m_allowedCcdPenetration);

            if (callback.hasHit()) {
                float t = callback.m_closestHitFraction;
                const btVector3 stairOffset = up * stairHeight * (0.5 - t);

                if (t < 0.5) {
                    std::cout << "Found stair: " << t << " ... " << stairOffset.x() << ", " << stairOffset.y() << ", " << stairOffset.z() << " ... " << obj0 << " " << obj1 << std::endl;
                } else {
                    std::cout << "Found not stair: " << t << " ... " << stairOffset.x() << ", " << stairOffset.y() << ", " << stairOffset.z() << " ... " << obj0 << " " << obj1 << std::endl;
                }
            } else {
                std::cout << "Has a collision pair without a hit..." << std::endl;
            }

            (this->getNearCallback())(pair, *this, dispatchInfo);
        } else {
            if (dynamic) {
                std::cout << "Not attempting a stair step because... " << dynamic->hasContactResponse() << stairHeight << shape->isConvex() << std::endl;
            }

            (this->getNearCallback())(pair, *this, dispatchInfo);
        }

        i += 1;

		// if (false) {
		// 	removeOverlappingPair(pair->m_pProxy0, pair->m_pProxy1, dispatcher);
		// } else {
		// 	i++;
		// }
	}

    // for (int i = 0; i < pairArray.size(); i++) {
    //     btBroadphasePair& pair = pairArray[i];

    //     btCollisionObject* obj0 = static_cast<btCollisionObject*>(pair.m_pProxy0->m_clientObject);
    //     btCollisionObject* obj1 = static_cast<btCollisionObject*>(pair.m_pProxy1->m_clientObject);

	// 	if (!obj0 || !obj0->hasContactResponse() || !obj1 || !obj1->hasContactResponse())
	// 		continue;

	// 	if (!needsCollision(obj0, obj1))
	// 		continue;

	// 	if (pair->m_algorithm)
	// 		pair->m_algorithm->getAllContactManifolds(m_manifoldArray);
    // }
}
