
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

class btKinematicClosestNotMeConvexResultCallback : public btCollisionWorld::ClosestConvexResultCallback {
public:
	btKinematicClosestNotMeConvexResultCallback(btCollisionObject* me, const btVector3& up, float minSlopeDot)
		: btCollisionWorld::ClosestConvexResultCallback(btVector3(0.0, 0.0, 0.0), btVector3(0.0, 0.0, 0.0)), m_me(me), m_up(up), m_minSlopeDot(minSlopeDot) {}

	virtual float addSingleResult(btCollisionWorld::LocalConvexResult& convexResult, bool normalInWorldSpace) {
		if (convexResult.m_hitCollisionObject == m_me)
			return 1.0;

		if (!convexResult.m_hitCollisionObject->hasContactResponse())
			return 1.0;

        if (!canCollide(m_me, convexResult.m_hitCollisionObject))
            return 1.0;

		btVector3 hitNormalWorld;
		if (normalInWorldSpace) {
			hitNormalWorld = convexResult.m_hitNormalLocal;
		} else {
			/// need to transform normal into worldspace
			hitNormalWorld = convexResult.m_hitCollisionObject->getWorldTransform().getBasis() * convexResult.m_hitNormalLocal;
		}

		float dotUp = m_up.dot(hitNormalWorld);
		if (dotUp < m_minSlopeDot) {
			return 1.0;
		}

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

        if (dynamic && dynamic->hasContactResponse() && stairHeight > 0 && shape && shape->isConvex()) {
            btConvexShape* convexShape = static_cast<btConvexShape*>(shape);

            const btVector3& gravity = m_collisionWorld->getGravity();
            const btVector3 up = -gravity.normalized();
            const float minDot = 0.7;

            const btTransform& end = dynamic->getWorldTransform();
            btTransform start = end;
            start.setOrigin(start.getOrigin() + up * stairHeight);

            btKinematicClosestNotMeConvexResultCallback callback(dynamic, up, minDot);
            callback.m_collisionFilterGroup = dynamic->getBroadphaseHandle()->m_collisionFilterGroup;
	        callback.m_collisionFilterMask = dynamic->getBroadphaseHandle()->m_collisionFilterMask;

            m_collisionWorld->convexSweepTest(convexShape, start, end, callback, m_collisionWorld->getDispatchInfo().m_allowedCcdPenetration);

            if (callback.hasHit()) {
                float t = callback.m_closestHitFraction;
                const btVector3 stairOffset = up * stairHeight * (1 - t);

                std::cout << "Found stair: " << stairOffset.x() << ", " << stairOffset.y() << ", " << stairOffset.z() << std::endl;
            }

            (this->getNearCallback())(pair, *this, dispatchInfo);
        } else {
            (this->getNearCallback())(pair, *this, dispatchInfo);
        }

        i += 1;

		// if (false) {
		// 	removeOverlappingPair(pair->m_pProxy0, pair->m_pProxy1, dispatcher);
		// } else {
			// i++;
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
