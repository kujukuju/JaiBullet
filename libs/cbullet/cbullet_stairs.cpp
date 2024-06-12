
#include "cbullet_stairs.h"
#include "cbullet.h"
#include <assert.h>
#include <stdint.h>
#include "btBulletCollisionCommon.h"
#include "btBulletDynamicsCommon.h"
#include "BulletDynamics/ConstraintSolver/btContactConstraint.h"
#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolverMt.h"
#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorldMt.h"
#include "BulletDynamics/Character/btKinematicCharacterController.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcherMt.h"
#include "BulletCollision/CollisionDispatch/btGhostObject.h"

CbtStairCollisionDispatcher::CbtStairCollisionDispatcher(btCollisionConfiguration* collisionConfiguration) : btCollisionDispatcher(collisionConfiguration) {}

CbtStairCollisionDispatcher::~CbtStairCollisionDispatcher() {}

void CbtStairCollisionDispatcher::dispatchAllCollisionPairs(btOverlappingPairCache* pairCache, const btDispatcherInfo& dispatchInfo, btDispatcher* dispatcher) {
    btBroadphasePairArray& overlappingPairArray = pairCache->getOverlappingPairArray();

    int i;
	for (i = 0; i < overlappingPairArray.size();) {
		btBroadphasePair& pair = overlappingPairArray[i];
        (this->getNearCallback())(pair, *this, dispatchInfo);
		// if (false) {
		// 	removeOverlappingPair(pair->m_pProxy0, pair->m_pProxy1, dispatcher);
		// } else {
			i++;
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
