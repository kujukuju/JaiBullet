
#pragma once

#include "cbullet.h"
#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"

class CbtStairCollisionDispatcher : btCollisionDispatcher {
public:
	CbtStairCollisionDispatcher(btCollisionConfiguration* collisionConfiguration, btDiscreteDynamicsWorld* collisionWorld, CbtGetStairHeight getStairHeight);
	virtual ~CbtStairCollisionDispatcher();
	void dispatchAllCollisionPairs(btOverlappingPairCache* pairCache, const btDispatcherInfo& dispatchInfo, btDispatcher* dispatcher) override;

    btDiscreteDynamicsWorld* m_collisionWorld;
    CbtGetStairHeight m_getStairHeight;
};
