
#pragma once

#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"

class CbtStairCollisionDispatcher : btCollisionDispatcher {
public:
	CbtStairCollisionDispatcher(btCollisionConfiguration* collisionConfiguration);
	virtual ~CbtStairCollisionDispatcher();
	void dispatchAllCollisionPairs(btOverlappingPairCache* pairCache, const btDispatcherInfo& dispatchInfo, btDispatcher* dispatcher) override;
};
