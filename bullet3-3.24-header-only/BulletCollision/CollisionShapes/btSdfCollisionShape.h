#ifndef BT_SDF_COLLISION_SHAPE_H
#define BT_SDF_COLLISION_SHAPE_H

#include "btApi.h"

#include "btConcaveShape.h"

class BT_API btSdfCollisionShape : public btConcaveShape
{
	struct btSdfCollisionShapeInternalData* m_data;

public:
	btSdfCollisionShape();
	virtual ~btSdfCollisionShape();

	bool initializeSDF(const char* sdfData, int sizeInBytes);

	virtual void getAabb(const btTransform& t, btVector3& aabbMin, btVector3& aabbMax) const;
	virtual void setLocalScaling(const btVector3& scaling);
	virtual const btVector3& getLocalScaling() const;
	virtual void calculateLocalInertia(btScalar mass, btVector3& inertia) const;
	virtual const char* getName() const;
	virtual void setMargin(btScalar margin);
	virtual btScalar getMargin() const;

	virtual void processAllTriangles(btTriangleCallback* callback, const btVector3& aabbMin, const btVector3& aabbMax) const;

	bool queryPoint(const btVector3& ptInSDF, btScalar& distOut, btVector3& normal);
};

#endif  //BT_SDF_COLLISION_SHAPE_H
