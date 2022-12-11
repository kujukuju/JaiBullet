
#ifndef B3_CONTACT_SPHERE_SPHERE_H
#define B3_CONTACT_SPHERE_SPHERE_H

#include "btApi.h"

void computeContactSphereConvex(int pairIndex,
								int bodyIndexA, int bodyIndexB,
								int collidableIndexA, int collidableIndexB,
								const b3RigidBodyData* rigidBodies,
								const b3Collidable* collidables,
								const b3ConvexPolyhedronData* convexShapes,
								const b3Vector3* convexVertices,
								const int* convexIndices,
								const b3GpuFace* faces,
								b3Contact4* globalContactsOut,
								int& nGlobalContactsOut,
								int maxContactCapacity)
{
}
#endif  //B3_CONTACT_SPHERE_SPHERE_H
