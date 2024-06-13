#include "cbullet.h"
#include "cbullet_stairs.h"
#include <assert.h>
#include <stdint.h>
#include "btBulletCollisionCommon.h"
#include "btBulletDynamicsCommon.h"
#include "BulletDynamics/ConstraintSolver/btContactConstraint.h"
#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolverMt.h"
#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorldMt.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcherMt.h"
#include "BulletCollision/CollisionDispatch/btGhostObject.h"

void cbtCalculateAABB(CbtShapeHandle shape_handle) {
    assert(shape_handle && cbtShapeIsCreated(shape_handle));

    int shape_type = cbtShapeGetType(shape_handle);
    
    size_t offset = 0;
    switch (shape_type) {
        case CBT_SHAPE_TYPE_BOX: offset = sizeof(btBoxShape); break;
        case CBT_SHAPE_TYPE_SPHERE: offset = sizeof(btSphereShape); break;
        case CBT_SHAPE_TYPE_CAPSULE: offset = sizeof(btCapsuleShape); break;
        case CBT_SHAPE_TYPE_CONE: offset = sizeof(btConeShape); break;
        case CBT_SHAPE_TYPE_CYLINDER: offset = sizeof(btCylinderShape); break;
        case CBT_SHAPE_TYPE_COMPOUND: offset = sizeof(btCompoundShape); break;
        case CBT_SHAPE_TYPE_TRIANGLE_MESH: offset = sizeof(btBvhTriangleMeshShape) + sizeof(btTriangleIndexVertexArray); break;
        case CBT_SHAPE_TYPE_CONVEX: offset = sizeof(btConvexHullShape); break;
        default:
            assert(0);
    }

    auto shape = (btCollisionShape*) shape_handle;
    auto shape_aabb = *((AABB3*)((uint8_t*)shape_handle + offset));

    btVector3 aabbMin;
    btVector3 aabbMax;
    shape->getAabb(btTransform::getIdentity(), aabbMin, aabbMax);

    shape_aabb[0] = aabbMin.x();
    shape_aabb[1] = aabbMin.y();
    shape_aabb[2] = aabbMin.z();
    shape_aabb[3] = aabbMax.x();
    shape_aabb[4] = aabbMax.y();
    shape_aabb[5] = aabbMax.z();
}

void cbtAlignedAllocSetCustom(CbtAllocFunc alloc, CbtFreeFunc free) {
    btAlignedAllocSetCustom(alloc, free);
}

void cbtAlignedAllocSetCustomAligned(CbtAlignedAllocFunc alloc, CbtAlignedFreeFunc free) {
    btAlignedAllocSetCustomAligned(alloc, free);
}

struct DebugDraw : public btIDebugDraw {
    CbtDebugDraw drawer = {};
    int mode = 0;

    virtual void drawLine(const btVector3& from, const btVector3& to, const btVector3& color) override {
        assert(drawer.drawLine1);
        const Vector3 p0 = { from.x(), from.y(), from.z() };
        const Vector3 p1 = { to.x(), to.y(), to.z() };
        const Vector3 c = { color.x(), color.y(), color.z() };
        drawer.drawLine1(drawer.context, p0, p1, c);
    }

    virtual void drawLine(
        const btVector3& from,
        const btVector3& to,
        const btVector3& color0,
        const btVector3& color1
    ) override {
        if (drawer.drawLine2) {
            const Vector3 p0 = { from.x(), from.y(), from.z() };
            const Vector3 p1 = { to.x(), to.y(), to.z() };
            const Vector3 c0 = { color0.x(), color0.y(), color0.z() };
            const Vector3 c1 = { color1.x(), color1.y(), color1.z() };
            drawer.drawLine2(drawer.context, p0, p1, c0, c1);
        } else {
            btIDebugDraw::drawLine(from, to, color0, color1);
        }
    }

    virtual void drawContactPoint(
        const btVector3& point,
        const btVector3& normal,
        btScalar distance,
        int life_time,
        const btVector3& color
    ) override {
        if (drawer.drawContactPoint) {
            const Vector3 p = { point.x(), point.y(), point.z() };
            const Vector3 n = { normal.x(), normal.y(), normal.z() };
            const Vector3 c = { color.x(), color.y(), color.z() };
            drawer.drawContactPoint(drawer.context, p, n, distance, life_time, c);
        }
    }

    virtual void reportErrorWarning(const char*) override {}

    virtual void draw3dText(const btVector3&, const char*) override {}

    virtual void setDebugMode(int in_mode) override {
        mode = in_mode;
    }

    virtual int getDebugMode() const override {
        return mode;
    }
};

struct WorldData {
    btDiscreteDynamicsWorld* world = nullptr;
    btDefaultCollisionConfiguration* collision_config = nullptr;
    btCollisionDispatcher* dispatcher = nullptr;
    btDbvtBroadphase* broadphase = nullptr;
    btSequentialImpulseConstraintSolver* solver = nullptr;

    btConstraintSolverPoolMt* solver_pool = nullptr;
    DebugDraw* debug = nullptr;
};

static btITaskScheduler* s_task_scheduler = nullptr;

// static float distConv = 1.0;
static float timeConv = 1.0;
static float velConv = 1.0;
static float accelConv = 1.0;

void cbtSetUnits(float distance, float time) {
    assert(distance == 1);
    
    // if you pass in 0.01 to go from pixels -> meters, your distConv becomes 0.01
    timeConv = time;
    // if you pass in 0.001 to go from milliseconds -> seconds, your velConv becomes 1000
    // a velocity you pass in of 0.0004 becomes 0.4
    velConv = distance / time;
    accelConv = distance / (time * time);
}

void cbtTaskSchedInit(void) {
    assert(s_task_scheduler == nullptr);
    s_task_scheduler = btCreateDefaultTaskScheduler();
    btSetTaskScheduler(s_task_scheduler);
}

void cbtTaskSchedDeinit(void) {
    assert(s_task_scheduler != nullptr);
    btSetTaskScheduler(nullptr);
    delete s_task_scheduler;
    s_task_scheduler = nullptr;
}

int cbtTaskSchedGetNumThreads(void) {
    assert(s_task_scheduler != nullptr);
    return s_task_scheduler->getNumThreads();
}

int cbtTaskSchedGetMaxNumThreads(void) {
    assert(s_task_scheduler != nullptr);
    return s_task_scheduler->getMaxNumThreads();
}

void cbtTaskSchedSetNumThreads(int num_threads) {
    assert(s_task_scheduler != nullptr);
    s_task_scheduler->setNumThreads(num_threads);
}

CbtWorldHandle cbtWorldCreate() {
    auto world_data = (WorldData*)btAlignedAlloc(sizeof(WorldData), 16);
    new (world_data) WorldData();

    world_data->collision_config = (btDefaultCollisionConfiguration*)btAlignedAlloc(
        sizeof(btDefaultCollisionConfiguration),
            16
    );
    world_data->broadphase = (btDbvtBroadphase*)btAlignedAlloc(sizeof(btDbvtBroadphase), 16);

    new (world_data->collision_config) btDefaultCollisionConfiguration();
    new (world_data->broadphase) btDbvtBroadphase();

    if (!s_task_scheduler) {
        world_data->dispatcher = (btCollisionDispatcher*)btAlignedAlloc(sizeof(btCollisionDispatcher), 16);
        world_data->solver = (btSequentialImpulseConstraintSolver*)btAlignedAlloc(
            sizeof(btSequentialImpulseConstraintSolver),
            16
        );
        world_data->world = (btDiscreteDynamicsWorld*)btAlignedAlloc(sizeof(btDiscreteDynamicsWorld), 16);

        new (world_data->dispatcher) btCollisionDispatcher(world_data->collision_config);
        new (world_data->solver) btSequentialImpulseConstraintSolver();

        new (world_data->world) btDiscreteDynamicsWorld(
            world_data->dispatcher,
            world_data->broadphase,
            world_data->solver,
            world_data->collision_config
        );
    } else {
        world_data->dispatcher = (btCollisionDispatcherMt*)btAlignedAlloc(sizeof(btCollisionDispatcherMt), 16);
        world_data->solver_pool = (btConstraintSolverPoolMt*)btAlignedAlloc(
            sizeof(btConstraintSolverPoolMt),
            16
        );
        world_data->solver = (btSequentialImpulseConstraintSolverMt*)btAlignedAlloc(
            sizeof(btSequentialImpulseConstraintSolverMt),
            16
        );
        world_data->world = (btDiscreteDynamicsWorldMt*)btAlignedAlloc(sizeof(btDiscreteDynamicsWorldMt), 16);

        new (world_data->dispatcher) btCollisionDispatcherMt(world_data->collision_config);
        new (world_data->solver_pool) btConstraintSolverPoolMt(s_task_scheduler->getNumThreads());
        new (world_data->solver) btSequentialImpulseConstraintSolverMt();

        new (world_data->world) btDiscreteDynamicsWorldMt(
            world_data->dispatcher,
            world_data->broadphase,
            world_data->solver_pool,
            world_data->solver,
            world_data->collision_config
        );
    }

    return (CbtWorldHandle)world_data;
}

CbtWorldHandle cbtWorldCreateWithStairs(CbtGetStairHeight get_stair_height) {
    assert(get_stair_height && !s_task_scheduler);

    auto world_data = (WorldData*)btAlignedAlloc(sizeof(WorldData), 16);
    new (world_data) WorldData();

    world_data->collision_config = (btDefaultCollisionConfiguration*)btAlignedAlloc(
        sizeof(btDefaultCollisionConfiguration),
            16
    );
    world_data->broadphase = (btDbvtBroadphase*)btAlignedAlloc(sizeof(btDbvtBroadphase), 16);

    new (world_data->collision_config) btDefaultCollisionConfiguration();
    new (world_data->broadphase) btDbvtBroadphase();

    {
        world_data->dispatcher = (btCollisionDispatcher*)btAlignedAlloc(sizeof(CbtStairCollisionDispatcher), 16);
        world_data->solver = (btSequentialImpulseConstraintSolver*)btAlignedAlloc(
            sizeof(btSequentialImpulseConstraintSolver),
            16
        );
        world_data->world = (btDiscreteDynamicsWorld*)btAlignedAlloc(sizeof(btDiscreteDynamicsWorld), 16);

        new (world_data->dispatcher) CbtStairCollisionDispatcher(world_data->collision_config, world_data->world, get_stair_height);
        new (world_data->solver) btSequentialImpulseConstraintSolver();

        new (world_data->world) btDiscreteDynamicsWorld(
            world_data->dispatcher,
            world_data->broadphase,
            world_data->solver,
            world_data->collision_config
        );

        // TODO
        world_data->world->getDispatchInfo().m_allowedCcdPenetration = 0.001;
    }

    return (CbtWorldHandle)world_data;
}

void cbtWorldDestroy(CbtWorldHandle world_handle) {
    assert(world_handle);
    auto world_data = (WorldData*)world_handle;

    world_data->dispatcher->~btCollisionDispatcher();
    world_data->collision_config->~btDefaultCollisionConfiguration();
    world_data->broadphase->~btDbvtBroadphase();
    world_data->solver->~btSequentialImpulseConstraintSolver();
    world_data->world->~btDiscreteDynamicsWorld();

    btAlignedFree(world_data->dispatcher);
    btAlignedFree(world_data->collision_config);
    btAlignedFree(world_data->broadphase);
    btAlignedFree(world_data->solver);
    btAlignedFree(world_data->world);

    if (world_data->solver_pool != nullptr) {
        world_data->solver_pool->~btConstraintSolverPoolMt();
        btAlignedFree(world_data->solver_pool);
    }

    if (world_data->debug) {
        world_data->debug->~DebugDraw();
        btAlignedFree(world_data->debug);
    }
    world_data->~WorldData();
    btAlignedFree(world_data);
}

void cbtWorldSetGravity(CbtWorldHandle world_handle, const Vector3 gravity) {
    assert(world_handle && gravity);
    auto world = ((WorldData*)world_handle)->world;
    world->setGravity(btVector3(gravity[0] * accelConv, gravity[1] * accelConv, gravity[2] * accelConv));
}

void cbtWorldGetGravity(CbtWorldHandle world_handle, Vector3 gravity) {
    assert(world_handle && gravity);
    auto world = ((WorldData*)world_handle)->world;
    auto tmp = world->getGravity();
    gravity[0] = tmp.x() / accelConv;
    gravity[1] = tmp.y() / accelConv;
    gravity[2] = tmp.z() / accelConv;
}

int cbtWorldStepSimulation(CbtWorldHandle world_handle, float time_step, int max_sub_steps, float fixed_time_step) {
    assert(world_handle);
    auto world = ((WorldData*)world_handle)->world;
    return world->stepSimulation(time_step * timeConv, max_sub_steps, fixed_time_step * timeConv);
}

void cbtWorldAddBody(CbtWorldHandle world_handle, CbtBodyHandle body_handle) {
    assert(world_handle);
    assert(body_handle && cbtBodyIsCreated(body_handle));
    auto world = ((WorldData*)world_handle)->world;
    auto body = (btRigidBody*)body_handle;
    world->addRigidBody(body);
}

void cbtWorldAddBodyWithGroup(CbtWorldHandle world_handle, CbtBodyHandle body_handle, int group, int mask) {
    assert(world_handle);
    assert(body_handle && cbtBodyIsCreated(body_handle));
    auto world = ((WorldData*)world_handle)->world;
    auto body = (btRigidBody*)body_handle;
    world->addRigidBody(body, group, mask);
}

void cbtWorldAddConstraint(
    CbtWorldHandle world_handle,
    CbtConstraintHandle con_handle,
    bool disable_collision_between_linked_bodies
) {
    assert(world_handle);
    assert(con_handle && cbtConIsCreated(con_handle));
    auto world = ((WorldData*)world_handle)->world;
    auto con = (btTypedConstraint*)con_handle;
    world->addConstraint(con, disable_collision_between_linked_bodies);
}

void cbtWorldRemoveBody(CbtWorldHandle world_handle, CbtBodyHandle body_handle) {
    assert(world_handle);
    assert(body_handle && cbtBodyIsCreated(body_handle));
    auto world = ((WorldData*)world_handle)->world;
    auto body = (btRigidBody*)body_handle;
    world->removeRigidBody(body);
}

void cbtWorldRemoveConstraint(CbtWorldHandle world_handle, CbtConstraintHandle con_handle) {
    assert(world_handle);
    assert(con_handle && cbtConIsCreated(con_handle));
    auto world = ((WorldData*)world_handle)->world;
    auto con = (btTypedConstraint*)con_handle;
    world->removeConstraint(con);
}

int cbtWorldGetNumBodies(CbtWorldHandle world_handle) {
    assert(world_handle);
    auto world = ((WorldData*)world_handle)->world;
    return (int)world->getCollisionObjectArray().size();
}

int cbtWorldGetNumConstraints(CbtWorldHandle world_handle) {
    assert(world_handle);
    auto world = ((WorldData*)world_handle)->world;
    return world->getNumConstraints();
}

CbtBodyHandle cbtWorldGetBody(CbtWorldHandle world_handle, int body_index) {
    assert(world_handle);
    auto world = ((WorldData*)world_handle)->world;
    return (CbtBodyHandle)world->getCollisionObjectArray()[body_index];
}

CbtConstraintHandle cbtWorldGetConstraint(CbtWorldHandle world_handle, int con_index) {
    assert(world_handle);
    auto world = ((WorldData*)world_handle)->world;
    return (CbtConstraintHandle)world->getConstraint(con_index);
}

bool cbtWorldRayTestClosest(
    CbtWorldHandle world_handle,
    const Vector3 ray_from_world,
    const Vector3 ray_to_world,
    int collision_filter_group,
    int collision_filter_mask,
    unsigned int flags,
    CbtRayCastResult* result
) {
    assert(world_handle);
    auto world = ((WorldData*)world_handle)->world;

    const btVector3 from(ray_from_world[0], ray_from_world[1], ray_from_world[2]);
    const btVector3 to(ray_to_world[0], ray_to_world[1], ray_to_world[2]);

    btCollisionWorld::ClosestRayResultCallback closest(from, to);
    closest.m_collisionFilterGroup = collision_filter_group;
    closest.m_collisionFilterMask = collision_filter_mask;
    closest.m_flags = flags;

    world->rayTest(from, to, closest);

    if (result) {
        result->hit_normal_world[0] = closest.m_hitNormalWorld.x();
        result->hit_normal_world[1] = closest.m_hitNormalWorld.y();
        result->hit_normal_world[2] = closest.m_hitNormalWorld.z();
        result->hit_point_world[0] = closest.m_hitPointWorld.x();
        result->hit_point_world[1] = closest.m_hitPointWorld.y();
        result->hit_point_world[2] = closest.m_hitPointWorld.z();
        result->hit_fraction = closest.m_closestHitFraction;
        result->body = (CbtBodyHandle)closest.m_collisionObject;
    }
    return closest.m_collisionObject != 0;
}

void cbtWorldGetContactPoints(
    CbtWorldHandle world_handle,
    CbtGetCollision get_collision,
    void* data
) {

    // to get the collision normals for an object
    // you would go through all the manifolds
    // via accessing the world dispatcher
    // the manifold data should include the object pointers
    // as well as the resolve directions

    // from a given peristent manifold you can get the number of contacts
    // and then iterate over these points and access the contact points via getContactPoint
    // a given contact point should haves the world normal direction of the collision
    // that can be used to determine if it's ground

    assert(world_handle);
    auto world_data = (WorldData*)world_handle;
    auto dispatcher = (btCollisionDispatcher*) world_data->dispatcher;
    int num_manifolds = dispatcher->getNumManifolds();

    btPersistentManifold** manifold_pointer = dispatcher->getInternalManifoldPointer();
    for (int i = 0; i < num_manifolds; i++) {
        btPersistentManifold* manifold = manifold_pointer[i];
        const btCollisionObject* body0 = manifold->getBody0();
        const btCollisionObject* body1 = manifold->getBody1();

        auto handle0 = (CbtBodyHandle) body0;
        auto handle1 = (CbtBodyHandle) body1;

        int num_contacts = manifold->getNumContacts();
        for (int a = 0; a < num_contacts; a++) {
            const btManifoldPoint& contact = manifold->getContactPoint(a);

            const btVector3& normal_vector = contact.m_normalWorldOnB;
            float impulse = contact.m_appliedImpulse;

            const Vector3 normal = {normal_vector.x(), normal_vector.y(), normal_vector.z()};

            if (!get_collision(handle0, handle1, normal, impulse, data)) {
                return;
            }
        }
    }
}

void cbtWorldDebugSetDrawer(CbtWorldHandle world_handle, const CbtDebugDraw* drawer) {
    assert(world_handle && drawer);
    auto world_data = (WorldData*)world_handle;

    if (world_data->debug == nullptr) {
        world_data->debug = (DebugDraw*)btAlignedAlloc(sizeof(DebugDraw), 16);
        new (world_data->debug) DebugDraw();
    }
    world_data->debug->drawer = *drawer;
    world_data->debug->setDebugMode(CBT_DBGMODE_NO_DEBUG);

    world_data->world->setDebugDrawer(world_data->debug);
}

void cbtWorldDebugSetMode(CbtWorldHandle world_handle, int mode) {
    assert(world_handle);
    auto world_data = (WorldData*)world_handle;
    assert(world_data->debug != nullptr);

    if (mode == CBT_DBGMODE_DISABLED) {
        world_data->debug->setDebugMode(CBT_DBGMODE_NO_DEBUG);
        world_data->world->setDebugDrawer(nullptr);
    } else {
        world_data->debug->setDebugMode(mode);
        world_data->world->setDebugDrawer(world_data->debug);
    }
}

int cbtWorldDebugGetMode(CbtWorldHandle world_handle) {
    assert(world_handle);
    auto world_data = (WorldData*)world_handle;
    assert(world_data->debug != nullptr);
    return world_data->debug->mode;
}

void cbtWorldDebugDrawAll(CbtWorldHandle world_handle) {
    assert(world_handle);
    auto world = ((WorldData*)world_handle)->world;
    world->debugDrawWorld();
}

void cbtWorldDebugDrawLine1(
    CbtWorldHandle world_handle,
    const Vector3 p0,
    const Vector3 p1,
    const Vector3 color
) {
    assert(world_handle);
    assert(p0 && p1 && color);
    auto world = ((WorldData*)world_handle)->world;
    assert(world->getDebugDrawer());

    world->getDebugDrawer()->drawLine(
        btVector3(p0[0], p0[1], p0[2]),
        btVector3(p1[0], p1[1], p1[2]),
        btVector3(color[0], color[1], color[2])
    );
}

void cbtWorldDebugDrawLine2(
    CbtWorldHandle world_handle,
    const Vector3 p0,
    const Vector3 p1,
    const Vector3 color0,
    const Vector3 color1
) {
    assert(world_handle);
    assert(p0 && p1 && color0 && color1);
    auto world = ((WorldData*)world_handle)->world;
    assert(world->getDebugDrawer());

    world->getDebugDrawer()->drawLine(
        btVector3(p0[0], p0[1], p0[2]),
        btVector3(p1[0], p1[1], p1[2]),
        btVector3(color0[0], color0[1], color0[2]),
        btVector3(color1[0], color1[1], color1[2])
    );
}

void cbtWorldDebugDrawSphere(
    CbtWorldHandle world_handle,
    const Vector3 position,
    float radius,
    const Vector3 color
) {
    assert(world_handle);
    assert(position && radius > 0.0 && color);
    auto world = ((WorldData*)world_handle)->world;
    assert(world->getDebugDrawer());

    world->getDebugDrawer()->drawSphere(
        btVector3(position[0], position[1], position[2]),
        radius,
        btVector3(color[0], color[1], color[2])
    );
}

static_assert(sizeof(Vector3) == 12, "wrong size");
static_assert(sizeof(AABB3) == 24, "wrong size");
static_assert(sizeof(btCapsuleShape) == sizeof(btCapsuleShapeX), "wrong size");
static_assert(sizeof(btCapsuleShape) == sizeof(btCapsuleShapeZ), "wrong size");
static_assert(sizeof(btConeShape) == sizeof(btConeShapeX), "wrong size");
static_assert(sizeof(btConeShape) == sizeof(btConeShapeZ), "wrong size");
static_assert(sizeof(btCylinderShape) == sizeof(btCylinderShapeX), "wrong size");
static_assert(sizeof(btCylinderShape) == sizeof(btCylinderShapeZ), "wrong size");

CbtShapeHandle cbtShapeAllocate(int shape_type) {
    size_t size = 0;
    switch (shape_type) {
        case CBT_SHAPE_TYPE_BOX: size = sizeof(btBoxShape) + sizeof(AABB3); break;
        case CBT_SHAPE_TYPE_SPHERE: size = sizeof(btSphereShape) + sizeof(AABB3); break;
        case CBT_SHAPE_TYPE_CAPSULE: size = sizeof(btCapsuleShape) + sizeof(AABB3); break;
        case CBT_SHAPE_TYPE_CONE: size = sizeof(btConeShape) + sizeof(AABB3); break;
        case CBT_SHAPE_TYPE_CYLINDER: size = sizeof(btCylinderShape) + sizeof(AABB3); break;
        case CBT_SHAPE_TYPE_COMPOUND: size = sizeof(btCompoundShape) + sizeof(AABB3); break;
        case CBT_SHAPE_TYPE_TRIANGLE_MESH: size = sizeof(btBvhTriangleMeshShape) + sizeof(btTriangleIndexVertexArray) + sizeof(AABB3); break;
        case CBT_SHAPE_TYPE_CONVEX: size = sizeof(btConvexHullShape) + sizeof(AABB3); break;
        default:
            assert(0);
    }
    auto shape = (int*)btAlignedAlloc(size, 16);

    // Set vtable to 0, this means that object is not created.
    shape[0] = 0;
    shape[1] = 0;
    // btCollisionShape::m_shapeType field is just after vtable (offset: 8).
    shape[2] = shape_type;

    if (shape_type == CBT_SHAPE_TYPE_TRIANGLE_MESH) {
        ((uint64_t*)((uint8_t*)shape + sizeof(btBvhTriangleMeshShape)))[0] = 0;
    }

    return (CbtShapeHandle)shape;
}

void cbtShapeDeallocate(CbtShapeHandle shape_handle) {
    assert(shape_handle && !cbtShapeIsCreated(shape_handle));
    btAlignedFree(shape_handle);
}

void cbtShapeDestroy(CbtShapeHandle shape_handle) {
    assert(shape_handle && cbtShapeIsCreated(shape_handle));
    assert(cbtShapeGetType(shape_handle) != CBT_SHAPE_TYPE_TRIANGLE_MESH);
    auto shape = (btCollisionShape*)shape_handle;
    shape->~btCollisionShape();
    // Set vtable to 0, this means that object is not created.
    ((uint64_t*)shape_handle)[0] = 0;
}

bool cbtShapeIsCreated(CbtShapeHandle shape_handle) {
    assert(shape_handle);
    // vtable == 0 means that object is not created.
    return ((uint64_t*)shape_handle)[0] != 0;
}

int cbtShapeGetType(CbtShapeHandle shape_handle) {
    assert(shape_handle);
    auto shape = (int*)shape_handle;
    // btCollisionShape::m_shapeType field is just after vtable (offset: 8).
    // This function works even for not yet created shapes (cbtShapeAllocate sets the type).
    return shape[2];
}

void cbtShapeSetMargin(CbtShapeHandle shape_handle, float margin) {
    assert(shape_handle && cbtShapeIsCreated(shape_handle));
    auto shape = (btCollisionShape*)shape_handle;
    shape->setMargin(margin);
}

float cbtShapeGetMargin(CbtShapeHandle shape_handle) {
    assert(shape_handle && cbtShapeIsCreated(shape_handle));
    auto shape = (btCollisionShape*)shape_handle;
    return shape->getMargin();
}

void cbtShapeGetAABB(CbtShapeHandle shape_handle, AABB3 aabb) {
    assert(shape_handle && cbtShapeIsCreated(shape_handle));

    int shape_type = cbtShapeGetType(shape_handle);

    size_t offset = 0;
    switch (shape_type) {
        case CBT_SHAPE_TYPE_BOX: offset = sizeof(btBoxShape); break;
        case CBT_SHAPE_TYPE_SPHERE: offset = sizeof(btSphereShape); break;
        case CBT_SHAPE_TYPE_CAPSULE: offset = sizeof(btCapsuleShape); break;
        case CBT_SHAPE_TYPE_CONE: offset = sizeof(btConeShape); break;
        case CBT_SHAPE_TYPE_CYLINDER: offset = sizeof(btCylinderShape); break;
        case CBT_SHAPE_TYPE_COMPOUND: offset = sizeof(btCompoundShape); break;
        case CBT_SHAPE_TYPE_TRIANGLE_MESH: offset = sizeof(btBvhTriangleMeshShape) + sizeof(btTriangleIndexVertexArray); break;
        case CBT_SHAPE_TYPE_CONVEX: offset = sizeof(btConvexHullShape); break;
        default:
            assert(0);
    }
    
    auto shape_aabb = *((AABB3*)((uint8_t*)shape_handle + offset));

    aabb[0] = shape_aabb[0];
    aabb[1] = shape_aabb[1];
    aabb[2] = shape_aabb[2];
    aabb[3] = shape_aabb[3];
    aabb[4] = shape_aabb[4];
    aabb[5] = shape_aabb[5];
}

void cbtShapeBoxCreate(CbtShapeHandle shape_handle, const Vector3 half_extents) {
    assert(shape_handle && !cbtShapeIsCreated(shape_handle));
    assert(cbtShapeGetType(shape_handle) == CBT_SHAPE_TYPE_BOX);
    assert(half_extents && half_extents[0] > 0.0 && half_extents[1] > 0.0 && half_extents[2] > 0.0);
    new (shape_handle) btBoxShape(btVector3(half_extents[0], half_extents[1], half_extents[2]));

    cbtCalculateAABB(shape_handle);
}

void cbtShapeBoxGetHalfExtentsWithoutMargin(CbtShapeHandle shape_handle, Vector3 half_extents) {
    assert(shape_handle && cbtShapeIsCreated(shape_handle) && half_extents);
    assert(cbtShapeGetType(shape_handle) == CBT_SHAPE_TYPE_BOX);
    auto shape = (btBoxShape*)shape_handle;
    auto tmp = shape->getHalfExtentsWithoutMargin();
    half_extents[0] = tmp.x();
    half_extents[1] = tmp.y();
    half_extents[2] = tmp.z();
}

void cbtShapeBoxGetHalfExtentsWithMargin(CbtShapeHandle shape_handle, Vector3 half_extents) {
    assert(shape_handle && cbtShapeIsCreated(shape_handle) && half_extents);
    assert(cbtShapeGetType(shape_handle) == CBT_SHAPE_TYPE_BOX);
    auto shape = (btBoxShape*)shape_handle;
    auto tmp = shape->getHalfExtentsWithMargin();
    half_extents[0] = tmp.x();
    half_extents[1] = tmp.y();
    half_extents[2] = tmp.z();
}

void cbtShapeSphereCreate(CbtShapeHandle shape_handle, float radius) {
    assert(shape_handle && !cbtShapeIsCreated(shape_handle));
    assert(cbtShapeGetType(shape_handle) == CBT_SHAPE_TYPE_SPHERE);
    assert(radius > 0.0);
    new (shape_handle) btSphereShape(radius);

    cbtCalculateAABB(shape_handle);
}

void cbtShapeSphereSetUnscaledRadius(CbtShapeHandle shape_handle, float radius) {
    assert(shape_handle && cbtShapeIsCreated(shape_handle));
    assert(cbtShapeGetType(shape_handle) == CBT_SHAPE_TYPE_SPHERE);
    assert(radius > 0.0);
    auto shape = (btSphereShape*)shape_handle;
    shape->setUnscaledRadius(radius);
}

float cbtShapeSphereGetRadius(CbtShapeHandle shape_handle) {
    assert(shape_handle && cbtShapeIsCreated(shape_handle));
    assert(cbtShapeGetType(shape_handle) == CBT_SHAPE_TYPE_SPHERE);
    auto shape = (btSphereShape*)shape_handle;
    return shape->getRadius();
}

void cbtShapeCapsuleCreate(CbtShapeHandle shape_handle, float radius, float height, int up_axis) {
    assert(shape_handle && !cbtShapeIsCreated(shape_handle));
    assert(cbtShapeGetType(shape_handle) == CBT_SHAPE_TYPE_CAPSULE);
    assert(radius > 0.0 && height > 0 && up_axis >= CBT_LINEAR_AXIS_X && up_axis <= CBT_LINEAR_AXIS_Z);

    if (up_axis == CBT_LINEAR_AXIS_X) {
        new (shape_handle) btCapsuleShapeX(radius, height);
    } else if (up_axis == CBT_LINEAR_AXIS_Y) {
        new (shape_handle) btCapsuleShape(radius, height);
    } else {
        new (shape_handle) btCapsuleShapeZ(radius, height);
    }

    cbtCalculateAABB(shape_handle);
}

int cbtShapeCapsuleGetUpAxis(CbtShapeHandle shape_handle) {
    assert(shape_handle && cbtShapeIsCreated(shape_handle));
    assert(cbtShapeGetType(shape_handle) == CBT_SHAPE_TYPE_CAPSULE);
    auto shape = (btCapsuleShape*)shape_handle;
    return shape->getUpAxis();
}

float cbtShapeCapsuleGetHalfHeight(CbtShapeHandle shape_handle) {
    assert(shape_handle && cbtShapeIsCreated(shape_handle));
    assert(cbtShapeGetType(shape_handle) == CBT_SHAPE_TYPE_CAPSULE);
    auto shape = (btCapsuleShape*)shape_handle;
    return shape->getHalfHeight();
}

float cbtShapeCapsuleGetRadius(CbtShapeHandle shape_handle) {
    assert(shape_handle && cbtShapeIsCreated(shape_handle));
    assert(cbtShapeGetType(shape_handle) == CBT_SHAPE_TYPE_CAPSULE);
    auto shape = (btCapsuleShape*)shape_handle;
    return shape->getRadius();
}

void cbtShapeCylinderCreate(CbtShapeHandle shape_handle, const Vector3 half_extents, int up_axis) {
    assert(shape_handle && !cbtShapeIsCreated(shape_handle));
    assert(cbtShapeGetType(shape_handle) == CBT_SHAPE_TYPE_CYLINDER);
    assert(half_extents && half_extents[0] > 0.0 && half_extents[1] > 0.0 && half_extents[2] > 0.0);
    assert(up_axis >= CBT_LINEAR_AXIS_X && up_axis <= CBT_LINEAR_AXIS_Z);

    if (up_axis == CBT_LINEAR_AXIS_X) {
        new (shape_handle) btCylinderShapeX(btVector3(half_extents[0], half_extents[1], half_extents[2]));
    } else if (up_axis == CBT_LINEAR_AXIS_Y) {
        new (shape_handle) btCylinderShape(btVector3(half_extents[0], half_extents[1], half_extents[2]));
    } else {
        new (shape_handle) btCylinderShapeZ(btVector3(half_extents[0], half_extents[1], half_extents[2]));
    }

    cbtCalculateAABB(shape_handle);
}

void cbtShapeCylinderGetHalfExtentsWithoutMargin(CbtShapeHandle shape_handle, Vector3 half_extents) {
    assert(shape_handle && cbtShapeIsCreated(shape_handle) && half_extents);
    assert(cbtShapeGetType(shape_handle) == CBT_SHAPE_TYPE_CYLINDER);
    auto shape = (btCylinderShape*)shape_handle;
    auto tmp = shape->getHalfExtentsWithoutMargin();
    half_extents[0] = tmp.x();
    half_extents[1] = tmp.y();
    half_extents[2] = tmp.z();
}

void cbtShapeCylinderGetHalfExtentsWithMargin(CbtShapeHandle shape_handle, Vector3 half_extents) {
    assert(shape_handle && cbtShapeIsCreated(shape_handle) && half_extents);
    assert(cbtShapeGetType(shape_handle) == CBT_SHAPE_TYPE_CYLINDER);
    auto shape = (btCylinderShape*)shape_handle;
    auto tmp = shape->getHalfExtentsWithMargin();
    half_extents[0] = tmp.x();
    half_extents[1] = tmp.y();
    half_extents[2] = tmp.z();
}

int cbtShapeCylinderGetUpAxis(CbtShapeHandle shape_handle) {
    assert(shape_handle && cbtShapeIsCreated(shape_handle));
    assert(cbtShapeGetType(shape_handle) == CBT_SHAPE_TYPE_CYLINDER);
    auto shape = (btCylinderShape*)shape_handle;
    return shape->getUpAxis();
}

void cbtShapeConeCreate(CbtShapeHandle shape_handle, float radius, float height, int up_axis) {
    assert(shape_handle && !cbtShapeIsCreated(shape_handle));
    assert(cbtShapeGetType(shape_handle) == CBT_SHAPE_TYPE_CONE);
    assert(radius > 0.0 && height > 0 && up_axis >= CBT_LINEAR_AXIS_X && up_axis <= CBT_LINEAR_AXIS_Z);

    if (up_axis == CBT_LINEAR_AXIS_X) {
        new (shape_handle) btConeShapeX(radius, height);
    } else if (up_axis == CBT_LINEAR_AXIS_Y) {
        new (shape_handle) btConeShape(radius, height);
    } else {
        new (shape_handle) btConeShapeZ(radius, height);
    }

    cbtCalculateAABB(shape_handle);
}

float cbtShapeConeGetRadius(CbtShapeHandle shape_handle) {
    assert(shape_handle && cbtShapeIsCreated(shape_handle));
    assert(cbtShapeGetType(shape_handle) == CBT_SHAPE_TYPE_CONE);
    auto shape = (btConeShape*)shape_handle;
    return shape->getRadius();
}

float cbtShapeConeGetHeight(CbtShapeHandle shape_handle) {
    assert(shape_handle && cbtShapeIsCreated(shape_handle));
    assert(cbtShapeGetType(shape_handle) == CBT_SHAPE_TYPE_CONE);
    auto shape = (btConeShape*)shape_handle;
    return shape->getHeight();
}

int cbtShapeConeGetUpAxis(CbtShapeHandle shape_handle) {
    assert(shape_handle && cbtShapeIsCreated(shape_handle));
    assert(cbtShapeGetType(shape_handle) == CBT_SHAPE_TYPE_CONE);
    auto shape = (btConeShape*)shape_handle;
    return shape->getConeUpIndex();
}

void cbtShapeConvexHullCreate(CbtShapeHandle shape_handle, const void* points, int numPoints) {
    assert(shape_handle && cbtShapeIsCreated(shape_handle));
    assert(cbtShapeGetType(shape_handle) == CBT_SHAPE_TYPE_CONVEX);
    assert(numPoints > 0);

    new (shape_handle) btConvexHullShape((const float*) points, numPoints, 12);

    cbtCalculateAABB(shape_handle);
}

void cbtShapeConvexHullOptimize(CbtShapeHandle shape_handle) {
    assert(shape_handle && cbtShapeIsCreated(shape_handle));
    assert(cbtShapeGetType(shape_handle) == CBT_SHAPE_TYPE_CONVEX);
    auto shape = (btConvexHullShape*) shape_handle;
    shape->optimizeConvexHull();
}

void cbtShapeConvexHullGetPoints(CbtShapeHandle shape_handle, void** points, int* numPoints) {
    assert(shape_handle && cbtShapeIsCreated(shape_handle));
    assert(cbtShapeGetType(shape_handle) == CBT_SHAPE_TYPE_CONVEX);
    auto shape = (btConvexHullShape*) shape_handle;
    *points = shape->getUnscaledPoints();
    *numPoints = shape->getNumPoints();
}

void cbtShapeCompoundCreate(
    CbtShapeHandle shape_handle,
    bool enable_dynamic_aabb_tree,
    int initial_child_capacity
) {
    assert(shape_handle && !cbtShapeIsCreated(shape_handle));
    assert(initial_child_capacity >= 0);
    assert(cbtShapeGetType(shape_handle) == CBT_SHAPE_TYPE_COMPOUND);
    new (shape_handle) btCompoundShape(enable_dynamic_aabb_tree, initial_child_capacity);

    cbtCalculateAABB(shape_handle);
}

static inline btTransform makeBtTransform(const Vector3 transform[4]) {
    // NOTE(mziulek): Bullet uses M * v order/convention so we need to transpose matrix.
    return btTransform(
        btMatrix3x3(
            btVector3(transform[0][0], transform[1][0], transform[2][0]),
            btVector3(transform[0][1], transform[1][1], transform[2][1]),
            btVector3(transform[0][2], transform[1][2], transform[2][2])
        ),
        btVector3(transform[3][0], transform[3][1], transform[3][2])
    );
}

void cbtShapeCompoundAddChild(
    CbtShapeHandle shape_handle,
    const Vector3 local_transform[4],
    CbtShapeHandle child_shape_handle
) {
    assert(shape_handle && cbtShapeIsCreated(shape_handle));
    assert(child_shape_handle && cbtShapeIsCreated(child_shape_handle));
    assert(local_transform);
    assert(cbtShapeGetType(shape_handle) == CBT_SHAPE_TYPE_COMPOUND);

    auto parent = (btCompoundShape*)shape_handle;
    auto child = (btCollisionShape*)child_shape_handle;

    parent->addChildShape(makeBtTransform(local_transform), child);
}

void cbtShapeCompoundRemoveChild(CbtShapeHandle shape_handle, CbtShapeHandle child_shape_handle) {
    assert(shape_handle && cbtShapeIsCreated(shape_handle));
    assert(child_shape_handle && cbtShapeIsCreated(child_shape_handle));
    assert(cbtShapeGetType(shape_handle) == CBT_SHAPE_TYPE_COMPOUND);
    auto parent = (btCompoundShape*)shape_handle;
    auto child = (btCollisionShape*)child_shape_handle;
    parent->removeChildShape(child);
}

void cbtShapeCompoundRemoveChildByIndex(CbtShapeHandle shape_handle, int child_shape_index) {
    assert(shape_handle && cbtShapeIsCreated(shape_handle));
    assert(cbtShapeGetType(shape_handle) == CBT_SHAPE_TYPE_COMPOUND);
    auto shape = (btCompoundShape*)shape_handle;
    shape->removeChildShapeByIndex(child_shape_index);
}

int cbtShapeCompoundGetNumChilds(CbtShapeHandle shape_handle) {
    assert(shape_handle && cbtShapeIsCreated(shape_handle));
    assert(cbtShapeGetType(shape_handle) == CBT_SHAPE_TYPE_COMPOUND);
    auto shape = (btCompoundShape*)shape_handle;
    return shape->getNumChildShapes();
}

CbtShapeHandle cbtShapeCompoundGetChild(CbtShapeHandle shape_handle, int child_shape_index) {
    assert(shape_handle && cbtShapeIsCreated(shape_handle));
    assert(cbtShapeGetType(shape_handle) == CBT_SHAPE_TYPE_COMPOUND);
    auto shape = (btCompoundShape*)shape_handle;
    return (CbtShapeHandle)shape->getChildShape(child_shape_index);
}

void cbtShapeCompoundGetChildTransform(
    CbtShapeHandle shape_handle,
    int child_shape_index,
    Vector3 transform[4]
) {
    assert(shape_handle && cbtShapeIsCreated(shape_handle));
    assert(transform && child_shape_index >= 0);
    auto shape = (btCompoundShape*)shape_handle;

    const btTransform& trans = shape->getChildTransform(child_shape_index);
    const btMatrix3x3& basis = trans.getBasis();
    const btVector3& origin = trans.getOrigin();

    // NOTE(mziulek): We transpose Bullet matrix here to make it compatible with: v * M convention.
    transform[0][0] = basis.getRow(0).x();
    transform[1][0] = basis.getRow(0).y();
    transform[2][0] = basis.getRow(0).z();
    transform[0][1] = basis.getRow(1).x();
    transform[1][1] = basis.getRow(1).y();
    transform[2][1] = basis.getRow(1).z();
    transform[0][2] = basis.getRow(2).x();
    transform[1][2] = basis.getRow(2).y();
    transform[2][2] = basis.getRow(2).z();

    transform[3][0] = origin.x();
    transform[3][1] = origin.y();
    transform[3][2] = origin.z();
}

static_assert((sizeof(btBvhTriangleMeshShape) % 8) == 0, "sizeof(btBvhTriangleMeshShape) is not multiple of 8");
static_assert(
    (sizeof(btTriangleIndexVertexArray) % 8) == 0,
    "sizeof(btTriangleIndexVertexArray) is not multiple of 8"
);
static_assert(
    ((sizeof(btBvhTriangleMeshShape) + sizeof(btTriangleIndexVertexArray)) % 8) == 0,
    "sizeof(btBvhTriangleMeshShape) + sizeof(btTriangleIndexVertexArray) is not multiple of 8"
);

void cbtShapeTriMeshCreateBegin(CbtShapeHandle shape_handle) {
    assert(shape_handle && !cbtShapeIsCreated(shape_handle));
    assert(cbtShapeGetType(shape_handle) == CBT_SHAPE_TYPE_TRIANGLE_MESH);
    assert(((uint64_t*)((uint8_t*)shape_handle + sizeof(btBvhTriangleMeshShape)))[0] == 0);

    void* mesh_interface_mem = (void*)((uint8_t*)shape_handle + sizeof(btBvhTriangleMeshShape));
    new (mesh_interface_mem) btTriangleIndexVertexArray();
}

void cbtShapeTriMeshCreateEnd(CbtShapeHandle shape_handle) {
    assert(shape_handle && !cbtShapeIsCreated(shape_handle));
    assert(cbtShapeGetType(shape_handle) == CBT_SHAPE_TYPE_TRIANGLE_MESH);
    assert(((uint64_t*)((uint8_t*)shape_handle + sizeof(btBvhTriangleMeshShape)))[0] != 0);

    auto mesh_interface = (btTriangleIndexVertexArray*)((uint8_t*)shape_handle + sizeof(btBvhTriangleMeshShape));
    assert(mesh_interface->getNumSubParts() > 0);

    new (shape_handle) btBvhTriangleMeshShape(mesh_interface, false, true);

    cbtCalculateAABB(shape_handle);
}

void cbtShapeTriMeshDestroy(CbtShapeHandle shape_handle) {
    assert(shape_handle && cbtShapeIsCreated(shape_handle));
    assert(cbtShapeGetType(shape_handle) == CBT_SHAPE_TYPE_TRIANGLE_MESH);

    auto shape = (btBvhTriangleMeshShape*)shape_handle;
    auto mesh_interface = (btTriangleIndexVertexArray*)((uint8_t*)shape_handle + sizeof(btBvhTriangleMeshShape));

    const IndexedMeshArray& arr = mesh_interface->getIndexedMeshArray();
    for (size_t i = 0; i < arr.size(); ++i) {
        // We keep triangles and vertices in one memory buffer, so only one deallocation is needed.
        btAlignedFree((void*)arr[i].m_triangleIndexBase);
    }

    mesh_interface->~btTriangleIndexVertexArray();
    shape->~btBvhTriangleMeshShape();

    // Set vtable to 0, this means that object is not created.
    ((uint64_t*)shape_handle)[0] = 0;
    ((uint64_t*)((uint8_t*)shape_handle + sizeof(btBvhTriangleMeshShape)))[0] = 0;
}

void cbtShapeTriMeshAddIndexVertexArray(
    CbtShapeHandle shape_handle,
    int num_triangles,
    const void* triangles_base,
    int triangle_stride,
    int num_vertices,
    const void* vertices_base,
    int vertex_stride
) {
    assert(shape_handle && !cbtShapeIsCreated(shape_handle));
    assert(cbtShapeGetType(shape_handle) == CBT_SHAPE_TYPE_TRIANGLE_MESH);
    assert(((uint64_t*)((uint8_t*)shape_handle + sizeof(btBvhTriangleMeshShape)))[0] != 0);
    assert(num_triangles > 0 && num_vertices > 0);
    assert(triangles_base != nullptr && vertices_base != nullptr);
    assert(vertex_stride >= 12);
    // NOTE(mziulek): We currently require triangles to be tightly packed.
    assert(triangle_stride == 3 || triangle_stride == 6 || triangle_stride == 12);

    assert(shape_handle && !cbtShapeIsCreated(shape_handle));
    assert(cbtShapeGetType(shape_handle) == CBT_SHAPE_TYPE_TRIANGLE_MESH);

    const size_t vertices_size = num_vertices * 3 * sizeof(float);
    const size_t indices_size = num_triangles * triangle_stride;
    const size_t vertices_size_aligned = (vertices_size + 15) & ~15;
    const size_t indices_size_aligned = (indices_size + 15) & ~15;

    auto mem = (uint8_t*)btAlignedAlloc(vertices_size_aligned + indices_size_aligned, 16);

    btIndexedMesh indexed_mesh;
    indexed_mesh.m_numTriangles = num_triangles;
    indexed_mesh.m_triangleIndexBase = mem;
    indexed_mesh.m_triangleIndexStride = triangle_stride;
    indexed_mesh.m_numVertices = num_vertices;
    indexed_mesh.m_vertexBase = mem + indices_size_aligned;
    indexed_mesh.m_vertexStride = 3 * sizeof(float);

    if (vertex_stride == 3 * sizeof(float)) {
        memcpy((void*)indexed_mesh.m_vertexBase, vertices_base, vertices_size);
    } else {
        for (int i = 0; i < num_vertices; ++i) {
            const float* src_vertex = (const float*)((uint8_t*)vertices_base + i * vertex_stride);
            float* dst_vertex = (float*)((uint8_t*)indexed_mesh.m_vertexBase + i * 3 * sizeof(float));
            dst_vertex[0] = src_vertex[0];
            dst_vertex[1] = src_vertex[1];
            dst_vertex[2] = src_vertex[2];
        }
    }
    memcpy((void*)indexed_mesh.m_triangleIndexBase, triangles_base, indices_size);

    auto mesh_interface = (btTriangleIndexVertexArray*)((uint8_t*)shape_handle + sizeof(btBvhTriangleMeshShape));
    mesh_interface->addIndexedMesh(
        indexed_mesh,
        triangle_stride == 12 ? PHY_INTEGER : (triangle_stride == 6 ? PHY_SHORT : PHY_UCHAR)
    );
}

bool cbtShapeIsPolyhedral(CbtShapeHandle shape_handle) {
    assert(shape_handle && cbtShapeIsCreated(shape_handle));
    auto shape = (btCollisionShape*)shape_handle;
    return shape->isPolyhedral();
}

bool cbtShapeIsConvex2d(CbtShapeHandle shape_handle) {
    assert(shape_handle && cbtShapeIsCreated(shape_handle));
    auto shape = (btCollisionShape*)shape_handle;
    return shape->isConvex2d();
}

bool cbtShapeIsConvex(CbtShapeHandle shape_handle) {
    assert(shape_handle && cbtShapeIsCreated(shape_handle));
    auto shape = (btCollisionShape*)shape_handle;
    return shape->isConvex();
}

bool cbtShapeIsNonMoving(CbtShapeHandle shape_handle) {
    assert(shape_handle && cbtShapeIsCreated(shape_handle));
    auto shape = (btCollisionShape*)shape_handle;
    return shape->isNonMoving();
}

bool cbtShapeIsConcave(CbtShapeHandle shape_handle) {
    assert(shape_handle && cbtShapeIsCreated(shape_handle));
    auto shape = (btCollisionShape*)shape_handle;
    return shape->isConcave();
}

bool cbtShapeIsCompound(CbtShapeHandle shape_handle) {
    assert(shape_handle && cbtShapeIsCreated(shape_handle));
    auto shape = (btCollisionShape*)shape_handle;
    return shape->isCompound();
}

void cbtShapeCalculateLocalInertia(CbtShapeHandle shape_handle, float mass, Vector3 inertia) {
    assert(shape_handle && cbtShapeIsCreated(shape_handle));
    assert(mass > 0.0);
    auto shape = (btCollisionShape*)shape_handle;
    btVector3 ine;
    shape->calculateLocalInertia(mass, ine);
    inertia[0] = ine.x();
    inertia[1] = ine.y();
    inertia[2] = ine.z();
}

void cbtShapeSetUserPointer(CbtShapeHandle shape_handle, void* user_pointer) {
    assert(shape_handle && cbtShapeIsCreated(shape_handle));
    auto shape = (btCollisionShape*)shape_handle;
    shape->setUserPointer(user_pointer);
}

void* cbtShapeGetUserPointer(CbtShapeHandle shape_handle) {
    assert(shape_handle && cbtShapeIsCreated(shape_handle));
    auto shape = (btCollisionShape*)shape_handle;
    return shape->getUserPointer();
}

void cbtShapeSetUserIndex(CbtShapeHandle shape_handle, int slot, int user_index) {
    assert(shape_handle && cbtShapeIsCreated(shape_handle));
    assert(slot == 0 || slot == 1);
    auto shape = (btCollisionShape*)shape_handle;
    if (slot == 0) {
        shape->setUserIndex(user_index);
    } else {
        shape->setUserIndex2(user_index);
    }
}

int cbtShapeGetUserIndex(CbtShapeHandle shape_handle, int slot) {
    assert(shape_handle && cbtShapeIsCreated(shape_handle));
    assert(slot == 0 || slot == 1);
    auto shape = (btCollisionShape*)shape_handle;
    if (slot == 0) {
        return shape->getUserIndex();
    }
    return shape->getUserIndex2();
}

static_assert((sizeof(btRigidBody) % 8) == 0, "sizeof(btRigidBody) is not multiple of 8");
static_assert((sizeof(btDefaultMotionState) % 8) == 0, "sizeof(btDefaultMotionState) is not multiple of 8");
static_assert(
    ((sizeof(btRigidBody) + sizeof(btDefaultMotionState)) % 8) == 0,
    "sizeof(btRigidBody) + sizeof(btDefaultMotionState) is not multiple of 8"
);

CbtBodyHandle cbtBodyAllocate(void) {
    auto base = (uint64_t*)btAlignedAlloc(sizeof(btRigidBody) + sizeof(btDefaultMotionState), 16);
    // Set vtable to 0. This means that body is not created.
    base[0] = 0;
    return (CbtBodyHandle)base;
}

void cbtBodyDeallocate(CbtBodyHandle body_handle) {
    assert(body_handle && !cbtBodyIsCreated(body_handle));
    btAlignedFree(body_handle);
}

void cbtBodyAllocateBatch(unsigned int num, CbtBodyHandle* body_handles) {
    assert(num > 0 && body_handles);
    const size_t element_size = sizeof(btRigidBody) + sizeof(btDefaultMotionState);
    uint8_t* base = (uint8_t*)btAlignedAlloc(num * element_size, 16);
    for (unsigned int i = 0; i < num; ++i) {
        body_handles[i] = (CbtBodyHandle)(base + i * element_size);
        // Set vtable to 0. This means that body is not created.
        ((uint64_t*)body_handles[i])[0] = 0;
    }
}

void cbtBodyDeallocateBatch(unsigned int num, CbtBodyHandle* body_handles) {
    assert(num > 0 && body_handles);
#ifdef _DEBUG
    for (unsigned int i = 0; i < num; ++i) {
        assert(!cbtBodyIsCreated(body_handles[i]));
    }
#endif
    // NOTE(mziulek): All handles must come from a single 'batch'.
    btAlignedFree(body_handles[0]);
}

void cbtBodyCreate(
    CbtBodyHandle body_handle,
    float mass,
    const Vector3 transform[4],
    CbtShapeHandle shape_handle
) {
    assert(body_handle && shape_handle);
    assert(transform && mass >= 0.0);
    assert(!cbtBodyIsCreated(body_handle));
    assert(cbtShapeIsCreated(shape_handle));

    auto shape = (btCollisionShape*)shape_handle;

    void* body_mem = (void*)body_handle;
    void* motion_state_mem = (void*)((uint8_t*)body_handle + sizeof(btRigidBody));

    const bool is_dynamic = (mass != 0.0);

    btVector3 local_inertia(0.0, 0.0, 0.0);
    if (is_dynamic) {
        shape->calculateLocalInertia(mass, local_inertia);
    }

    btDefaultMotionState* motion_state = new (motion_state_mem) btDefaultMotionState(makeBtTransform(transform));
    btRigidBody::btRigidBodyConstructionInfo info(mass, motion_state, shape, local_inertia);
    new (body_mem) btRigidBody(info);
}

void cbtBodyDestroy(CbtBodyHandle body_handle) {
    assert(body_handle && cbtBodyIsCreated(body_handle));

    auto body = (btRigidBody*)body_handle;
    auto motion_state = (btDefaultMotionState*)((uint8_t*)body_handle + sizeof(btRigidBody));

    motion_state->~btDefaultMotionState();
    body->~btRigidBody();

    // Set vtable to 0, this means that object is not created.
    ((uint64_t*)body)[0] = 0;
}

bool cbtBodyIsCreated(CbtBodyHandle body_handle) {
    assert(body_handle);
    // vtable == 0 means that object is not created.
    return ((uint64_t*)body_handle)[0] != 0;
}

void cbtBodySetShape(CbtBodyHandle body_handle, CbtShapeHandle shape_handle) {
    assert(body_handle && shape_handle);
    assert(cbtBodyIsCreated(body_handle) && cbtShapeIsCreated(shape_handle));
    auto body = (btRigidBody*)body_handle;
    auto shape = (btCollisionShape*)shape_handle;
    body->setCollisionShape(shape);
}

CbtShapeHandle cbtBodyGetShape(CbtBodyHandle body_handle) {
    assert(body_handle && cbtBodyIsCreated(body_handle));
    auto body = (btRigidBody*)body_handle;
    return (CbtShapeHandle)body->getCollisionShape();
}

void cbtBodySetRestitution(CbtBodyHandle body_handle, float restitution) {
    assert(body_handle && cbtBodyIsCreated(body_handle));
    auto body = (btRigidBody*)body_handle;
    body->setRestitution(restitution);
}

void cbtBodySetFriction(CbtBodyHandle body_handle, float friction) {
    assert(body_handle && cbtBodyIsCreated(body_handle));
    auto body = (btRigidBody*)body_handle;
    body->setFriction(friction * accelConv);
}

void cbtBodySetRollingFriction(CbtBodyHandle body_handle, float friction) {
    assert(body_handle && cbtBodyIsCreated(body_handle));
    auto body = (btRigidBody*)body_handle;
    body->setRollingFriction(friction * accelConv);
}

void cbtBodySetSpinningFriction(CbtBodyHandle body_handle, float friction) {
    assert(body_handle && cbtBodyIsCreated(body_handle));
    auto body = (btRigidBody*)body_handle;
    body->setSpinningFriction(friction * accelConv);
}

void cbtBodySetAnisotropicFriction(CbtBodyHandle body_handle, const Vector3 friction, int mode) {
    assert(body_handle && cbtBodyIsCreated(body_handle));
    assert(friction);
    auto body = (btRigidBody*)body_handle;
    body->setAnisotropicFriction(btVector3(friction[0] * accelConv, friction[1] * accelConv, friction[2] * accelConv), mode);
}

void cbtBodySetContactStiffnessAndDamping(CbtBodyHandle body_handle, float stiffness, float damping) {
    assert(body_handle && cbtBodyIsCreated(body_handle));
    auto body = (btRigidBody*)body_handle;
    body->setContactStiffnessAndDamping(stiffness, damping);
}

void cbtBodySetMassProps(CbtBodyHandle body_handle, float mass, const Vector3 inertia) {
    assert(body_handle && cbtBodyIsCreated(body_handle));
    assert(inertia);
    auto body = (btRigidBody*)body_handle;
    body->setMassProps(mass, btVector3(inertia[0], inertia[1], inertia[2]));
}

void cbtBodySetDamping(CbtBodyHandle body_handle, float linear, float angular) {
    assert(body_handle && cbtBodyIsCreated(body_handle));
    auto body = (btRigidBody*)body_handle;
    body->setDamping(linear, angular);
}

void cbtBodySetLinearVelocity(CbtBodyHandle body_handle, const Vector3 velocity) {
    assert(body_handle && cbtBodyIsCreated(body_handle));
    assert(velocity);
    auto body = (btRigidBody*)body_handle;
    body->setLinearVelocity(btVector3(velocity[0] * velConv, velocity[1] * velConv, velocity[2] * velConv));
}

void cbtBodySetAngularVelocity(CbtBodyHandle body_handle, const Vector3 velocity) {
    assert(body_handle && cbtBodyIsCreated(body_handle));
    assert(velocity);
    auto body = (btRigidBody*)body_handle;
    body->setAngularVelocity(btVector3(velocity[0] * velConv, velocity[1] * velConv, velocity[2] * velConv));
}

void cbtBodySetLinearFactor(CbtBodyHandle body_handle, const Vector3 factor) {
    assert(body_handle && cbtBodyIsCreated(body_handle));
    assert(factor);
    auto body = (btRigidBody*)body_handle;
    body->setLinearFactor(btVector3(factor[0], factor[1], factor[2]));
}

void cbtBodySetAngularFactor(CbtBodyHandle body_handle, const Vector3 factor) {
    assert(body_handle && cbtBodyIsCreated(body_handle));
    assert(factor);
    auto body = (btRigidBody*)body_handle;
    body->setAngularFactor(btVector3(factor[0], factor[1], factor[2]));
}

void cbtBodySetGravity(CbtBodyHandle body_handle, const Vector3 gravity) {
    assert(body_handle && cbtBodyIsCreated(body_handle));
    assert(gravity);
    auto body = (btRigidBody*)body_handle;
    body->setGravity(btVector3(gravity[0] * accelConv, gravity[1] * accelConv, gravity[2] * accelConv));
}

void cbtBodyGetGravity(CbtBodyHandle body_handle, Vector3 gravity) {
    assert(body_handle && cbtBodyIsCreated(body_handle));
    assert(gravity);
    auto body = (btRigidBody*)body_handle;
    auto tmp = body->getGravity();
    gravity[0] = tmp.x() / accelConv;
    gravity[1] = tmp.y() / accelConv;
    gravity[2] = tmp.z() / accelConv;
}

int cbtBodyGetNumConstraints(CbtBodyHandle body_handle) {
    assert(body_handle && cbtBodyIsCreated(body_handle));
    auto body = (btRigidBody*)body_handle;
    return body->getNumConstraintRefs();
}

CbtConstraintHandle cbtBodyGetConstraint(CbtBodyHandle body_handle, int index) {
    assert(body_handle && cbtBodyIsCreated(body_handle));
    auto body = (btRigidBody*)body_handle;
    return (CbtConstraintHandle)body->getConstraintRef(index);
}

void cbtBodyApplyCentralForce(CbtBodyHandle body_handle, const Vector3 force) {
    assert(body_handle && cbtBodyIsCreated(body_handle));
    assert(force);
    auto body = (btRigidBody*)body_handle;
    body->applyCentralForce(btVector3(force[0] * accelConv, force[1] * accelConv, force[2] * accelConv));
}

void cbtBodyApplyCentralImpulse(CbtBodyHandle body_handle, const Vector3 impulse) {
    assert(body_handle && cbtBodyIsCreated(body_handle));
    assert(impulse);
    auto body = (btRigidBody*)body_handle;
    body->applyCentralImpulse(btVector3(impulse[0] * velConv, impulse[1] * velConv, impulse[2] * velConv));
}

void cbtBodyApplyForce(CbtBodyHandle body_handle, const Vector3 force, const Vector3 rel_pos) {
    assert(body_handle && cbtBodyIsCreated(body_handle));
    assert(force && rel_pos);
    auto body = (btRigidBody*)body_handle;
    body->applyForce(btVector3(force[0] * accelConv, force[1] * accelConv, force[2] * accelConv), btVector3(rel_pos[0], rel_pos[1], rel_pos[2]));
}

void cbtBodyClearForces(CbtBodyHandle body_handle) {
    assert(body_handle && cbtBodyIsCreated(body_handle));
    auto body = (btRigidBody*)body_handle;
    body->clearForces();
}

void cbtBodyApplyImpulse(CbtBodyHandle body_handle, const Vector3 impulse, const Vector3 rel_pos) {
    assert(body_handle && cbtBodyIsCreated(body_handle));
    assert(impulse && rel_pos);
    auto body = (btRigidBody*)body_handle;
    body->applyImpulse(
        btVector3(impulse[0] * velConv, impulse[1] * velConv, impulse[2] * velConv),
        btVector3(rel_pos[0], rel_pos[1], rel_pos[2])
    );
}

void cbtBodyApplyTorque(CbtBodyHandle body_handle, const Vector3 torque) {
    assert(body_handle && cbtBodyIsCreated(body_handle));
    assert(torque);
    auto body = (btRigidBody*)body_handle;
    body->applyTorque(btVector3(torque[0] * accelConv, torque[1] * accelConv, torque[2] * accelConv));
}

void cbtBodyApplyTorqueImpulse(CbtBodyHandle body_handle, const Vector3 impulse) {
    assert(body_handle && cbtBodyIsCreated(body_handle));
    assert(impulse);
    auto body = (btRigidBody*)body_handle;
    body->applyTorqueImpulse(btVector3(impulse[0] * velConv, impulse[1] * velConv, impulse[2] * velConv));
}

float cbtBodyGetRestitution(CbtBodyHandle body_handle) {
    assert(body_handle && cbtBodyIsCreated(body_handle));
    auto body = (btRigidBody*)body_handle;
    return body->getRestitution();
}

float cbtBodyGetFriction(CbtBodyHandle body_handle) {
    assert(body_handle && cbtBodyIsCreated(body_handle));
    auto body = (btRigidBody*)body_handle;
    return body->getFriction();
}

float cbtBodyGetRollingFriction(CbtBodyHandle body_handle) {
    assert(body_handle && cbtBodyIsCreated(body_handle));
    auto body = (btRigidBody*)body_handle;
    return body->getRollingFriction();
}

float cbtBodyGetSpinningFriction(CbtBodyHandle body_handle) {
    assert(body_handle && cbtBodyIsCreated(body_handle));
    auto body = (btRigidBody*)body_handle;
    return body->getSpinningFriction();
}

void cbtBodyGetAnisotropicFriction(CbtBodyHandle body_handle, Vector3 friction) {
    assert(body_handle && cbtBodyIsCreated(body_handle));
    assert(friction);
    auto body = (btRigidBody*)body_handle;
    const btVector3& f = body->getAnisotropicFriction();
    friction[0] = f.x() / accelConv;
    friction[1] = f.y() / accelConv;
    friction[2] = f.z() / accelConv;
}

float cbtBodyGetContactStiffness(CbtBodyHandle body_handle) {
    assert(body_handle && cbtBodyIsCreated(body_handle));
    auto body = (btRigidBody*)body_handle;
    return body->getContactStiffness();
}

float cbtBodyGetContactDamping(CbtBodyHandle body_handle) {
    assert(body_handle && cbtBodyIsCreated(body_handle));
    auto body = (btRigidBody*)body_handle;
    return body->getContactDamping();
}

float cbtBodyGetMass(CbtBodyHandle body_handle) {
    assert(body_handle && cbtBodyIsCreated(body_handle));
    auto body = (btRigidBody*)body_handle;
    return body->getMass();
}

float cbtBodyGetLinearDamping(CbtBodyHandle body_handle) {
    assert(body_handle && cbtBodyIsCreated(body_handle));
    auto body = (btRigidBody*)body_handle;
    return body->getLinearDamping();
}

float cbtBodyGetAngularDamping(CbtBodyHandle body_handle) {
    assert(body_handle && cbtBodyIsCreated(body_handle));
    auto body = (btRigidBody*)body_handle;
    return body->getAngularDamping();
}

void cbtBodyGetLinearVelocity(CbtBodyHandle body_handle, Vector3 velocity) {
    assert(body_handle && cbtBodyIsCreated(body_handle));
    assert(velocity);
    auto body = (btRigidBody*)body_handle;
    const btVector3& vel = body->getLinearVelocity();
    velocity[0] = vel.x() / velConv;
    velocity[1] = vel.y() / velConv;
    velocity[2] = vel.z() / velConv;
}

void cbtBodyGetAngularVelocity(CbtBodyHandle body_handle, Vector3 velocity) {
    assert(body_handle && cbtBodyIsCreated(body_handle));
    assert(velocity);
    auto body = (btRigidBody*)body_handle;
    const btVector3& vel = body->getAngularVelocity();
    velocity[0] = vel.x() / velConv;
    velocity[1] = vel.y() / velConv;
    velocity[2] = vel.z() / velConv;
}

void cbtBodyGetTotalForce(CbtBodyHandle body_handle, Vector3 force) {
    assert(body_handle && cbtBodyIsCreated(body_handle));
    assert(force);
    auto body = (btRigidBody*)body_handle;
    auto tmp = body->getTotalForce();
    force[0] = tmp.x() / accelConv;
    force[1] = tmp.y() / accelConv;
    force[2] = tmp.z() / accelConv;
}

void cbtBodyGetTotalTorque(CbtBodyHandle body_handle, Vector3 torque) {
    assert(body_handle && cbtBodyIsCreated(body_handle));
    assert(torque);
    auto body = (btRigidBody*)body_handle;
    auto tmp = body->getTotalTorque();
    torque[0] = tmp.x() / accelConv;
    torque[1] = tmp.y() / accelConv;
    torque[2] = tmp.z() / accelConv;
}

bool cbtBodyIsStatic(CbtBodyHandle body_handle) {
    assert(body_handle && cbtBodyIsCreated(body_handle));
    auto body = (btRigidBody*)body_handle;
    return body->isStaticObject();
}

bool cbtBodyIsKinematic(CbtBodyHandle body_handle) {
    assert(body_handle && cbtBodyIsCreated(body_handle));
    auto body = (btRigidBody*)body_handle;
    return body->isKinematicObject();
}

bool cbtBodyIsStaticOrKinematic(CbtBodyHandle body_handle) {
    assert(body_handle && cbtBodyIsCreated(body_handle));
    auto body = (btRigidBody*)body_handle;
    return body->isStaticOrKinematicObject();
}

float cbtBodyGetDeactivationTime(CbtBodyHandle body_handle) {
    assert(body_handle && cbtBodyIsCreated(body_handle));
    assert(body_handle);
    auto body = (btRigidBody*)body_handle;
    return body->getDeactivationTime() / timeConv;
}

void cbtBodySetDeactivationTime(CbtBodyHandle body_handle, float time) {
    assert(body_handle && cbtBodyIsCreated(body_handle));
    auto body = (btRigidBody*)body_handle;
    return body->setDeactivationTime(time * timeConv);
}

int cbtBodyGetActivationState(CbtBodyHandle body_handle) {
    assert(body_handle && cbtBodyIsCreated(body_handle));
    auto body = (btRigidBody*)body_handle;
    return body->getActivationState();
}

void cbtBodySetActivationState(CbtBodyHandle body_handle, int state) {
    assert(body_handle && cbtBodyIsCreated(body_handle));
    auto body = (btRigidBody*)body_handle;
    return body->setActivationState(state);
}

void cbtBodyForceActivationState(CbtBodyHandle body_handle, int state) {
    assert(body_handle && cbtBodyIsCreated(body_handle));
    auto body = (btRigidBody*)body_handle;
    return body->forceActivationState(state);
}

bool cbtBodyIsActive(CbtBodyHandle body_handle) {
    assert(body_handle && cbtBodyIsCreated(body_handle));
    auto body = (btRigidBody*)body_handle;
    return body->isActive();
}

bool cbtBodyIsInWorld(CbtBodyHandle body_handle) {
    assert(body_handle && cbtBodyIsCreated(body_handle));
    auto body = (btRigidBody*)body_handle;
    return body->isInWorld();
}

void cbtBodySetUserPointer(CbtBodyHandle body_handle, void* user_pointer) {
    assert(body_handle && cbtBodyIsCreated(body_handle));
    auto body = (btRigidBody*)body_handle;
    body->setUserPointer(user_pointer);
}

void* cbtBodyGetUserPointer(CbtBodyHandle body_handle) {
    assert(body_handle && cbtBodyIsCreated(body_handle));
    auto body = (btRigidBody*)body_handle;
    return body->getUserPointer();
}

void cbtBodySetUserIndex(CbtBodyHandle body_handle, int slot, int user_index) {
    assert(body_handle && cbtBodyIsCreated(body_handle));
    assert(slot >= 0 && slot <= 2);
    auto body = (btRigidBody*)body_handle;
    if (slot == 0) {
        body->setUserIndex(user_index);
    } else if (slot == 1) {
        body->setUserIndex2(user_index);
    } else {
        body->setUserIndex3(user_index);
    }
}

int cbtBodyGetUserIndex(CbtBodyHandle body_handle, int slot) {
    assert(body_handle && cbtBodyIsCreated(body_handle));
    assert(slot >= 0 && slot <= 2);
    auto body = (btRigidBody*)body_handle;
    if (slot == 0) {
        return body->getUserIndex();
    }
    if (slot == 1) {
        return body->getUserIndex2();
    }
    return body->getUserIndex3();
}

void cbtBodySetCenterOfMassTransform(CbtBodyHandle body_handle, const Vector3 transform[4]) {
    assert(body_handle && cbtBodyIsCreated(body_handle));
    auto body = (btRigidBody*)body_handle;
    body->setCenterOfMassTransform(makeBtTransform(transform));
}

void cbtBodyGetCenterOfMassTransform(CbtBodyHandle body_handle, Vector3 transform[4]) {
    assert(body_handle && cbtBodyIsCreated(body_handle));
    assert(transform);
    auto body = (btRigidBody*)body_handle;

    const btTransform& trans = body->getCenterOfMassTransform();
    const btMatrix3x3& basis = trans.getBasis();
    const btVector3& origin = trans.getOrigin();

    // NOTE(mziulek): We transpose Bullet matrix here to make it compatible with: v * M convention.
    transform[0][0] = basis.getRow(0).x();
    transform[1][0] = basis.getRow(0).y();
    transform[2][0] = basis.getRow(0).z();
    transform[0][1] = basis.getRow(1).x();
    transform[1][1] = basis.getRow(1).y();
    transform[2][1] = basis.getRow(1).z();
    transform[0][2] = basis.getRow(2).x();
    transform[1][2] = basis.getRow(2).y();
    transform[2][2] = basis.getRow(2).z();

    transform[3][0] = origin.x();
    transform[3][1] = origin.y();
    transform[3][2] = origin.z();
}

void cbtBodySetCenterOfMassPosition(CbtBodyHandle body_handle, const Vector3 position) {
    assert(body_handle && cbtBodyIsCreated(body_handle));
    assert(position);
    auto body = (btRigidBody*) body_handle;

    btTransform& trans = body->getWorldTransform();
    btVector3& origin = trans.getOrigin();

    origin.setValue(position[0], position[1], position[2]);
}

void cbtBodyGetCenterOfMassPosition(CbtBodyHandle body_handle, Vector3 position) {
    assert(body_handle && cbtBodyIsCreated(body_handle));
    assert(position);
    auto body = (btRigidBody*) body_handle;

    const btTransform& trans = body->getCenterOfMassTransform();
    const btVector3& origin = trans.getOrigin();

    position[0] = origin.x();
    position[1] = origin.y();
    position[2] = origin.z();
}

void cbtBodyGetInvCenterOfMassTransform(CbtBodyHandle body_handle, Vector3 transform[4]) {
    assert(body_handle && cbtBodyIsCreated(body_handle));
    assert(transform);
    auto body = (btRigidBody*)body_handle;

    const btTransform trans = body->getCenterOfMassTransform().inverse();
    const btMatrix3x3& basis = trans.getBasis();
    const btVector3& origin = trans.getOrigin();

    // NOTE(mziulek): We transpose Bullet matrix here to make it compatible with: v * M convention.
    transform[0][0] = basis.getRow(0).x();
    transform[1][0] = basis.getRow(0).y();
    transform[2][0] = basis.getRow(0).z();
    transform[0][1] = basis.getRow(1).x();
    transform[1][1] = basis.getRow(1).y();
    transform[2][1] = basis.getRow(1).z();
    transform[0][2] = basis.getRow(2).x();
    transform[1][2] = basis.getRow(2).y();
    transform[2][2] = basis.getRow(2).z();

    transform[3][0] = origin.x();
    transform[3][1] = origin.y();
    transform[3][2] = origin.z();
}

void cbtBodyGetGraphicsWorldTransform(CbtBodyHandle body_handle, Vector3 transform[4]) {
    assert(body_handle && cbtBodyIsCreated(body_handle));
    assert(transform);
    auto body = (btRigidBody*)body_handle;

    btTransform trans;
    body->getMotionState()->getWorldTransform(trans);

    const btMatrix3x3& basis = trans.getBasis();
    const btVector3& origin = trans.getOrigin();

    // NOTE(mziulek): We transpose Bullet matrix here to make it compatible with: v * M convention.
    transform[0][0] = basis.getRow(0).x();
    transform[1][0] = basis.getRow(0).y();
    transform[2][0] = basis.getRow(0).z();
    transform[0][1] = basis.getRow(1).x();
    transform[1][1] = basis.getRow(1).y();
    transform[2][1] = basis.getRow(1).z();
    transform[0][2] = basis.getRow(2).x();
    transform[1][2] = basis.getRow(2).y();
    transform[2][2] = basis.getRow(2).z();

    transform[3][0] = origin.x();
    transform[3][1] = origin.y();
    transform[3][2] = origin.z();
}

float cbtBodyGetCcdSweptSphereRadius(CbtBodyHandle body_handle) {
    assert(body_handle && cbtBodyIsCreated(body_handle));
    auto body = (btRigidBody*)body_handle;
    return body->getCcdSweptSphereRadius();
}

void cbtBodySetCcdSweptSphereRadius(CbtBodyHandle body_handle, float radius) {
    assert(body_handle && cbtBodyIsCreated(body_handle));
    auto body = (btRigidBody*)body_handle;
    body->setCcdSweptSphereRadius(radius);
}

float cbtBodyGetCcdMotionThreshold(CbtBodyHandle body_handle) {
    assert(body_handle && cbtBodyIsCreated(body_handle));
    auto body = (btRigidBody*)body_handle;
    return body->getCcdMotionThreshold();
}

void cbtBodySetCcdMotionThreshold(CbtBodyHandle body_handle, float threshold) {
    assert(body_handle && cbtBodyIsCreated(body_handle));
    auto body = (btRigidBody*)body_handle;
    body->setCcdMotionThreshold(threshold);
}

int cbtBodyGetCollisionFlags(CbtBodyHandle body_handle) {
    assert(body_handle && cbtBodyIsCreated(body_handle));
    auto body = (btRigidBody*) body_handle;
    return body->getCollisionFlags();
}

void cbtBodyAddCollisionFlag(CbtBodyHandle body_handle, int flag) {
    assert(body_handle && cbtBodyIsCreated(body_handle));
    auto body = (btRigidBody*) body_handle;
    body->setCollisionFlags(body->getCollisionFlags() | flag);
}

void cbtBodyRemoveCollisionFlag(CbtBodyHandle body_handle, int flag) {
    assert(body_handle && cbtBodyIsCreated(body_handle));
    auto body = (btRigidBody*) body_handle;
    body->setCollisionFlags(body->getCollisionFlags() & ~flag);
}

// constraints
CbtBodyHandle cbtConGetFixedBody(void) {
    return (CbtBodyHandle)&btTypedConstraint::getFixedBody();
}

void cbtConDestroyFixedBody(void) {
    btTypedConstraint::destroyFixedBody();
}

CbtConstraintHandle cbtConAllocate(int con_type) {
    size_t size = 0;
    switch (con_type) {
        case CBT_CONSTRAINT_TYPE_POINT2POINT: size = sizeof(btPoint2PointConstraint); break;
        case CBT_CONSTRAINT_TYPE_HINGE: size = sizeof(btHingeConstraint); break;
        case CBT_CONSTRAINT_TYPE_CONETWIST: size = sizeof(btConeTwistConstraint); break;
        case CBT_CONSTRAINT_TYPE_SLIDER: size = sizeof(btSliderConstraint); break;
        case CBT_CONSTRAINT_TYPE_GEAR: size = sizeof(btGearConstraint); break;
        case CBT_CONSTRAINT_TYPE_D6_SPRING_2: size = sizeof(btGeneric6DofSpring2Constraint); break;
        default: assert(0);
    }
    auto constraint = (int*)btAlignedAlloc(size, 16);
    // Set vtable to 0, this means that object is not created.
    constraint[0] = 0;
    constraint[1] = 0;
    // btTypedObject::m_objectType field is just after vtable (offset: 8).
    constraint[2] = con_type;
    return (CbtConstraintHandle)constraint;
}

void cbtConDeallocate(CbtConstraintHandle con_handle) {
    assert(con_handle && !cbtConIsCreated(con_handle));
    btAlignedFree(con_handle);
}

void cbtConDestroy(CbtConstraintHandle con_handle) {
    assert(con_handle && cbtConIsCreated(con_handle));
    auto constraint = (btTypedConstraint*)con_handle;
    constraint->~btTypedConstraint();
    // Set vtable to 0, this means that object is not created.
    ((uint64_t*)con_handle)[0] = 0;
}

bool cbtConIsCreated(CbtConstraintHandle con_handle) {
    assert(con_handle);
    // vtable == 0 means that object is not created.
    return ((uint64_t*)con_handle)[0] != 0;
}

int cbtConGetType(CbtConstraintHandle con_handle) {
    assert(con_handle);
    auto constraint = (int*)con_handle;
    // btTypedObject::m_objectType field is just after vtable (offset: 8).
    // This function works even for not yet created shapes (cbtConAllocate sets the type).
    return constraint[2];
}

void cbtConSetParam(CbtConstraintHandle con_handle, int param, float value, int up_axis) {
    assert(con_handle && cbtConIsCreated(con_handle));
    auto con = (btTypedConstraint*)con_handle;
    con->setParam(param, value, up_axis);
}

float cbtConGetParam(CbtConstraintHandle con_handle, int param, int up_axis) {
    assert(con_handle && cbtConIsCreated(con_handle));
    auto con = (btTypedConstraint*)con_handle;
    return con->getParam(param, up_axis);
}

void cbtConSetEnabled(CbtConstraintHandle con_handle, bool enabled) {
    assert(con_handle && cbtConIsCreated(con_handle));
    auto con = (btTypedConstraint*)con_handle;
    con->setEnabled(enabled);
}

bool cbtConIsEnabled(CbtConstraintHandle con_handle) {
    assert(con_handle && cbtConIsCreated(con_handle));
    auto con = (btTypedConstraint*)con_handle;
    return con->isEnabled();
}

CbtBodyHandle cbtConGetBodyA(CbtConstraintHandle con_handle) {
    assert(con_handle && cbtConIsCreated(con_handle));
    auto con = (btTypedConstraint*)con_handle;
    return (CbtBodyHandle)&con->getRigidBodyA();
}

CbtBodyHandle cbtConGetBodyB(CbtConstraintHandle con_handle) {
    assert(con_handle && cbtConIsCreated(con_handle));
    auto con = (btTypedConstraint*)con_handle;
    return (CbtBodyHandle)&con->getRigidBodyB();
}

void cbtConSetBreakingImpulseThreshold(CbtConstraintHandle con_handle, float threshold) {
    assert(con_handle && cbtConIsCreated(con_handle));
    auto con = (btTypedConstraint*)con_handle;
    con->setBreakingImpulseThreshold(threshold);
}

float cbtConGetBreakingImpulseThreshold(CbtConstraintHandle con_handle) {
    assert(con_handle && cbtConIsCreated(con_handle));
    auto con = (btTypedConstraint*)con_handle;
    return con->getBreakingImpulseThreshold();
}

void cbtConSetDebugDrawSize(CbtConstraintHandle con_handle, float size) {
    assert(con_handle && cbtConIsCreated(con_handle));
    auto con = (btTypedConstraint*)con_handle;
    con->setDbgDrawSize(size);
}

float cbtConGetDebugDrawSize(CbtConstraintHandle con_handle) {
    assert(con_handle && cbtConIsCreated(con_handle));
    auto con = (btTypedConstraint*)con_handle;
    return con->getDbgDrawSize();
}

void cbtConPoint2PointCreate1(
    CbtConstraintHandle con_handle,
    CbtBodyHandle body_handle_a,
    const Vector3 pivot_a
) {
    assert(con_handle && !cbtConIsCreated(con_handle));
    assert(cbtConGetType(con_handle) == CBT_CONSTRAINT_TYPE_POINT2POINT);
    assert(body_handle_a && cbtBodyIsCreated(body_handle_a));
    assert(pivot_a);

    auto body_a = (btRigidBody*)body_handle_a;
    new (con_handle) btPoint2PointConstraint( *body_a, btVector3(pivot_a[0], pivot_a[1], pivot_a[2]));
}

void cbtConPoint2PointCreate2(
    CbtConstraintHandle con_handle,
    CbtBodyHandle body_handle_a,
    CbtBodyHandle body_handle_b,
    const Vector3 pivot_a,
    const Vector3 pivot_b
) {
    assert(con_handle && !cbtConIsCreated(con_handle));
    assert(cbtConGetType(con_handle) == CBT_CONSTRAINT_TYPE_POINT2POINT);
    assert(body_handle_a && cbtBodyIsCreated(body_handle_a));
    assert(body_handle_b && cbtBodyIsCreated(body_handle_b));
    assert(pivot_a && pivot_b);

    auto body_a = (btRigidBody*)body_handle_a;
    auto body_b = (btRigidBody*)body_handle_b;
    new (con_handle) btPoint2PointConstraint(
        *body_a,
        *body_b,
        btVector3(pivot_a[0], pivot_a[1], pivot_a[2]),
        btVector3(pivot_b[0], pivot_b[1], pivot_b[2])
    );
}

void cbtConPoint2PointSetPivotA(CbtConstraintHandle con_handle, const Vector3 pivot) {
    assert(con_handle && cbtConIsCreated(con_handle));
    assert(cbtConGetType(con_handle) == CBT_CONSTRAINT_TYPE_POINT2POINT);
    btPoint2PointConstraint* con = (btPoint2PointConstraint*)con_handle;
    con->setPivotA(btVector3(pivot[0], pivot[1], pivot[2]));
}

void cbtConPoint2PointSetPivotB(CbtConstraintHandle con_handle, const Vector3 pivot) {
    assert(con_handle && cbtConIsCreated(con_handle));
    assert(cbtConGetType(con_handle) == CBT_CONSTRAINT_TYPE_POINT2POINT);
    btPoint2PointConstraint* con = (btPoint2PointConstraint*)con_handle;
    con->setPivotB(btVector3(pivot[0], pivot[1], pivot[2]));
}

void cbtConPoint2PointSetTau(CbtConstraintHandle con_handle, float tau) {
    assert(con_handle && cbtConIsCreated(con_handle));
    assert(cbtConGetType(con_handle) == CBT_CONSTRAINT_TYPE_POINT2POINT);
    btPoint2PointConstraint* con = (btPoint2PointConstraint*)con_handle;
    con->m_setting.m_tau = tau;
}

void cbtConPoint2PointSetDamping(CbtConstraintHandle con_handle, float damping) {
    assert(con_handle && cbtConIsCreated(con_handle));
    assert(cbtConGetType(con_handle) == CBT_CONSTRAINT_TYPE_POINT2POINT);
    btPoint2PointConstraint* con = (btPoint2PointConstraint*)con_handle;
    con->m_setting.m_damping = damping;
}

void cbtConPoint2PointSetImpulseClamp(CbtConstraintHandle con_handle, float impulse_clamp) {
    assert(con_handle && cbtConIsCreated(con_handle));
    assert(cbtConGetType(con_handle) == CBT_CONSTRAINT_TYPE_POINT2POINT);
    btPoint2PointConstraint* con = (btPoint2PointConstraint*)con_handle;
    con->m_setting.m_impulseClamp = impulse_clamp * velConv;
}

void cbtConPoint2PointGetPivotA(CbtConstraintHandle con_handle, Vector3 pivot) {
    assert(con_handle && cbtConIsCreated(con_handle) && pivot);
    assert(cbtConGetType(con_handle) == CBT_CONSTRAINT_TYPE_POINT2POINT);
    btPoint2PointConstraint* con = (btPoint2PointConstraint*)con_handle;
    auto tmp = con->getPivotInA();
    pivot[0] = tmp.x();
    pivot[1] = tmp.y();
    pivot[2] = tmp.z();
}

void cbtConPoint2PointGetPivotB(CbtConstraintHandle con_handle, Vector3 pivot) {
    assert(con_handle && cbtConIsCreated(con_handle) && pivot);
    assert(cbtConGetType(con_handle) == CBT_CONSTRAINT_TYPE_POINT2POINT);
    btPoint2PointConstraint* con = (btPoint2PointConstraint*)con_handle;
    auto tmp = con->getPivotInB();
    pivot[0] = tmp.x();
    pivot[1] = tmp.y();
    pivot[2] = tmp.z();
}

void cbtConHingeCreate1(
    CbtConstraintHandle con_handle,
    CbtBodyHandle body_handle_a,
    const Vector3 pivot_a,
    const Vector3 axis_a,
    bool use_reference_frame_a
) {
    assert(con_handle && !cbtConIsCreated(con_handle));
    assert(cbtConGetType(con_handle) == CBT_CONSTRAINT_TYPE_HINGE);
    assert(body_handle_a && cbtBodyIsCreated(body_handle_a));
    assert(pivot_a && axis_a);

    auto body_a = (btRigidBody*)body_handle_a;
    new (con_handle) btHingeConstraint(
        *body_a,
        btVector3(pivot_a[0], pivot_a[1], pivot_a[2]),
        btVector3(axis_a[0], axis_a[1], axis_a[2]),
        use_reference_frame_a
    );
}

void cbtConHingeCreate2(
    CbtConstraintHandle con_handle,
    CbtBodyHandle body_handle_a,
    CbtBodyHandle body_handle_b,
    const Vector3 pivot_a,
    const Vector3 pivot_b,
    const Vector3 axis_a,
    const Vector3 axis_b,
    bool use_reference_frame_a
) {
    assert(con_handle && !cbtConIsCreated(con_handle));
    assert(cbtConGetType(con_handle) == CBT_CONSTRAINT_TYPE_HINGE);
    assert(body_handle_a && cbtBodyIsCreated(body_handle_a));
    assert(body_handle_b && cbtBodyIsCreated(body_handle_b));
    assert(pivot_a && pivot_b);
    assert(axis_a && axis_b);

    auto body_a = (btRigidBody*)body_handle_a;
    auto body_b = (btRigidBody*)body_handle_b;
    new (con_handle) btHingeConstraint(
        *body_a,
        *body_b,
        btVector3(pivot_a[0], pivot_a[1], pivot_a[2]),
        btVector3(pivot_b[0], pivot_b[1], pivot_b[2]),
        btVector3(axis_a[0], axis_a[1], axis_a[2]),
        btVector3(axis_b[0], axis_b[1], axis_b[2]),
        use_reference_frame_a
    );
}

void cbtConHingeCreate3(
    CbtConstraintHandle con_handle,
    CbtBodyHandle body_handle_a,
    const Vector3 frame_a[4],
    bool use_reference_frame_a
) {
    assert(con_handle && !cbtConIsCreated(con_handle));
    assert(cbtConGetType(con_handle) == CBT_CONSTRAINT_TYPE_HINGE);
    assert(body_handle_a && cbtBodyIsCreated(body_handle_a));
    assert(frame_a);

    auto body_a = (btRigidBody*)body_handle_a;
    new (con_handle) btHingeConstraint(
        *body_a,
        makeBtTransform(frame_a),
        use_reference_frame_a
    );
}

void cbtConHingeSetAngularOnly(CbtConstraintHandle con_handle, bool angular_only) {
    assert(con_handle && cbtConIsCreated(con_handle));
    assert(cbtConGetType(con_handle) == CBT_CONSTRAINT_TYPE_HINGE);
    auto con = (btHingeConstraint*)con_handle;
    con->setAngularOnly(angular_only);
}

void cbtConHingeEnableAngularMotor(
    CbtConstraintHandle con_handle,
    bool enable,
    float target_velocity,
    float max_motor_impulse
) {
    assert(con_handle && cbtConIsCreated(con_handle));
    assert(cbtConGetType(con_handle) == CBT_CONSTRAINT_TYPE_HINGE);
    auto con = (btHingeConstraint*)con_handle;
    con->enableAngularMotor(enable, target_velocity * velConv, max_motor_impulse * velConv);
}

void cbtConHingeSetLimit(
    CbtConstraintHandle con_handle,
    float low,
    float high,
    float softness,
    float bias_factor,
    float relaxation_factor
) {
    assert(con_handle && cbtConIsCreated(con_handle));
    assert(cbtConGetType(con_handle) == CBT_CONSTRAINT_TYPE_HINGE);
    auto con = (btHingeConstraint*)con_handle;
    con->setLimit(low, high, softness, bias_factor, relaxation_factor);
}

void cbtConGearCreate(
    CbtConstraintHandle con_handle,
    CbtBodyHandle body_handle_a,
    CbtBodyHandle body_handle_b,
    const Vector3 axis_a,
    const Vector3 axis_b,
    float ratio
) {
    assert(con_handle && !cbtConIsCreated(con_handle));
    assert(cbtConGetType(con_handle) == CBT_CONSTRAINT_TYPE_GEAR);
    assert(body_handle_a && cbtBodyIsCreated(body_handle_a));
    assert(body_handle_b && cbtBodyIsCreated(body_handle_b));
    assert(axis_a && axis_b);

    auto body_a = (btRigidBody*)body_handle_a;
    auto body_b = (btRigidBody*)body_handle_b;
    new (con_handle) btGearConstraint(
        *body_a,
        *body_b,
        btVector3(axis_a[0], axis_a[1], axis_a[2]),
        btVector3(axis_b[0], axis_b[1], axis_b[2]),
        ratio
    );
}

void cbtConGearSetAxisA(CbtConstraintHandle con_handle, const Vector3 axis) {
    assert(con_handle && cbtConIsCreated(con_handle));
    assert(cbtConGetType(con_handle) == CBT_CONSTRAINT_TYPE_GEAR);
    assert(axis);
    auto con = (btGearConstraint*)con_handle;
    auto tmp = btVector3(axis[0], axis[1], axis[2]);
    con->setAxisA(tmp);
}

void cbtConGearSetAxisB(CbtConstraintHandle con_handle, const Vector3 axis) {
    assert(con_handle && cbtConIsCreated(con_handle));
    assert(cbtConGetType(con_handle) == CBT_CONSTRAINT_TYPE_GEAR);
    assert(axis);
    auto con = (btGearConstraint*)con_handle;
    auto tmp = btVector3(axis[0], axis[1], axis[2]);
    con->setAxisB(tmp);
}

void cbtConGearSetRatio(CbtConstraintHandle con_handle, float ratio) {
    assert(con_handle && cbtConIsCreated(con_handle));
    assert(cbtConGetType(con_handle) == CBT_CONSTRAINT_TYPE_GEAR);
    assert(ratio > 0.0);
    auto con = (btGearConstraint*)con_handle;
    con->setRatio(ratio);
}

void cbtConGearGetAxisA(CbtConstraintHandle con_handle, Vector3 axis) {
    assert(con_handle && cbtConIsCreated(con_handle));
    assert(cbtConGetType(con_handle) == CBT_CONSTRAINT_TYPE_GEAR);
    assert(axis);
    auto con = (btGearConstraint*)con_handle;
    auto tmp = con->getAxisA();
    axis[0] = tmp.x();
    axis[1] = tmp.y();
    axis[2] = tmp.z();
}

void cbtConGearGetAxisB(CbtConstraintHandle con_handle, Vector3 axis) {
    assert(con_handle && cbtConIsCreated(con_handle));
    assert(cbtConGetType(con_handle) == CBT_CONSTRAINT_TYPE_GEAR);
    assert(axis);
    auto con = (btGearConstraint*)con_handle;
    auto tmp = con->getAxisB();
    axis[0] = tmp.x();
    axis[1] = tmp.y();
    axis[2] = tmp.z();
}

float cbtConGearGetRatio(CbtConstraintHandle con_handle) {
    assert(con_handle && cbtConIsCreated(con_handle));
    assert(cbtConGetType(con_handle) == CBT_CONSTRAINT_TYPE_GEAR);
    auto con = (btGearConstraint*)con_handle;
    return con->getRatio();
}

void cbtConSliderCreate1(
    CbtConstraintHandle con_handle,
    CbtBodyHandle body_handle_b,
    const Vector3 frame_b[4],
    bool use_reference_frame_a
) {
    assert(con_handle && !cbtConIsCreated(con_handle));
    assert(cbtConGetType(con_handle) == CBT_CONSTRAINT_TYPE_SLIDER);
    assert(body_handle_b && cbtBodyIsCreated(body_handle_b));
    assert(frame_b);

    auto body_b = (btRigidBody*)body_handle_b;
    new (con_handle) btSliderConstraint(
        *body_b,
        makeBtTransform(frame_b),
        use_reference_frame_a
    );
}

void cbtConSliderCreate2(
    CbtConstraintHandle con_handle,
    CbtBodyHandle body_handle_a,
    CbtBodyHandle body_handle_b,
    const Vector3 frame_a[4],
    const Vector3 frame_b[4],
    bool use_reference_frame_a
) {
    assert(con_handle && !cbtConIsCreated(con_handle));
    assert(cbtConGetType(con_handle) == CBT_CONSTRAINT_TYPE_SLIDER);
    assert(body_handle_a && cbtBodyIsCreated(body_handle_a));
    assert(body_handle_b && cbtBodyIsCreated(body_handle_b));
    assert(frame_a && frame_b);

    auto body_a = (btRigidBody*)body_handle_a;
    auto body_b = (btRigidBody*)body_handle_b;
    new (con_handle) btSliderConstraint(
        *body_a,
        *body_b,
        makeBtTransform(frame_a),
        makeBtTransform(frame_b),
        use_reference_frame_a
    );
}

void cbtConSliderSetLinearLowerLimit(CbtConstraintHandle con_handle, float limit) {
    assert(con_handle && cbtConIsCreated(con_handle));
    assert(cbtConGetType(con_handle) == CBT_CONSTRAINT_TYPE_SLIDER);
    auto con = (btSliderConstraint*)con_handle;
    con->setLowerLinLimit(limit);
}

void cbtConSliderSetLinearUpperLimit(CbtConstraintHandle con_handle, float limit) {
    assert(con_handle && cbtConIsCreated(con_handle));
    assert(cbtConGetType(con_handle) == CBT_CONSTRAINT_TYPE_SLIDER);
    auto con = (btSliderConstraint*)con_handle;
    con->setUpperLinLimit(limit);
}

float cbtConSliderGetLinearLowerLimit(CbtConstraintHandle con_handle) {
    assert(con_handle && cbtConIsCreated(con_handle));
    assert(cbtConGetType(con_handle) == CBT_CONSTRAINT_TYPE_SLIDER);
    auto con = (btSliderConstraint*)con_handle;
    return con->getLowerLinLimit();
}

float cbtConSliderGetLinearUpperLimit(CbtConstraintHandle con_handle) {
    assert(con_handle && cbtConIsCreated(con_handle));
    assert(cbtConGetType(con_handle) == CBT_CONSTRAINT_TYPE_SLIDER);
    auto con = (btSliderConstraint*)con_handle;
    return con->getUpperLinLimit();
}

void cbtConSliderSetAngularLowerLimit(CbtConstraintHandle con_handle, float limit) {
    assert(con_handle && cbtConIsCreated(con_handle));
    assert(cbtConGetType(con_handle) == CBT_CONSTRAINT_TYPE_SLIDER);
    auto con = (btSliderConstraint*)con_handle;
    con->setLowerAngLimit(limit);
}

void cbtConSliderSetAngularUpperLimit(CbtConstraintHandle con_handle, float limit) {
    assert(con_handle && cbtConIsCreated(con_handle));
    assert(cbtConGetType(con_handle) == CBT_CONSTRAINT_TYPE_SLIDER);
    auto con = (btSliderConstraint*)con_handle;
    con->setUpperAngLimit(limit);
}

float cbtConSliderGetAngularLowerLimit(CbtConstraintHandle con_handle) {
    assert(con_handle && cbtConIsCreated(con_handle));
    assert(cbtConGetType(con_handle) == CBT_CONSTRAINT_TYPE_SLIDER);
    auto con = (btSliderConstraint*)con_handle;
    return con->getLowerAngLimit();
}

float cbtConSliderGetAngularUpperLimit(CbtConstraintHandle con_handle) {
    assert(con_handle && cbtConIsCreated(con_handle));
    assert(cbtConGetType(con_handle) == CBT_CONSTRAINT_TYPE_SLIDER);
    auto con = (btSliderConstraint*)con_handle;
    return con->getUpperAngLimit();
}

void cbtConSliderEnableLinearMotor(
    CbtConstraintHandle con_handle,
    bool enable,
    float target_velocity,
    float max_motor_force
) {
    assert(con_handle && cbtConIsCreated(con_handle));
    assert(cbtConGetType(con_handle) == CBT_CONSTRAINT_TYPE_SLIDER);
    auto con = (btSliderConstraint*)con_handle;
    con->setPoweredLinMotor(enable);
    con->setTargetLinMotorVelocity(target_velocity * velConv);
    con->setMaxLinMotorForce(max_motor_force * accelConv);
}

void cbtConSliderEnableAngularMotor(
    CbtConstraintHandle con_handle,
    bool enable,
    float target_velocity,
    float max_force
) {
    assert(con_handle && cbtConIsCreated(con_handle));
    assert(cbtConGetType(con_handle) == CBT_CONSTRAINT_TYPE_SLIDER);
    auto con = (btSliderConstraint*)con_handle;
    con->setPoweredAngMotor(enable);
    con->setTargetAngMotorVelocity(target_velocity * velConv);
    con->setMaxAngMotorForce(max_force * accelConv);
}

bool cbtConSliderIsLinearMotorEnabled(CbtConstraintHandle con_handle) {
    assert(con_handle && cbtConIsCreated(con_handle));
    assert(cbtConGetType(con_handle) == CBT_CONSTRAINT_TYPE_SLIDER);
    auto con = (btSliderConstraint*)con_handle;
    return con->getPoweredLinMotor();
}

bool cbtConSliderIsAngularMotorEnabled(CbtConstraintHandle con_handle) {
    assert(con_handle && cbtConIsCreated(con_handle));
    assert(cbtConGetType(con_handle) == CBT_CONSTRAINT_TYPE_SLIDER);
    auto con = (btSliderConstraint*)con_handle;
    return con->getPoweredAngMotor();
}

void cbtConSliderGetAngularMotor(
    CbtConstraintHandle con_handle,
    float* target_velocity,
    float* max_force
) {
    assert(con_handle && cbtConIsCreated(con_handle));
    assert(cbtConGetType(con_handle) == CBT_CONSTRAINT_TYPE_SLIDER);
    auto con = (btSliderConstraint*)con_handle;
    if (target_velocity) {
        *target_velocity = con->getTargetAngMotorVelocity() / velConv;
    }
    if (max_force) {
        *max_force = con->getMaxAngMotorForce() / accelConv;
    }
}

float cbtConSliderGetLinearPosition(CbtConstraintHandle con_handle) {
    assert(con_handle && cbtConIsCreated(con_handle));
    assert(cbtConGetType(con_handle) == CBT_CONSTRAINT_TYPE_SLIDER);
    auto con = (btSliderConstraint*)con_handle;
    return con->getLinearPos();
}

float cbtConSliderGetAngularPosition(CbtConstraintHandle con_handle) {
    assert(con_handle && cbtConIsCreated(con_handle));
    assert(cbtConGetType(con_handle) == CBT_CONSTRAINT_TYPE_SLIDER);
    auto con = (btSliderConstraint*)con_handle;
    return con->getAngularPos();
}

void cbtConD6Spring2Create1(
    CbtConstraintHandle con_handle,
    CbtBodyHandle body_handle_b,
    const Vector3 frame_b[4],
    int rotate_order
) {
    assert(con_handle && !cbtConIsCreated(con_handle));
    assert(cbtConGetType(con_handle) == CBT_CONSTRAINT_TYPE_D6_SPRING_2);
    assert(body_handle_b && cbtBodyIsCreated(body_handle_b));
    assert(frame_b);
    assert(rotate_order >= CBT_ROTATE_ORDER_XYZ && rotate_order <= CBT_ROTATE_ORDER_ZYX);

    auto body_b = (btRigidBody*)body_handle_b;
    new (con_handle) btGeneric6DofSpring2Constraint(
        *body_b,
        makeBtTransform(frame_b),
        (RotateOrder)rotate_order
    );
}

void cbtConD6Spring2Create2(
    CbtConstraintHandle con_handle,
    CbtBodyHandle body_handle_a,
    CbtBodyHandle body_handle_b,
    const Vector3 frame_a[4],
    const Vector3 frame_b[4],
    int rotate_order
) {
    assert(con_handle && !cbtConIsCreated(con_handle));
    assert(cbtConGetType(con_handle) == CBT_CONSTRAINT_TYPE_D6_SPRING_2);
    assert(body_handle_a && cbtBodyIsCreated(body_handle_a));
    assert(body_handle_b && cbtBodyIsCreated(body_handle_b));
    assert(frame_a && frame_b);
    assert(rotate_order >= CBT_ROTATE_ORDER_XYZ && rotate_order <= CBT_ROTATE_ORDER_ZYX);

    auto body_a = (btRigidBody*)body_handle_a;
    auto body_b = (btRigidBody*)body_handle_b;
    new (con_handle) btGeneric6DofSpring2Constraint(
        *body_a,
        *body_b,
        makeBtTransform(frame_a),
        makeBtTransform(frame_b),
        (RotateOrder)rotate_order
    );
}

void cbtConD6Spring2SetLinearLowerLimit(CbtConstraintHandle con_handle, const Vector3 limit) {
    assert(con_handle && cbtConIsCreated(con_handle));
    assert(cbtConGetType(con_handle) == CBT_CONSTRAINT_TYPE_D6_SPRING_2);
    assert(limit);
    auto con = (btGeneric6DofSpring2Constraint*)con_handle;
    con->setLinearLowerLimit(btVector3(limit[0], limit[1], limit[2]));
}

void cbtConD6Spring2SetLinearUpperLimit(CbtConstraintHandle con_handle, const Vector3 limit) {
    assert(con_handle && cbtConIsCreated(con_handle));
    assert(cbtConGetType(con_handle) == CBT_CONSTRAINT_TYPE_D6_SPRING_2);
    assert(limit);
    auto con = (btGeneric6DofSpring2Constraint*)con_handle;
    con->setLinearUpperLimit(btVector3(limit[0], limit[1], limit[2]));
}

void cbtConD6Spring2GetLinearLowerLimit(CbtConstraintHandle con_handle, Vector3 limit) {
    assert(con_handle && cbtConIsCreated(con_handle));
    assert(cbtConGetType(con_handle) == CBT_CONSTRAINT_TYPE_D6_SPRING_2);
    assert(limit);
    auto con = (btGeneric6DofSpring2Constraint*)con_handle;
    btVector3 lim;
    con->getLinearLowerLimit(lim);
    limit[0] = lim.x();
    limit[1] = lim.y();
    limit[2] = lim.z();
}

void cbtConD6Spring2GetLinearUpperLimit(CbtConstraintHandle con_handle, Vector3 limit) {
    assert(con_handle && cbtConIsCreated(con_handle));
    assert(cbtConGetType(con_handle) == CBT_CONSTRAINT_TYPE_D6_SPRING_2);
    assert(limit);
    auto con = (btGeneric6DofSpring2Constraint*)con_handle;
    btVector3 lim;
    con->getLinearUpperLimit(lim);
    limit[0] = lim.x();
    limit[1] = lim.y();
    limit[2] = lim.z();
}

void cbtConD6Spring2SetAngularLowerLimit(CbtConstraintHandle con_handle, const Vector3 limit) {
    assert(con_handle && cbtConIsCreated(con_handle));
    assert(cbtConGetType(con_handle) == CBT_CONSTRAINT_TYPE_D6_SPRING_2);
    assert(limit);
    auto con = (btGeneric6DofSpring2Constraint*)con_handle;
    con->setAngularLowerLimit(btVector3(limit[0], limit[1], limit[2]));
}

void cbtConD6Spring2SetAngularUpperLimit(CbtConstraintHandle con_handle, const Vector3 limit) {
    assert(con_handle && cbtConIsCreated(con_handle));
    assert(cbtConGetType(con_handle) == CBT_CONSTRAINT_TYPE_D6_SPRING_2);
    assert(limit);
    auto con = (btGeneric6DofSpring2Constraint*)con_handle;
    con->setAngularUpperLimit(btVector3(limit[0], limit[1], limit[2]));
}

void cbtConD6Spring2GetAngularLowerLimit(CbtConstraintHandle con_handle, Vector3 limit) {
    assert(con_handle && cbtConIsCreated(con_handle));
    assert(cbtConGetType(con_handle) == CBT_CONSTRAINT_TYPE_D6_SPRING_2);
    assert(limit);
    auto con = (btGeneric6DofSpring2Constraint*)con_handle;
    btVector3 lim;
    con->getAngularLowerLimit(lim);
    limit[0] = lim.x();
    limit[1] = lim.y();
    limit[2] = lim.z();
}

void cbtConD6Spring2GetAngularUpperLimit(CbtConstraintHandle con_handle, Vector3 limit) {
    assert(con_handle && cbtConIsCreated(con_handle));
    assert(cbtConGetType(con_handle) == CBT_CONSTRAINT_TYPE_D6_SPRING_2);
    assert(limit);
    auto con = (btGeneric6DofSpring2Constraint*)con_handle;
    btVector3 lim;
    con->getAngularUpperLimit(lim);
    limit[0] = lim.x();
    limit[1] = lim.y();
    limit[2] = lim.z();
}

void cbtConConeTwistCreate1(
    CbtConstraintHandle con_handle,
    CbtBodyHandle body_handle_a,
    const Vector3 frame_a[4]
) {
    assert(con_handle && !cbtConIsCreated(con_handle));
    assert(cbtConGetType(con_handle) == CBT_CONSTRAINT_TYPE_CONETWIST);
    assert(body_handle_a && cbtBodyIsCreated(body_handle_a));
    assert(frame_a);

    auto body_a = (btRigidBody*)body_handle_a;
    new (con_handle) btConeTwistConstraint(*body_a, makeBtTransform(frame_a));
}

void cbtConConeTwistCreate2(
    CbtConstraintHandle con_handle,
    CbtBodyHandle body_handle_a,
    CbtBodyHandle body_handle_b,
    const Vector3 frame_a[4],
    const Vector3 frame_b[4]
) {
    assert(con_handle && !cbtConIsCreated(con_handle));
    assert(cbtConGetType(con_handle) == CBT_CONSTRAINT_TYPE_CONETWIST);
    assert(body_handle_a && cbtBodyIsCreated(body_handle_a));
    assert(body_handle_b && cbtBodyIsCreated(body_handle_b));
    assert(frame_a && frame_b);

    auto body_a = (btRigidBody*)body_handle_a;
    auto body_b = (btRigidBody*)body_handle_b;
    new (con_handle) btConeTwistConstraint(
        *body_a,
        *body_b,
        makeBtTransform(frame_a),
        makeBtTransform(frame_b)
    );
}

void cbtConConeTwistSetLimit(
    CbtConstraintHandle con_handle,
    float swing_span1,
    float swing_span2,
    float twist_span,
    float softness,
    float bias_factor,
    float relaxation_factor
) {
    assert(con_handle && cbtConIsCreated(con_handle));
    assert(cbtConGetType(con_handle) == CBT_CONSTRAINT_TYPE_CONETWIST);
    auto con = (btConeTwistConstraint*)con_handle;
    con->setLimit(swing_span1, swing_span2, twist_span, softness, bias_factor, relaxation_factor);
}
