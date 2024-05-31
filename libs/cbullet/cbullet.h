// cbullet v0.2
// C API for Bullet Physics SDK

#pragma once

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define BT_API __attribute__ ((dllexport))
  #else
    #define BT_API __declspec(dllexport)
  #endif
#else
#if __GNUC__ >= 4
    #define BT_API __attribute__ ((visibility ("default")))
#else
    #define BT_API
#endif
#endif

#include <stddef.h>

#define CBT_DECLARE_HANDLE(name) typedef struct name##__ { int unused; } *name

// cbtWorldRayTestClosest
#define CBT_COLLISION_FILTER_DEFAULT 1
#define CBT_COLLISION_FILTER_STATIC 2
#define CBT_COLLISION_FILTER_KINEMATIC 4
#define CBT_COLLISION_FILTER_DEBRIS 8
#define CBT_COLLISION_FILTER_SENSOR_TRIGGER 16
#define CBT_COLLISION_FILTER_CHARACTER 32
#define CBT_COLLISION_FILTER_ALL -1

// cbtWorldRayTestClosest
#define CBT_RAYCAST_FLAG_NONE 0
#define CBT_RAYCAST_FLAG_TRIMESH_SKIP_BACKFACES 1
#define CBT_RAYCAST_FLAG_TRIMESH_KEEP_UNFLIPPED_NORMALS 2
#define CBT_RAYCAST_FLAG_USE_SUB_SIMPLEX_CONVEX_TEST 4 // default, faster but less accurate
#define CBT_RAYCAST_FLAG_USE_GJK_CONVEX_TEST 8

// cbtBodySetAnisotropicFriction
#define CBT_ANISOTROPIC_FRICTION_DISABLED 0
#define CBT_ANISOTROPIC_FRICTION 1
#define CBT_ANISOTROPIC_ROLLING_FRICTION 2

// cbtShapeGetType, cbtShapeAllocate
#define CBT_SHAPE_TYPE_BOX 0
#define CBT_SHAPE_TYPE_SPHERE 8
#define CBT_SHAPE_TYPE_CAPSULE 10
#define CBT_SHAPE_TYPE_CONE 11
#define CBT_SHAPE_TYPE_CYLINDER 13
#define CBT_SHAPE_TYPE_COMPOUND 31
#define CBT_SHAPE_TYPE_TRIANGLE_MESH 21

// cbtConGetType, cbtConAllocate
#define CBT_CONSTRAINT_TYPE_POINT2POINT 3
#define CBT_CONSTRAINT_TYPE_HINGE 4
#define CBT_CONSTRAINT_TYPE_CONETWIST 5
#define CBT_CONSTRAINT_TYPE_SLIDER 7
#define CBT_CONSTRAINT_TYPE_GEAR 10
#define CBT_CONSTRAINT_TYPE_D6_SPRING_2 12

// cbtConSetParam
#define CBT_CONSTRAINT_PARAM_ERP 1
#define CBT_CONSTRAINT_PARAM_STOP_ERP 2
#define CBT_CONSTRAINT_PARAM_CFM 3
#define CBT_CONSTRAINT_PARAM_STOP_CFM 4

// cbtBodyGetActivationState, cbtBodySetActivationState
#define CBT_ACTIVE_TAG 1
#define CBT_ISLAND_SLEEPING 2
#define CBT_WANTS_DEACTIVATION 3
#define CBT_DISABLE_DEACTIVATION 4
#define CBT_DISABLE_SIMULATION 5

// cbtShapeCapsuleCreate, cbtShapeCylinderCreate, cbtShapeConeCreate, cbtConSetParam
#define CBT_LINEAR_AXIS_X 0
#define CBT_LINEAR_AXIS_Y 1
#define CBT_LINEAR_AXIS_Z 2
#define CBT_ANGULAR_AXIS_X 3
#define CBT_ANGULAR_AXIS_Y 4
#define CBT_ANGULAR_AXIS_Z 5

// cbtCon6DofSpring2Create
#define CBT_ROTATE_ORDER_XYZ 0
#define CBT_ROTATE_ORDER_XZY 1
#define CBT_ROTATE_ORDER_YXZ 2
#define CBT_ROTATE_ORDER_YZX 3
#define CBT_ROTATE_ORDER_ZXY 4
#define CBT_ROTATE_ORDER_ZYX 5

#define CBT_DBGMODE_DISABLED -1
#define CBT_DBGMODE_NO_DEBUG 0
#define CBT_DBGMODE_DRAW_WIREFRAME 1
#define CBT_DBGMODE_DRAW_AABB 2

typedef float Vector3[3];

#ifdef __cplusplus
extern "C" {
#else
#include <stdbool.h>
#endif

CBT_DECLARE_HANDLE(CbtWorldHandle);
CBT_DECLARE_HANDLE(CbtShapeHandle);
CBT_DECLARE_HANDLE(CbtBodyHandle);
CBT_DECLARE_HANDLE(CbtConstraintHandle);
CBT_DECLARE_HANDLE(CbtDebugDrawHandle);

typedef void* (CbtAlignedAllocFunc)(size_t size, int alignment);
typedef void (CbtAlignedFreeFunc)(void* memblock);
typedef void* (CbtAllocFunc)(size_t size);
typedef void (CbtFreeFunc)(void* memblock);

BT_API void cbtAlignedAllocSetCustom(CbtAllocFunc alloc, CbtFreeFunc free);
BT_API void cbtAlignedAllocSetCustomAligned(CbtAlignedAllocFunc alloc, CbtAlignedFreeFunc free);

typedef void (*CbtDrawLine1Callback)(
    void* context,
    const Vector3 p0,
    const Vector3 p1,
    const Vector3 color
);
typedef void (*CbtDrawLine2Callback)(
    void* context,
    const Vector3 p0,
    const Vector3 p1,
    const Vector3 color0,
    const Vector3 color1
);
typedef void (*CbtDrawContactPointCallback)(
    void* context,
    const Vector3 point,
    const Vector3 normal,
    float distance,
    int life_time,
    const Vector3 color
);

typedef struct CbtDebugDraw {
    CbtDrawLine1Callback drawLine1;
    CbtDrawLine2Callback drawLine2;
    CbtDrawContactPointCallback drawContactPoint;
    void* context;
} CbtDebugDraw;

typedef struct CbtRayCastResult {
    Vector3 hit_normal_world;
    Vector3 hit_point_world;
    float hit_fraction;
    CbtBodyHandle body;
} CbtRayCastResult;

//
// Task scheduler
//
BT_API void cbtTaskSchedInit(void);
BT_API void cbtTaskSchedDeinit(void);
BT_API int cbtTaskSchedGetNumThreads(void);
BT_API int cbtTaskSchedGetMaxNumThreads(void);
BT_API void cbtTaskSchedSetNumThreads(int num_threads);

//
// World
//
BT_API CbtWorldHandle cbtWorldCreate(void);
BT_API void cbtWorldDestroy(CbtWorldHandle world_handle);
BT_API void cbtWorldSetGravity(CbtWorldHandle world_handle, const Vector3 gravity);
BT_API void cbtWorldGetGravity(CbtWorldHandle world_handle, Vector3 gravity);
BT_API int cbtWorldStepSimulation(
    CbtWorldHandle world_handle,
    float time_step,
    int max_sub_steps, // 1
    float fixed_time_step // 1.0 / 60.0
);

BT_API void cbtWorldAddBody(CbtWorldHandle world_handle, CbtBodyHandle body_handle);
BT_API void cbtWorldAddBodyWithGroup(CbtWorldHandle world_handle, CbtBodyHandle body_handle, int group, int mask);
BT_API void cbtWorldAddConstraint(
    CbtWorldHandle world_handle,
    CbtConstraintHandle con_handle,
    bool disable_collision_between_linked_bodies // false
);

BT_API void cbtWorldRemoveBody(CbtWorldHandle world_handle, CbtBodyHandle body_handle);
BT_API void cbtWorldRemoveConstraint(CbtWorldHandle world_handle, CbtConstraintHandle constraint_handle);

BT_API int cbtWorldGetNumBodies(CbtWorldHandle world_handle);
BT_API int cbtWorldGetNumConstraints(CbtWorldHandle world_handle);
BT_API CbtBodyHandle cbtWorldGetBody(CbtWorldHandle world_handle, int body_index);
BT_API CbtConstraintHandle cbtWorldGetConstraint(CbtWorldHandle world_handle, int con_index);

// Returns `true` when hits something, `false` otherwise
BT_API bool cbtWorldRayTestClosest(
    CbtWorldHandle world_handle,
    const Vector3 ray_from_world,
    const Vector3 ray_to_world,
    int collision_filter_group,
    int collision_filter_mask,
    unsigned int flags,
    CbtRayCastResult* result
);

BT_API void cbtWorldDebugSetDrawer(CbtWorldHandle world_handle, const CbtDebugDraw* drawer);
BT_API void cbtWorldDebugSetMode(CbtWorldHandle world_handle, int mode);
BT_API int cbtWorldDebugGetMode(CbtWorldHandle world_handle);
BT_API void cbtWorldDebugDrawAll(CbtWorldHandle world_handle);
BT_API void cbtWorldDebugDrawLine1(
    CbtWorldHandle world_handle,
    const Vector3 p0,
    const Vector3 p1,
    const Vector3 color
);
BT_API void cbtWorldDebugDrawLine2(
    CbtWorldHandle world_handle,
    const Vector3 p0,
    const Vector3 p1,
    const Vector3 color0,
    const Vector3 color1
);
BT_API void cbtWorldDebugDrawSphere(
    CbtWorldHandle world_handle,
    const Vector3 position,
    float radius,
    const Vector3 color
);

//
// Shape
//
BT_API CbtShapeHandle cbtShapeAllocate(int shape_type);
BT_API void cbtShapeDeallocate(CbtShapeHandle shape_handle);

BT_API void cbtShapeDestroy(CbtShapeHandle shape_handle);
BT_API bool cbtShapeIsCreated(CbtShapeHandle shape_handle);
BT_API int cbtShapeGetType(CbtShapeHandle shape_handle);
BT_API void cbtShapeSetMargin(CbtShapeHandle shape_handle, float margin);
BT_API float cbtShapeGetMargin(CbtShapeHandle shape_handle);

BT_API bool cbtShapeIsPolyhedral(CbtShapeHandle shape_handle);
BT_API bool cbtShapeIsConvex2d(CbtShapeHandle shape_handle);
BT_API bool cbtShapeIsConvex(CbtShapeHandle shape_handle);
BT_API bool cbtShapeIsNonMoving(CbtShapeHandle shape_handle);
BT_API bool cbtShapeIsConcave(CbtShapeHandle shape_handle);
BT_API bool cbtShapeIsCompound(CbtShapeHandle shape_handle);

BT_API void cbtShapeCalculateLocalInertia(CbtShapeHandle shape_handle, float mass, Vector3 inertia);

BT_API void cbtShapeSetUserPointer(CbtShapeHandle shape_handle, void* user_pointer);
BT_API void* cbtShapeGetUserPointer(CbtShapeHandle shape_handle);
BT_API void cbtShapeSetUserIndex(CbtShapeHandle shape_handle, int slot, int user_index); // slot can be 0 or 1
BT_API int cbtShapeGetUserIndex(CbtShapeHandle shape_handle, int slot); // slot can be 0 or 1

BT_API void cbtShapeBoxCreate(CbtShapeHandle shape_handle, const Vector3 half_extents);
BT_API void cbtShapeBoxGetHalfExtentsWithoutMargin(CbtShapeHandle shape_handle, Vector3 half_extents);
BT_API void cbtShapeBoxGetHalfExtentsWithMargin(CbtShapeHandle shape_handle, Vector3 half_extents);

BT_API void cbtShapeSphereCreate(CbtShapeHandle shape_handle, float radius);
BT_API void cbtShapeSphereSetUnscaledRadius(CbtShapeHandle shape_handle, float radius);
BT_API float cbtShapeSphereGetRadius(CbtShapeHandle shape_handle);

BT_API void cbtShapeCapsuleCreate(CbtShapeHandle shape_handle, float radius, float height, int up_axis);
BT_API int cbtShapeCapsuleGetUpAxis(CbtShapeHandle shape_handle);
BT_API float cbtShapeCapsuleGetHalfHeight(CbtShapeHandle shape_handle);
BT_API float cbtShapeCapsuleGetRadius(CbtShapeHandle shape_handle);

BT_API void cbtShapeCylinderCreate(CbtShapeHandle shape_handle, const Vector3 half_extents, int up_axis);
BT_API void cbtShapeCylinderGetHalfExtentsWithoutMargin(CbtShapeHandle shape_handle, Vector3 half_extents);
BT_API void cbtShapeCylinderGetHalfExtentsWithMargin(CbtShapeHandle shape_handle, Vector3 half_extents);
BT_API int cbtShapeCylinderGetUpAxis(CbtShapeHandle shape_handle);

BT_API void cbtShapeConeCreate(CbtShapeHandle shape_handle, float radius, float height, int up_axis);
BT_API float cbtShapeConeGetRadius(CbtShapeHandle shape_handle);
BT_API float cbtShapeConeGetHeight(CbtShapeHandle shape_handle);
BT_API int cbtShapeConeGetUpAxis(CbtShapeHandle shape_handle);

BT_API void cbtShapeCompoundCreate(
    CbtShapeHandle shape_handle,
    bool enable_dynamic_aabb_tree, // true
    int initial_child_capacity // 0
);
BT_API void cbtShapeCompoundAddChild(
    CbtShapeHandle shape_handle,
    const Vector3 local_transform[4],
    CbtShapeHandle child_shape_handle
);
BT_API void cbtShapeCompoundRemoveChild(CbtShapeHandle shape_handle, CbtShapeHandle child_shape_handle);
BT_API void cbtShapeCompoundRemoveChildByIndex(CbtShapeHandle shape_handle, int child_shape_index);
BT_API int cbtShapeCompoundGetNumChilds(CbtShapeHandle shape_handle);
BT_API CbtShapeHandle cbtShapeCompoundGetChild(CbtShapeHandle shape_handle, int child_shape_index);
BT_API void cbtShapeCompoundGetChildTransform(CbtShapeHandle shape_handle, int child_shape_index, Vector3 transform[4]);

BT_API void cbtShapeTriMeshCreateBegin(CbtShapeHandle shape_handle);
BT_API void cbtShapeTriMeshCreateEnd(CbtShapeHandle shape_handle);
BT_API void cbtShapeTriMeshDestroy(CbtShapeHandle shape_handle);
BT_API void cbtShapeTriMeshAddIndexVertexArray(
    CbtShapeHandle shape_handle,
    int num_triangles,
    const void* triangle_base,
    int triangle_stride,
    int num_vertices,
    const void* vertex_base,
    int vertex_stride
);

//
// Body
//
BT_API CbtBodyHandle cbtBodyAllocate(void);
BT_API void cbtBodyAllocateBatch(unsigned int num, CbtBodyHandle* body_handles);
BT_API void cbtBodyDeallocate(CbtBodyHandle body_handle);
BT_API void cbtBodyDeallocateBatch(unsigned int num, CbtBodyHandle* body_handles);

BT_API void cbtBodyCreate(
    CbtBodyHandle body_handle,
    float mass,
    const Vector3 transform[4],
    CbtShapeHandle shape_handle
);
BT_API void cbtBodyDestroy(CbtBodyHandle body_handle);
BT_API bool cbtBodyIsCreated(CbtBodyHandle body_handle);

BT_API void cbtBodySetShape(CbtBodyHandle body_handle, CbtShapeHandle shape_handle);
BT_API CbtShapeHandle cbtBodyGetShape(CbtBodyHandle body_handle);

BT_API void cbtBodySetRestitution(CbtBodyHandle body_handle, float restitution);

BT_API void cbtBodySetFriction(CbtBodyHandle body_handle, float friction);
BT_API void cbtBodySetRollingFriction(CbtBodyHandle body_handle, float friction);
BT_API void cbtBodySetSpinningFriction(CbtBodyHandle body_handle, float friction);
BT_API void cbtBodySetAnisotropicFriction(CbtBodyHandle body_handle, const Vector3 friction, int mode);

BT_API void cbtBodySetContactStiffnessAndDamping(CbtBodyHandle body_handle, float stiffness, float damping);

BT_API void cbtBodySetMassProps(CbtBodyHandle body_handle, float mass, const Vector3 inertia);

BT_API void cbtBodySetDamping(CbtBodyHandle body_handle, float linear, float angular);

BT_API void cbtBodySetLinearVelocity(CbtBodyHandle body_handle, const Vector3 velocity);
BT_API void cbtBodySetAngularVelocity(CbtBodyHandle body_handle, const Vector3 velocity);

BT_API void cbtBodySetLinearFactor(CbtBodyHandle body_handle, const Vector3 factor);
BT_API void cbtBodySetAngularFactor(CbtBodyHandle body_handle, const Vector3 factor);

BT_API void cbtBodySetGravity(CbtBodyHandle body_handle, const Vector3 gravity);
BT_API void cbtBodyGetGravity(CbtBodyHandle body_handle, Vector3 gravity);

BT_API int cbtBodyGetNumConstraints(CbtBodyHandle body_handle);
BT_API CbtConstraintHandle cbtBodyGetConstraint(CbtBodyHandle body_handle, int index);

BT_API void cbtBodyApplyCentralForce(CbtBodyHandle body_handle, const Vector3 force);
BT_API void cbtBodyApplyCentralImpulse(CbtBodyHandle body_handle, const Vector3 impulse);
BT_API void cbtBodyApplyForce(CbtBodyHandle body_handle, const Vector3 force, const Vector3 rel_pos);
BT_API void cbtBodyApplyImpulse(CbtBodyHandle body_handle, const Vector3 impulse, const Vector3 rel_pos);
BT_API void cbtBodyApplyTorque(CbtBodyHandle body_handle, const Vector3 torque);
BT_API void cbtBodyApplyTorqueImpulse(CbtBodyHandle body_handle, const Vector3 impulse);

BT_API float cbtBodyGetRestitution(CbtBodyHandle body_handle);

BT_API float cbtBodyGetFriction(CbtBodyHandle body_handle);
BT_API float cbtBodyGetRollingFriction(CbtBodyHandle body_handle);
BT_API float cbtBodyGetSpinningFriction(CbtBodyHandle body_handle);
BT_API void cbtBodyGetAnisotropicFriction(CbtBodyHandle body_handle, Vector3 friction);

BT_API float cbtBodyGetContactStiffness(CbtBodyHandle body_handle);
BT_API float cbtBodyGetContactDamping(CbtBodyHandle body_handle);

BT_API float cbtBodyGetMass(CbtBodyHandle body_handle);

BT_API float cbtBodyGetLinearDamping(CbtBodyHandle body_handle);
BT_API float cbtBodyGetAngularDamping(CbtBodyHandle body_handle);

BT_API void cbtBodyGetLinearVelocity(CbtBodyHandle body_handle, Vector3 velocity);
BT_API void cbtBodyGetAngularVelocity(CbtBodyHandle body_handle, Vector3 velocity);

BT_API void cbtBodyGetTotalForce(CbtBodyHandle body_handle, Vector3 force);
BT_API void cbtBodyGetTotalTorque(CbtBodyHandle body_handle, Vector3 torque);

BT_API bool cbtBodyIsStatic(CbtBodyHandle body_handle);
BT_API bool cbtBodyIsKinematic(CbtBodyHandle body_handle);
BT_API bool cbtBodyIsStaticOrKinematic(CbtBodyHandle body_handle);

BT_API float cbtBodyGetDeactivationTime(CbtBodyHandle body_handle);
BT_API void cbtBodySetDeactivationTime(CbtBodyHandle body_handle, float time);
BT_API int cbtBodyGetActivationState(CbtBodyHandle body_handle);
BT_API void cbtBodySetActivationState(CbtBodyHandle body_handle, int state);
BT_API void cbtBodyForceActivationState(CbtBodyHandle body_handle, int state);
BT_API bool cbtBodyIsActive(CbtBodyHandle body_handle);
BT_API bool cbtBodyIsInWorld(CbtBodyHandle body_handle);

BT_API void cbtBodySetUserPointer(CbtBodyHandle body_handle, void* user_pointer);
BT_API void* cbtBodyGetUserPointer(CbtBodyHandle body_handle);
BT_API void cbtBodySetUserIndex(CbtBodyHandle body_handle, int slot, int user_index); // slot can be 0, 1 or 2
BT_API int cbtBodyGetUserIndex(CbtBodyHandle body_handle, int slot); // slot can be 0, 1 or 2

BT_API void cbtBodySetCenterOfMassTransform(CbtBodyHandle body_handle, const Vector3 transform[4]);
BT_API void cbtBodyGetCenterOfMassTransform(CbtBodyHandle body_handle, Vector3 transform[4]);
BT_API void cbtBodyGetCenterOfMassPosition(CbtBodyHandle body_handle, Vector3 position);
BT_API void cbtBodyGetInvCenterOfMassTransform(CbtBodyHandle body_handle, Vector3 transform[4]);
BT_API void cbtBodyGetGraphicsWorldTransform(CbtBodyHandle body_handle, Vector3 transform[4]);

BT_API float cbtBodyGetCcdSweptSphereRadius(CbtBodyHandle body_handle);
BT_API void cbtBodySetCcdSweptSphereRadius(CbtBodyHandle body_handle, float radius);

BT_API float cbtBodyGetCcdMotionThreshold(CbtBodyHandle body_handle);
BT_API void cbtBodySetCcdMotionThreshold(CbtBodyHandle body_handle, float threshold);

//
// Constraints
//
BT_API CbtBodyHandle cbtConGetFixedBody(void);
BT_API void cbtConDestroyFixedBody(void);

BT_API CbtConstraintHandle cbtConAllocate(int con_type);
BT_API void cbtConDeallocate(CbtConstraintHandle con_handle);

BT_API void cbtConDestroy(CbtConstraintHandle con_handle);
BT_API bool cbtConIsCreated(CbtConstraintHandle con_handle);
BT_API int cbtConGetType(CbtConstraintHandle con_handle);

BT_API void cbtConSetParam(CbtConstraintHandle con_handle, int param, float value, int axis /* -1 */);
BT_API float cbtConGetParam(CbtConstraintHandle con_handle, int param, int axis /* -1 */);

BT_API void cbtConSetEnabled(CbtConstraintHandle con_handle, bool enabled);
BT_API bool cbtConIsEnabled(CbtConstraintHandle con_handle);
CbtBodyHandle cbtConGetBodyA(CbtConstraintHandle con_handle);
BT_API CbtBodyHandle cbtConGetBodyB(CbtConstraintHandle con_handle);
BT_API void cbtConSetBreakingImpulseThreshold(CbtConstraintHandle con_handle, float threshold);
BT_API float cbtConGetBreakingImpulseThreshold(CbtConstraintHandle con_handle);

BT_API void cbtConSetDebugDrawSize(CbtConstraintHandle con_handle, float size);
BT_API float cbtConGetDebugDrawSize(CbtConstraintHandle con_handle);

// Point2Point
BT_API void cbtConPoint2PointCreate1(
    CbtConstraintHandle con_handle,
    CbtBodyHandle body_handle_a,
    const Vector3 pivot_a
);
BT_API void cbtConPoint2PointCreate2(
    CbtConstraintHandle con_handle,
    CbtBodyHandle body_handle_a,
    CbtBodyHandle body_handle_b,
    const Vector3 pivot_a,
    const Vector3 pivot_b
);
BT_API void cbtConPoint2PointSetPivotA(CbtConstraintHandle con_handle, const Vector3 pivot);
BT_API void cbtConPoint2PointSetPivotB(CbtConstraintHandle con_handle, const Vector3 pivot);
BT_API void cbtConPoint2PointSetTau(CbtConstraintHandle con_handle, float tau);
BT_API void cbtConPoint2PointSetDamping(CbtConstraintHandle con_handle, float damping);
BT_API void cbtConPoint2PointSetImpulseClamp(CbtConstraintHandle con_handle, float impulse_clamp);

BT_API void cbtConPoint2PointGetPivotA(CbtConstraintHandle con_handle, Vector3 pivot);
BT_API void cbtConPoint2PointGetPivotB(CbtConstraintHandle con_handle, Vector3 pivot);

// Hinge
BT_API void cbtConHingeCreate1(
    CbtConstraintHandle con_handle,
    CbtBodyHandle body_handle_a,
    const Vector3 pivot_a,
    const Vector3 axis_a,
    bool use_reference_frame_a // false
);
BT_API void cbtConHingeCreate2(
    CbtConstraintHandle con_handle,
    CbtBodyHandle body_handle_a,
    CbtBodyHandle body_handle_b,
    const Vector3 pivot_a,
    const Vector3 pivot_b,
    const Vector3 axis_a,
    const Vector3 axis_b,
    bool use_reference_frame_a // false
);
BT_API void cbtConHingeCreate3(
    CbtConstraintHandle con_handle,
    CbtBodyHandle body_handle_a,
    const Vector3 frame_a[4],
    bool use_reference_frame_a // false
);
BT_API void cbtConHingeSetAngularOnly(CbtConstraintHandle con_handle, bool angular_only);
BT_API void cbtConHingeEnableAngularMotor(
    CbtConstraintHandle con_handle,
    bool enable,
    float target_velocity,
    float max_motor_impulse
);
BT_API void cbtConHingeSetLimit(
    CbtConstraintHandle con_handle,
    float low,
    float high,
    float softness, // 0.9
    float bias_factor, // 0.3
    float relaxation_factor // 1.0
);

// Gear
BT_API void cbtConGearCreate(
    CbtConstraintHandle con_handle,
    CbtBodyHandle body_handle_a,
    CbtBodyHandle body_handle_b,
    const Vector3 axis_a,
    const Vector3 axis_b,
    float ratio // 1.0
);
BT_API void cbtConGearSetAxisA(CbtConstraintHandle con_handle, const Vector3 axis);
BT_API void cbtConGearSetAxisB(CbtConstraintHandle con_handle, const Vector3 axis);
BT_API void cbtConGearSetRatio(CbtConstraintHandle con_handle, float ratio);
BT_API void cbtConGearGetAxisA(CbtConstraintHandle con_handle, Vector3 axis);
BT_API void cbtConGearGetAxisB(CbtConstraintHandle con_handle, Vector3 axis);
BT_API float cbtConGearGetRatio(CbtConstraintHandle con_handle);

// Slider
BT_API void cbtConSliderCreate1(
    CbtConstraintHandle con_handle,
    CbtBodyHandle body_handle_b,
    const Vector3 frame_b[4],
    bool use_reference_frame_a
);
BT_API void cbtConSliderCreate2(
    CbtConstraintHandle con_handle,
    CbtBodyHandle body_handle_a,
    CbtBodyHandle body_handle_b,
    const Vector3 frame_a[4],
    const Vector3 frame_b[4],
    bool use_reference_frame_a
);
BT_API void cbtConSliderSetLinearLowerLimit(CbtConstraintHandle con_handle, float limit);
BT_API void cbtConSliderSetLinearUpperLimit(CbtConstraintHandle con_handle, float limit);
BT_API float cbtConSliderGetLinearLowerLimit(CbtConstraintHandle con_handle);
BT_API float cbtConSliderGetLinearUpperLimit(CbtConstraintHandle con_handle);

BT_API void cbtConSliderSetAngularLowerLimit(CbtConstraintHandle con_handle, float limit);
BT_API void cbtConSliderSetAngularUpperLimit(CbtConstraintHandle con_handle, float limit);
BT_API float cbtConSliderGetAngularLowerLimit(CbtConstraintHandle con_handle);
BT_API float cbtConSliderGetAngularUpperLimit(CbtConstraintHandle con_handle);

BT_API void cbtConSliderEnableLinearMotor(
    CbtConstraintHandle con_handle,
    bool enable,
    float target_velocity,
    float max_motor_force
);
BT_API void cbtConSliderEnableAngularMotor(
    CbtConstraintHandle con_handle,
    bool enable,
    float target_velocity,
    float max_force
);
BT_API bool cbtConSliderIsLinearMotorEnabled(CbtConstraintHandle con_handle);
BT_API bool cbtConSliderIsAngularMotorEnabled(CbtConstraintHandle con_handle);

BT_API void cbtConSliderGetAngularMotor(CbtConstraintHandle con_handle, float* target_velocity, float* max_force);

BT_API float cbtConSliderGetLinearPosition(CbtConstraintHandle con_handle);
BT_API float cbtConSliderGetAngularPosition(CbtConstraintHandle con_handle);

// Generic 6Dof Spring Constraint (ver. 2)
BT_API void cbtConD6Spring2Create1(
    CbtConstraintHandle con_handle,
    CbtBodyHandle body_handle_b,
    const Vector3 frame_b[4],
    int rotate_order // CBT_ROTATE_ORDER_XYZ
);
BT_API void cbtConD6Spring2Create2(
    CbtConstraintHandle con_handle,
    CbtBodyHandle body_handle_a,
    CbtBodyHandle body_handle_b,
    const Vector3 frame_a[4],
    const Vector3 frame_b[4],
    int rotate_order // CBT_ROTATE_ORDER_XYZ
);
BT_API void cbtConD6Spring2SetLinearLowerLimit(CbtConstraintHandle con_handle, const Vector3 limit);
BT_API void cbtConD6Spring2SetLinearUpperLimit(CbtConstraintHandle con_handle, const Vector3 limit);
BT_API void cbtConD6Spring2GetLinearLowerLimit(CbtConstraintHandle con_handle, Vector3 limit);
BT_API void cbtConD6Spring2GetLinearUpperLimit(CbtConstraintHandle con_handle, Vector3 limit);

BT_API void cbtConD6Spring2SetAngularLowerLimit(CbtConstraintHandle con_handle, const Vector3 limit);
BT_API void cbtConD6Spring2SetAngularUpperLimit(CbtConstraintHandle con_handle, const Vector3 limit);
BT_API void cbtConD6Spring2GetAngularLowerLimit(CbtConstraintHandle con_handle, Vector3 limit);
BT_API void cbtConD6Spring2GetAngularUpperLimit(CbtConstraintHandle con_handle, Vector3 limit);

// Cone Twist
BT_API void cbtConConeTwistCreate1(
    CbtConstraintHandle con_handle,
    CbtBodyHandle body_handle_a,
    const Vector3 frame_a[4]
);
BT_API void cbtConConeTwistCreate2(
    CbtConstraintHandle con_handle,
    CbtBodyHandle body_handle_a,
    CbtBodyHandle body_handle_b,
    const Vector3 frame_a[4],
    const Vector3 frame_b[4]
);
BT_API void cbtConConeTwistSetLimit(
    CbtConstraintHandle con_handle,
    float swing_span1,
    float swing_span2,
    float twist_span,
    float softness, // 1.0
    float bias_factor, // 0.3
    float relaxation_factor // 1.0
);

#ifdef __cplusplus
}
#endif
