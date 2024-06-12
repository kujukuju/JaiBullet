//
// This file was auto-generated using the following command:
//
// jai ../ ./generate.jai
//
size_t :: u64;


CBT_COLLISION_FILTER_DEFAULT :: 1;
CBT_COLLISION_FILTER_STATIC :: 2;
CBT_COLLISION_FILTER_KINEMATIC :: 4;
CBT_COLLISION_FILTER_DEBRIS :: 8;
CBT_COLLISION_FILTER_SENSOR_TRIGGER :: 16;
CBT_COLLISION_FILTER_CHARACTER :: 32;
CBT_COLLISION_FILTER_ALL :: -1;

CBT_RAYCAST_FLAG_NONE :: 0;
CBT_RAYCAST_FLAG_TRIMESH_SKIP_BACKFACES :: 1;
CBT_RAYCAST_FLAG_TRIMESH_KEEP_UNFLIPPED_NORMALS :: 2;
CBT_RAYCAST_FLAG_USE_SUB_SIMPLEX_CONVEX_TEST :: 4;
CBT_RAYCAST_FLAG_USE_GJK_CONVEX_TEST :: 8;

CBT_ANISOTROPIC_FRICTION_DISABLED :: 0;
CBT_ANISOTROPIC_FRICTION :: 1;
CBT_ANISOTROPIC_ROLLING_FRICTION :: 2;

CBT_SHAPE_TYPE_BOX :: 0;
CBT_SHAPE_TYPE_SPHERE :: 8;
CBT_SHAPE_TYPE_CAPSULE :: 10;
CBT_SHAPE_TYPE_CONE :: 11;
CBT_SHAPE_TYPE_CYLINDER :: 13;
CBT_SHAPE_TYPE_COMPOUND :: 31;
CBT_SHAPE_TYPE_TRIANGLE_MESH :: 21;
CBT_SHAPE_TYPE_CONVEX :: 4;

CBT_CONSTRAINT_TYPE_POINT2POINT :: 3;
CBT_CONSTRAINT_TYPE_HINGE :: 4;
CBT_CONSTRAINT_TYPE_CONETWIST :: 5;
CBT_CONSTRAINT_TYPE_SLIDER :: 7;
CBT_CONSTRAINT_TYPE_GEAR :: 10;
CBT_CONSTRAINT_TYPE_D6_SPRING_2 :: 12;

CBT_CONSTRAINT_PARAM_ERP :: 1;
CBT_CONSTRAINT_PARAM_STOP_ERP :: 2;
CBT_CONSTRAINT_PARAM_CFM :: 3;
CBT_CONSTRAINT_PARAM_STOP_CFM :: 4;

CBT_ACTIVE_TAG :: 1;
CBT_ISLAND_SLEEPING :: 2;
CBT_WANTS_DEACTIVATION :: 3;
CBT_DISABLE_DEACTIVATION :: 4;
CBT_DISABLE_SIMULATION :: 5;

CBT_LINEAR_AXIS_X :: 0;
CBT_LINEAR_AXIS_Y :: 1;
CBT_LINEAR_AXIS_Z :: 2;
CBT_ANGULAR_AXIS_X :: 3;
CBT_ANGULAR_AXIS_Y :: 4;
CBT_ANGULAR_AXIS_Z :: 5;

CBT_ROTATE_ORDER_XYZ :: 0;
CBT_ROTATE_ORDER_XZY :: 1;
CBT_ROTATE_ORDER_YXZ :: 2;
CBT_ROTATE_ORDER_YZX :: 3;
CBT_ROTATE_ORDER_ZXY :: 4;
CBT_ROTATE_ORDER_ZYX :: 5;

CBT_DBGMODE_DISABLED :: -1;
CBT_DBGMODE_NO_DEBUG :: 0;
CBT_DBGMODE_DRAW_WIREFRAME :: 1;
CBT_DBGMODE_DRAW_AABB :: 2;

CbtWorldHandle__ :: struct {
    unused: s32;
}
CbtWorldHandle :: *CbtWorldHandle__;
CbtShapeHandle__ :: struct {
    unused: s32;
}
CbtShapeHandle :: *CbtShapeHandle__;
CbtBodyHandle__ :: struct {
    unused: s32;
}
CbtBodyHandle :: *CbtBodyHandle__;

CbtCharacterControllerHandle__ :: struct {
    unused: s32;
}
CbtCharacterControllerHandle :: *CbtCharacterControllerHandle__;
CbtConstraintHandle__ :: struct {
    unused: s32;
}
CbtConstraintHandle :: *CbtConstraintHandle__;
CbtDebugDrawHandle__ :: struct {
    unused: s32;
}
CbtDebugDrawHandle :: *CbtDebugDrawHandle__;

CbtAlignedAllocFunc :: #type (size: u64, alignment: s32) -> *void #c_call;
CbtAlignedFreeFunc :: #type (memblock: *void) -> void #c_call;
CbtAllocFunc :: #type (size: u64) -> *void #c_call;
CbtFreeFunc :: #type (memblock: *void) -> void #c_call;

cbtCalculateAABB :: (shape_handle: CbtShapeHandle) -> void #foreign bullet3;

cbtAlignedAllocSetCustom :: (alloc: CbtAllocFunc, free: CbtFreeFunc) -> void #foreign bullet3;
cbtAlignedAllocSetCustomAligned :: (alloc: CbtAlignedAllocFunc, free: CbtAlignedFreeFunc) -> void #foreign bullet3;

CbtGetStairHeight :: #type (body_a: CbtBodyHandle) -> float #c_call;
CbtGetCollision :: #type (body_a: CbtBodyHandle, body_b: CbtBodyHandle, normal: *Vector3, impulse: float, data: *void) -> bool #c_call;

CbtDrawLine1Callback :: #type (_context: *void, p0: *Vector3, p1: *Vector3, color: *Vector3) -> void #c_call;

CbtDrawLine2Callback :: #type (_context: *void, p0: *Vector3, p1: *Vector3, color0: *Vector3, color1: *Vector3) -> void #c_call;

CbtDrawContactPointCallback :: #type (_context: *void, point: *Vector3, normal: *Vector3, distance: float, life_time: s32, color: *Vector3) -> void #c_call;

CbtDebugDraw :: struct {
    drawLine1:        CbtDrawLine1Callback;
    drawLine2:        CbtDrawLine2Callback;
    drawContactPoint: CbtDrawContactPointCallback;
    _context:         *void;
}

CbtRayCastResult :: struct {
    hit_normal_world: Vector3;
    hit_point_world:  Vector3;
    hit_fraction:     float;
    body:             CbtBodyHandle;
}

//
// Conversion
//
cbtSetUnits :: (distance: float, time: float) -> void #foreign bullet3;

//
// Task scheduler
//
cbtTaskSchedInit :: () -> void #foreign bullet3;
cbtTaskSchedDeinit :: () -> void #foreign bullet3;
cbtTaskSchedGetNumThreads :: () -> s32 #foreign bullet3;
cbtTaskSchedGetMaxNumThreads :: () -> s32 #foreign bullet3;
cbtTaskSchedSetNumThreads :: (num_threads: s32) -> void #foreign bullet3;

//
// World
//
cbtWorldCreate :: (__args: ..Any) -> CbtWorldHandle #foreign bullet3;
cbtWorldCreateWithStairs :: (get_stair_height: CbtGetStairHeight) -> CbtWorldHandle #foreign bullet3;
cbtWorldDestroy :: (world_handle: CbtWorldHandle) -> void #foreign bullet3;
cbtWorldSetGravity :: (world_handle: CbtWorldHandle, gravity: *Vector3) -> void #foreign bullet3;
cbtWorldSetGravity :: (world_handle: CbtWorldHandle, gravity: Vector3) -> void {
	return cbtWorldSetGravity(world_handle, *gravity);
}

cbtWorldGetGravity :: (world_handle: CbtWorldHandle, gravity: *Vector3) -> void #foreign bullet3;
cbtWorldGetGravity :: (world_handle: CbtWorldHandle, gravity: Vector3) -> void {
	return cbtWorldGetGravity(world_handle, *gravity);
}

cbtWorldStepSimulation :: (world_handle: CbtWorldHandle, time_step: float, max_sub_steps: s32, fixed_time_step: float) -> s32 #foreign bullet3;

cbtWorldAddBody :: (world_handle: CbtWorldHandle, body_handle: CbtBodyHandle) -> void #foreign bullet3;
cbtWorldAddBodyWithGroup :: (world_handle: CbtWorldHandle, body_handle: CbtBodyHandle, group: s32, mask: s32) -> void #foreign bullet3;
cbtWorldAddConstraint :: (world_handle: CbtWorldHandle, con_handle: CbtConstraintHandle, disable_collision_between_linked_bodies: bool) -> void #foreign bullet3;

cbtWorldRemoveBody :: (world_handle: CbtWorldHandle, body_handle: CbtBodyHandle) -> void #foreign bullet3;
cbtWorldRemoveConstraint :: (world_handle: CbtWorldHandle, constraint_handle: CbtConstraintHandle) -> void #foreign bullet3;

cbtWorldGetNumBodies :: (world_handle: CbtWorldHandle) -> s32 #foreign bullet3;
cbtWorldGetNumConstraints :: (world_handle: CbtWorldHandle) -> s32 #foreign bullet3;
cbtWorldGetBody :: (world_handle: CbtWorldHandle, body_index: s32) -> CbtBodyHandle #foreign bullet3;
cbtWorldGetConstraint :: (world_handle: CbtWorldHandle, con_index: s32) -> CbtConstraintHandle #foreign bullet3;

// Returns `true` when hits something, `false` otherwise
cbtWorldRayTestClosest :: (world_handle: CbtWorldHandle, ray_from_world: *Vector3, ray_to_world: *Vector3, collision_filter_group: s32, collision_filter_mask: s32, flags: u32, result: *CbtRayCastResult) -> bool #foreign bullet3;
cbtWorldRayTestClosest :: (world_handle: CbtWorldHandle, ray_from_world: Vector3, ray_to_world: Vector3, collision_filter_group: s32, collision_filter_mask: s32, flags: u32, result: *CbtRayCastResult) -> bool {
	return cbtWorldRayTestClosest(world_handle, *ray_from_world, *ray_to_world, collision_filter_group, collision_filter_mask, flags, result);
}


cbtWorldGetContactPoints :: (world_handle: CbtWorldHandle, get_collision: CbtGetCollision, data: *void) -> void #foreign bullet3;

cbtWorldDebugSetDrawer :: (world_handle: CbtWorldHandle, drawer: *CbtDebugDraw) -> void #foreign bullet3;
cbtWorldDebugSetMode :: (world_handle: CbtWorldHandle, mode: s32) -> void #foreign bullet3;
cbtWorldDebugGetMode :: (world_handle: CbtWorldHandle) -> s32 #foreign bullet3;
cbtWorldDebugDrawAll :: (world_handle: CbtWorldHandle) -> void #foreign bullet3;
cbtWorldDebugDrawLine1 :: (world_handle: CbtWorldHandle, p0: *Vector3, p1: *Vector3, color: *Vector3) -> void #foreign bullet3;
cbtWorldDebugDrawLine1 :: (world_handle: CbtWorldHandle, p0: Vector3, p1: Vector3, color: Vector3) -> void {
	return cbtWorldDebugDrawLine1(world_handle, *p0, *p1, *color);
}


cbtWorldDebugDrawLine2 :: (world_handle: CbtWorldHandle, p0: *Vector3, p1: *Vector3, color0: *Vector3, color1: *Vector3) -> void #foreign bullet3;
cbtWorldDebugDrawLine2 :: (world_handle: CbtWorldHandle, p0: Vector3, p1: Vector3, color0: Vector3, color1: Vector3) -> void {
	return cbtWorldDebugDrawLine2(world_handle, *p0, *p1, *color0, *color1);
}


cbtWorldDebugDrawSphere :: (world_handle: CbtWorldHandle, position: *Vector3, radius: float, color: *Vector3) -> void #foreign bullet3;
cbtWorldDebugDrawSphere :: (world_handle: CbtWorldHandle, position: Vector3, radius: float, color: Vector3) -> void {
	return cbtWorldDebugDrawSphere(world_handle, *position, radius, *color);
}


//
// Shape
//
cbtShapeAllocate :: (shape_type: s32) -> CbtShapeHandle #foreign bullet3;
cbtShapeDeallocate :: (shape_handle: CbtShapeHandle) -> void #foreign bullet3;

cbtShapeDestroy :: (shape_handle: CbtShapeHandle) -> void #foreign bullet3;
cbtShapeIsCreated :: (shape_handle: CbtShapeHandle) -> bool #foreign bullet3;
cbtShapeGetType :: (shape_handle: CbtShapeHandle) -> s32 #foreign bullet3;
cbtShapeSetMargin :: (shape_handle: CbtShapeHandle, margin: float) -> void #foreign bullet3;
cbtShapeGetMargin :: (shape_handle: CbtShapeHandle) -> float #foreign bullet3;
cbtShapeGetAABB :: (shape_handle: CbtShapeHandle, aabb: *AABB3) -> void #foreign bullet3;

cbtShapeIsPolyhedral :: (shape_handle: CbtShapeHandle) -> bool #foreign bullet3;
cbtShapeIsConvex2d :: (shape_handle: CbtShapeHandle) -> bool #foreign bullet3;
cbtShapeIsConvex :: (shape_handle: CbtShapeHandle) -> bool #foreign bullet3;
cbtShapeIsNonMoving :: (shape_handle: CbtShapeHandle) -> bool #foreign bullet3;
cbtShapeIsConcave :: (shape_handle: CbtShapeHandle) -> bool #foreign bullet3;
cbtShapeIsCompound :: (shape_handle: CbtShapeHandle) -> bool #foreign bullet3;

cbtShapeCalculateLocalInertia :: (shape_handle: CbtShapeHandle, mass: float, inertia: *Vector3) -> void #foreign bullet3;
cbtShapeCalculateLocalInertia :: (shape_handle: CbtShapeHandle, mass: float, inertia: Vector3) -> void {
	return cbtShapeCalculateLocalInertia(shape_handle, mass, *inertia);
}


cbtShapeSetUserPointer :: (shape_handle: CbtShapeHandle, user_pointer: *void) -> void #foreign bullet3;
cbtShapeGetUserPointer :: (shape_handle: CbtShapeHandle) -> *void #foreign bullet3;
cbtShapeSetUserIndex :: (shape_handle: CbtShapeHandle, slot: s32, user_index: s32) -> void #foreign bullet3;
cbtShapeGetUserIndex :: (shape_handle: CbtShapeHandle, slot: s32) -> s32 #foreign bullet3;

cbtShapeBoxCreate :: (shape_handle: CbtShapeHandle, half_extents: *Vector3) -> void #foreign bullet3;
cbtShapeBoxCreate :: (shape_handle: CbtShapeHandle, half_extents: Vector3) -> void {
	return cbtShapeBoxCreate(shape_handle, *half_extents);
}

cbtShapeBoxGetHalfExtentsWithoutMargin :: (shape_handle: CbtShapeHandle, half_extents: *Vector3) -> void #foreign bullet3;
cbtShapeBoxGetHalfExtentsWithoutMargin :: (shape_handle: CbtShapeHandle, half_extents: Vector3) -> void {
	return cbtShapeBoxGetHalfExtentsWithoutMargin(shape_handle, *half_extents);
}

cbtShapeBoxGetHalfExtentsWithMargin :: (shape_handle: CbtShapeHandle, half_extents: *Vector3) -> void #foreign bullet3;
cbtShapeBoxGetHalfExtentsWithMargin :: (shape_handle: CbtShapeHandle, half_extents: Vector3) -> void {
	return cbtShapeBoxGetHalfExtentsWithMargin(shape_handle, *half_extents);
}


cbtShapeSphereCreate :: (shape_handle: CbtShapeHandle, radius: float) -> void #foreign bullet3;
cbtShapeSphereSetUnscaledRadius :: (shape_handle: CbtShapeHandle, radius: float) -> void #foreign bullet3;
cbtShapeSphereGetRadius :: (shape_handle: CbtShapeHandle) -> float #foreign bullet3;

cbtShapeCapsuleCreate :: (shape_handle: CbtShapeHandle, radius: float, height: float, up_axis: s32) -> void #foreign bullet3;
cbtShapeCapsuleGetUpAxis :: (shape_handle: CbtShapeHandle) -> s32 #foreign bullet3;
cbtShapeCapsuleGetHalfHeight :: (shape_handle: CbtShapeHandle) -> float #foreign bullet3;
cbtShapeCapsuleGetRadius :: (shape_handle: CbtShapeHandle) -> float #foreign bullet3;

cbtShapeCylinderCreate :: (shape_handle: CbtShapeHandle, half_extents: *Vector3, up_axis: s32) -> void #foreign bullet3;
cbtShapeCylinderCreate :: (shape_handle: CbtShapeHandle, half_extents: Vector3, up_axis: s32) -> void {
	return cbtShapeCylinderCreate(shape_handle, *half_extents, up_axis);
}

cbtShapeCylinderGetHalfExtentsWithoutMargin :: (shape_handle: CbtShapeHandle, half_extents: *Vector3) -> void #foreign bullet3;
cbtShapeCylinderGetHalfExtentsWithoutMargin :: (shape_handle: CbtShapeHandle, half_extents: Vector3) -> void {
	return cbtShapeCylinderGetHalfExtentsWithoutMargin(shape_handle, *half_extents);
}

cbtShapeCylinderGetHalfExtentsWithMargin :: (shape_handle: CbtShapeHandle, half_extents: *Vector3) -> void #foreign bullet3;
cbtShapeCylinderGetHalfExtentsWithMargin :: (shape_handle: CbtShapeHandle, half_extents: Vector3) -> void {
	return cbtShapeCylinderGetHalfExtentsWithMargin(shape_handle, *half_extents);
}

cbtShapeCylinderGetUpAxis :: (shape_handle: CbtShapeHandle) -> s32 #foreign bullet3;

cbtShapeConeCreate :: (shape_handle: CbtShapeHandle, radius: float, height: float, up_axis: s32) -> void #foreign bullet3;
cbtShapeConeGetRadius :: (shape_handle: CbtShapeHandle) -> float #foreign bullet3;
cbtShapeConeGetHeight :: (shape_handle: CbtShapeHandle) -> float #foreign bullet3;
cbtShapeConeGetUpAxis :: (shape_handle: CbtShapeHandle) -> s32 #foreign bullet3;

cbtShapeConvexHullCreate :: (shape_handle: CbtShapeHandle, points: *void, numPoints: s32) -> void #foreign bullet3;
cbtShapeConvexHullOptimize :: (shape_handle: CbtShapeHandle) -> void #foreign bullet3;
cbtShapeConvexHullGetPoints :: (shape_handle: CbtShapeHandle, points: **void, numPoints: *s32) -> void #foreign bullet3;

cbtShapeCompoundCreate :: (shape_handle: CbtShapeHandle, enable_dynamic_aabb_tree: bool, initial_child_capacity: s32) -> void #foreign bullet3;

cbtShapeCompoundAddChild :: (shape_handle: CbtShapeHandle, local_transform: *[4] Vector3, child_shape_handle: CbtShapeHandle) -> void #foreign bullet3;
cbtShapeCompoundAddChild :: (shape_handle: CbtShapeHandle, local_transform: [4] Vector3, child_shape_handle: CbtShapeHandle) -> void {
	return cbtShapeCompoundAddChild(shape_handle, *local_transform, child_shape_handle);
}


cbtShapeCompoundRemoveChild :: (shape_handle: CbtShapeHandle, child_shape_handle: CbtShapeHandle) -> void #foreign bullet3;
cbtShapeCompoundRemoveChildByIndex :: (shape_handle: CbtShapeHandle, child_shape_index: s32) -> void #foreign bullet3;
cbtShapeCompoundGetNumChilds :: (shape_handle: CbtShapeHandle) -> s32 #foreign bullet3;
cbtShapeCompoundGetChild :: (shape_handle: CbtShapeHandle, child_shape_index: s32) -> CbtShapeHandle #foreign bullet3;
cbtShapeCompoundGetChildTransform :: (shape_handle: CbtShapeHandle, child_shape_index: s32, transform: *[4] Vector3) -> void #foreign bullet3;
cbtShapeCompoundGetChildTransform :: (shape_handle: CbtShapeHandle, child_shape_index: s32, transform: [4] Vector3) -> void {
	return cbtShapeCompoundGetChildTransform(shape_handle, child_shape_index, *transform);
}


cbtShapeTriMeshCreateBegin :: (shape_handle: CbtShapeHandle) -> void #foreign bullet3;
cbtShapeTriMeshCreateEnd :: (shape_handle: CbtShapeHandle) -> void #foreign bullet3;
cbtShapeTriMeshDestroy :: (shape_handle: CbtShapeHandle) -> void #foreign bullet3;
cbtShapeTriMeshAddIndexVertexArray :: (shape_handle: CbtShapeHandle, num_triangles: s32, triangle_base: *void, triangle_stride: s32, num_vertices: s32, vertex_base: *void, vertex_stride: s32) -> void #foreign bullet3;

//
// Body
//
cbtBodyAllocate :: () -> CbtBodyHandle #foreign bullet3;
cbtBodyAllocateBatch :: (num: u32, body_handles: *CbtBodyHandle) -> void #foreign bullet3;
cbtBodyDeallocate :: (body_handle: CbtBodyHandle) -> void #foreign bullet3;
cbtBodyDeallocateBatch :: (num: u32, body_handles: *CbtBodyHandle) -> void #foreign bullet3;

cbtBodyCreate :: (body_handle: CbtBodyHandle, mass: float, transform: *[4] Vector3, shape_handle: CbtShapeHandle) -> void #foreign bullet3;
cbtBodyCreate :: (body_handle: CbtBodyHandle, mass: float, transform: [4] Vector3, shape_handle: CbtShapeHandle) -> void {
	return cbtBodyCreate(body_handle, mass, *transform, shape_handle);
}


cbtBodyDestroy :: (body_handle: CbtBodyHandle) -> void #foreign bullet3;
cbtBodyIsCreated :: (body_handle: CbtBodyHandle) -> bool #foreign bullet3;

cbtBodySetShape :: (body_handle: CbtBodyHandle, shape_handle: CbtShapeHandle) -> void #foreign bullet3;
cbtBodyGetShape :: (body_handle: CbtBodyHandle) -> CbtShapeHandle #foreign bullet3;

cbtBodySetRestitution :: (body_handle: CbtBodyHandle, restitution: float) -> void #foreign bullet3;

cbtBodySetFriction :: (body_handle: CbtBodyHandle, friction: float) -> void #foreign bullet3;
cbtBodySetRollingFriction :: (body_handle: CbtBodyHandle, friction: float) -> void #foreign bullet3;
cbtBodySetSpinningFriction :: (body_handle: CbtBodyHandle, friction: float) -> void #foreign bullet3;
cbtBodySetAnisotropicFriction :: (body_handle: CbtBodyHandle, friction: *Vector3, mode: s32) -> void #foreign bullet3;
cbtBodySetAnisotropicFriction :: (body_handle: CbtBodyHandle, friction: Vector3, mode: s32) -> void {
	return cbtBodySetAnisotropicFriction(body_handle, *friction, mode);
}


cbtBodySetContactStiffnessAndDamping :: (body_handle: CbtBodyHandle, stiffness: float, damping: float) -> void #foreign bullet3;

cbtBodySetMassProps :: (body_handle: CbtBodyHandle, mass: float, inertia: *Vector3) -> void #foreign bullet3;
cbtBodySetMassProps :: (body_handle: CbtBodyHandle, mass: float, inertia: Vector3) -> void {
	return cbtBodySetMassProps(body_handle, mass, *inertia);
}


cbtBodySetDamping :: (body_handle: CbtBodyHandle, linear: float, angular: float) -> void #foreign bullet3;

cbtBodySetLinearVelocity :: (body_handle: CbtBodyHandle, velocity: *Vector3) -> void #foreign bullet3;
cbtBodySetLinearVelocity :: (body_handle: CbtBodyHandle, velocity: Vector3) -> void {
	return cbtBodySetLinearVelocity(body_handle, *velocity);
}

cbtBodySetAngularVelocity :: (body_handle: CbtBodyHandle, velocity: *Vector3) -> void #foreign bullet3;
cbtBodySetAngularVelocity :: (body_handle: CbtBodyHandle, velocity: Vector3) -> void {
	return cbtBodySetAngularVelocity(body_handle, *velocity);
}


cbtBodySetLinearFactor :: (body_handle: CbtBodyHandle, factor: *Vector3) -> void #foreign bullet3;
cbtBodySetLinearFactor :: (body_handle: CbtBodyHandle, factor: Vector3) -> void {
	return cbtBodySetLinearFactor(body_handle, *factor);
}

cbtBodySetAngularFactor :: (body_handle: CbtBodyHandle, factor: *Vector3) -> void #foreign bullet3;
cbtBodySetAngularFactor :: (body_handle: CbtBodyHandle, factor: Vector3) -> void {
	return cbtBodySetAngularFactor(body_handle, *factor);
}


cbtBodySetGravity :: (body_handle: CbtBodyHandle, gravity: *Vector3) -> void #foreign bullet3;
cbtBodySetGravity :: (body_handle: CbtBodyHandle, gravity: Vector3) -> void {
	return cbtBodySetGravity(body_handle, *gravity);
}

cbtBodyGetGravity :: (body_handle: CbtBodyHandle, gravity: *Vector3) -> void #foreign bullet3;
cbtBodyGetGravity :: (body_handle: CbtBodyHandle, gravity: Vector3) -> void {
	return cbtBodyGetGravity(body_handle, *gravity);
}


cbtBodyGetNumConstraints :: (body_handle: CbtBodyHandle) -> s32 #foreign bullet3;
cbtBodyGetConstraint :: (body_handle: CbtBodyHandle, index: s32) -> CbtConstraintHandle #foreign bullet3;

cbtBodyApplyCentralForce :: (body_handle: CbtBodyHandle, force: *Vector3) -> void #foreign bullet3;
cbtBodyApplyCentralForce :: (body_handle: CbtBodyHandle, force: Vector3) -> void {
	return cbtBodyApplyCentralForce(body_handle, *force);
}

cbtBodyApplyCentralImpulse :: (body_handle: CbtBodyHandle, impulse: *Vector3) -> void #foreign bullet3;
cbtBodyApplyCentralImpulse :: (body_handle: CbtBodyHandle, impulse: Vector3) -> void {
	return cbtBodyApplyCentralImpulse(body_handle, *impulse);
}

cbtBodyApplyForce :: (body_handle: CbtBodyHandle, force: *Vector3, rel_pos: *Vector3) -> void #foreign bullet3;
cbtBodyApplyForce :: (body_handle: CbtBodyHandle, force: Vector3, rel_pos: Vector3) -> void {
	return cbtBodyApplyForce(body_handle, *force, *rel_pos);
}

cbtBodyApplyImpulse :: (body_handle: CbtBodyHandle, impulse: *Vector3, rel_pos: *Vector3) -> void #foreign bullet3;
cbtBodyApplyImpulse :: (body_handle: CbtBodyHandle, impulse: Vector3, rel_pos: Vector3) -> void {
	return cbtBodyApplyImpulse(body_handle, *impulse, *rel_pos);
}

cbtBodyApplyTorque :: (body_handle: CbtBodyHandle, torque: *Vector3) -> void #foreign bullet3;
cbtBodyApplyTorque :: (body_handle: CbtBodyHandle, torque: Vector3) -> void {
	return cbtBodyApplyTorque(body_handle, *torque);
}

cbtBodyApplyTorqueImpulse :: (body_handle: CbtBodyHandle, impulse: *Vector3) -> void #foreign bullet3;
cbtBodyApplyTorqueImpulse :: (body_handle: CbtBodyHandle, impulse: Vector3) -> void {
	return cbtBodyApplyTorqueImpulse(body_handle, *impulse);
}


cbtBodyGetRestitution :: (body_handle: CbtBodyHandle) -> float #foreign bullet3;

cbtBodyGetFriction :: (body_handle: CbtBodyHandle) -> float #foreign bullet3;
cbtBodyGetRollingFriction :: (body_handle: CbtBodyHandle) -> float #foreign bullet3;
cbtBodyGetSpinningFriction :: (body_handle: CbtBodyHandle) -> float #foreign bullet3;
cbtBodyGetAnisotropicFriction :: (body_handle: CbtBodyHandle, friction: *Vector3) -> void #foreign bullet3;
cbtBodyGetAnisotropicFriction :: (body_handle: CbtBodyHandle, friction: Vector3) -> void {
	return cbtBodyGetAnisotropicFriction(body_handle, *friction);
}


cbtBodyGetContactStiffness :: (body_handle: CbtBodyHandle) -> float #foreign bullet3;
cbtBodyGetContactDamping :: (body_handle: CbtBodyHandle) -> float #foreign bullet3;

cbtBodyGetMass :: (body_handle: CbtBodyHandle) -> float #foreign bullet3;

cbtBodyGetLinearDamping :: (body_handle: CbtBodyHandle) -> float #foreign bullet3;
cbtBodyGetAngularDamping :: (body_handle: CbtBodyHandle) -> float #foreign bullet3;

cbtBodyGetLinearVelocity :: (body_handle: CbtBodyHandle, velocity: *Vector3) -> void #foreign bullet3;
cbtBodyGetLinearVelocity :: (body_handle: CbtBodyHandle, velocity: Vector3) -> void {
	return cbtBodyGetLinearVelocity(body_handle, *velocity);
}

cbtBodyGetAngularVelocity :: (body_handle: CbtBodyHandle, velocity: *Vector3) -> void #foreign bullet3;
cbtBodyGetAngularVelocity :: (body_handle: CbtBodyHandle, velocity: Vector3) -> void {
	return cbtBodyGetAngularVelocity(body_handle, *velocity);
}


cbtBodyGetTotalForce :: (body_handle: CbtBodyHandle, force: *Vector3) -> void #foreign bullet3;
cbtBodyGetTotalForce :: (body_handle: CbtBodyHandle, force: Vector3) -> void {
	return cbtBodyGetTotalForce(body_handle, *force);
}

cbtBodyGetTotalTorque :: (body_handle: CbtBodyHandle, torque: *Vector3) -> void #foreign bullet3;
cbtBodyGetTotalTorque :: (body_handle: CbtBodyHandle, torque: Vector3) -> void {
	return cbtBodyGetTotalTorque(body_handle, *torque);
}


cbtBodyIsStatic :: (body_handle: CbtBodyHandle) -> bool #foreign bullet3;
cbtBodyIsKinematic :: (body_handle: CbtBodyHandle) -> bool #foreign bullet3;
cbtBodyIsStaticOrKinematic :: (body_handle: CbtBodyHandle) -> bool #foreign bullet3;

cbtBodyGetDeactivationTime :: (body_handle: CbtBodyHandle) -> float #foreign bullet3;
cbtBodySetDeactivationTime :: (body_handle: CbtBodyHandle, time: float) -> void #foreign bullet3;
cbtBodyGetActivationState :: (body_handle: CbtBodyHandle) -> s32 #foreign bullet3;
cbtBodySetActivationState :: (body_handle: CbtBodyHandle, state: s32) -> void #foreign bullet3;
cbtBodyForceActivationState :: (body_handle: CbtBodyHandle, state: s32) -> void #foreign bullet3;
cbtBodyIsActive :: (body_handle: CbtBodyHandle) -> bool #foreign bullet3;
cbtBodyIsInWorld :: (body_handle: CbtBodyHandle) -> bool #foreign bullet3;

cbtBodySetUserPointer :: (body_handle: CbtBodyHandle, user_pointer: *void) -> void #foreign bullet3;
cbtBodyGetUserPointer :: (body_handle: CbtBodyHandle) -> *void #foreign bullet3;
cbtBodySetUserIndex :: (body_handle: CbtBodyHandle, slot: s32, user_index: s32) -> void #foreign bullet3;
cbtBodyGetUserIndex :: (body_handle: CbtBodyHandle, slot: s32) -> s32 #foreign bullet3;

cbtBodySetCenterOfMassTransform :: (body_handle: CbtBodyHandle, transform: *[4] Vector3) -> void #foreign bullet3;
cbtBodySetCenterOfMassTransform :: (body_handle: CbtBodyHandle, transform: [4] Vector3) -> void {
	return cbtBodySetCenterOfMassTransform(body_handle, *transform);
}

cbtBodyGetCenterOfMassTransform :: (body_handle: CbtBodyHandle, transform: *[4] Vector3) -> void #foreign bullet3;
cbtBodyGetCenterOfMassTransform :: (body_handle: CbtBodyHandle, transform: [4] Vector3) -> void {
	return cbtBodyGetCenterOfMassTransform(body_handle, *transform);
}

cbtBodySetCenterOfMassPosition :: (body_handle: CbtBodyHandle, position: *Vector3) -> void #foreign bullet3;
cbtBodySetCenterOfMassPosition :: (body_handle: CbtBodyHandle, position: Vector3) -> void {
	return cbtBodySetCenterOfMassPosition(body_handle, *position);
}

cbtBodyGetCenterOfMassPosition :: (body_handle: CbtBodyHandle, position: *Vector3) -> void #foreign bullet3;
cbtBodyGetCenterOfMassPosition :: (body_handle: CbtBodyHandle, position: Vector3) -> void {
	return cbtBodyGetCenterOfMassPosition(body_handle, *position);
}

cbtBodyGetInvCenterOfMassTransform :: (body_handle: CbtBodyHandle, transform: *[4] Vector3) -> void #foreign bullet3;
cbtBodyGetInvCenterOfMassTransform :: (body_handle: CbtBodyHandle, transform: [4] Vector3) -> void {
	return cbtBodyGetInvCenterOfMassTransform(body_handle, *transform);
}

cbtBodyGetGraphicsWorldTransform :: (body_handle: CbtBodyHandle, transform: *[4] Vector3) -> void #foreign bullet3;
cbtBodyGetGraphicsWorldTransform :: (body_handle: CbtBodyHandle, transform: [4] Vector3) -> void {
	return cbtBodyGetGraphicsWorldTransform(body_handle, *transform);
}


cbtBodyGetCcdSweptSphereRadius :: (body_handle: CbtBodyHandle) -> float #foreign bullet3;
cbtBodySetCcdSweptSphereRadius :: (body_handle: CbtBodyHandle, radius: float) -> void #foreign bullet3;

cbtBodyGetCcdMotionThreshold :: (body_handle: CbtBodyHandle) -> float #foreign bullet3;
cbtBodySetCcdMotionThreshold :: (body_handle: CbtBodyHandle, threshold: float) -> void #foreign bullet3;

cbtBodyGetCollisionFlags :: (body_handle: CbtBodyHandle) -> s32 #foreign bullet3;
cbtBodyAddCollisionFlag :: (body_handle: CbtBodyHandle, flag: s32) -> void #foreign bullet3;
cbtBodyRemoveCollisionFlag :: (body_handle: CbtBodyHandle, flag: s32) -> void #foreign bullet3;

//
// Kinematic Character
//
cbtCharacterControllerAllocate :: () -> CbtCharacterControllerHandle #foreign bullet3;
cbtCharacterControllerDeallocate :: (character_handle: CbtCharacterControllerHandle) -> void #foreign bullet3;

cbtCharacterControllerCreate :: (character_handle: CbtCharacterControllerHandle, shape_handle: CbtShapeHandle, stepHeight: float, up: *Vector3) -> void #foreign bullet3;
cbtCharacterControllerCreate :: (character_handle: CbtCharacterControllerHandle, shape_handle: CbtShapeHandle, stepHeight: float, up: Vector3) -> void {
	return cbtCharacterControllerCreate(character_handle, shape_handle, stepHeight, *up);
}

cbtCharacterControllerDestroy :: (character_handle: CbtCharacterControllerHandle) -> void #foreign bullet3;
cbtCharacterControllerIsCreated :: (character_handle: CbtCharacterControllerHandle) -> bool #foreign bullet3;

cbtCharacterControllerSetLinearVelocity :: (character_handle: CbtCharacterControllerHandle, velocity: *Vector3) -> void #foreign bullet3;
cbtCharacterControllerSetLinearVelocity :: (character_handle: CbtCharacterControllerHandle, velocity: Vector3) -> void {
	return cbtCharacterControllerSetLinearVelocity(character_handle, *velocity);
}

cbtCharacterControllerGetLinearVelocity :: (character_handle: CbtCharacterControllerHandle, velocity: *Vector3) -> void #foreign bullet3;
cbtCharacterControllerGetLinearVelocity :: (character_handle: CbtCharacterControllerHandle, velocity: Vector3) -> void {
	return cbtCharacterControllerGetLinearVelocity(character_handle, *velocity);
}

cbtCharacterControllerSetStepHeight :: (character_handle: CbtCharacterControllerHandle, stepHeight: float) -> void #foreign bullet3;
cbtCharacterControllerGetStepHeight :: (character_handle: CbtCharacterControllerHandle) -> float #foreign bullet3;
cbtCharacterControllerOnGround :: (character_handle: CbtCharacterControllerHandle) -> bool #foreign bullet3;
cbtCharacterControllerApplyImpulse :: (character_handle: CbtCharacterControllerHandle, impulse: *Vector3) -> void #foreign bullet3;
cbtCharacterControllerSetGravity :: (character_handle: CbtCharacterControllerHandle, gravity: *Vector3) -> void #foreign bullet3;
cbtCharacterControllerGetGravity :: (character_handle: CbtCharacterControllerHandle, gravity: *Vector3) -> void #foreign bullet3;
cbtCharacterControllerSetMaxSlope :: (character_handle: CbtCharacterControllerHandle, radians: float) -> void #foreign bullet3;
cbtCharacterControllerGetMaxSlope :: (character_handle: CbtCharacterControllerHandle) -> float #foreign bullet3;
cbtCharacterControllerSetMaxPenetrationDepth :: (character_handle: CbtCharacterControllerHandle, d: float) -> void #foreign bullet3;
cbtCharacterControllerGetMaxPenetrationDepth :: (character_handle: CbtCharacterControllerHandle) -> float #foreign bullet3;

//
// Constraints
//
cbtConGetFixedBody :: () -> CbtBodyHandle #foreign bullet3;
cbtConDestroyFixedBody :: () -> void #foreign bullet3;

cbtConAllocate :: (con_type: s32) -> CbtConstraintHandle #foreign bullet3;
cbtConDeallocate :: (con_handle: CbtConstraintHandle) -> void #foreign bullet3;

cbtConDestroy :: (con_handle: CbtConstraintHandle) -> void #foreign bullet3;
cbtConIsCreated :: (con_handle: CbtConstraintHandle) -> bool #foreign bullet3;
cbtConGetType :: (con_handle: CbtConstraintHandle) -> s32 #foreign bullet3;

cbtConSetParam :: (con_handle: CbtConstraintHandle, param: s32, value: float, axis: s32) -> void #foreign bullet3;
cbtConGetParam :: (con_handle: CbtConstraintHandle, param: s32, axis: s32) -> float #foreign bullet3;

cbtConSetEnabled :: (con_handle: CbtConstraintHandle, enabled: bool) -> void #foreign bullet3;
cbtConIsEnabled :: (con_handle: CbtConstraintHandle) -> bool #foreign bullet3;
cbtConGetBodyA :: (con_handle: CbtConstraintHandle) -> CbtBodyHandle #foreign bullet3;
cbtConGetBodyB :: (con_handle: CbtConstraintHandle) -> CbtBodyHandle #foreign bullet3;
cbtConSetBreakingImpulseThreshold :: (con_handle: CbtConstraintHandle, threshold: float) -> void #foreign bullet3;
cbtConGetBreakingImpulseThreshold :: (con_handle: CbtConstraintHandle) -> float #foreign bullet3;

cbtConSetDebugDrawSize :: (con_handle: CbtConstraintHandle, size: float) -> void #foreign bullet3;
cbtConGetDebugDrawSize :: (con_handle: CbtConstraintHandle) -> float #foreign bullet3;

// Point2Point
cbtConPoint2PointCreate1 :: (con_handle: CbtConstraintHandle, body_handle_a: CbtBodyHandle, pivot_a: *Vector3) -> void #foreign bullet3;

cbtConPoint2PointCreate2 :: (con_handle: CbtConstraintHandle, body_handle_a: CbtBodyHandle, body_handle_b: CbtBodyHandle, pivot_a: *Vector3, pivot_b: *Vector3) -> void #foreign bullet3;

cbtConPoint2PointSetPivotA :: (con_handle: CbtConstraintHandle, pivot: *Vector3) -> void #foreign bullet3;
cbtConPoint2PointSetPivotB :: (con_handle: CbtConstraintHandle, pivot: *Vector3) -> void #foreign bullet3;
cbtConPoint2PointSetTau :: (con_handle: CbtConstraintHandle, tau: float) -> void #foreign bullet3;
cbtConPoint2PointSetDamping :: (con_handle: CbtConstraintHandle, damping: float) -> void #foreign bullet3;
cbtConPoint2PointSetImpulseClamp :: (con_handle: CbtConstraintHandle, impulse_clamp: float) -> void #foreign bullet3;

cbtConPoint2PointGetPivotA :: (con_handle: CbtConstraintHandle, pivot: *Vector3) -> void #foreign bullet3;
cbtConPoint2PointGetPivotB :: (con_handle: CbtConstraintHandle, pivot: *Vector3) -> void #foreign bullet3;

// Hinge
cbtConHingeCreate1 :: (con_handle: CbtConstraintHandle, body_handle_a: CbtBodyHandle, pivot_a: *Vector3, axis_a: *Vector3, use_reference_frame_a: bool) -> void #foreign bullet3;

cbtConHingeCreate2 :: (con_handle: CbtConstraintHandle, body_handle_a: CbtBodyHandle, body_handle_b: CbtBodyHandle, pivot_a: *Vector3, pivot_b: *Vector3, axis_a: *Vector3, axis_b: *Vector3, use_reference_frame_a: bool) -> void #foreign bullet3;

cbtConHingeCreate3 :: (con_handle: CbtConstraintHandle, body_handle_a: CbtBodyHandle, frame_a: *[4] Vector3, use_reference_frame_a: bool) -> void #foreign bullet3;

cbtConHingeSetAngularOnly :: (con_handle: CbtConstraintHandle, angular_only: bool) -> void #foreign bullet3;
cbtConHingeEnableAngularMotor :: (con_handle: CbtConstraintHandle, enable: bool, target_velocity: float, max_motor_impulse: float) -> void #foreign bullet3;

cbtConHingeSetLimit :: (con_handle: CbtConstraintHandle, low: float, high: float, softness: float, bias_factor: float, relaxation_factor: float) -> void #foreign bullet3;

// Gear
cbtConGearCreate :: (con_handle: CbtConstraintHandle, body_handle_a: CbtBodyHandle, body_handle_b: CbtBodyHandle, axis_a: *Vector3, axis_b: *Vector3, ratio: float) -> void #foreign bullet3;

cbtConGearSetAxisA :: (con_handle: CbtConstraintHandle, axis: *Vector3) -> void #foreign bullet3;
cbtConGearSetAxisB :: (con_handle: CbtConstraintHandle, axis: *Vector3) -> void #foreign bullet3;
cbtConGearSetRatio :: (con_handle: CbtConstraintHandle, ratio: float) -> void #foreign bullet3;
cbtConGearGetAxisA :: (con_handle: CbtConstraintHandle, axis: *Vector3) -> void #foreign bullet3;
cbtConGearGetAxisB :: (con_handle: CbtConstraintHandle, axis: *Vector3) -> void #foreign bullet3;
cbtConGearGetRatio :: (con_handle: CbtConstraintHandle) -> float #foreign bullet3;

// Slider
cbtConSliderCreate1 :: (con_handle: CbtConstraintHandle, body_handle_b: CbtBodyHandle, frame_b: *[4] Vector3, use_reference_frame_a: bool) -> void #foreign bullet3;

cbtConSliderCreate2 :: (con_handle: CbtConstraintHandle, body_handle_a: CbtBodyHandle, body_handle_b: CbtBodyHandle, frame_a: *[4] Vector3, frame_b: *[4] Vector3, use_reference_frame_a: bool) -> void #foreign bullet3;

cbtConSliderSetLinearLowerLimit :: (con_handle: CbtConstraintHandle, limit: float) -> void #foreign bullet3;
cbtConSliderSetLinearUpperLimit :: (con_handle: CbtConstraintHandle, limit: float) -> void #foreign bullet3;
cbtConSliderGetLinearLowerLimit :: (con_handle: CbtConstraintHandle) -> float #foreign bullet3;
cbtConSliderGetLinearUpperLimit :: (con_handle: CbtConstraintHandle) -> float #foreign bullet3;

cbtConSliderSetAngularLowerLimit :: (con_handle: CbtConstraintHandle, limit: float) -> void #foreign bullet3;
cbtConSliderSetAngularUpperLimit :: (con_handle: CbtConstraintHandle, limit: float) -> void #foreign bullet3;
cbtConSliderGetAngularLowerLimit :: (con_handle: CbtConstraintHandle) -> float #foreign bullet3;
cbtConSliderGetAngularUpperLimit :: (con_handle: CbtConstraintHandle) -> float #foreign bullet3;

cbtConSliderEnableLinearMotor :: (con_handle: CbtConstraintHandle, enable: bool, target_velocity: float, max_motor_force: float) -> void #foreign bullet3;

cbtConSliderEnableAngularMotor :: (con_handle: CbtConstraintHandle, enable: bool, target_velocity: float, max_force: float) -> void #foreign bullet3;

cbtConSliderIsLinearMotorEnabled :: (con_handle: CbtConstraintHandle) -> bool #foreign bullet3;
cbtConSliderIsAngularMotorEnabled :: (con_handle: CbtConstraintHandle) -> bool #foreign bullet3;

cbtConSliderGetAngularMotor :: (con_handle: CbtConstraintHandle, target_velocity: *float, max_force: *float) -> void #foreign bullet3;

cbtConSliderGetLinearPosition :: (con_handle: CbtConstraintHandle) -> float #foreign bullet3;
cbtConSliderGetAngularPosition :: (con_handle: CbtConstraintHandle) -> float #foreign bullet3;

// Generic 6Dof Spring Constraint (ver. 2)
cbtConD6Spring2Create1 :: (con_handle: CbtConstraintHandle, body_handle_b: CbtBodyHandle, frame_b: *[4] Vector3, rotate_order: s32) -> void #foreign bullet3;

cbtConD6Spring2Create2 :: (con_handle: CbtConstraintHandle, body_handle_a: CbtBodyHandle, body_handle_b: CbtBodyHandle, frame_a: *[4] Vector3, frame_b: *[4] Vector3, rotate_order: s32) -> void #foreign bullet3;

cbtConD6Spring2SetLinearLowerLimit :: (con_handle: CbtConstraintHandle, limit: *Vector3) -> void #foreign bullet3;
cbtConD6Spring2SetLinearUpperLimit :: (con_handle: CbtConstraintHandle, limit: *Vector3) -> void #foreign bullet3;
cbtConD6Spring2GetLinearLowerLimit :: (con_handle: CbtConstraintHandle, limit: *Vector3) -> void #foreign bullet3;
cbtConD6Spring2GetLinearUpperLimit :: (con_handle: CbtConstraintHandle, limit: *Vector3) -> void #foreign bullet3;

cbtConD6Spring2SetAngularLowerLimit :: (con_handle: CbtConstraintHandle, limit: *Vector3) -> void #foreign bullet3;
cbtConD6Spring2SetAngularUpperLimit :: (con_handle: CbtConstraintHandle, limit: *Vector3) -> void #foreign bullet3;
cbtConD6Spring2GetAngularLowerLimit :: (con_handle: CbtConstraintHandle, limit: *Vector3) -> void #foreign bullet3;
cbtConD6Spring2GetAngularUpperLimit :: (con_handle: CbtConstraintHandle, limit: *Vector3) -> void #foreign bullet3;

// Cone Twist
cbtConConeTwistCreate1 :: (con_handle: CbtConstraintHandle, body_handle_a: CbtBodyHandle, frame_a: *[4] Vector3) -> void #foreign bullet3;

cbtConConeTwistCreate2 :: (con_handle: CbtConstraintHandle, body_handle_a: CbtBodyHandle, body_handle_b: CbtBodyHandle, frame_a: *[4] Vector3, frame_b: *[4] Vector3) -> void #foreign bullet3;

cbtConConeTwistSetLimit :: (con_handle: CbtConstraintHandle, swing_span1: float, swing_span2: float, twist_span: float, softness: float, bias_factor: float, relaxation_factor: float) -> void #foreign bullet3;

#scope_file

bullet3 :: #library "./bullet3";
#import "Math";
AABB3 :: struct {
    lower: Vector3;
    upper: Vector3;
}
