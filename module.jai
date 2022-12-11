//
// This file was auto-generated using the following command:
//
// jai ./generate.jai
//



B3_USE_CLEW :: 1;
BT_SHARED :: 1;
DEACTIVATION_ENABLE :: 3;

DEACTIVATION_DISABLE :: 4;

// copied from btBroadphaseProxy.h so C app doesn't have to import
// bullet c++ headers
//
// TODO !!! find some better way!
// 
// NB will need to recopy this if it changes in bullet - not ideal...
// have to rename enums so no duplicate definition for C++ "wrapper"
/** \cond HIDDEN */
BroadphaseNativeTypesCOPY :: enum s32 {
    BOX_SHAPE_PROXYTYPE                    :: 0;
    TRIANGLE_SHAPE_PROXYTYPE               :: 1;
    TETRAHEDRAL_SHAPE_PROXYTYPE            :: 2;
    CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE    :: 3;
    CONVEX_HULL_SHAPE_PROXYTYPE            :: 4;
    CONVEX_POINT_CLOUD_SHAPE_PROXYTYPE     :: 5;
    CUSTOM_POLYHEDRAL_SHAPE_TYPE           :: 6;

    IMPLICIT_CONVEX_SHAPES_START_HERE      :: 7;
    SPHERE_SHAPE_PROXYTYPE                 :: 8;
    MULTI_SPHERE_SHAPE_PROXYTYPE           :: 9;
    CAPSULE_SHAPE_PROXYTYPE                :: 10;
    CONE_SHAPE_PROXYTYPE                   :: 11;
    CONVEX_SHAPE_PROXYTYPE                 :: 12;
    CYLINDER_SHAPE_PROXYTYPE               :: 13;
    UNIFORM_SCALING_SHAPE_PROXYTYPE        :: 14;
    MINKOWSKI_SUM_SHAPE_PROXYTYPE          :: 15;
    MINKOWSKI_DIFFERENCE_SHAPE_PROXYTYPE   :: 16;
    BOX_2D_SHAPE_PROXYTYPE                 :: 17;
    CONVEX_2D_SHAPE_PROXYTYPE              :: 18;
    CUSTOM_CONVEX_SHAPE_TYPE               :: 19;

    CONCAVE_SHAPES_START_HERE              :: 20;

    TRIANGLE_MESH_SHAPE_PROXYTYPE          :: 21;
    SCALED_TRIANGLE_MESH_SHAPE_PROXYTYPE   :: 22;

    FAST_CONCAVE_MESH_PROXYTYPE            :: 23;

    TERRAIN_SHAPE_PROXYTYPE                :: 24;

    GIMPACT_SHAPE_PROXYTYPE                :: 25;

    MULTIMATERIAL_TRIANGLE_MESH_PROXYTYPE  :: 26;

    EMPTY_SHAPE_PROXYTYPE                  :: 27;
    STATIC_PLANE_PROXYTYPE                 :: 28;
    CUSTOM_CONCAVE_SHAPE_TYPE              :: 29;
    SDF_SHAPE_PROXYTYPE                    :: 29;
    CONCAVE_SHAPES_END_HERE                :: 30;

    COMPOUND_SHAPE_PROXYTYPE               :: 31;

    SOFTBODY_SHAPE_PROXYTYPE               :: 32;
    HFFLUID_SHAPE_PROXYTYPE                :: 33;
    HFFLUID_BUOYANT_CONVEX_SHAPE_PROXYTYPE :: 34;
    INVALID_SHAPE_PROXYTYPE                :: 35;

    MAX_BROADPHASE_COLLISION_TYPES         :: 36;

    xBOX_SHAPE_PROXYTYPE                    :: BOX_SHAPE_PROXYTYPE;
    xTRIANGLE_SHAPE_PROXYTYPE               :: TRIANGLE_SHAPE_PROXYTYPE;
    xTETRAHEDRAL_SHAPE_PROXYTYPE            :: TETRAHEDRAL_SHAPE_PROXYTYPE;
    xCONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE    :: CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE;
    xCONVEX_HULL_SHAPE_PROXYTYPE            :: CONVEX_HULL_SHAPE_PROXYTYPE;
    xCONVEX_POINT_CLOUD_SHAPE_PROXYTYPE     :: CONVEX_POINT_CLOUD_SHAPE_PROXYTYPE;
    xCUSTOM_POLYHEDRAL_SHAPE_TYPE           :: CUSTOM_POLYHEDRAL_SHAPE_TYPE;

    xIMPLICIT_CONVEX_SHAPES_START_HERE      :: IMPLICIT_CONVEX_SHAPES_START_HERE;
    xSPHERE_SHAPE_PROXYTYPE                 :: SPHERE_SHAPE_PROXYTYPE;
    xMULTI_SPHERE_SHAPE_PROXYTYPE           :: MULTI_SPHERE_SHAPE_PROXYTYPE;
    xCAPSULE_SHAPE_PROXYTYPE                :: CAPSULE_SHAPE_PROXYTYPE;
    xCONE_SHAPE_PROXYTYPE                   :: CONE_SHAPE_PROXYTYPE;
    xCONVEX_SHAPE_PROXYTYPE                 :: CONVEX_SHAPE_PROXYTYPE;
    xCYLINDER_SHAPE_PROXYTYPE               :: CYLINDER_SHAPE_PROXYTYPE;
    xUNIFORM_SCALING_SHAPE_PROXYTYPE        :: UNIFORM_SCALING_SHAPE_PROXYTYPE;
    xMINKOWSKI_SUM_SHAPE_PROXYTYPE          :: MINKOWSKI_SUM_SHAPE_PROXYTYPE;
    xMINKOWSKI_DIFFERENCE_SHAPE_PROXYTYPE   :: MINKOWSKI_DIFFERENCE_SHAPE_PROXYTYPE;
    xBOX_2D_SHAPE_PROXYTYPE                 :: BOX_2D_SHAPE_PROXYTYPE;
    xCONVEX_2D_SHAPE_PROXYTYPE              :: CONVEX_2D_SHAPE_PROXYTYPE;
    xCUSTOM_CONVEX_SHAPE_TYPE               :: CUSTOM_CONVEX_SHAPE_TYPE;

    xCONCAVE_SHAPES_START_HERE              :: CONCAVE_SHAPES_START_HERE;

    xTRIANGLE_MESH_SHAPE_PROXYTYPE          :: TRIANGLE_MESH_SHAPE_PROXYTYPE;
    xSCALED_TRIANGLE_MESH_SHAPE_PROXYTYPE   :: SCALED_TRIANGLE_MESH_SHAPE_PROXYTYPE;

    xFAST_CONCAVE_MESH_PROXYTYPE            :: FAST_CONCAVE_MESH_PROXYTYPE;

    xTERRAIN_SHAPE_PROXYTYPE                :: TERRAIN_SHAPE_PROXYTYPE;

    xGIMPACT_SHAPE_PROXYTYPE                :: GIMPACT_SHAPE_PROXYTYPE;

    xMULTIMATERIAL_TRIANGLE_MESH_PROXYTYPE  :: MULTIMATERIAL_TRIANGLE_MESH_PROXYTYPE;

    xEMPTY_SHAPE_PROXYTYPE                  :: EMPTY_SHAPE_PROXYTYPE;
    xSTATIC_PLANE_PROXYTYPE                 :: STATIC_PLANE_PROXYTYPE;
    xCUSTOM_CONCAVE_SHAPE_TYPE              :: CUSTOM_CONCAVE_SHAPE_TYPE;
    xSDF_SHAPE_PROXYTYPE                    :: SDF_SHAPE_PROXYTYPE;
    xCONCAVE_SHAPES_END_HERE                :: CONCAVE_SHAPES_END_HERE;

    xCOMPOUND_SHAPE_PROXYTYPE               :: COMPOUND_SHAPE_PROXYTYPE;

    xSOFTBODY_SHAPE_PROXYTYPE               :: SOFTBODY_SHAPE_PROXYTYPE;
    xHFFLUID_SHAPE_PROXYTYPE                :: HFFLUID_SHAPE_PROXYTYPE;
    xHFFLUID_BUOYANT_CONVEX_SHAPE_PROXYTYPE :: HFFLUID_BUOYANT_CONVEX_SHAPE_PROXYTYPE;
    xINVALID_SHAPE_PROXYTYPE                :: INVALID_SHAPE_PROXYTYPE;

    xMAX_BROADPHASE_COLLISION_TYPES         :: MAX_BROADPHASE_COLLISION_TYPES;
}

/** types used to distinguish shape types 
*
@param T_SPHERE sphere type
@param T_BOX box type
@param T_CYLINDER cylinder type
@param T_COMPOUND compound type
*/
ShapeType :: enum s32 {
    SPHERE   :: 8;
    BOX      :: 0;
    CYLINDER :: 13;
    COMPOUND :: 31;

    T_SPHERE   :: SPHERE;
    T_BOX      :: BOX;
    T_CYLINDER :: CYLINDER;
    T_COMPOUND :: COMPOUND;
}

// See TODO re: BroadphaseNativeTypesCOPY
/** values used to set constraint params 
* @param C_ERP
* @param C_STOP_ERP
* @param C_CFM
* @param C_STOP_CFM
* */
ConstraintParams :: enum s32 {
    ERP      :: 1;
    STOP_ERP :: 2;
    CFM      :: 3;
    STOP_CFM :: 4;

    C_ERP      :: ERP;
    C_STOP_ERP :: STOP_ERP;
    C_CFM      :: CFM;
    C_STOP_CFM :: STOP_CFM;
}

/** apply an impulse to a body
* @param body
* @param i impulse vector
* @param p where on the body to apply the impulse
*/
body_apply_impulse :: (body: *void, i: *Vector4, p: *Vector4) -> void #foreign bullet3;

/** apply a rotational force to the body
* @param body
* @param t torque vector
*/
body_apply_torque :: (body: *void, t: *Vector4) -> void #foreign bullet3;

/** create a body
* @param u the universe or environment the body is in
* @param shape pointer to the shape to create the body with
* @param mass mass of the body
* @param x,y,z position to start the body from
*/
body_create :: (u: *void, shape: *void, mass: float, x: float, y: float, z: float) -> *void #foreign bullet3;

/** delete a body (release its resources)
* @param b body
*/
body_delete :: (b: *void) -> void #foreign bullet3;

/** get the angular velocity
* @param body
* @param v the velocity
*/
body_get_angular_velocity :: (body: *void, v: *Vector4) -> void #foreign bullet3;

/** get the bodies friction setting
* @param s body
* @return friction setting
*/
body_get_friction :: (s: *void) -> float #foreign bullet3;

/** get the linear velocity of a body
* @param body
* @param v pointer to a Vector4 for the velocity
*/
body_get_linear_velocity :: (body: *void, v: *Vector4) -> void #foreign bullet3;

/** get a matrix representing the orientaion and position of a body
* @param body
* @param m the pointer to a matrix struct (16 floats) can be directly
* used by OpenGL
*/
body_get_open_gl_matrix :: (body: *void, m: *float) -> void #foreign bullet3;

/** get body orientation
* @param body
* @param r rotation returned in this Vector4
*/
body_get_orientation :: (body: *void, r: *Vector4) -> void #foreign bullet3;

/** get the bodies position
* @param body
* @param pos the supplied structured is filled in with the position
*/
body_get_position :: (body: *void, pos: *Vector4) -> void #foreign bullet3;

/** get the position and orientation of a body
* @param body
* @param pos the position
* @param r the rotation
*/
body_get_position_and_orientation :: (body: *void, pos: *Vector4, r: *Vector4) -> void #foreign bullet3;

/** get the shape attached to the body
* @param b body
*/
body_get_shape :: (b: *void) -> *void #foreign bullet3;

/** get the bodies shape type - see ShapeType enum
* @param body pointer to the body to query
* @param T_SPHERE sphere type
* @param T_BOX box type
* @param T_CYLINDER cylinder type
* @param T_COMPOUND compound type
*/
body_get_shape_type :: (body: *void) -> s32 #foreign bullet3;

/** remove body from the universe
* @param u universe pointer
* @param b body
*/
body_remove :: (u: *void, b: *void) -> void #foreign bullet3;

/** set angluar velocity - see notes in bodySetLinearVelocity
* @param body
* @param v velocity
*/
body_set_angular_velocity :: (body: *void, v: Vector4) -> void #foreign bullet3;

/** set body deactivation
* @param b body
* @param v true if body should be deactivated after a period without collisions
*/
body_set_deactivation :: (b: *void, v: bool) -> void #foreign bullet3;

/** set bodies friction
* @param s the body
* @param f friction
*/
body_set_friction :: (s: *void, f: float) -> void #foreign bullet3;

/** set the velocity of a body - 
* this should not be used every frame rather this should be used one
* off in special circumstances like teleportation
* @param body
* @param v the velocity
*/
body_set_linear_velocity :: (body: *void, v: Vector4) -> void #foreign bullet3;

/** set the bodies position - see notes in bodySetLinearVelocity
* @param body
* @param pos the position
*/
body_set_position :: (body: *void, pos: Vector4) -> void #foreign bullet3;

/** set the restitution of a body (bouncy-ness)
* @param body
* @param r restitution
*/
body_set_restitution :: (body: *void, r: float) -> void #foreign bullet3;

/** sets a bodies rotation - see warning in bodySetLinearVelocity
* @param body
* @param r rotation (as a vec instead of three floats - see bodySetRotationEular)
*/
body_set_rotation :: (body: *void, r: Vector4) -> void #foreign bullet3;

/** set the rotation of the body (in radians)
* @param body
* @param pitch,yaw,roll rotations around the x,y & z axis respectivly
*/
body_set_rotation_eular :: (body: *void, pitch: float, yaw: float, roll: float) -> void #foreign bullet3;

/** set the collision callback
* @param u the universe
* @param callback function pointer to the contact callback
* 
* void contact(void* b1, void* b2, const Vector4* ptA, const Vector4* ptB, const Vector4* norm)
* @param b1 body A
* @param b2 body B
* @param ptA point in body A of the contact
* @param ptB point in body B of the contact
* @param norm the collision normal
*/
collision_callback :: (u: *void, callback: #type (unknown0: *void, unknown1: *void, unknown2: *Vector4, unknown3: *Vector4, unknown4: *Vector4) -> void #c_call) -> void #foreign bullet3;

/** adds other primatives to the compound shape
* @param compound pointer to the compound shape
* @param child pointer to the child shape
* @param x,y,z local position of shape in the compound
* @param yaw,pitch,roll local rotation in the compound
*/
compound_add_child :: (compound: *void, child: *void, x: float, y: float, z: float, yaw: float, pitch: float, roll: float) -> void #foreign bullet3;

/** number of shapes in a compound
* @param s shape
*/
compound_get_num_children :: (s: *void) -> s32 #foreign bullet3;

/** remove a shape from a compound shape
* @param s shape
* @param index
*/
compound_remove_shape :: (s: *void, index: s32) -> void #foreign bullet3;

/** is a constraint enabled
* @param c constraint pointer
* @return is the constraint enabled?
*/
constraint_is_enabled :: (c: *void) -> bool #foreign bullet3;

/** enable or disable a constraint
* @param c constraint pointer
* @param en true or false to enable / disable
*/
constraint_set_enabled :: (c: *void, en: bool) -> void #foreign bullet3;

/** sets a constrains parameters
* @param c constraint pointer
* @param num see ConstraintParams enum
* @param value
* @param axis 0-5 but see implementation as some are handled differently
* for example hinge only uses axis 5 (which can be refered to as -1)
*/
constraint_set_param :: (c: *void, num: s32, value: float, axis: s32) -> void #foreign bullet3;

/** create a hinge2 constraint
* @param u universe
* @param bodyA parent body constrained
* @param bodyB child body constrained
* @param anchor constraint centre
* @param parentAxis orientation of parent
* @param childAxis orientation of child
*/
hinge2_create :: (u: *void, bodyA: *void, bodyB: *void, anchor: Vector4, parentAxis: Vector4, childAxis: Vector4, collide: bool) -> *void #foreign bullet3;

/** enable (or disable) a motor on a hinge2 constraint
* @param h pointer to hinge2 constraint
* @param index which motor to effect
* @param onOff true for on, false for off
*/
hinge2enable_motor :: (h: *void, index: s32, onOff: bool) -> void #foreign bullet3;

/** gets the angle of the first hinge
* @param h pointer to hinge2 constraint
*/
hinge2get_angle1 :: (h: *void) -> float #foreign bullet3;

/** gets the angle of the second hinge
* @param h pointer to hinge2 constraint
*/
hinge2get_angle2 :: (h: *void) -> float #foreign bullet3;

/** sets damping for a hinge2 axis
* @param h pointer to hinge2 constraint
* @param index the axis
* @param damping how much damping for the axis
* @param limitIfNeeded normally defaults to true
*/
hinge2set_damping :: (h: *void, index: s32, damping: float, limitIfNeeded: bool) -> void #foreign bullet3;

/**  set the lower limit of a hinge2 constraint
* @param h pointer to the hinge2
* @param l value of the limit
*/
hinge2set_lower_limit :: (h: *void, l: float) -> void #foreign bullet3;

/** set the maximum motor force for a hinge2 motor
* @param h pointer to hinge2 constraint
* @param index which motor to change
* @param force maximum force
*/
hinge2set_max_motor_force :: (h: *void, index: s32, force: float) -> void #foreign bullet3;

/** sets stifness for a hinge2 axis
* @param h pointer to hinge2 constraint
* @param index the axis
* @param stiffness how stiff the axis is
* @param limitIfNeeded normally defaults to true
*/
hinge2set_stiffness :: (h: *void, index: s32, stiffness: float, limitIfNeeded: bool) -> void #foreign bullet3;

/** set the target velocity for a hinge2 axis motor
* @param h pointer to hinge2 constraint
* @param index which axis
* @param vel the target velocity
*/
hinge2set_target_velocity :: (h: *void, index: s32, vel: float) -> void #foreign bullet3;

/**  set the upper limit of a hinge2 constraint
* @param h pointer to the hinge2
* @param l value of the limit
*/
hinge2set_upper_limit :: (h: *void, l: float) -> void #foreign bullet3;

/** creates a hinge constraint (joint)
* @param uni the universe the joint should be in
* @param bodyA,bodyB the two bodies involved
* @param pivA,rotA the pivot point and rotation of the hinge axis
* @param pivB,rotB pivot and rotation relative to bodyB
* @param refA use reference frame A or not
* @param collide should these bodies collide or not
*/
hinge_create :: (uni: *void, bodyA: *void, bodyB: *void, pivA: Vector4, rotA: Vector4, pivB: Vector4, rotB: Vector4, refA: bool, collide: bool) -> *void #foreign bullet3;

/** enable rotational motor for a hinge
* @param hinge pointer to the constraint
* @param enableMotor true enable
* @param targetVelocity the motor will always try to achive this velocity
* @param maxMotorImpulse limits the impulse the motor can use
*/
hinge_enable_angular_motor :: (hinge: *void, enableMotor: bool, targetVelocity: float, maxMotorImpulse: float) -> void #foreign bullet3;

/** set the hinge angluar limit
* @param hinge pointer to the joint
* @param low set the lower limit  < -PI is unlimited
* @param hi set the upper limit  > PI is unlimited
*/
hinge_set_limit :: (hinge: *void, low: float, hi: float) -> void #foreign bullet3;

/** creat a box shape
* @param uni the universe that the shape is intended for
* 			this is for tidying up when the universe is destroyed
* @param ex X extent of the box
* @param ey Y extent of the box
* @param ez Z extent of the box
*/
shape_create_box :: (uni: *void, ex: float, ey: float, ez: float) -> *void #foreign bullet3;

/** create a compound shape
* @param u the universe that this shape is in
* 
* The compound shape is a "empty" shape you add other shapes
* to, to make a single more complex shape.
*/
shape_create_compound :: (u: *void) -> *void #foreign bullet3;

/** creates a cylinder shape, length along the X axis
* @param u the universe that releases this shape
* @param r the radius of the cylinder
* @param l the length of the cylinder
*/
shape_create_cylinder_x :: (u: *void, r: float, l: float) -> *void #foreign bullet3;

/** creates a cylinder shape, length along the Y axis
* @param u the universe that releases this shape
* @param r the radius of the cylinder
* @param l the length of the cylinder
*/
shape_create_cylinder_y :: (u: *void, r: float, l: float) -> *void #foreign bullet3;

/** creates a cylinder shape, length along the Z axis
* @param u the universe that releases this shape
* @param r the radius of the cylinder
* @param l the length of the cylinder
*/
shape_create_cylinder_z :: (u: *void, r: float, l: float) -> *void #foreign bullet3;

/** creates a sphere shape
* @param u the universe that releases this shape
* @param re radius of sphere
*/
shape_create_sphere :: (u: *void, re: float) -> *void #foreign bullet3;

/** delete a shape
* @param u universe pointer
* @param s shape
*/
shape_delete :: (u: *void, s: *void) -> void #foreign bullet3;

/** creates a physics environment 
* @return pointer to the environment 
*/
universe_create :: () -> *void #foreign bullet3;

/** releases the environment
* @param uni pointer to the previously created environment
*/
universe_destroy :: (uni: *void) -> void #foreign bullet3;

/** sets the environments gravity */
universe_set_gravity :: (uni: *void, x: float, y: float, z: float) -> void #foreign bullet3;

/** step the universe (time)
* @param u the environment to step
* @param dt the amount of time to step
* @param i number of iterations 
*/
universe_step :: (u: *void, dt: float, i: s32) -> void #foreign bullet3;

#scope_file

bullet3 :: #foreign_library "./bullet3";
#import "Math";