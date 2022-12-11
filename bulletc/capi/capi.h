
/** @file capi.h
 * @brief a C wrapper for the bullet physics engine
 * @author Chris Camacho (codifies)
 * @author http://bedroomcoders.co.uk/captcha/
 */

#ifndef CAPI_H
#define CAPI_H

#include "btApi.h"

#include <stdbool.h>

// #define PI 3.14159265359

/** DEACTIVATION_ENABLE enable deactivation used with bodySetDeactivation*/
#define DEACTIVATION_ENABLE 3
/** DEACTIVATION_DISABLE disable deactivation used with bodySetDeactivation*/
#define DEACTIVATION_DISABLE 4

// Vec has 4 components so it can be used for quaternions too...
/** struct to hold vector or quaternions */
typedef struct {
	float x,y,z,w;
} Vector4;

// copied from btBroadphaseProxy.h so C app doesn't have to import
// bullet c++ headers
//
// TODO !!! find some better way!
// 
// NB will need to recopy this if it changes in bullet - not ideal...
// have to rename enums so no duplicate definition for C++ "wrapper"
/** \cond HIDDEN */
enum BroadphaseNativeTypesCOPY
{
    // polyhedral convex shapes
    xBOX_SHAPE_PROXYTYPE,
    xTRIANGLE_SHAPE_PROXYTYPE,
    xTETRAHEDRAL_SHAPE_PROXYTYPE,
    xCONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE,
    xCONVEX_HULL_SHAPE_PROXYTYPE,
    xCONVEX_POINT_CLOUD_SHAPE_PROXYTYPE,
    xCUSTOM_POLYHEDRAL_SHAPE_TYPE,
    //implicit convex shapes
    xIMPLICIT_CONVEX_SHAPES_START_HERE,
    xSPHERE_SHAPE_PROXYTYPE,
    xMULTI_SPHERE_SHAPE_PROXYTYPE,
    xCAPSULE_SHAPE_PROXYTYPE,
    xCONE_SHAPE_PROXYTYPE,
    xCONVEX_SHAPE_PROXYTYPE,
    xCYLINDER_SHAPE_PROXYTYPE,
    xUNIFORM_SCALING_SHAPE_PROXYTYPE,
    xMINKOWSKI_SUM_SHAPE_PROXYTYPE,
    xMINKOWSKI_DIFFERENCE_SHAPE_PROXYTYPE,
    xBOX_2D_SHAPE_PROXYTYPE,
    xCONVEX_2D_SHAPE_PROXYTYPE,
    xCUSTOM_CONVEX_SHAPE_TYPE,
    //concave shapes
    xCONCAVE_SHAPES_START_HERE,
    //keep all the convex shapetype below here, for the check IsConvexShape in broadphase proxy!
    xTRIANGLE_MESH_SHAPE_PROXYTYPE,
    xSCALED_TRIANGLE_MESH_SHAPE_PROXYTYPE,
    ///used for demo integration FAST/Swift collision library and Bullet
    xFAST_CONCAVE_MESH_PROXYTYPE,
    //terrain
    xTERRAIN_SHAPE_PROXYTYPE,
    ///Used for GIMPACT Trimesh integration
    xGIMPACT_SHAPE_PROXYTYPE,
    ///Multimaterial mesh
    xMULTIMATERIAL_TRIANGLE_MESH_PROXYTYPE,

    xEMPTY_SHAPE_PROXYTYPE,
    xSTATIC_PLANE_PROXYTYPE,
    xCUSTOM_CONCAVE_SHAPE_TYPE,
    xSDF_SHAPE_PROXYTYPE = xCUSTOM_CONCAVE_SHAPE_TYPE,
    xCONCAVE_SHAPES_END_HERE,

    xCOMPOUND_SHAPE_PROXYTYPE,

    xSOFTBODY_SHAPE_PROXYTYPE,
    xHFFLUID_SHAPE_PROXYTYPE,
    xHFFLUID_BUOYANT_CONVEX_SHAPE_PROXYTYPE,
    xINVALID_SHAPE_PROXYTYPE,

    xMAX_BROADPHASE_COLLISION_TYPES
};
/** \endcond */

/** types used to distinguish shape types 
 *
	@param T_SPHERE sphere type
	@param T_BOX box type
	@param T_CYLINDER cylinder type
	@param T_COMPOUND compound type
*/
typedef enum {
	T_SPHERE		=	xSPHERE_SHAPE_PROXYTYPE,
	T_BOX			=	xBOX_SHAPE_PROXYTYPE,
	T_CYLINDER		=	xCYLINDER_SHAPE_PROXYTYPE,
	T_COMPOUND		=	xCOMPOUND_SHAPE_PROXYTYPE
} ShapeType;

// See TODO re: BroadphaseNativeTypesCOPY
/** values used to set constraint params 
 * @param C_ERP
 * @param C_STOP_ERP
 * @param C_CFM
 * @param C_STOP_CFM
 * */
typedef enum  {
	C_ERP=1,		//BT_CONSTRAINT_ERP=1,	
	C_STOP_ERP,		//BT_CONSTRAINT_STOP_ERP,
	C_CFM,			//BT_CONSTRAINT_CFM,
	C_STOP_CFM		//BT_CONSTRAINT_STOP_CFM
} ConstraintParams;


/** apply an impulse to a body
 * @param body
 * @param i impulse vector
 * @param p where on the body to apply the impulse
 */
BT_API void body_apply_impulse(void* body, Vector4* i, Vector4* p);

/** apply a rotational force to the body
 * @param body
 * @param t torque vector
 */
BT_API void body_apply_torque(void* body, Vector4* t);

/** create a body
 * @param u the universe or environment the body is in
 * @param shape pointer to the shape to create the body with
 * @param mass mass of the body
 * @param x,y,z position to start the body from
 */
BT_API void* body_create(void* u, void* shape, float mass, float x, float y, float z);

/** delete a body (release its resources)
 * @param b body
 */
BT_API void body_delete(void* b);

/** get the angular velocity
 * @param body
 * @param v the velocity
 */
BT_API void body_get_angular_velocity(void* body, Vector4* v);

/** get the bodies friction setting
 * @param s body
 * @return friction setting
 */
BT_API float body_get_friction(void* s);

/** get the linear velocity of a body
 * @param body
 * @param v pointer to a Vector4 for the velocity
 */
BT_API void body_get_linear_velocity(void* body, Vector4* v);

/** get a matrix representing the orientaion and position of a body
 * @param body
 * @param m the pointer to a matrix struct (16 floats) can be directly
 * used by OpenGL
 */
BT_API void body_get_open_gl_matrix(void* body, float* m);

/** get body orientation
 * @param body
 * @param r rotation returned in this Vector4
 */
BT_API void body_get_orientation(void* body, Vector4* r);

/** get the bodies position
 * @param body
 * @param pos the supplied structured is filled in with the position
 */
BT_API void body_get_position(void* body, Vector4* pos );

/** get the position and orientation of a body
 * @param body
 * @param pos the position
 * @param r the rotation
 */
BT_API void body_get_position_and_orientation(void* body, Vector4* pos, Vector4* r);

/** get the shape attached to the body
 * @param b body
 */
BT_API void* body_get_shape(void* b);

/** get the bodies shape type - see ShapeType enum
 * @param body pointer to the body to query
 * @param T_SPHERE sphere type
 * @param T_BOX box type
 * @param T_CYLINDER cylinder type
 * @param T_COMPOUND compound type
 */
BT_API int body_get_shape_type(void* body);

/** remove body from the universe
 * @param u universe pointer
 * @param b body
 */
BT_API void body_remove(void* u, void* b);

/** set angluar velocity - see notes in bodySetLinearVelocity
 * @param body
 * @param v velocity
 */
BT_API void body_set_angular_velocity(void* body, Vector4 v);

/** set body deactivation
 * @param b body
 * @param v true if body should be deactivated after a period without collisions
 */
BT_API void body_set_deactivation(void* b, bool v);

/** set bodies friction
 * @param s the body
 * @param f friction
 */ 
BT_API void body_set_friction(void* s, float f);

/** set the velocity of a body - 
 * this should not be used every frame rather this should be used one
 * off in special circumstances like teleportation
 * @param body
 * @param v the velocity
 */
BT_API void body_set_linear_velocity(void* body, Vector4 v);

/** set the bodies position - see notes in bodySetLinearVelocity
 * @param body
 * @param pos the position
 */
BT_API void body_set_position(void* body, Vector4 pos );

/** set the restitution of a body (bouncy-ness)
 * @param body
 * @param r restitution
 */
BT_API void body_set_restitution(void* body, float r);
 
/** sets a bodies rotation - see warning in bodySetLinearVelocity
 * @param body
 * @param r rotation (as a vec instead of three floats - see bodySetRotationEular)
 */
BT_API void body_set_rotation(void* body, Vector4 r);

/** set the rotation of the body (in radians)
 * @param body
 * @param pitch,yaw,roll rotations around the x,y & z axis respectivly
 */
BT_API void body_set_rotation_eular(void* body, float pitch, float yaw, float roll);

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
BT_API void collision_callback(void* u, void(*callback)(void*, void*, const Vector4*, const Vector4*, const Vector4*) );

/** adds other primatives to the compound shape
 * @param compound pointer to the compound shape
 * @param child pointer to the child shape
 * @param x,y,z local position of shape in the compound
 * @param yaw,pitch,roll local rotation in the compound
 */
BT_API void compound_add_child(void* compound, void* child, float x, float y, float z,
						float yaw, float pitch, float roll);

/** number of shapes in a compound
 * @param s shape
 */
BT_API int compound_get_num_children(void* s);

/** remove a shape from a compound shape
 * @param s shape
 * @param index
 */
BT_API void compound_remove_shape(void* s, int index);

/** is a constraint enabled
 * @param c constraint pointer
 * @return is the constraint enabled?
 */
BT_API bool constraint_is_enabled(void* c);

/** enable or disable a constraint
 * @param c constraint pointer
 * @param en true or false to enable / disable
 */ 
BT_API void constraint_set_enabled(void* c, bool en);

/** sets a constrains parameters
 * @param c constraint pointer
 * @param num see ConstraintParams enum
 * @param value
 * @param axis 0-5 but see implementation as some are handled differently
 * for example hinge only uses axis 5 (which can be refered to as -1)
 */
BT_API void constraint_set_param(void* c, int num, float value, int axis);

/** create a hinge2 constraint
 * @param u universe
 * @param bodyA parent body constrained
 * @param bodyB child body constrained
 * @param anchor constraint centre
 * @param parentAxis orientation of parent
 * @param childAxis orientation of child
 */
BT_API void* hinge2_create(void* u, void* bodyA, void* bodyB, Vector4 anchor,
								Vector4 parentAxis, Vector4 childAxis, bool collide);

/** enable (or disable) a motor on a hinge2 constraint
 * @param h pointer to hinge2 constraint
 * @param index which motor to effect
 * @param onOff true for on, false for off
 */
BT_API void hinge2enable_motor(void* h, int index, bool onOff);

/** gets the angle of the first hinge
 * @param h pointer to hinge2 constraint
 */
BT_API float hinge2get_angle1(void* h);

/** gets the angle of the second hinge
 * @param h pointer to hinge2 constraint
 */
BT_API float hinge2get_angle2(void* h);

/** sets damping for a hinge2 axis
 * @param h pointer to hinge2 constraint
 * @param index the axis
 * @param damping how much damping for the axis
 * @param limitIfNeeded normally defaults to true
 */
BT_API void hinge2set_damping(void* h, int index, float damping, bool limitIfNeeded);

/**  set the lower limit of a hinge2 constraint
 * @param h pointer to the hinge2
 * @param l value of the limit
 */
BT_API void hinge2set_lower_limit(void* h, float l);

/** set the maximum motor force for a hinge2 motor
 * @param h pointer to hinge2 constraint
 * @param index which motor to change
 * @param force maximum force
 */
BT_API void hinge2set_max_motor_force(void* h, int index, float force);

/** sets stifness for a hinge2 axis
 * @param h pointer to hinge2 constraint
 * @param index the axis
 * @param stiffness how stiff the axis is
 * @param limitIfNeeded normally defaults to true
 */
BT_API void hinge2set_stiffness(void* h, int index, float stiffness, bool limitIfNeeded);

/** set the target velocity for a hinge2 axis motor
 * @param h pointer to hinge2 constraint
 * @param index which axis
 * @param vel the target velocity
 */
BT_API void hinge2set_target_velocity(void* h, int index, float vel);

/**  set the upper limit of a hinge2 constraint
 * @param h pointer to the hinge2
 * @param l value of the limit
 */
BT_API void hinge2set_upper_limit(void* h, float l);

/** creates a hinge constraint (joint)
 * @param uni the universe the joint should be in
 * @param bodyA,bodyB the two bodies involved
 * @param pivA,rotA the pivot point and rotation of the hinge axis
 * @param pivB,rotB pivot and rotation relative to bodyB
 * @param refA use reference frame A or not
 * @param collide should these bodies collide or not
 */
BT_API void* hinge_create(void* uni, void* bodyA, void* bodyB,
					Vector4 pivA, Vector4 rotA, 
					Vector4 pivB, Vector4 rotB, bool refA, bool collide);

/** enable rotational motor for a hinge
 * @param hinge pointer to the constraint
 * @param enableMotor true enable
 * @param targetVelocity the motor will always try to achive this velocity
 * @param maxMotorImpulse limits the impulse the motor can use
 */
BT_API void hinge_enable_angular_motor(void* hinge, bool enableMotor, float targetVelocity,
								float maxMotorImpulse);

/** set the hinge angluar limit
 * @param hinge pointer to the joint
 * @param low set the lower limit  < -PI is unlimited
 * @param hi set the upper limit  > PI is unlimited
 */
BT_API void hinge_set_limit(void* hinge, float low, float hi);

/** creat a box shape
 * @param uni the universe that the shape is intended for
 * 			this is for tidying up when the universe is destroyed
 * @param ex X extent of the box
 * @param ey Y extent of the box
 * @param ez Z extent of the box
 */ 
BT_API void* shape_create_box(void* uni, float ex, float ey, float ez);

/** create a compound shape
 * @param u the universe that this shape is in
 * 
 * The compound shape is a "empty" shape you add other shapes
 * to, to make a single more complex shape.
 */
BT_API void* shape_create_compound(void* u);

/** creates a cylinder shape, length along the X axis
 * @param u the universe that releases this shape
 * @param r the radius of the cylinder
 * @param l the length of the cylinder
 */
BT_API void* shape_create_cylinder_x(void* u,  float r, float l);

/** creates a cylinder shape, length along the Y axis
 * @param u the universe that releases this shape
 * @param r the radius of the cylinder
 * @param l the length of the cylinder
 */
BT_API void* shape_create_cylinder_y(void* u,  float r, float l);

/** creates a cylinder shape, length along the Z axis
 * @param u the universe that releases this shape
 * @param r the radius of the cylinder
 * @param l the length of the cylinder
 */
BT_API void* shape_create_cylinder_z(void* u,  float r, float l);

/** creates a sphere shape
 * @param u the universe that releases this shape
 * @param re radius of sphere
 */
BT_API void* shape_create_sphere(void* u, float re);

/** delete a shape
 * @param u universe pointer
 * @param s shape
 */ 
BT_API void shape_delete(void* u, void* s);

/** creates a physics environment 
 * @return pointer to the environment 
 */
BT_API void* universe_create();

/** releases the environment
 * @param uni pointer to the previously created environment
 */
BT_API void universe_destroy(void* uni); /// muhahahaha

/** sets the environments gravity */
BT_API void universe_set_gravity(void* uni, float x, float y, float z);

/** step the universe (time)
 * @param u the environment to step
 * @param dt the amount of time to step
 * @param i number of iterations 
 */
BT_API void universe_step(void* u, float dt, int i);


#endif




