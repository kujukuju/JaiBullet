#include "capi.hpp"
#include "btBulletDynamicsCommon.h"

void* universe_create() {
	universe* u = (universe*)malloc(sizeof(universe));
	u->collisionConfiguration = new btDefaultCollisionConfiguration();
	u->dispatcher = new btCollisionDispatcher(u->collisionConfiguration);
	u->broadphase = new btDbvtBroadphase();
	u->solver = new btSequentialImpulseConstraintSolver;
	u->dynamicsWorld = new btDiscreteDynamicsWorld(u->dispatcher, u->broadphase, u->solver, u->collisionConfiguration);

	// possibly could have an iteration callback function so all shapes in the
	// world get passed one at a time to a callback function(s)
	// only used for clean up for now...
	u->collisionShapes =  new btAlignedObjectArray<btCollisionShape*>();
	
	//float a;
	//btScalar b;
	//printf("sizeof float %i sizeof btScalar %i\n", sizeof(a),sizeof(b));

	return (void*)u;
}

void body_remove(void* u, void* b) {
	UNI(u)->dynamicsWorld->removeRigidBody(BODY(b));
}

void shape_delete(void* u, void* s) {
	UNI(u)->collisionShapes->remove(SHAPE(s));
	delete SHAPE(s);
}

void body_delete(void* b) {
	if (BODY(b) && BODY(b)->getMotionState()) {
		delete BODY(b)->getMotionState();
	}
	delete BODY(b);
}

void* body_get_shape(void* b) {
	return BODY(b)->getCollisionShape();
}

int compound_get_num_children(void* s) {
	return ((btCompoundShape*)s)->getNumChildShapes();
}

void compound_remove_shape(void* s, int index) {
	((btCompoundShape*)s)->removeChildShapeByIndex(index);
}

void universe_destroy(void* u) {
	
	for (int i = UNI(u)->dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--)
	{
		btCollisionObject* obj = UNI(u)->dynamicsWorld->getCollisionObjectArray().at(i);
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			while (body->getNumConstraintRefs())
			{
				btTypedConstraint* constraint = body->getConstraintRef(0);
				UNI(u)->dynamicsWorld->removeConstraint(constraint);
				delete constraint;
			}
			delete body->getMotionState();
		}
		UNI(u)->dynamicsWorld->removeCollisionObject(obj);
		delete obj;
	}

	for (int j = 0; j < UNI(u)->collisionShapes->size(); j++)
	{
		btCollisionShape* shape = (btCollisionShape*)(UNI(u)->collisionShapes->at(j));
		//collisionShapes[j] = 0;
		delete shape;
	}
	
	delete UNI(u)->dynamicsWorld;
	delete UNI(u)->solver;
	delete UNI(u)->broadphase;
	delete UNI(u)->dispatcher;
	delete UNI(u)->collisionConfiguration;
	UNI(u)->collisionShapes->clear();
	delete UNI(u)->collisionShapes;
	
	free(u);
}

void universe_set_gravity(void* u, float x, float y, float z) {
	UNI(u)->dynamicsWorld->setGravity(btVector3(x,y,z));
}

void* shape_create_box(void* u, float ex, float ey, float ez) {
	btCollisionShape* shape = new btBoxShape(btVector3(btScalar(ex), btScalar(ey), btScalar(ez)));
	UNI(u)->collisionShapes->push_back(shape);
	return (void*)shape;
}

void* shape_create_compound(void* u) {
	btCollisionShape* shape = new btCompoundShape();
	UNI(u)->collisionShapes->push_back(shape);
	return (void*)shape;
}

void compound_add_child(void* compound, void* child, float x, float y, float z,
						float yaw, float pitch, float roll) {
	
	btTransform localTrans;
	localTrans.setIdentity();
	localTrans.setOrigin(btVector3(x,y,z));
	
	btQuaternion quat;
	quat.setEuler(yaw, pitch, roll);
	localTrans.setRotation(quat);

	((btCompoundShape*)compound)->addChildShape(localTrans,SHAPE(child));
}

void* shape_create_sphere(void* u, float re) {
	btCollisionShape* shape = new btSphereShape(re);
	UNI(u)->collisionShapes->push_back(shape);
	return (void*)shape;
}

void* shape_create_cylinder_z(void* u,  float r, float l) {
	btCollisionShape* shape = new btCylinderShapeZ(btVector3(r,r,l));
	UNI(u)->collisionShapes->push_back(shape);
	return (void*)shape;	
}

void* shape_create_cylinder_y(void* u,  float r, float l) {
	btCollisionShape* shape = new btCylinderShape(btVector3(r,l,r));
	UNI(u)->collisionShapes->push_back(shape);
	return (void*)shape;	
}

void* shape_create_cylinder_x(void* u,  float r, float l) {
	btCollisionShape* shape = new btCylinderShapeX(btVector3(l,r,r));
	UNI(u)->collisionShapes->push_back(shape);
	return (void*)shape;	
}

// void* shape_create_static_triangle_mesh(void* u, float* triangles, int triangle_count) {
//     // not going to pursue this further because mesh has to be a pointer, but its not deleted when the shape is deleted,
//     // so it would have to be managed separately which sounds like a huge pain in the ass
//     btTriangleMesh mesh = new btTriangleMesh(true, false);
//     for (int i = 0; i < triangle_count; i++) {
//         btVector3 vertex0(vertices[i * 9 + 0], vertices[i * 9 + 1], vertices[i * 9 + 2]);
//         btVector3 vertex1(vertices[i * 9 + 3], vertices[i * 9 + 4], vertices[i * 9 + 5]);
//         btVector3 vertex2(vertices[i * 9 + 6], vertices[i * 9 + 7], vertices[i * 9 + 8]);
//         mesh.addTriangle(vertex0, vertex1, vertex2);
//     }

//     btBvhTriangleMeshShape shape = new btBvhTriangleMeshShape();
//     UNI(u)->collisionShapes->push_back(shape);
//     return (void*) shape;
// }

void* hinge2_create(void* u, void* bodyA, void* bodyB, Vector4 anchor,
								Vector4 parentAxis, Vector4 childAxis, bool collide) {

		btVector3 anc(anchor.x, anchor.y, anchor.z);
		btVector3 pax(parentAxis.x, parentAxis.y, parentAxis.z);
		btVector3 cax(childAxis.x, childAxis.y, childAxis.z); 

		btHinge2Constraint* hinge2 = new btHinge2Constraint(
				*BODY(bodyA), *BODY(bodyB), 
				anc, pax, cax);
		UNI(u)->dynamicsWorld->addConstraint(hinge2, collide);

		//TODO add to global universe list of constraints for 
		// iteration / auto freeing on destroy universe
		
		return (void*)hinge2;
}

void hinge2set_lower_limit(void* h, float l) {
	((btHinge2Constraint*)h)->setLowerLimit(l);
}
void hinge2set_upper_limit(void* h, float l) {
	((btHinge2Constraint*)h)->setUpperLimit(l);
}

void hinge2enable_motor(void* h, int index, bool onOff) {
	((btHinge2Constraint*)h)->enableMotor(index, onOff);
}

void hinge2set_max_motor_force(void* h, int index, float force) {
	((btHinge2Constraint*)h)->setMaxMotorForce(index, force);
}

void hinge2set_target_velocity(void* h, int index, float vel) {
	((btHinge2Constraint*)h)->setTargetVelocity(index, vel);
}

void hinge2set_damping(void* h, int index, float damping, bool limitIfNeeded) {
	((btHinge2Constraint*)h)->setDamping(index, damping, limitIfNeeded);
}

void hinge2set_stiffness(void* h, int index, float stiffness, bool limitIfNeeded) {
	((btHinge2Constraint*)h)->setStiffness(index, stiffness, limitIfNeeded);
}

float hinge2get_angle1(void* h) {
    return ((btHinge2Constraint*)h)->getAngle1();
}

float hinge2get_angle2(void* h) {
    return ((btHinge2Constraint*)h)->getAngle2();
}

void* hinge_create(void* uni, void* bodyA, void* bodyB, 
					Vector4 pivA, Vector4 rotA, 
					Vector4 pivB, Vector4 rotB, bool refA, bool collide) {

	btTransform localA, localB;
	localA.setIdentity();
	localB.setIdentity();
	localA.getBasis().setEulerZYX(rotA.z, rotA.y, rotA.x);
	localA.setOrigin(btVector3(pivA.x, pivA.y, pivA.z));
	localB.getBasis().setEulerZYX(rotB.z, rotB.y, rotB.x);
	localB.setOrigin(btVector3(pivB.x, pivB.y, pivB.z));
	btHingeConstraint* hinge = new btHingeConstraint(
									*BODY(bodyA), *BODY(bodyB), 
									localA, localB, refA);
	UNI(uni)->dynamicsWorld->addConstraint(hinge, collide);
	// TODO research proper way to dispose of constraints later
	// possibly add to global constraint list for destroyUniverse			
	return (void*) hinge;

}


void hinge_set_limit(void* hinge, float low, float hi) {
	((btHingeConstraint*)hinge)->setLimit(low, hi);
}

void hinge_enable_angular_motor(void* hinge, bool enableMotor, 
						float targetVelocity, float maxMotorImpulse) {
	((btHingeConstraint*)hinge)->enableAngularMotor(enableMotor,
									targetVelocity, maxMotorImpulse);
}

// TODO can't get any sense from this...
//float hingeGetAngle(void* hinge) {
	//return ((btHingeConstraint*)hinge)->getHingeAngle();
	//return ((btHingeAccumulatedAngleConstraint*)hinge)->getAccumulatedHingeAngle();
	//btTransform transA = ((btHingeConstraint*)hinge)->getFrameOffsetA();
	//btTransform transB = ((btHingeConstraint*)hinge)->getFrameOffsetB();
	//return ((btHingeConstraint*)hinge)->getHingeAngle(transA, transB);
//}

void constraint_set_param(void* c, int num, float value, int axis) {
	JOINT(c)->setParam(num, value, axis);
}

bool constraint_is_enabled(void* c) {
	return JOINT(c)->isEnabled();
}
 
void constraint_set_enabled(void* c, bool en) {
	JOINT(c)->setEnabled(en);
}

void* body_create(void* u, void* shape, float mass, float x, float y, float z) {
	// heavily "influenced" from bullet manual hello world console example
	btTransform trans;
	trans.setIdentity();
	trans.setOrigin(btVector3(x, y, z));
	
	btScalar M(mass);
	bool isDynamic = (M != 0.f);

	btVector3 localInertia(0, 0, 0);
	if (isDynamic)
		SHAPE(shape)->calculateLocalInertia(M, localInertia);

	btDefaultMotionState* motionState = new btDefaultMotionState(trans);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(M, motionState, SHAPE(shape), localInertia);
	btRigidBody* body = new btRigidBody(rbInfo);

	UNI(u)->dynamicsWorld->addRigidBody(body);
	return body;
}

void universe_step(void* u, float dt, int i) {
	UNI(u)->dynamicsWorld->stepSimulation(dt, i);
}

void body_get_position_and_orientation(void* body, Vector4* pos, Vector4* r) {
	btTransform trans;
	//if (BODY(body) && BODY(body)->getMotionState())	{
	//	BODY(body)->getMotionState()->getWorldTransform(trans);
	//} else {
		trans = BODY(body)->getWorldTransform();
	//}
	pos->x = trans.getOrigin().getX();
	pos->y = trans.getOrigin().getY();
	pos->z = trans.getOrigin().getZ();	
	
	btQuaternion q = trans.getRotation();
	
	r->x = q.getX();
	r->y = q.getY();
	r->z = q.getZ();
	r->w = q.getW();
}

void body_get_position(void* body, Vector4* pos ) {
	btTransform trans;
	//if (BODY(body) && BODY(body)->getMotionState())	{
	//	BODY(body)->getMotionState()->getWorldTransform(trans);
	//} else {
		trans = BODY(body)->getWorldTransform();
	//}
	pos->x = trans.getOrigin().getX();
	pos->y = trans.getOrigin().getY();
	pos->z = trans.getOrigin().getZ();
}

void body_set_position(void* body, Vector4 pos ) {
	btTransform trans;
	trans.setOrigin(btVector3(pos.x,pos.y,pos.z));
	BODY(body)->setWorldTransform(trans);
}

void body_get_orientation(void* body, Vector4* r) {
	btTransform trans;
	//if (BODY(body) && BODY(body)->getMotionState())	{
	//	BODY(body)->getMotionState()->getWorldTransform(trans);
	//} else {
		trans = BODY(body)->getWorldTransform();
	//}
	btQuaternion q = trans.getRotation();
	
	r->x = q.getX();
	r->y = q.getY();
	r->z = q.getZ();
	r->w = q.getW();
}

int body_get_shape_type(void* body) {
	return BODY(body)->getCollisionShape()->getShapeType();
}

void body_get_open_gl_matrix(void* body, float* m) {
	btTransform trans;
	
	// TODO look into this...
	// commented only worked for dynamic bodies, getting world trans
	// direct from body works for both static and dynamic
	
//	if (BODY(body) && BODY(body)->getMotionState())	{
//		BODY(body)->getMotionState()->getWorldTransform(trans);
//	} else {
		trans = BODY(body)->getWorldTransform();
//	}
	trans.getOpenGLMatrix(m);
}

void body_apply_impulse(void* body, Vector4* i, Vector4* p) {
	BODY(body)->applyImpulse(btVector3(i->x,i->y,i->z), btVector3(p->x,p->y,p->z)); 	
}

void body_apply_torque(void* body, Vector4* t) {
	BODY(body)->applyTorque(btVector3(t->x,t->y,t->z));
}

void body_set_rotation_eular(void* body, float pitch, float yaw, float roll) {
	btQuaternion q = btQuaternion();
	q.setEuler(btScalar(yaw),btScalar(pitch),btScalar(roll));
	
	btTransform trans;

	trans = BODY(body)->getCenterOfMassTransform();
	trans.setRotation(q);
	
	BODY(body)->setCenterOfMassTransform(trans);
	
}

void body_set_rotation(void* body, Vector4 r) {
	body_set_rotation_eular(body, r.x, r.y, r.z);
}

void body_set_restitution(void* body, float r) {
	BODY(body)->setRestitution(r);
}

void body_get_linear_velocity(void* body, Vector4* v) {
	const btVector3 bv = BODY(body)->getLinearVelocity();
	v->x = bv.getX();
	v->y = bv.getY();
	v->z = bv.getZ();
}

void body_set_linear_velocity(void* body, Vector4 v) {
	BODY(body)->setLinearVelocity(btVector3(v.x,v.y,v.z));
}

void body_get_angular_velocity(void* body, Vector4* v) {
	const btVector3 bv = BODY(body)->getAngularVelocity();
	v->x = bv.getX();
	v->y = bv.getY();
	v->z = bv.getZ();
}

void body_set_angular_velocity(void* body, Vector4 v) {
	BODY(body)->setAngularVelocity(btVector3(v.x,v.y,v.z));
}

void body_set_friction(void* s, float f) {
	BODY(s)->setFriction(f);
}

float body_get_friction(void* s) {
	return BODY(s)->getFriction();
}

void body_set_deactivation(void* b, bool v) {
	if (v) {
		BODY(b)->forceActivationState(DEACTIVATION_ENABLE);
	} else {
		BODY(b)->forceActivationState(DEACTIVATION_DISABLE);
	}
}

void collision_callback(void* u, void (*callback)(void*, void*, const Vector4*, const Vector4*, const Vector4*) ) {
	int numManifolds = UNI(u)->dispatcher->getNumManifolds();
	
    for (int i = 0; i < numManifolds; i++) {
        btPersistentManifold* contactManifold = UNI(u)->dispatcher->getManifoldByIndexInternal(i);
        
        const btCollisionObject* obA = contactManifold->getBody0();
        const btCollisionObject* obB = contactManifold->getBody1();

        int numContacts = contactManifold->getNumContacts();
        for (int j = 0; j < numContacts; j++) {
            btManifoldPoint& pt = contactManifold->getContactPoint(j);
            if (pt.getDistance() < 0.f) {
                const btVector3& ptA = pt.getPositionWorldOnA();
                const btVector3& ptB = pt.getPositionWorldOnB();
                const btVector3& normalOnB = pt.m_normalWorldOnB;
                
                const Vector4 pa={ptA.getX(),ptA.getY(),ptA.getZ()};
                const Vector4 pb={ptB.getX(),ptB.getY(),ptB.getZ()};
                const Vector4 n={normalOnB.getX(),normalOnB.getY(),normalOnB.getZ()};
                callback((void*)obA, (void*)obB, &pa, &pb, &n);
            }
        }
    }
}
