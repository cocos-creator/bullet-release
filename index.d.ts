// tslint:disable
declare function instantiate(env: any, buffer: ArrayBuffer): Bullet.instance;

declare namespace Bullet {
    type ptr = number;
    interface instance {

        // math

        Vector3_create(x: number, y: number, z: number): ptr;
        Vector3_length(p: ptr): number;
        Vector3_x(p: ptr): number;
        Vector3_y(p: ptr): number;
        Vector3_z(p: ptr): number;
        Vector3_setValue(p: ptr, x: number, y: number, z: number): void;
        Vector3_normalize(p: ptr): void;

        Quaternion_create(x: number, y: number, z: number, w: number): ptr;
        Quaternion_x(p: ptr): number;
        Quaternion_y(p: ptr): number;
        Quaternion_z(p: ptr): number;
        Quaternion_w(p: ptr): number;
        Quaternion_setValue(p: ptr, x: number, y: number, z: number, w: number): void;

        Transform_create(): ptr;
        Transform_setIdentity(p: ptr): void;
        Transform_getOrigin(p: ptr): ptr;
        Transform_setRotation(p: ptr, quate: ptr): void;
        Transform_getRotationRef(p: ptr, quat: ptr): void;

        // shapes

        CollisionShape_isCompound(p: ptr): boolean;
        CollisionShape_setLocalScaling(p: ptr, scale: ptr): void;

        EmptyShape_create(): ptr;

        BoxShape_create(p: ptr): ptr;
        BoxShape_setUnscaledHalfExtents(p: ptr, halfExtents: ptr): void;

        SphereShape_create(radius: number): ptr;
        SphereShape_setUnscaledRadius(p: ptr, radius: number): void;

        StaticPlaneShape_create(normal: ptr, constant: number): ptr;
        StaticPlaneShape_getPlaneNormal(p: ptr): ptr;
        StaticPlaneShape_setPlaneConstant(p: ptr, constant: number): void;

        CompoundShape_create(enableDynamicAabbTree: boolean): ptr;
        CompoundShape_getNumChildShapes(p: ptr): number;
        CompoundShape_getChildShape(p: ptr, i: number): ptr;
        CompoundShape_addChildShape(p: ptr, local: ptr, shape: ptr): void;
        CompoundShape_removeChildShape(p: ptr, shape: ptr): void;
        CompoundShape_updateChildTransform(p: ptr, i: number, trans: ptr, shouldRecalculateLocalAabb: boolean): void;

        // collision

        CollisionObject_create(): number;
        CollisionObject_getCollisionShape(p: ptr): ptr;
        CollisionObject_setContactProcessingThreshold(p: ptr, contactProcessingThreshold: number): void;
        CollisionObject_getActivationState(p: ptr): number;
        CollisionObject_setActivationState(p: ptr, newState: number): void;
        CollisionObject_forceActivationState(p: ptr, newState: number): void;
        CollisionObject_activate(p: ptr, forceActivation?: boolean): void;
        CollisionObject_isActive(p: ptr): boolean;
        CollisionObject_isKinematicObject(p: ptr): boolean;
        CollisionObject_isStaticObject(p: ptr): boolean;
        CollisionObject_isStaticOrKinematicObject(p: ptr): boolean;
        CollisionObject_setRestitution(p: ptr, rest: number): void;
        CollisionObject_setFriction(p: ptr, frict: number): void;
        CollisionObject_setRollingFriction(p: ptr, frict: number): void;
        CollisionObject_getWorldTransform(p: ptr): ptr;
        CollisionObject_getCollisionFlags(p: ptr): number;
        CollisionObject_setCollisionFlags(p: ptr, flags: number): void;
        CollisionObject_setWorldTransform(p: ptr, transform: ptr): void;
        CollisionObject_setCollisionShape(p: ptr, shape: ptr): void;
        CollisionObject_setCcdMotionThreshold(p: ptr, ccdMotionThreshold: number): void;
        CollisionObject_setCcdSweptSphereRadius(p: ptr, radius: number): void;
        CollisionObject_getUserIndex(p: ptr): number;
        CollisionObject_setUserIndex(p: ptr, index: number): void;
        CollisionObject_getUserPointer(p: ptr): number;
        CollisionObject_setUserPointer(p: ptr, userPointer: number): void;

        RigidBodyConstructionInfo_create(m: number, ms: ptr, shape: ptr, localInertia: ptr): ptr;
        RigidBody_create(p: ptr): ptr;
        RigidBody_setMassProps(p: ptr, m: number, localInertia: ptr): void;
        RigidBody_clearState(p: ptr): void;
        RigidBody_setSleepingThresholds(p: ptr, linear: number, angular: number): void;
        RigidBody_getMotionState(p: ptr): ptr;

        // dynamic

        DefaultMotionState_create(transform: ptr): ptr;
        MotionState_getWorldTransform(p: ptr, transform: ptr): void;
        MotionState_setWorldTransform(p: ptr, transform: ptr): void;

        DefaultCollisionConfiguration_create(): ptr;
        CollisionDispatcher_create(p: ptr): ptr;
        Dispatcher_getNumManifolds(p: ptr): number;
        Dispatcher_getManifoldByIndexInternal(p: ptr, i: number): ptr;

        ManifoldPoint_getShape0(p: ptr): ptr;
        ManifoldPoint_getShape1(p: ptr): ptr;
        ManifoldPoint_get_m_index0(p: ptr): number;
        ManifoldPoint_get_m_index1(p: ptr): number;
        PersistentManifold_getBody0(p: ptr): ptr;
        PersistentManifold_getBody1(p: ptr): ptr;
        PersistentManifold_getNumContacts(p: ptr): number;
        PersistentManifold_getContactPoint(p: ptr, i: number): ptr;

        DbvtBroadphase_create(): ptr;
        SequentialImpulseConstraintSolver_create(): ptr;

        CollisionWorld_addCollisionObject(p: ptr, body: ptr, g: number, m: number): void;
        CollisionWorld_removeCollisionObject(p: ptr, body: ptr): void;

        DiscreteDynamicsWorld_create(dispatcher: ptr, pairCache: ptr, solver: ptr, config: ptr): ptr;
        DiscreteDynamicsWorld_setGravity(p: ptr, g: ptr): void;
        DiscreteDynamicsWorld_stepSimulation(p: ptr, timeStep: number, maxSubSteps: number, fixedTimeStep: number): ptr;
        DiscreteDynamicsWorld_addRigidBody(p: ptr, body: ptr, g: number, m: number): void;
        DiscreteDynamicsWorld_removeRigidBody(p: ptr, body: ptr): void;
    }
}

declare module '@cocos/bullet' {
    export = instantiate;
}
