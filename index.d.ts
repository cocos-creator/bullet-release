// tslint:disable
declare function instantiate(env: any, buffer: ArrayBuffer): Bullet.instance;

declare namespace Bullet {
    type ptr = number;
    interface instance {
        //btVector3
        btVector3_create(x: number, y: number, z: number): ptr;
        btVector3_length(p: ptr,): number;
        btVector3_x(p: ptr,): number;
        btVector3_y(p: ptr,): number;
        btVector3_z(p: ptr,): number;
        btVector3_setX(p: ptr, x: number): void;
        btVector3_setY(p: ptr, y: number): void;
        btVector3_setZ(p: ptr, z: number): void;
        btVector3_setValue(p: ptr, x: number, y: number, z: number): void;
        btVector3_normalize(p: ptr,): void;
        // /**[Value] */
        // btVector3_rotate (p: ptr, wAxis: btVector3, angle: number): btVector3;
        // btVector3_dot (p: ptr, v: btVector3): number;
        // btVector3_op_mul (p: ptr, x: number): btVector3;
        // btVector3_op_add (p: ptr, v: btVector3): btVector3;
        // btVector3_op_sub (p: ptr, v: btVector3): btVector3;

        //btVector4
        btVector4_create(x: number, y: number, z: number, w: number): number;
        btVector4_w(p: ptr): number;
        setValue(x: number, y: number, z: number): void;
        setValue(x: number, y: number, z: number, w: number): void;

        /**
         * btQuaternion
         */

        btQuaternion_x(p: ptr): number;
        btQuaternion_y(p: ptr): number;
        btQuaternion_z(p: ptr): number;
        btQuaternion_w(p: ptr): number;
        btQuaternion_setX(p: ptr, x: number): void;
        btQuaternion_setY(p: ptr, y: number): void;
        btQuaternion_setZ(p: ptr, z: number): void;
        btQuaternion_setW(p: ptr, w: number): void;
        btQuaternion_setValue(x: number, y: number, z: number, w: number): void;
        // btQuaternion_setEulerZYX (z: number, y: number, x: number): void;
        // btQuaternion_setRotation (axis: btVector3, angle: number): void;
        // btQuaternion_normalize (): void;
        // btQuaternion_length2 (): number;
        // btQuaternion_length (): number;
        // btQuaternion_dot (q: btQuaternion): number;
        // /**[Value] */
        // btQuaternion_normalized (): btQuaternion;
        // /**[Value] */
        // btQuaternion_getAxis (): btVector3;
        // /**[Value] */
        // btQuaternion_inverse (): btQuaternion;
        // btQuaternion_getAngle (): number;
        // btQuaternion_getAngleShortestPath (): number;
        // btQuaternion_angle (q: btQuaternion): number;
        // btQuaternion_angleShortestPath (q: btQuaternion): number;
        // btQuaternion_op_add (q: btQuaternion): btQuaternion;
        // btQuaternion_op_sub (q: btQuaternion): btQuaternion;
        // btQuaternion_op_mul (s: number): btQuaternion;
        // btQuaternion_op_mulq (q: btQuaternion): btQuaternion;
        // btQuaternion_op_div (s: number): btQuaternion;

        /**
         * btMatrix3x3 
         */

        btMatrix3x3_setEulerZYX(ex: number, ey: number, ez: number): void;
        // btMatrix3x3_getRotation (q: btQuaternion): void;
        // btMatrix3x3_getRow (y: number): btVector3;

        /**
         * btTransform
         */

        btTransform_create(): number;
        btTransform_setIdentity(): void;
        // btTransform_setOrigin (origin: btVector3): void;
        // btTransform_setRotation (rotation: btQuaternion): void;
        // btTransform_getOrigin (): btVector3;
        // /**[Value] */
        // btTransform_getRotation (): btQuaternion;
        // btTransform_getBasis (): btMatrix3x3;
        // btTransform_setFromOpenGLMatrix (m: number[]): void;
        // /**[Value] */
        // btTransform_inverse (): btTransform;
        // btTransform_op_mul (t: btTransform): btTransform;

        // class btMotionState {
        //     public getWorldTransform (worldTrans: btTransform): void;
        //     public setWorldTransform (worldTrans: btTransform): void;
        // }

        // class btDefaultMotionState extends btMotionState {
        //     public m_graphicsWorldTrans: btTransform;
        //     constructor (startTrans?: btTransform, centerOfMassOffset?: btTransform);
        // }

        /**
         * btCollisionObject
         */

        btCollisionObject_create(): number;
        // btCollisionObject_setAnisotropicFriction(anisotropicFriction: btVector3, frictionMode: number): void;
        // btCollisionObject_getCollisionShape (): btCollisionShape;
        btCollisionObject_setContactProcessingThreshold(contactProcessingThreshold: number): void;
        btCollisionObject_getActivationState(): number;
        btCollisionObject_setActivationState(newState: number): void;
        btCollisionObject_forceActivationState(newState: number): void;
        btCollisionObject_activate(forceActivation?: boolean): void;
        btCollisionObject_isActive(): boolean;
        btCollisionObject_isKinematicObject(): boolean;
        btCollisionObject_isStaticObject(): boolean;
        btCollisionObject_isStaticOrKinematicObject(): boolean;
        btCollisionObject_setRestitution(rest: number): void;
        btCollisionObject_setFriction(frict: number): void;
        btCollisionObject_setRollingFriction(frict: number): void;
        // btCollisionObject_getWorldTransform(): btTransform;
        btCollisionObject_getCollisionFlags(): number;
        btCollisionObject_setCollisionFlags(flags: number): void;
        // btCollisionObject_setWorldTransform(worldTrans: btTransform): void;
        // btCollisionObject_setCollisionShape (collisionShape: btCollisionShape): void;
        btCollisionObject_setCcdMotionThreshold(ccdMotionThreshold: number): void;
        btCollisionObject_setCcdSweptSphereRadius(radius: number): void;
        btCollisionObject_getUserIndex(): number;
        btCollisionObject_setUserIndex(index: number): void;
        btCollisionObject_getUserPointer(): number;
        btCollisionObject_setUserPointer(userPointer: number): void;

        // shape
        btEmptyShape_create(): ptr;
        btBoxShape_create(p: ptr): ptr;
        btSphereShape_create(radius: number): ptr;
    }
}

declare module '@cocos/bullet' {
    export = instantiate;
}
