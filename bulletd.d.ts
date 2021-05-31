// tslint:disable
declare function instantiate(env: any, buffer: ArrayBuffer): Bullet.instance;

declare namespace Bullet {
    interface instance {
        //btVector3
        btVector3_create(x: number, y: number, z: number): number;
        btVector3_length(ptr: number,): number;
        btVector3_x(ptr: number,): number;
        btVector3_y(ptr: number,): number;
        btVector3_z(ptr: number,): number;
        btVector3_setX(ptr: number, x: number): void;
        btVector3_setY(ptr: number, y: number): void;
        btVector3_setZ(ptr: number, z: number): void;
        btVector3_setValue(ptr: number, x: number, y: number, z: number): void;
        btVector3_normalize(ptr: number,): void;
        // /**[Value] */
        // btVector3_rotate (ptr: number, wAxis: btVector3, angle: number): btVector3;
        // btVector3_dot (ptr: number, v: btVector3): number;
        // btVector3_op_mul (ptr: number, x: number): btVector3;
        // btVector3_op_add (ptr: number, v: btVector3): btVector3;
        // btVector3_op_sub (ptr: number, v: btVector3): btVector3;

        //btVector4
        btVector4_create(x: number, y: number, z: number, w: number): number;
        btVector4_w(ptr: number): number;
        setValue(x: number, y: number, z: number): void;
        setValue(x: number, y: number, z: number, w: number): void;

        /**
         * btQuaternion
         */

        btQuaternion_x(): number;
        btQuaternion_y(): number;
        btQuaternion_z(): number;
        btQuaternion_w(): number;
        btQuaternion_setX(x: number): void;
        btQuaternion_setY(y: number): void;
        btQuaternion_setZ(z: number): void;
        btQuaternion_setW(w: number): void;
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
    }
}

declare module '@cocos/bullet' {
    export = instantiate;
}
