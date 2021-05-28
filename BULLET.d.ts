// tslint:disable
declare function Bullet(): Promise<void>;

declare namespace Bullet {

    //btVector3
    function btVector3_create(x: number, y: number, z: number): number;
    function btVector3_length(ptr: number,): number;
    function btVector3_x(ptr: number,): number;
    function btVector3_y(ptr: number,): number;
    function btVector3_z(ptr: number,): number;
    function btVector3_setX(ptr: number, x: number): void;
    function btVector3_setY(ptr: number, y: number): void;
    function btVector3_setZ(ptr: number, z: number): void;
    function btVector3_setValue(ptr: number, x: number, y: number, z: number): void;
    function btVector3_normalize(ptr: number,): void;
    // /**[Value] */
    // function btVector3_rotate (ptr: number, wAxis: btVector3, angle: number): btVector3;
    // function btVector3_dot (ptr: number, v: btVector3): number;
    // function btVector3_op_mul (ptr: number, x: number): btVector3;
    // function btVector3_op_add (ptr: number, v: btVector3): btVector3;
    // function btVector3_op_sub (ptr: number, v: btVector3): btVector3;

    //btVector4
    function btVector4_create(x: number, y: number, z: number, w: number): number;
    function btVector4_w(ptr: number): number;
    function setValue(x: number, y: number, z: number): void;
    function setValue(x: number, y: number, z: number, w: number): void;

    /**
     * btQuaternion
     */

    function btQuaternion_x(): number;
    function btQuaternion_y(): number;
    function btQuaternion_z(): number;
    function btQuaternion_w(): number;
    function btQuaternion_setX(x: number): void;
    function btQuaternion_setY(y: number): void;
    function btQuaternion_setZ(z: number): void;
    function btQuaternion_setW(w: number): void;
    function btQuaternion_setValue(x: number, y: number, z: number, w: number): void;
    // function btQuaternion_setEulerZYX (z: number, y: number, x: number): void;
    // function btQuaternion_setRotation (axis: btVector3, angle: number): void;
    // function btQuaternion_normalize (): void;
    // function btQuaternion_length2 (): number;
    // function btQuaternion_length (): number;
    // function btQuaternion_dot (q: btQuaternion): number;
    // /**[Value] */
    // function btQuaternion_normalized (): btQuaternion;
    // /**[Value] */
    // function btQuaternion_getAxis (): btVector3;
    // /**[Value] */
    // function btQuaternion_inverse (): btQuaternion;
    // function btQuaternion_getAngle (): number;
    // function btQuaternion_getAngleShortestPath (): number;
    // function btQuaternion_angle (q: btQuaternion): number;
    // function btQuaternion_angleShortestPath (q: btQuaternion): number;
    // function btQuaternion_op_add (q: btQuaternion): btQuaternion;
    // function btQuaternion_op_sub (q: btQuaternion): btQuaternion;
    // function btQuaternion_op_mul (s: number): btQuaternion;
    // function btQuaternion_op_mulq (q: btQuaternion): btQuaternion;
    // function btQuaternion_op_div (s: number): btQuaternion;

    /**
     * btMatrix3x3 
     */

    function btMatrix3x3_setEulerZYX(ex: number, ey: number, ez: number): void;
    // function btMatrix3x3_getRotation (q: btQuaternion): void;
    // function btMatrix3x3_getRow (y: number): btVector3;

    /**
     * btTransform
     */

    function btTransform_create(): number;
    function btTransform_setIdentity(): void;
    // function btTransform_setOrigin (origin: btVector3): void;
    // function btTransform_setRotation (rotation: btQuaternion): void;
    // function btTransform_getOrigin (): btVector3;
    // /**[Value] */
    // function btTransform_getRotation (): btQuaternion;
    // function btTransform_getBasis (): btMatrix3x3;
    // function btTransform_setFromOpenGLMatrix (m: number[]): void;
    // /**[Value] */
    // function btTransform_inverse (): btTransform;
    // function btTransform_op_mul (t: btTransform): btTransform;

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

    function btCollisionObject_create(): number;
    // function btCollisionObject_setAnisotropicFriction(anisotropicFriction: btVector3, frictionMode: number): void;
    // function btCollisionObject_getCollisionShape (): btCollisionShape;
    function btCollisionObject_setContactProcessingThreshold(contactProcessingThreshold: number): void;
    function btCollisionObject_getActivationState(): number;
    function btCollisionObject_setActivationState(newState: number): void;
    function btCollisionObject_forceActivationState(newState: number): void;
    function btCollisionObject_activate(forceActivation?: boolean): void;
    function btCollisionObject_isActive(): boolean;
    function btCollisionObject_isKinematicObject(): boolean;
    function btCollisionObject_isStaticObject(): boolean;
    function btCollisionObject_isStaticOrKinematicObject(): boolean;
    function btCollisionObject_setRestitution(rest: number): void;
    function btCollisionObject_setFriction(frict: number): void;
    function btCollisionObject_setRollingFriction(frict: number): void;
    // function btCollisionObject_getWorldTransform(): btTransform;
    function btCollisionObject_getCollisionFlags(): number;
    function btCollisionObject_setCollisionFlags(flags: number): void;
    // function btCollisionObject_setWorldTransform(worldTrans: btTransform): void;
    // function btCollisionObject_setCollisionShape (collisionShape: btCollisionShape): void;
    function btCollisionObject_setCcdMotionThreshold(ccdMotionThreshold: number): void;
    function btCollisionObject_setCcdSweptSphereRadius(radius: number): void;
    function btCollisionObject_getUserIndex(): number;
    function btCollisionObject_setUserIndex(index: number): void;
    function btCollisionObject_getUserPointer(): number;
    function btCollisionObject_setUserPointer(userPointer: number): void;
    // function btCollisionObject_getBroadphaseHandle (): btBroadphaseProxy;

    // class RayResultCallback {
    //     public m_shapePart: number;
    //     public m_collisionFilterGroup: number;
    //     public m_collisionFilterMask: number;
    //     public m_closestHitFraction: number;
    //     public m_collisionObject: btCollisionObject;
    //     public hasHit (): boolean;
    //     public get_m_collisionObject (): btCollisionObject;
    // }

    // class ClosestRayResultCallback extends RayResultCallback {
    //     public m_rayFromWorld: btVector3;
    //     public m_rayToWorld: btVector3;
    //     public m_hitNormalWorld: btVector3;
    //     public m_hitPointWorld: btVector3;
    //     constructor (from: btVector3, to: btVector3);
    //     public get_m_hitPointWorld (): btVector3;
    //     public get_m_hitNormalWorld (): btVector3;
    // }

    // interface btArray<T> {
    //     clear (): void;
    //     size (): number;
    //     at (n: number): T;
    // }
    // interface btNumberArray extends btArray<number> { }
    // interface btConstCollisionObjectArray extends btArray<btCollisionObject> { }

    // class AllHitsRayResultCallback extends RayResultCallback {
    //     public m_rayFromWorld: btVector3;
    //     public m_rayToWorld: btVector3;
    //     public m_hitNormalWorld: btVector3;
    //     public m_hitPointWorld: btVector3;
    //     constructor (from: btVector3, to: btVector3);
    //     public get_m_hitPointWorld (): btVector3;
    //     public get_m_hitNormalWorld (): btVector3;
    //     public m_hitFractions: btNumberArray;
    //     public m_collisionObjects: btConstCollisionObjectArray;
    //     public m_shapeParts: btNumberArray;
    // }

    // class btManifoldPoint {
    //     public m_localPointA: btVector3;
    //     public m_localPointB: btVector3;
    //     public m_positionWorldOnB: btVector3;
    //     public m_positionWorldOnA: btVector3;
    //     public m_normalWorldOnB: btVector3;
    //     public getPositionWorldOnA (): btVector3;
    //     public getPositionWorldOnB (): btVector3;
    //     public getAppliedImpulse (): number;
    //     public getDistance (): number;

    //     public get_m_positionWorldOnB (): btVector3;
    //     public get_m_normalWorldOnB (): btVector3;

    //     public m_distance1: number;
    //     public m_combinedFriction: number;
    //     public m_combinedRollingFriction: number;
    //     public m_combinedRestitution: number;

    //     //BP mod, store contact triangles.
    //     public m_partId0: number;
    //     public m_partId1: number;
    //     public m_index0: number;
    //     public m_index1: number;

    //     // Contact callback support
    //     public m_userPersistentData: any;
    //     public m_userPersistentData0: any;
    //     public m_userPersistentData1: any;

    //     public getShape0 (): btCollisionShape;
    //     public getShape1 (): btCollisionShape;
    // }

    // class ContactResultCallback {
    //     public addSingleResult (
    //         cp: btManifoldPoint,
    //         colObj0Wrap: btCollisionObjectWrapper,
    //         partId0: number,
    //         index0: number,
    //         colObj1Wrap: btCollisionObjectWrapper,
    //         partId1: number,
    //         index1: number,
    //     ): number;
    // }

    // class ConcreteContactResultCallback extends ContactResultCallback { }

    // class LocalShapeInfo {
    //     public m_shapePart: number;
    //     public m_triangleIndex: number;
    // }

    // class LocalConvexResult {
    //     public m_hitCollisionObject: btCollisionObject;
    //     public m_localShapeInfo: LocalShapeInfo;
    //     public m_hitNormalLocal: btVector3;
    //     public m_hitPointLocal: btVector3;
    //     public m_hitFraction: number;
    //     constructor (
    //         hitCollisionObject: btCollisionObject,
    //         localShapeInfo: LocalShapeInfo,
    //         hitNormalLocal: btVector3,
    //         hitPointLocal: btVector3,
    //         hitFraction: number,
    //     );
    // }

    // class ConvexResultCallback {
    //     public m_collisionFilterGroup: number;
    //     public m_collisionFilterMask: number;
    //     public m_closestHitFraction: number;
    //     public hasHit (): boolean;
    // }

    // class ClosestConvexResultCallback extends ConvexResultCallback {
    //     public m_convexFromWorld: btVector3;
    //     public m_convexToWorld: btVector3;
    //     public m_hitNormalWorld: btVector3;
    //     public m_hitPointWorld: btVector3;
    //     constructor (convexFromWorld: btVector3, convexToWorld: btVector3);
    // }

    // class btCollisionShape {
    //     public setLocalScaling (scaling: btVector3): void;
    //     public getLocalScaling (): btVector3;
    //     public calculateLocalInertia (mass: number, inertia: btVector3): void;
    //     public setMargin (margin: number): void;
    //     public getMargin (): number;
    //     public setUserIndex (i: number): void;
    //     public isCompound (): boolean;
    // }

    // class btConvexShape extends btCollisionShape { }

    // class btEmptyShape extends btCollisionShape { }

    // class btConvexTriangleMeshShape extends btConvexShape {
    //     constructor (meshInterface: btStridingMeshInterface, calcAabb?: boolean);
    // }

    // class btBoxShape extends btCollisionShape {
    //     constructor (boxHalfExtents: btVector3);
    //     public setMargin (margin: number): void;
    //     public getMargin (): number;
    // }

    // class btCapsuleShape extends btCollisionShape {
    //     constructor (radius: number, height: number);
    //     public setMargin (margin: number): void;
    //     public getMargin (): number;
    //     public getUpAxis (): number;
    //     public getRadius (): number;
    //     public getHalfHeight (): number;
    //     public setUpAxis (upAxis: number): void;
    //     public getImplicitShapeDimensions (): btVector3;
    // }

    // class btCapsuleShapeX extends btCapsuleShape {
    //     constructor (radius: number, height: number);
    //     public setMargin (margin: number): void;
    //     public getMargin (): number;
    // }

    // class btCapsuleShapeZ extends btCapsuleShape {
    //     constructor (radius: number, height: number);
    //     public setMargin (margin: number): void;
    //     public getMargin (): number;
    // }

    // class btCylinderShape extends btCollisionShape {
    //     constructor (halfExtents: btVector3);
    //     public setMargin (margin: number): void;
    //     public getMargin (): number;
    // }

    // class btCylinderShapeX extends btCylinderShape {
    //     constructor (halfExtents: btVector3);
    //     public setMargin (margin: number): void;
    //     public getMargin (): number;
    // }

    // class btCylinderShapeZ extends btCylinderShape {
    //     constructor (halfExtents: btVector3);
    //     public setMargin (margin: number): void;
    //     public getMargin (): number;
    // }

    // class btSphereShape extends btCollisionShape {
    //     constructor (radius: number);
    //     public setMargin (margin: number): void;
    //     public getMargin (): number;
    // }

    // class btConeShape extends btCollisionShape {
    //     constructor (radius: number, height: number);
    // }

    // class btConvexHullShape extends btCollisionShape {
    //     constructor (points?: number[], numPoints?: number);
    //     public addPoint (point: btVector3, recalculateLocalAABB?: boolean): void;
    //     public setMargin (margin: number): void;
    //     public getMargin (): number;
    //     public getNumVertices (): number;
    //     public initializePolyhedralFeatures (shiftVerticesByMargin: number): boolean;
    // }

    // class btShapeHull {
    //     constructor (shape: btConvexShape);
    //     public buildHull (margin: number): boolean;
    //     public numVertices (): number;
    //     public getVertexPointer (): btVector3;
    // }

    // class btConeShapeX extends btConeShape {
    //     constructor (radius: number, height: number);
    // }

    // class btConeShapeZ extends btConeShape {
    //     constructor (radius: number, height: number);
    // }

    // class btCompoundShape extends btCollisionShape {
    //     constructor (enableDynamicAabbTree?: boolean);
    //     public addChildShape (localTransform: btTransform, shape: btCollisionShape): void;
    //     public removeChildShape (shape: btCollisionShape): void;
    //     public removeChildShapeByIndex (childShapeindex: number): void;
    //     public getNumChildShapes (): number;
    //     public getChildShape (index: number): btCollisionShape;
    //     public setMargin (margin: number): void;
    //     public getMargin (): number;
    //     public updateChildTransform (childIndex: number, newChildTransform: btTransform, shouldRecalculateLocalAabb?: boolean): void;
    //     public setMaterial (childShapeindex: number, f: number, r: number, rf?: number): void;
    // }

    // class btStridingMeshInterface { }

    // class btTriangleMesh extends btStridingMeshInterface {
    //     constructor (use32bitIndices?: boolean, use4componentVertices?: boolean);
    //     public addTriangle (
    //         vertex0: btVector3,
    //         vertex1: btVector3,
    //         vertex2: btVector3,
    //         removeDuplicateVertices?: boolean,
    //     ): void;
    // }

    // enum PHY_ScalarType {
    //     PHY_FLOAT,
    //     PHY_DOUBLE,
    //     PHY_INTEGER,
    //     PHY_SHORT,
    //     PHY_FIXEDPOINT88,
    //     PHY_UCHAR,
    // }

    // class btConcaveShape extends btCollisionShape { }

    // class btStaticPlaneShape extends btConcaveShape {
    //     constructor (planeNormal: btVector3, planeConstant: number);
    // }

    // class btTriangleMeshShape extends btConcaveShape { }

    // class btBvhTriangleMeshShape extends btTriangleMeshShape {
    //     constructor (
    //         meshInterface: btStridingMeshInterface,
    //         useQuantizedAabbCompression: boolean,
    //         buildBvh?: boolean,
    //     );
    // }

    // class btHeightfieldTerrainShape extends btConcaveShape {
    //     constructor (
    //         heightStickWidth: number,
    //         heightStickLength: number,
    //         heightfieldData: VoidPtr,
    //         heightScale: number,
    //         minHeight: number,
    //         maxHeight: number,
    //         upAxis: number,
    //         hdt: PHY_ScalarType,
    //         flipQuadEdges: boolean,
    //     );
    //     public setMargin (margin: number): void;
    //     public getMargin (): number;
    // }

    // class btDefaultCollisionConstructionInfo {
    //     constructor ();
    // }

    // class btDefaultCollisionConfiguration {
    //     constructor (info?: btDefaultCollisionConstructionInfo);
    // }

    // class btPersistentManifold {
    //     constructor ();
    //     public getBody0 (): btCollisionObject;
    //     public getBody1 (): btCollisionObject;
    //     public getNumContacts (): number;
    //     public getContactPoint (index: number): btManifoldPoint;
    // }

    // class btDispatcher {
    //     public getNumManifolds (): number;
    //     public getManifoldByIndexInternal (index: number): btPersistentManifold;
    // }

    // class btCollisionDispatcher extends btDispatcher {
    //     constructor (conf: btDefaultCollisionConfiguration);
    // }

    // class btOverlappingPairCallback { }

    // class btOverlappingPairCache {
    //     public setInternalGhostPairCallback (
    //         ghostPairCallback: btOverlappingPairCallback,
    //     ): void;
    //     getNumOverlappingPairs (): number;
    // }

    // class btAxisSweep3 {
    //     constructor (
    //         worldAabbMin: btVector3,
    //         worldAabbMax: btVector3,
    //         maxHandles?: number,
    //         pairCache?: btOverlappingPairCache,
    //         disableRaycastAccelerator?: boolean,
    //     );
    // }

    // class btBroadphaseInterface {
    //     getOverlappingPairCache (): btOverlappingPairCache;
    // }

    // class btCollisionConfiguration { }

    // class btDbvtBroadphase extends btBroadphaseInterface {
    //     constructor ();
    // }

    // interface btBroadphaseProxy {
    //     m_collisionFilterGroup: number;
    //     m_collisionFilterMask: number;
    // }

    // class btRigidBodyConstructionInfo {

    //     public m_linearDamping: number;
    //     public m_angularDamping: number;
    //     public m_friction: number;
    //     public m_rollingFriction: number;
    //     public m_restitution: number;
    //     public m_linearSleepingThreshold: number;
    //     public m_angularSleepingThreshold: number;
    //     public m_additionalDamping: boolean;
    //     public m_additionalDampingFactor: number;
    //     public m_additionalLinearDampingThresholdSqr: number;
    //     public m_additionalAngularDampingThresholdSqr: number;
    //     public m_additionalAngularDampingFactor: number;
    //     constructor (
    //         mass: number,
    //         motionState: btMotionState,
    //         collisionShape: btCollisionShape,
    //         localInertia?: btVector3,
    //     );
    // }

    // class btRigidBody extends btCollisionObject {
    //     constructor (constructionInfo: btRigidBodyConstructionInfo);
    //     public getCenterOfMassTransform (): btTransform;
    //     public setCenterOfMassTransform (xform: btTransform): void;
    //     public setSleepingThresholds (linear: number, angular: number): void;
    //     public getLinearDamping (): number;
    //     public getAngularDamping (): number;
    //     public setDamping (lin_damping: number, ang_damping: number): void;
    //     public setMassProps (mass: number, inertia: btVector3): void;
    //     public setLinearFactor (linearFactor: btVector3): void;
    //     public applyTorque (torque: btVector3): void;
    //     public applyLocalTorque (torque: btVector3): void;
    //     public applyForce (force: btVector3, rel_pos: btVector3): void;
    //     public applyCentralForce (force: btVector3): void;
    //     public applyCentralLocalForce (force: btVector3): void;
    //     public applyTorqueImpulse (torque: btVector3): void;
    //     public applyImpulse (impulse: btVector3, rel_pos: btVector3): void;
    //     public applyCentralImpulse (impulse: btVector3): void;
    //     public updateInertiaTensor (): void;
    //     public getLinearVelocity (): btVector3;
    //     public getAngularVelocity (): btVector3;
    //     public setLinearVelocity (lin_vel: btVector3): void;
    //     public setAngularVelocity (ang_vel: btVector3): void;
    //     public getMotionState (): btMotionState;
    //     public setMotionState (motionState: btMotionState): void;
    //     public setAngularFactor (angularFactor: btVector3): void;
    //     public getAngularFactor (): btVector3;
    //     public getLinearFactor (): btVector3;
    //     public upcast (colObj: btCollisionObject): btRigidBody;
    //     public getAabb (aabbMin: btVector3, aabbMax: btVector3): void;
    //     public applyGravity (): void;
    //     public getGravity (): btVector3;
    //     public setGravity (acceleration: btVector3): void;
    //     public getBroadphaseProxy (): btBroadphaseProxy;
    //     public getFlags (): number;
    //     public setFlags (flags: number): void;
    //     public wantsSleeping (): boolean;

    //     // XXX
    //     public clearState (): void;
    // }

    // class btConstraintSetting {
    //     public m_tau: number;
    //     public m_damping: number;
    //     public m_impulseClamp: number;
    //     constructor ();
    // }

    // class btTypedConstraint {
    //     public enableFeedback (needsFeedback: boolean): void;
    //     public getBreakingImpulseThreshold (): number;
    //     public setBreakingImpulseThreshold (threshold: number): void;
    //     public getParam (num: number, axis: number): number;
    //     public setParam (num: number, value: number, axis: number): void;
    // }

    // enum btConstraintParams {
    //     BT_CONSTRAINT_ERP,
    //     BT_CONSTRAINT_STOP_ERP,
    //     BT_CONSTRAINT_CFM,
    //     BT_CONSTRAINT_STOP_CFM,
    // }

    // class btPoint2PointConstraint extends btTypedConstraint {

    //     public m_setting: btConstraintSetting;
    //     constructor (
    //         rbA: btRigidBody,
    //         rbB: btRigidBody,
    //         pivotInA: btVector3,
    //         pivotInB: btVector3,
    //     );
    //     constructor (rbA: btRigidBody, pivotInA: btVector3);
    //     public setPivotA (pivotA: btVector3): void;
    //     public setPivotB (pivotB: btVector3): void;
    //     public getPivotInA (): btVector3;
    //     public getPivotInB (): btVector3;
    // }

    // class btGeneric6DofConstraint extends btTypedConstraint {
    //     constructor (
    //         rbA: btRigidBody,
    //         rbB: btRigidBody,
    //         frameInA: btTransform,
    //         frameInB: btTransform,
    //         useLinearFrameReferenceFrameA: boolean,
    //     );
    //     constructor (
    //         rbB: btRigidBody,
    //         frameInB: btTransform,
    //         useLinearFrameReferenceFrameB: boolean,
    //     );
    //     public setLinearLowerLimit (linearLower: btVector3): void;
    //     public setLinearUpperLimit (linearUpper: btVector3): void;
    //     public setAngularLowerLimit (angularLower: btVector3): void;
    //     public setAngularUpperLimit (angularUpper: btVector3): void;
    //     public getFrameOffsetA (): btTransform;
    // }

    // class btGeneric6DofSpringConstraint extends btGeneric6DofConstraint {
    //     constructor (
    //         rbA: btRigidBody,
    //         rbB: btRigidBody,
    //         frameInA: btTransform,
    //         frameInB: btTransform,
    //         useLinearFrameReferenceFrameA: boolean,
    //     );
    //     constructor (
    //         rbB: btRigidBody,
    //         frameInB: btTransform,
    //         useLinearFrameReferenceFrameB: boolean,
    //     );
    //     public enableSpring (index: number, onOff: boolean): void;
    //     public setStiffness (index: number, stiffness: number): void;
    //     public setDamping (index: number, damping: number): void;
    // }

    // class btSequentialImpulseConstraintSolver {
    //     constructor ();
    // }

    // class btConeTwistConstraint extends btTypedConstraint {
    //     constructor (
    //         rbA: btRigidBody,
    //         rbB: btRigidBody,
    //         rbAFrame: btTransform,
    //         rbBFrame: btTransform,
    //     );
    //     constructor (rbA: btRigidBody, rbAFrame: btTransform);
    //     public setLimit (limitIndex: number, limitValue: number): void;
    //     public setAngularOnly (angularOnly: boolean): void;
    //     public setDamping (damping: number): void;
    //     public enableMotor (b: boolean): void;
    //     public setMaxMotorImpulse (maxMotorImpulse: number): void;
    //     public setMaxMotorImpulseNormalized (maxMotorImpulse: number): void;
    //     public setMotorTarget (q: btQuaternion): void;
    //     public setMotorTargetInConstraintSpace (q: btQuaternion): void;
    // }

    // class btHingeConstraint extends btTypedConstraint {
    //     constructor (
    //         rbA: btRigidBody,
    //         rbB: btRigidBody,
    //         pivotInA: btVector3,
    //         pivotInB: btVector3,
    //         axisInA: btVector3,
    //         axisInB: btVector3,
    //         useReferenceFrameA?: boolean,
    //     );
    //     constructor (
    //         rbA: btRigidBody,
    //         rbB: btRigidBody,
    //         rbAFrame: btTransform,
    //         rbBFrame: btTransform,
    //         useReferenceFrameA?: boolean,
    //     );
    //     constructor (
    //         rbA: btRigidBody,
    //         rbAFrame: btTransform,
    //         useReferenceFrameA?: boolean,
    //     );
    //     public setLimit (
    //         low: number,
    //         high: number,
    //         softness: number,
    //         biasFactor: number,
    //         relaxationFactor?: number,
    //     ): void;
    //     public enableAngularMotor (
    //         enableMotor: boolean,
    //         targetVelocity: number,
    //         maxMotorImpulse: number,
    //     ): void;
    //     public setAngularOnly (angularOnly: boolean): void;
    //     public enableMotor (enableMotor: boolean): void;
    //     public setMaxMotorImpulse (maxMotorImpulse: number): void;
    //     public setMotorTarget (targetAngle: number, dt: number): void;
    // }

    // class btSliderConstraint extends btTypedConstraint {
    //     constructor (
    //         rbA: btRigidBody,
    //         rbB: btRigidBody,
    //         frameInA: btTransform,
    //         frameInB: btTransform,
    //         useLinearReferenceFrameA: boolean,
    //     );
    //     constructor (
    //         rbB: btRigidBody,
    //         frameInB: btTransform,
    //         useLinearReferenceFrameA: boolean,
    //     );
    //     public setLowerLinLimit (lowerLimit: number): void;
    //     public setUpperLinLimit (upperLimit: number): void;
    //     public setLowerAngLimit (lowerAngLimit: number): void;
    //     public setUpperAngLimit (upperAngLimit: number): void;
    // }

    // class btFixedConstraint extends btTypedConstraint {
    //     constructor (
    //         rbA: btRigidBody,
    //         rbB: btRigidBody,
    //         frameInA: btTransform,
    //         frameInB: btTransform,
    //     );
    // }

    // class btConstraintSolver { }

    // class btDispatcherInfo {
    //     public m_timeStep: number;
    //     public m_stepCount: number;
    //     public m_dispatchFunc: number;
    //     public m_timeOfImpact: number;
    //     public m_useContinuous: boolean;
    //     public m_enableSatConvex: boolean;
    //     public m_enableSPU: boolean;
    //     public m_useEpa: boolean;
    //     public m_allowedCcdPenetration: number;
    //     public m_useConvexConservativeDistanceUtil: boolean;
    //     public m_convexConservativeDistanceThreshold: number;
    // }

    // class btCollisionWorld {
    //     public getDispatcher (): btDispatcher;
    //     public rayTest (
    //         rayFromWorld: btVector3,
    //         rayToWorld: btVector3,
    //         resultCallback: RayResultCallback,
    //     ): void;
    //     public getPairCache (): btOverlappingPairCache;
    //     public getDispatchInfo (): btDispatcherInfo;
    //     public addCollisionObject (
    //         collisionObject: btCollisionObject,
    //         collisionFilterGroup?: number,
    //         collisionFilterMask?: number,
    //     ): void;
    //     public removeCollisionObject (collisionObject: btCollisionObject): void;
    //     public getBroadphase (): btBroadphaseInterface;
    //     public convexSweepTest (
    //         castShape: btConvexShape,
    //         from: btTransform,
    //         to: btTransform,
    //         resultCallback: ConvexResultCallback,
    //         allowedCcdPenetration: number,
    //     ): void;
    //     public contactPairTest (
    //         colObjA: btCollisionObject,
    //         colObjB: btCollisionObject,
    //         resultCallback: ContactResultCallback,
    //     ): void;
    //     public contactTest (
    //         colObj: btCollisionObject,
    //         resultCallback: ContactResultCallback,
    //     ): void;
    //     public updateSingleAabb (colObj: btCollisionObject): void;
    //     public setDebugDrawer (debugDrawer: btIDebugDraw): void;
    //     public getDebugDrawer (): btIDebugDraw;
    //     public debugDrawWorld (): void;
    //     public debugDrawObject (
    //         worldTransform: btTransform,
    //         shape: btCollisionShape,
    //         color: btVector3,
    //     ): void;
    // }

    // class btContactSolverInfo {
    //     public m_splitImpulse: boolean;
    //     public m_splitImpulsePenetrationThreshold: number;
    //     public m_numIterations: number;
    // }

    // class btDynamicsWorld extends btCollisionWorld {
    //     public addAction (action: btActionInterface): void;
    //     public removeAction (action: btActionInterface): void;
    //     public getSolverInfo (): btContactSolverInfo;
    // }

    // class btDiscreteDynamicsWorld extends btDynamicsWorld {
    //     constructor (
    //         dispatcher: btDispatcher,
    //         pairCache: btBroadphaseInterface,
    //         constraintSolver: btConstraintSolver,
    //         collisionConfiguration: btCollisionConfiguration,
    //     );
    //     public setGravity (gravity: btVector3): void;
    //     public getGravity (): btVector3;
    //     public addRigidBody (body: btRigidBody): void;
    //     public addRigidBody (body: btRigidBody, group: number, mask: number): void;
    //     public removeRigidBody (body: btRigidBody): void;
    //     public addConstraint (
    //         constraint: btTypedConstraint,
    //         disableCollisionsBetweenLinkedBodies?: boolean,
    //     ): void;
    //     public removeConstraint (constraint: btTypedConstraint): void;
    //     public stepSimulation (
    //         timeStep: number,
    //         maxSubSteps?: number,
    //         fixedTimeStep?: number,
    //     ): number;

    //     // Contact callback support
    //     public setContactAddedCallback (funcpointer: number): void;
    //     public setContactProcessedCallback (funcpointer: number): void;
    //     public setContactDestroyedCallback (funcpointer: number): void;
    // }

    // class btVehicleTuning {
    //     public m_suspensionStiffness: number;
    //     public m_suspensionCompression: number;
    //     public m_suspensionDamping: number;
    //     public m_maxSuspensionTravelCm: number;
    //     public m_frictionSlip: number;
    //     public m_maxSuspensionForce: number;
    //     constructor ();
    // }

    // class btVehicleRaycasterResult {
    //     public m_hitPointInWorld: btVector3;
    //     public m_hitNormalInWorld: btVector3;
    //     public m_distFraction: number;
    // }

    // class btVehicleRaycaster {
    //     public castRay (
    //         from: btVector3,
    //         to: btVector3,
    //         result: btVehicleRaycasterResult,
    //     ): void;
    // }

    // class btDefaultVehicleRaycaster extends btVehicleRaycaster {
    //     // constructor (world: btDynamicsWorld);
    //     public addAction (action: btActionInterface): void;
    //     public removeAction (action: btActionInterface): void;
    //     public getSolverInfo (): btContactSolverInfo;
    // }

    // class RaycastInfo {
    //     public m_contactNormalWS: btVector3;
    //     public m_contactPointWS: btVector3;
    //     public m_suspensionLength: number;
    //     public m_hardPointWS: btVector3;
    //     public m_wheelDirectionWS: btVector3;
    //     public m_wheelAxleWS: btVector3;
    //     public m_isInContact: boolean;
    //     public m_groundObject: any;
    // }

    // class btWheelInfoConstructionInfo {
    //     public m_chassisConnectionCS: btVector3;
    //     public m_wheelDirectionCS: btVector3;
    //     public m_wheelAxleCS: btVector3;
    //     public m_suspensionRestLength: number;
    //     public m_maxSuspensionTravelCm: number;
    //     public m_wheelRadius: number;
    //     public m_suspensionStiffness: number;
    //     public m_wheelsDampingCompression: number;
    //     public m_wheelsDampingRelaxation: number;
    //     public m_frictionSlip: number;
    //     public m_maxSuspensionForce: number;
    //     public m_bIsFrontWheel: boolean;
    // }

    // class btWheelInfo {
    //     public m_suspensionStiffness: number;
    //     public m_frictionSlip: number;
    //     public m_engineForce: number;
    //     public m_rollInfluence: number;
    //     public m_suspensionRestLength1: number;
    //     public m_wheelsRadius: number;
    //     public m_wheelsDampingCompression: number;
    //     public m_wheelsDampingRelaxation: number;
    //     public m_steering: number;
    //     public m_maxSuspensionForce: number;
    //     public m_maxSuspensionTravelCm: number;
    //     public m_wheelsSuspensionForce: number;
    //     public m_bIsFrontWheel: boolean;
    //     public m_raycastInfo: RaycastInfo;
    //     public m_chassisConnectionPointCS: btVector3;
    //     public m_worldTransform: btTransform;
    //     public m_wheelDirectionCS: btVector3;
    //     public m_wheelAxleCS: btVector3;
    //     public m_rotation: number;
    //     public m_deltaRotation: number;
    //     public m_brake: number;
    //     public m_clippedInvContactDotSuspension: number;
    //     public m_suspensionRelativeVelocity: number;
    //     public m_skidInfo: number;
    //     constructor (ci: btWheelInfoConstructionInfo);
    //     public getSuspensionRestLength (): number;
    //     public updateWheel (chassis: btRigidBody, raycastInfo: RaycastInfo): void;
    // }

    // class btActionInterface {
    //     public updateAction (collisionWorld: btCollisionWorld, deltaTimeStep: number): void;
    // }

    // class btKinematicCharacterController extends btActionInterface {
    //     constructor (
    //         ghostObject: btPairCachingGhostObject,
    //         convexShape: btConvexShape,
    //         stepHeight: number,
    //         upAxis?: number,
    //     );
    //     public setUpAxis (axis: number): void;
    //     public setWalkDirection (walkDirection: btVector3): void;
    //     public setVelocityForTimeInterval (velocity: btVector3, timeInterval: number): void;
    //     public warp (origin: btVector3): void;
    //     public preStep (collisionWorld: btCollisionWorld): void;
    //     public playerStep (collisionWorld: btCollisionWorld, dt: number): void;
    //     public setFallSpeed (fallSpeed: number): void;
    //     public setJumpSpeed (jumpSpeed: number): void;
    //     public setMaxJumpHeight (maxJumpHeight: number): void;
    //     public canJump (): boolean;
    //     public jump (): void;
    //     public setGravity (gravity: number): void;
    //     public getGravity (): number;
    //     public setMaxSlope (slopeRadians: number): void;
    //     public getMaxSlope (): number;
    //     public getGhostObject (): btPairCachingGhostObject;
    //     public setUseGhostSweepTest (useGhostObjectSweepTest: boolean): void;
    //     public onGround (): boolean;
    //     public setUpInterpolate (value: boolean): void;
    // }

    // class btRaycastVehicle extends btActionInterface {
    //     constructor (
    //         tuning: btVehicleTuning,
    //         chassis: btRigidBody,
    //         raycaster: btVehicleRaycaster,
    //     );
    //     public applyEngineForce (force: number, wheel: number): void;
    //     public setSteeringValue (steering: number, wheel: number): void;
    //     public getWheelTransformWS (wheelIndex: number): btTransform;
    //     public updateWheelTransform (
    //         wheelIndex: number,
    //         interpolatedTransform: boolean,
    //     ): void;
    //     public addWheel (
    //         connectionPointCS0: btVector3,
    //         wheelDirectionCS0: btVector3,
    //         wheelAxleCS: btVector3,
    //         suspensionRestLength: number,
    //         wheelRadius: number,
    //         tuning: btVehicleTuning,
    //         isFrontWheel: boolean,
    //     ): btWheelInfo;
    //     public getNumWheels (): number;
    //     public getRigidBody (): btRigidBody;
    //     public getWheelInfo (index: number): btWheelInfo;
    //     public setBrake (brake: number, wheelIndex: number): void;
    //     public setCoordinateSystem (
    //         rightIndex: number,
    //         upIndex: number,
    //         forwardIndex: number,
    //     ): void;
    //     public getCurrentSpeedKmHour (): number;
    //     public getChassisWorldTransform (): btTransform;
    //     public rayCast (wheel: btWheelInfo): number;
    //     public updateVehicle (step: number): void;
    //     public resetSuspension (): void;
    //     public getSteeringValue (wheel: number): number;
    //     public updateWheelTransformsWS (
    //         wheel: btWheelInfo,
    //         interpolatedTransform?: boolean,
    //     ): void;
    //     public setPitchControl (pitch: number): void;
    //     public updateSuspension (deltaTime: number): void;
    //     public updateFriction (timeStep: number): void;
    //     public getRightAxis (): number;
    //     public getUpAxis (): number;
    //     public getForwardAxis (): number;
    //     public getForwardVector (): btVector3;
    //     public getUserConstraintType (): number;
    //     public setUserConstraintType (userConstraintType: number): void;
    //     public setUserConstraintId (uid: number): void;
    //     public getUserConstraintId (): number;
    // }

    // class btGhostObject extends btCollisionObject {
    //     constructor ();
    //     public getNumOverlappingObjects (): number;
    //     public getOverlappingObject (index: number): btCollisionObject;
    // }

    // class btPairCachingGhostObject extends btGhostObject {
    //     constructor ();
    // }

    // class btGhostPairCallback {
    //     constructor ();
    // }

    // class btSoftBodyWorldInfo {
    //     public air_density: number;
    //     public water_density: number;
    //     public water_offset: number;
    //     public m_maxDisplacement: number;
    //     public water_normal: btVector3;
    //     public m_broadphase: btBroadphaseInterface;
    //     public m_dispatcher: btDispatcher;
    //     public m_gravity: btVector3;
    //     constructor ();
    // }

    // class Node {
    //     public m_x: btVector3;
    //     public m_q: btVector3;
    //     public m_v: btVector3;
    //     public m_f: btVector3;
    //     public m_n: btVector3;
    //     public m_im: number;
    //     public m_area: number;
    // }

    // class tNodeArray {
    //     public size (): number;
    //     public at (n: number): Node;
    // }

    // class Material {
    //     public m_kLST: number;
    //     public m_kAST: number;
    //     public m_kVST: number;
    //     public m_flags: number;
    // }

    // class tMaterialArray {
    //     public size (): number;
    //     public at (n: number): Material;
    // }

    // class Anchor {
    //     public m_node: Node;
    //     public m_local: btVector3;
    //     public m_body: btRigidBody;
    //     public m_influence: number;
    //     public m_c0: btMatrix3x3;
    //     public m_c1: btVector3;
    //     public m_c2: number;
    // }

    // class tAnchorArray {
    //     public size (): number;
    //     public at (n: number): Anchor;
    //     public clear (): void;
    //     public push_back (val: Anchor): void;
    //     public pop_back (): void;
    // }

    // class Config {
    //     public kVCF: number;
    //     public kDP: number;
    //     public kDG: number;
    //     public kLF: number;
    //     public kPR: number;
    //     public kVC: number;
    //     public kDF: number;
    //     public kMT: number;
    //     public kCHR: number;
    //     public kKHR: number;
    //     public kSHR: number;
    //     public kAHR: number;
    //     public kSRHR_CL: number;
    //     public kSKHR_CL: number;
    //     public kSSHR_CL: number;
    //     public kSR_SPLT_CL: number;
    //     public kSK_SPLT_CL: number;
    //     public kSS_SPLT_CL: number;
    //     public maxvolume: number;
    //     public timescale: number;
    //     public viterations: number;
    //     public piterations: number;
    //     public diterations: number;
    //     public citerations: number;
    //     public collisions: number;
    // }

    // class btSoftBody extends btCollisionObject {
    //     public m_cfg: Config;
    //     public m_nodes: tNodeArray;
    //     public m_materials: tMaterialArray;
    //     public m_anchors: tAnchorArray;
    //     constructor (
    //         worldInfo: btSoftBodyWorldInfo,
    //         node_count: number,
    //         x: btVector3,
    //         m: number[],
    //     );
    //     public checkLink (node0: number, node1: number): boolean;
    //     public checkFace (node0: number, node1: number, node2: number): boolean;
    //     public appendMaterial (): Material;
    //     public appendNode (x: btVector3, m: number): void;
    //     public appendLink (
    //         node0: number,
    //         node1: number,
    //         mat: Material,
    //         bcheckexist: boolean,
    //     ): void;
    //     public appendFace (
    //         node0: number,
    //         node1: number,
    //         node2: number,
    //         mat: Material,
    //     ): void;
    //     public appendTetra (
    //         node0: number,
    //         node1: number,
    //         node2: number,
    //         node3: number,
    //         mat: Material,
    //     ): void;
    //     public appendAnchor (
    //         node: number,
    //         body: btRigidBody,
    //         disableCollisionBetweenLinkedBodies: boolean,
    //         influence: number,
    //     ): void;
    //     public addForce (force: btVector3): void;
    //     public addForce (force: btVector3, node: number): void;
    //     public addAeroForceToNode (windVelocity: btVector3, nodeIndex: number): void;
    //     public getTotalMass (): number;
    //     public setTotalMass (mass: number, fromfaces: boolean): void;
    //     public setMass (node: number, mass: number): void;
    //     public transform (trs: btTransform): void;
    //     public translate (trs: btVector3): void;
    //     public rotate (rot: btQuaternion): void;
    //     public scale (scl: btVector3): void;
    //     public generateClusters (k: number, maxiterations?: number): number;
    //     public generateBendingConstraints (distance: number, mat: Material): number;
    //     public upcast (colObj: btCollisionObject): btSoftBody;
    // }

    // class btSoftBodyRigidBodyCollisionConfiguration extends btDefaultCollisionConfiguration {
    //     constructor (info?: btDefaultCollisionConstructionInfo);
    // }

    // class btSoftBodySolver { }

    // class btDefaultSoftBodySolver extends btSoftBodySolver {
    //     constructor ();
    // }

    // class btSoftBodyArray {
    //     public size (): number;
    //     public at (n: number): btSoftBody;
    // }

    // class btSoftRigidDynamicsWorld extends btDiscreteDynamicsWorld {
    //     constructor (
    //         dispatcher: btDispatcher,
    //         pairCache: btBroadphaseInterface,
    //         constraintSolver: btConstraintSolver,
    //         collisionConfiguration: btCollisionConfiguration,
    //         softBodySolver: btSoftBodySolver,
    //     );
    //     public addSoftBody (
    //         body: btSoftBody,
    //         collisionFilterGroup: number,
    //         collisionFilterMask: number,
    //     ): void;
    //     public removeSoftBody (body: btSoftBody): void;
    //     public removeCollisionObject (collisionObject: btCollisionObject): void;
    //     public getWorldInfo (): btSoftBodyWorldInfo;
    //     public getSoftBodyArray (): btSoftBodyArray;
    // }

    // class btSoftBodyHelpers {
    //     constructor ();
    //     public CreateRope (
    //         worldInfo: btSoftBodyWorldInfo,
    //         from: btVector3,
    //         to: btVector3,
    //         res: number,
    //         fixeds: number,
    //     ): btSoftBody;
    //     public CreatePatch (
    //         worldInfo: btSoftBodyWorldInfo,
    //         corner00: btVector3,
    //         corner10: btVector3,
    //         corner01: btVector3,
    //         corner11: btVector3,
    //         resx: number,
    //         resy: number,
    //         fixeds: number,
    //         gendiags: boolean,
    //     ): btSoftBody;
    //     public CreatePatchUV (
    //         worldInfo: btSoftBodyWorldInfo,
    //         corner00: btVector3,
    //         corner10: btVector3,
    //         corner01: btVector3,
    //         corner11: btVector3,
    //         resx: number,
    //         resy: number,
    //         fixeds: number,
    //         gendiags: boolean,
    //         tex_coords: number[],
    //     ): btSoftBody;
    //     public CreateEllipsoid (
    //         worldInfo: btSoftBodyWorldInfo,
    //         center: btVector3,
    //         radius: btVector3,
    //         res: number,
    //     ): btSoftBody;
    //     public CreateFromTriMesh (
    //         worldInfo: btSoftBodyWorldInfo,
    //         vertices: number[],
    //         triangles: number[],
    //         ntriangles: number,
    //         randomizeConstraints: boolean,
    //     ): btSoftBody;
    //     public CreateFromConvexHull (
    //         worldInfo: btSoftBodyWorldInfo,
    //         vertices: btVector3,
    //         nvertices: number,
    //         randomizeConstraints: boolean,
    //     ): btSoftBody;
    // }

    // type Type =
    //     | btIDebugDraw
    //     | DebugDrawer
    //     | btVector3
    //     | btVector4
    //     | btQuadWord
    //     | btQuaternion
    //     | btMatrix3x3
    //     | btTransform
    //     | btMotionState
    //     | btDefaultMotionState
    //     | btCollisionObject
    //     | btCollisionObjectWrapper
    //     | RayResultCallback
    //     | ClosestRayResultCallback
    //     | btManifoldPoint
    //     | ContactResultCallback
    //     | ConcreteContactResultCallback
    //     | LocalShapeInfo
    //     | LocalConvexResult
    //     | ConvexResultCallback
    //     | ClosestConvexResultCallback
    //     | btCollisionShape
    //     | btConvexShape
    //     | btConvexTriangleMeshShape
    //     | btBoxShape
    //     | btCapsuleShape
    //     | btCapsuleShapeX
    //     | btCapsuleShapeZ
    //     | btCylinderShape
    //     | btCylinderShapeX
    //     | btCylinderShapeZ
    //     | btSphereShape
    //     | btConeShape
    //     | btConvexHullShape
    //     | btShapeHull
    //     | btConeShapeX
    //     | btConeShapeZ
    //     | btCompoundShape
    //     | btStridingMeshInterface
    //     | btTriangleMesh
    //     | btConcaveShape
    //     | btStaticPlaneShape
    //     | btTriangleMeshShape
    //     | btBvhTriangleMeshShape
    //     | btHeightfieldTerrainShape
    //     | btDefaultCollisionConstructionInfo
    //     | btDefaultCollisionConfiguration
    //     | btPersistentManifold
    //     | btDispatcher
    //     | btCollisionDispatcher
    //     | btOverlappingPairCallback
    //     | btOverlappingPairCache
    //     | btAxisSweep3
    //     | btBroadphaseInterface
    //     | btCollisionConfiguration
    //     | btDbvtBroadphase
    //     | btRigidBodyConstructionInfo
    //     | btRigidBody
    //     | btConstraintSetting
    //     | btTypedConstraint
    //     | btPoint2PointConstraint
    //     | btGeneric6DofConstraint
    //     | btGeneric6DofSpringConstraint
    //     | btSequentialImpulseConstraintSolver
    //     | btConeTwistConstraint
    //     | btHingeConstraint
    //     | btSliderConstraint
    //     | btFixedConstraint
    //     | btConstraintSolver
    //     | btDispatcherInfo
    //     | btCollisionWorld
    //     | btContactSolverInfo
    //     | btDynamicsWorld
    //     | btDiscreteDynamicsWorld
    //     | btVehicleTuning
    //     | btVehicleRaycasterResult
    //     | btVehicleRaycaster
    //     | btDefaultVehicleRaycaster
    //     | RaycastInfo
    //     | btWheelInfoConstructionInfo
    //     | btWheelInfo
    //     | btActionInterface
    //     | btKinematicCharacterController
    //     | btRaycastVehicle
    //     | btGhostObject
    //     | btPairCachingGhostObject
    //     | btGhostPairCallback
    //     | btSoftBodyWorldInfo
    //     | Node
    //     | tNodeArray
    //     | Material
    //     | tMaterialArray
    //     | Anchor
    //     | tAnchorArray
    //     | Config
    //     | btSoftBody
    //     | btSoftBodyRigidBodyCollisionConfiguration
    //     | btSoftBodySolver
    //     | btDefaultSoftBodySolver
    //     | btSoftBodyArray
    //     | btSoftRigidDynamicsWorld
    //     | btSoftBodyHelpers;

    //   function destroy (obj: Ammo.Type): void;

    //   function castObject<T> (...args: any): any;

    //   function wrapPointer<T extends Ammo.Type> (params: number, obj: new (...args: any[]) => T): T;

    //   function addFunction (params: Function): number;
}

declare module '@cocos/bullet' {
    export = Bullet;
}
