export default Ammo;
declare function Ammo<T>(target?: T): Promise<T & typeof Ammo>;
declare module Ammo {
    function destroy(obj: any): void;
    function _malloc(size: number): number;
    function _free(ptr: number): void;
    const HEAP8: Int8Array;
    const HEAP16: Int16Array;
    const HEAP32: Int32Array;
    const HEAPU8: Uint8Array;
    const HEAPU16: Uint16Array;
    const HEAPU32: Uint32Array;
    const HEAPF32: Float32Array;
    const HEAPF64: Float64Array;
    class btCompat {
        setInternalTickCallback(world: btDynamicsWorld, cb: unknown, worldUserInfo?: unknown, isPreTick?: boolean): void;
    }
    class btIDebugDraw {
        drawLine(from: btVector3, to: btVector3, color: btVector3): void;
        drawContactPoint(pointOnB: btVector3, normalOnB: btVector3, distance: number, lifeTime: number, color: btVector3): void;
        reportErrorWarning(warningString: string): void;
        draw3dText(location: btVector3, textString: string): void;
        setDebugMode(debugMode: number): void;
        getDebugMode(): number;
    }
    class DebugDrawer {
        constructor();
        drawLine(from: btVector3, to: btVector3, color: btVector3): void;
        drawContactPoint(pointOnB: btVector3, normalOnB: btVector3, distance: number, lifeTime: number, color: btVector3): void;
        reportErrorWarning(warningString: string): void;
        draw3dText(location: btVector3, textString: string): void;
        setDebugMode(debugMode: number): void;
        getDebugMode(): number;
    }
    class btVector3 {
        constructor();
        constructor(x: number, y: number, z: number);
        op_add(v: btVector3): btVector3;
        op_sub(v: btVector3): btVector3;
        op_mul(s: number): btVector3;
        op_div(s: number): btVector3;
        dot(v: btVector3): number;
        length2(): number;
        length(): number;
        norm(): number;
        distance2(v: btVector3): number;
        distance(v: btVector3): number;
        normalize(): btVector3;
        normalized(): btVector3;
        rotate(wAxis: btVector3, angle: number): btVector3;
        angle(v: btVector3): number;
        absolute(): btVector3;
        cross(v: btVector3): btVector3;
        triple(v1: btVector3, v2: btVector3): number;
        minAxis(): number;
        maxAxis(): number;
        furthestAxis(): number;
        closestAxis(): number;
        setInterpolate3(v0: btVector3, v1: btVector3, rt: number): void;
        lerp(v: btVector3, t: number): btVector3;
        op_mulv(v: btVector3): btVector3;
        getX(): number;
        getY(): number;
        getZ(): number;
        setX(x: number): void;
        setY(y: number): void;
        setZ(z: number): void;
        setW(w: number): void;
        x(): number;
        y(): number;
        z(): number;
        w(): number;
        op_eq(other: btVector3): boolean;
        op_neq(other: btVector3): boolean;
        setMax(other: btVector3): void;
        setMin(other: btVector3): void;
        setValue(x: number, y: number, z: number): void;
        getSkewSymmetricMatrix(v0: btVector3, v1: btVector3, v2: btVector3): void;
        setZero(): void;
        isZero(): boolean;
        fuzzyZero(): boolean;
        dot3(v0: btVector3, v1: btVector3, v2: btVector3): btVector3;
    }
    class btVector4 extends btVector3 {
        constructor();
        constructor(x: number, y: number, z: number, w: number);
        absolute4(): btVector4;
        getW(): number;
        maxAxis4(): number;
        minAxis4(): number;
        closestAxis4(): number;
        setValue(x: number, y: number, z: number, w: number): void;
    }
    class btQuadWord {
        constructor();
        constructor(x: number, y: number, z: number, w: number);
        getX(): number;
        getY(): number;
        getZ(): number;
        setX(x: number): void;
        setY(y: number): void;
        setZ(z: number): void;
        setW(w: number): void;
        x(): number;
        y(): number;
        z(): number;
        w(): number;
        op_eq(other: btQuadWord): boolean;
        op_neq(other: btQuadWord): boolean;
        setValue(x: number, y: number, z: number): void;
        setValue(x: number, y: number, z: number, w: number): void;
        setMax(other: btQuadWord): void;
        setMin(other: btQuadWord): void;
    }
    class btQuaternion extends btQuadWord {
        constructor();
        constructor(x: number, y: number, z: number, w: number);
        constructor(axis: btVector3, angle: number);
        constructor(yaw: number, pitch: number, roll: number);
        setRotation(axis: btVector3, angle: number): void;
        setEuler(yaw: number, pitch: number, roll: number): void;
        setEulerZYX(yawZ: number, pitchY: number, rollX: number): void;
        op_add(q: btQuaternion): btQuaternion;
        op_sub(q: btQuaternion): btQuaternion;
        op_mul(s: number): btQuaternion;
        op_mulq(q: btQuaternion): btQuaternion;
        dot(q: btQuaternion): number;
        length2(): number;
        length(): number;
        normalize(): void;
        op_div(s: number): btQuaternion;
        normalized(): btQuaternion;
        angle(q: btQuaternion): number;
        angleShortestPath(q: btQuaternion): number;
        getAngle(): number;
        getAngleShortestPath(): number;
        getAxis(): btVector3;
        inverse(): btQuaternion;
        slerp(q: btQuaternion, t: number): btQuaternion;
        getIdentity(): btQuaternion;
        getW(): number;
    }
    class btMatrix3x3 {
        constructor();
        constructor(q: btQuaternion);
        constructor(xx: number, xy: number, xz: number, yx: number, yy: number, yz: number, zx: number, zy: number, zz: number);
        constructor(v0: btVector3, v1: btVector3, v2: btVector3);
        getColumn(i: number): btVector3;
        getRow(i: number): btVector3;
        op_id(i: number): btVector3;
        op_mul(m: btMatrix3x3): btMatrix3x3;
        op_add(m: btMatrix3x3): btMatrix3x3;
        op_div(m: btMatrix3x3): btMatrix3x3;
        setValue(xx: number, xy: number, xz: number, yx: number, yy: number, yz: number, zx: number, zy: number, zz: number): void;
        setRotation(q: btQuaternion): void;
        setEulerYPR(yaw: number, pitch: number, roll: number): void;
        setEulerZYX(eulerX: number, eulerY: number, eulerZ: number): void;
        setIdentity(): void;
        setZero(): void;
        getIdentity(): btMatrix3x3;
        getRotation(q: btQuaternion): void;
        scaled(s: btVector3): btMatrix3x3;
        determinant(): number;
        adjoint(): btMatrix3x3;
        absolute(): btMatrix3x3;
        transpose(): btMatrix3x3;
        inverse(): btMatrix3x3;
        solve33(b: btVector3): btVector3;
        transposeTimes(m: btMatrix3x3): btMatrix3x3;
        timesTranspose(m: btMatrix3x3): btMatrix3x3;
        tdotx(v: btVector3): number;
        tdoty(v: btVector3): number;
        tdotz(v: btVector3): number;
        extractRotation(q: btQuaternion, tolerance?: number, maxIter?: number): void;
        diagonalize(rot: btMatrix3x3, threshold: number, maxSteps: number): void;
        cofac(r1: number, c1: number, r2: number, c2: number): number;
    }
    class btTransform {
        constructor();
        constructor(q: btQuaternion, v?: btVector3);
        op_set(other: btTransform): btTransform;
        mult(t1: btTransform, t2: btTransform): void;
        getBasis(): btMatrix3x3;
        getOrigin(): btVector3;
        getRotation(): btQuaternion;
        setOrigin(origin: btVector3): void;
        invXform(inVec: btVector3): btVector3;
        setBasis(basis: btMatrix3x3): void;
        setRotation(q: btQuaternion): void;
        setIdentity(): void;
        op_mul(t: btTransform): btTransform;
        inverse(): btTransform;
        inverseTimes(t: btTransform): btTransform;
        getIdentity(): btTransform;
    }
    class btTypedObject {
        constructor(objectType: number);
        get_m_objectType(): number;
        set_m_objectType(m_objectType: number): void;
        m_objectType: number;
        getObjectType(): number;
    }
    class btMotionState {
        getWorldTransform(worldTrans: btTransform): void;
        setWorldTransform(worldTrans: btTransform): void;
    }
    class btDefaultMotionState extends btMotionState {
        constructor(startTrans?: btTransform, centerOfMassOffset?: btTransform);
        get_m_graphicsWorldTrans(): btTransform;
        set_m_graphicsWorldTrans(m_graphicsWorldTrans: btTransform): void;
        m_graphicsWorldTrans: btTransform;
        get_m_centerOfMassOffset(): btTransform;
        set_m_centerOfMassOffset(m_centerOfMassOffset: btTransform): void;
        m_centerOfMassOffset: btTransform;
        get_m_startWorldTrans(): btTransform;
        set_m_startWorldTrans(m_startWorldTrans: btTransform): void;
        m_startWorldTrans: btTransform;
        get_m_userPointer(): unknown;
        set_m_userPointer(m_userPointer: unknown): void;
        m_userPointer: unknown;
        getWorldTransform(centerOfMassWorldTrans: btTransform): void;
        setWorldTransform(centerOfMassWorldTrans: btTransform): void;
    }
    class btIntArray {
        size(): number;
        at(n: number): number;
    }
    class btFace {
        get_m_indices(): btIntArray;
        set_m_indices(m_indices: btIntArray): void;
        m_indices: btIntArray;
        get_m_plane(): ReadonlyArray<number>;
        set_m_plane(m_plane: ReadonlyArray<number>): void;
        m_plane: ReadonlyArray<number>;
    }
    class btVector3Array {
        size(): number;
        at(n: number): btVector3;
    }
    class btFaceArray {
        size(): number;
        at(n: number): btFace;
    }
    class btCollisionObject {
        constructor();
        mergesSimulationIslands(): boolean;
        getAnisotropicFriction(): btVector3;
        setAnisotropicFriction(anisotropicFriction: btVector3, frictionMode?: number): void;
        hasAnisotropicFriction(frictionMode?: number): boolean;
        setContactProcessingThreshold(contactProcessingThreshold: number): void;
        getContactProcessingThreshold(): number;
        isStaticObject(): boolean;
        isKinematicObject(): boolean;
        isStaticOrKinematicObject(): boolean;
        hasContactResponse(): boolean;
        setCollisionShape(collisionShape: btCollisionShape): void;
        getCollisionShape(): btCollisionShape;
        setIgnoreCollisionCheck(co: btCollisionObject, ignoreCollisionCheck: boolean): void;
        getNumObjectsWithoutCollision(): number;
        getObjectWithoutCollision(index: number): btCollisionObject;
        checkCollideWithOverride(co: btCollisionObject): boolean;
        getActivationState(): number;
        setActivationState(newState: number): void;
        setDeactivationTime(time: number): void;
        getDeactivationTime(): number;
        forceActivationState(newState: number): void;
        activate(forceActivation?: boolean): void;
        isActive(): boolean;
        setRestitution(rest: number): void;
        getRestitution(): number;
        setFriction(frict: number): void;
        getFriction(): number;
        setRollingFriction(frict: number): void;
        getRollingFriction(): number;
        setSpinningFriction(frict: number): void;
        getSpinningFriction(): number;
        setContactStiffnessAndDamping(stiffness: number, damping: number): void;
        getContactStiffness(): number;
        getContactDamping(): number;
        getWorldTransform(): btTransform;
        setWorldTransform(worldTrans: btTransform): void;
        getBroadphaseHandle(): btBroadphaseProxy;
        setBroadphaseHandle(handle: btBroadphaseProxy): void;
        getInterpolationWorldTransform(): btTransform;
        setInterpolationWorldTransform(trans: btTransform): void;
        setInterpolationLinearVelocity(linvel: btVector3): void;
        setInterpolationAngularVelocity(angvel: btVector3): void;
        getInterpolationLinearVelocity(): btVector3;
        getInterpolationAngularVelocity(): btVector3;
        getIslandTag(): number;
        setIslandTag(tag: number): void;
        getCompanionId(): number;
        setCompanionId(id: number): void;
        getWorldArrayIndex(): number;
        getHitFraction(): number;
        setHitFraction(hitFraction: number): void;
        getCollisionFlags(): number;
        setCollisionFlags(flags: number): void;
        getCcdSweptSphereRadius(): number;
        setCcdSweptSphereRadius(radius: number): void;
        getCcdMotionThreshold(): number;
        setCcdMotionThreshold(ccdMotionThreshold: number): void;
        getUserPointer(): unknown;
        getUserIndex(): number;
        getUserIndex2(): number;
        getUserIndex3(): number;
        setUserPointer(userPointer: unknown): void;
        setUserIndex(index: number): void;
        setUserIndex2(index: number): void;
        setUserIndex3(index: number): void;
        getUpdateRevisionInternal(): number;
        checkCollideWith(co: btCollisionObject): boolean;
    }
    class btCollisionObjectWrapper {
        get_m_partId(): number;
        set_m_partId(m_partId: number): void;
        m_partId: number;
        get_m_index(): number;
        set_m_index(m_index: number): void;
        m_index: number;
        getWorldTransform(): btTransform;
        getCollisionObject(): btCollisionObject;
        getCollisionShape(): btCollisionShape;
    }
    class RayResultCallback {
        get_m_closestHitFraction(): number;
        set_m_closestHitFraction(m_closestHitFraction: number): void;
        m_closestHitFraction: number;
        get_m_collisionObject(): btCollisionObject;
        set_m_collisionObject(m_collisionObject: btCollisionObject): void;
        m_collisionObject: btCollisionObject;
        get_m_collisionFilterGroup(): number;
        set_m_collisionFilterGroup(m_collisionFilterGroup: number): void;
        m_collisionFilterGroup: number;
        get_m_collisionFilterMask(): number;
        set_m_collisionFilterMask(m_collisionFilterMask: number): void;
        m_collisionFilterMask: number;
        get_m_flags(): number;
        set_m_flags(m_flags: number): void;
        m_flags: number;
        hasHit(): boolean;
        needsCollision(proxy0: btBroadphaseProxy): boolean;
    }
    class LocalRayResult {
        constructor(collisionObject: btCollisionObject, localShapeInfo: LocalShapeInfo, hitNormalLocal: btVector3, hitFraction: number);
        get_m_collisionObject(): btCollisionObject;
        set_m_collisionObject(m_collisionObject: btCollisionObject): void;
        m_collisionObject: btCollisionObject;
        get_m_localShapeInfo(): LocalShapeInfo;
        set_m_localShapeInfo(m_localShapeInfo: LocalShapeInfo): void;
        m_localShapeInfo: LocalShapeInfo;
        get_m_hitNormalLocal(): btVector3;
        set_m_hitNormalLocal(m_hitNormalLocal: btVector3): void;
        m_hitNormalLocal: btVector3;
        get_m_hitFraction(): number;
        set_m_hitFraction(m_hitFraction: number): void;
        m_hitFraction: number;
    }
    class ClosestRayResultCallback extends RayResultCallback {
        constructor(from: btVector3, to: btVector3);
        get_m_rayFromWorld(): btVector3;
        set_m_rayFromWorld(m_rayFromWorld: btVector3): void;
        m_rayFromWorld: btVector3;
        get_m_rayToWorld(): btVector3;
        set_m_rayToWorld(m_rayToWorld: btVector3): void;
        m_rayToWorld: btVector3;
        get_m_hitNormalWorld(): btVector3;
        set_m_hitNormalWorld(m_hitNormalWorld: btVector3): void;
        m_hitNormalWorld: btVector3;
        get_m_hitPointWorld(): btVector3;
        set_m_hitPointWorld(m_hitPointWorld: btVector3): void;
        m_hitPointWorld: btVector3;
        addSingleResult(rayResult: LocalRayResult, normalInWorldSpace: boolean): number;
    }
    class btConstCollisionObjectArray {
        size(): number;
        at(n: number): btCollisionObject;
    }
    class btScalarArray {
        size(): number;
        at(n: number): number;
    }
    class AllHitsRayResultCallback extends RayResultCallback {
        constructor(from: btVector3, to: btVector3);
        get_m_collisionObjects(): btConstCollisionObjectArray;
        set_m_collisionObjects(m_collisionObjects: btConstCollisionObjectArray): void;
        m_collisionObjects: btConstCollisionObjectArray;
        get_m_rayFromWorld(): btVector3;
        set_m_rayFromWorld(m_rayFromWorld: btVector3): void;
        m_rayFromWorld: btVector3;
        get_m_rayToWorld(): btVector3;
        set_m_rayToWorld(m_rayToWorld: btVector3): void;
        m_rayToWorld: btVector3;
        get_m_hitNormalWorld(): btVector3Array;
        set_m_hitNormalWorld(m_hitNormalWorld: btVector3Array): void;
        m_hitNormalWorld: btVector3Array;
        get_m_hitPointWorld(): btVector3Array;
        set_m_hitPointWorld(m_hitPointWorld: btVector3Array): void;
        m_hitPointWorld: btVector3Array;
        get_m_hitFractions(): btScalarArray;
        set_m_hitFractions(m_hitFractions: btScalarArray): void;
        m_hitFractions: btScalarArray;
        addSingleResult(rayResult: LocalRayResult, normalInWorldSpace: boolean): number;
    }
    class LocalConvexResult {
        constructor(hitCollisionObject: btCollisionObject, localShapeInfo: LocalShapeInfo, hitNormalLocal: btVector3, hitPointLocal: btVector3, hitFraction: number);
        get_m_hitCollisionObject(): btCollisionObject;
        set_m_hitCollisionObject(m_hitCollisionObject: btCollisionObject): void;
        m_hitCollisionObject: btCollisionObject;
        get_m_localShapeInfo(): LocalShapeInfo;
        set_m_localShapeInfo(m_localShapeInfo: LocalShapeInfo): void;
        m_localShapeInfo: LocalShapeInfo;
        get_m_hitNormalLocal(): btVector3;
        set_m_hitNormalLocal(m_hitNormalLocal: btVector3): void;
        m_hitNormalLocal: btVector3;
        get_m_hitPointLocal(): btVector3;
        set_m_hitPointLocal(m_hitPointLocal: btVector3): void;
        m_hitPointLocal: btVector3;
        get_m_hitFraction(): number;
        set_m_hitFraction(m_hitFraction: number): void;
        m_hitFraction: number;
    }
    class btManifoldPoint {
        constructor();
        constructor(pointA: btVector3, pointB: btVector3, normal: btVector3, distance: number);
        get_m_localPointA(): btVector3;
        set_m_localPointA(m_localPointA: btVector3): void;
        m_localPointA: btVector3;
        get_m_localPointB(): btVector3;
        set_m_localPointB(m_localPointB: btVector3): void;
        m_localPointB: btVector3;
        get_m_positionWorldOnB(): btVector3;
        set_m_positionWorldOnB(m_positionWorldOnB: btVector3): void;
        m_positionWorldOnB: btVector3;
        get_m_positionWorldOnA(): btVector3;
        set_m_positionWorldOnA(m_positionWorldOnA: btVector3): void;
        m_positionWorldOnA: btVector3;
        get_m_normalWorldOnB(): btVector3;
        set_m_normalWorldOnB(m_normalWorldOnB: btVector3): void;
        m_normalWorldOnB: btVector3;
        get_m_distance1(): number;
        set_m_distance1(m_distance1: number): void;
        m_distance1: number;
        get_m_combinedFriction(): number;
        set_m_combinedFriction(m_combinedFriction: number): void;
        m_combinedFriction: number;
        get_m_combinedRollingFriction(): number;
        set_m_combinedRollingFriction(m_combinedRollingFriction: number): void;
        m_combinedRollingFriction: number;
        get_m_combinedSpinningFriction(): number;
        set_m_combinedSpinningFriction(m_combinedSpinningFriction: number): void;
        m_combinedSpinningFriction: number;
        get_m_combinedRestitution(): number;
        set_m_combinedRestitution(m_combinedRestitution: number): void;
        m_combinedRestitution: number;
        get_m_partId0(): number;
        set_m_partId0(m_partId0: number): void;
        m_partId0: number;
        get_m_partId1(): number;
        set_m_partId1(m_partId1: number): void;
        m_partId1: number;
        get_m_index0(): number;
        set_m_index0(m_index0: number): void;
        m_index0: number;
        get_m_index1(): number;
        set_m_index1(m_index1: number): void;
        m_index1: number;
        get_m_userPersistentData(): any;
        set_m_userPersistentData(m_userPersistentData: any): void;
        m_userPersistentData: any;
        get_m_contactPointFlags(): number;
        set_m_contactPointFlags(m_contactPointFlags: number): void;
        m_contactPointFlags: number;
        get_m_appliedImpulse(): number;
        set_m_appliedImpulse(m_appliedImpulse: number): void;
        m_appliedImpulse: number;
        get_m_prevRHS(): number;
        set_m_prevRHS(m_prevRHS: number): void;
        m_prevRHS: number;
        get_m_appliedImpulseLateral1(): number;
        set_m_appliedImpulseLateral1(m_appliedImpulseLateral1: number): void;
        m_appliedImpulseLateral1: number;
        get_m_appliedImpulseLateral2(): number;
        set_m_appliedImpulseLateral2(m_appliedImpulseLateral2: number): void;
        m_appliedImpulseLateral2: number;
        get_m_contactMotion1(): number;
        set_m_contactMotion1(m_contactMotion1: number): void;
        m_contactMotion1: number;
        get_m_contactMotion2(): number;
        set_m_contactMotion2(m_contactMotion2: number): void;
        m_contactMotion2: number;
        get_m_frictionCFM(): number;
        set_m_frictionCFM(m_frictionCFM: number): void;
        m_frictionCFM: number;
        get_m_lifeTime(): number;
        set_m_lifeTime(m_lifeTime: number): void;
        m_lifeTime: number;
        get_m_lateralFrictionDir1(): btVector3;
        set_m_lateralFrictionDir1(m_lateralFrictionDir1: btVector3): void;
        m_lateralFrictionDir1: btVector3;
        get_m_lateralFrictionDir2(): btVector3;
        set_m_lateralFrictionDir2(m_lateralFrictionDir2: btVector3): void;
        m_lateralFrictionDir2: btVector3;
        getDistance(): number;
        getLifeTime(): number;
        getPositionWorldOnA(): btVector3;
        getPositionWorldOnB(): btVector3;
        setDistance(dist: number): void;
        getAppliedImpulse(): number;
    }
    class ContactResultCallback {
        get_m_collisionFilterGroup(): number;
        set_m_collisionFilterGroup(m_collisionFilterGroup: number): void;
        m_collisionFilterGroup: number;
        get_m_collisionFilterMask(): number;
        set_m_collisionFilterMask(m_collisionFilterMask: number): void;
        m_collisionFilterMask: number;
        get_m_closestDistanceThreshold(): number;
        set_m_closestDistanceThreshold(m_closestDistanceThreshold: number): void;
        m_closestDistanceThreshold: number;
        needsCollision(proxy0: btBroadphaseProxy): boolean;
        addSingleResult(cp: btManifoldPoint, colObj0Wrap: btCollisionObjectWrapper, partId0: number, index0: number, colObj1Wrap: btCollisionObjectWrapper, partId1: number, index1: number): number;
    }
    class ConcreteContactResultCallback {
        constructor();
        addSingleResult(cp: btManifoldPoint, colObj0Wrap: btCollisionObjectWrapper, partId0: number, index0: number, colObj1Wrap: btCollisionObjectWrapper, partId1: number, index1: number): number;
    }
    class LocalShapeInfo {
        get_m_shapePart(): number;
        set_m_shapePart(m_shapePart: number): void;
        m_shapePart: number;
        get_m_triangleIndex(): number;
        set_m_triangleIndex(m_triangleIndex: number): void;
        m_triangleIndex: number;
    }
    class ConvexResultCallback {
        get_m_closestHitFraction(): number;
        set_m_closestHitFraction(m_closestHitFraction: number): void;
        m_closestHitFraction: number;
        get_m_collisionFilterGroup(): number;
        set_m_collisionFilterGroup(m_collisionFilterGroup: number): void;
        m_collisionFilterGroup: number;
        get_m_collisionFilterMask(): number;
        set_m_collisionFilterMask(m_collisionFilterMask: number): void;
        m_collisionFilterMask: number;
        hasHit(): boolean;
        needsCollision(proxy0: btBroadphaseProxy): boolean;
        addSingleResult(convexResult: LocalConvexResult, normalInWorldSpace: boolean): number;
    }
    class ClosestConvexResultCallback extends ConvexResultCallback {
        constructor(convexFromWorld: btVector3, convexToWorld: btVector3);
        get_m_convexFromWorld(): btVector3;
        set_m_convexFromWorld(m_convexFromWorld: btVector3): void;
        m_convexFromWorld: btVector3;
        get_m_convexToWorld(): btVector3;
        set_m_convexToWorld(m_convexToWorld: btVector3): void;
        m_convexToWorld: btVector3;
        get_m_hitNormalWorld(): btVector3;
        set_m_hitNormalWorld(m_hitNormalWorld: btVector3): void;
        m_hitNormalWorld: btVector3;
        get_m_hitPointWorld(): btVector3;
        set_m_hitPointWorld(m_hitPointWorld: btVector3): void;
        m_hitPointWorld: btVector3;
        get_m_hitCollisionObject(): btCollisionObject;
        set_m_hitCollisionObject(m_hitCollisionObject: btCollisionObject): void;
        m_hitCollisionObject: btCollisionObject;
        addSingleResult(convexResult: LocalConvexResult, normalInWorldSpace: boolean): number;
    }
    class btCollisionShape {
        getAabb(t: btTransform, aabbMin: btVector3, aabbMax: btVector3): void;
        getAngularMotionDisc(): number;
        getContactBreakingThreshold(defaultContactThresholdFactor: number): number;
        calculateTemporalAabb(curTrans: btTransform, linvel: btVector3, angvel: btVector3, timeStep: number, temporalAabbMin: btVector3, temporalAabbMax: btVector3): void;
        isPolyhedral(): boolean;
        isConvex2d(): boolean;
        isConvex(): boolean;
        isNonMoving(): boolean;
        isConcave(): boolean;
        isCompound(): boolean;
        isSoftBody(): boolean;
        isInfinite(): boolean;
        setLocalScaling(scaling: btVector3): void;
        getLocalScaling(): btVector3;
        calculateLocalInertia(mass: number, inertia: btVector3): void;
        getName(): string;
        getShapeType(): number;
        getAnisotropicRollingFrictionDirection(): btVector3;
        setMargin(margin: number): void;
        getMargin(): number;
        setUserPointer(userPtr: unknown): void;
        getUserPointer(): unknown;
        setUserIndex(index: number): void;
        getUserIndex(): number;
        setUserIndex2(index: number): void;
        getUserIndex2(): number;
    }
    class btConvexShape extends btCollisionShape {
        localGetSupportingVertex(vec: btVector3): btVector3;
        localGetSupportingVertexWithoutMargin(vec: btVector3): btVector3;
        localGetSupportVertexWithoutMarginNonVirtual(vec: btVector3): btVector3;
        localGetSupportVertexNonVirtual(vec: btVector3): btVector3;
        getMarginNonVirtual(): number;
        getAabbNonVirtual(t: btTransform, aabbMin: btVector3, aabbMax: btVector3): void;
        project(trans: btTransform, dir: btVector3, minProj: number, maxProj: number, witnesPtMin: btVector3, witnesPtMax: btVector3): void;
        batchedUnitVectorGetSupportingVertexWithoutMargin(vectors: btVector3, supportVerticesOut: btVector3, numVectors: number): void;
        getAabb(t: btTransform, aabbMin: btVector3, aabbMax: btVector3): void;
        getAabbSlow(t: btTransform, aabbMin: btVector3, aabbMax: btVector3): void;
        setLocalScaling(scaling: btVector3): void;
        getLocalScaling(): btVector3;
        setMargin(margin: number): void;
        getMargin(): number;
        getNumPreferredPenetrationDirections(): number;
        getPreferredPenetrationDirection(index: number, penetrationVector: btVector3): void;
    }
    class btConvexInternalShape extends btConvexShape {
        localGetSupportingVertex(vec: btVector3): btVector3;
        getImplicitShapeDimensions(): btVector3;
        setImplicitShapeDimensions(dimensions: btVector3): void;
        setSafeMargin(minDimension: number, defaultMarginMultiplier?: number): void;
        getAabb(t: btTransform, aabbMin: btVector3, aabbMax: btVector3): void;
        getAabbSlow(t: btTransform, aabbMin: btVector3, aabbMax: btVector3): void;
        setLocalScaling(scaling: btVector3): void;
        setMargin(margin: number): void;
        getMargin(): number;
        getNumPreferredPenetrationDirections(): number;
        getPreferredPenetrationDirection(index: number, penetrationVector: btVector3): void;
    }
    class btConvexInternalAabbCachingShape extends btConvexInternalShape {
        setLocalScaling(scaling: btVector3): void;
        getAabb(t: btTransform, aabbMin: btVector3, aabbMax: btVector3): void;
        recalcLocalAabb(): void;
    }
    class btConvexPolyhedron {
        constructor();
        get_m_vertices(): btVector3Array;
        set_m_vertices(m_vertices: btVector3Array): void;
        m_vertices: btVector3Array;
        get_m_faces(): btFaceArray;
        set_m_faces(m_faces: btFaceArray): void;
        m_faces: btFaceArray;
        get_m_uniqueEdges(): btVector3Array;
        set_m_uniqueEdges(m_uniqueEdges: btVector3Array): void;
        m_uniqueEdges: btVector3Array;
        get_m_localCenter(): btVector3;
        set_m_localCenter(m_localCenter: btVector3): void;
        m_localCenter: btVector3;
        get_m_extents(): btVector3;
        set_m_extents(m_extents: btVector3): void;
        m_extents: btVector3;
        get_m_radius(): number;
        set_m_radius(m_radius: number): void;
        m_radius: number;
        get_mC(): btVector3;
        set_mC(mC: btVector3): void;
        mC: btVector3;
        get_mE(): btVector3;
        set_mE(mE: btVector3): void;
        mE: btVector3;
        initialize(): void;
        initialize2(): void;
        testContainment(): boolean;
    }
    class btPolyhedralConvexShape extends btConvexInternalShape {
        initializePolyhedralFeatures(shiftVerticesByMargin?: number): boolean;
        setPolyhedralFeatures(polyhedron: btConvexPolyhedron): void;
        getConvexPolyhedron(): btConvexPolyhedron;
        localGetSupportingVertexWithoutMargin(vec: btVector3): btVector3;
        batchedUnitVectorGetSupportingVertexWithoutMargin(vectors: btVector3, supportVerticesOut: btVector3, numVectors: number): void;
        calculateLocalInertia(mass: number, inertia: btVector3): void;
        getNumVertices(): number;
        getNumEdges(): number;
        getEdge(i: number, pa: btVector3, pb: btVector3): void;
        getVertex(i: number, vtx: btVector3): void;
        getNumPlanes(): number;
        getPlane(planeNormal: btVector3, planeSupport: btVector3, i: number): void;
        isInside(pt: btVector3, tolerance: number): boolean;
    }
    class btPolyhedralConvexAabbCachingShape extends btPolyhedralConvexShape {
        setLocalScaling(scaling: btVector3): void;
        getAabb(t: btTransform, aabbMin: btVector3, aabbMax: btVector3): void;
        recalcLocalAabb(): void;
    }
    class btConvexTriangleMeshShape extends btPolyhedralConvexAabbCachingShape {
        constructor(meshInterface: btStridingMeshInterface, calcAabb?: boolean);
        getMeshInterface(): btStridingMeshInterface;
        localGetSupportingVertex(vec: btVector3): btVector3;
        localGetSupportingVertexWithoutMargin(vec: btVector3): btVector3;
        batchedUnitVectorGetSupportingVertexWithoutMargin(vectors: btVector3, supportVerticesOut: btVector3, numVectors: number): void;
        getName(): string;
        getNumVertices(): number;
        getNumEdges(): number;
        getEdge(i: number, pa: btVector3, pb: btVector3): void;
        getVertex(i: number, vtx: btVector3): void;
        getNumPlanes(): number;
        getPlane(planeNormal: btVector3, planeSupport: btVector3, i: number): void;
        isInside(pt: btVector3, tolerance: number): boolean;
        setLocalScaling(scaling: btVector3): void;
        getLocalScaling(): btVector3;
    }
    class btBoxShape extends btPolyhedralConvexShape {
        constructor(boxHalfExtents: btVector3);
        getHalfExtentsWithMargin(): btVector3;
        getHalfExtentsWithoutMargin(): btVector3;
        localGetSupportingVertex(vec: btVector3): btVector3;
        localGetSupportingVertexWithoutMargin(vec: btVector3): btVector3;
        batchedUnitVectorGetSupportingVertexWithoutMargin(vectors: btVector3, supportVerticesOut: btVector3, numVectors: number): void;
        setMargin(collisionMargin: number): void;
        getMargin(): number;
        setLocalScaling(scaling: btVector3): void;
        getAabb(t: btTransform, aabbMin: btVector3, aabbMax: btVector3): void;
        calculateLocalInertia(mass: number, inertia: btVector3): void;
        getPlane(planeNormal: btVector3, planeSupport: btVector3, i: number): void;
        getNumPlanes(): number;
        getNumVertices(): number;
        getNumEdges(): number;
        getVertex(i: number, vtx: btVector3): void;
        getPlaneEquation(plane: btVector4, i: number): void;
        getEdge(i: number, pa: btVector3, pb: btVector3): void;
        isInside(pt: btVector3, tolerance: number): boolean;
        getName(): string;
        getNumPreferredPenetrationDirections(): number;
        getPreferredPenetrationDirection(index: number, penetrationVector: btVector3): void;
    }
    class btCapsuleShape extends btConvexInternalShape {
        constructor(radius: number, height: number);
        calculateLocalInertia(mass: number, inertia: btVector3): void;
        batchedUnitVectorGetSupportingVertexWithoutMargin(vectors: btVector3, supportVerticesOut: btVector3, numVectors: number): void;
        setMargin(collisionMargin: number): void;
        getAabb(t: btTransform, aabbMin: btVector3, aabbMax: btVector3): void;
        getName(): string;
        getUpAxis(): number;
        getRadius(): number;
        getHalfHeight(): number;
        setLocalScaling(scaling: btVector3): void;
        getAnisotropicRollingFrictionDirection(): btVector3;
    }
    class btCapsuleShapeX extends btCapsuleShape {
        constructor(radius: number, height: number);
        getName(): string;
    }
    class btCapsuleShapeZ extends btCapsuleShape {
        constructor(radius: number, height: number);
        getName(): string;
    }
    class btCylinderShape extends btConvexInternalShape {
        constructor(halfExtents: btVector3);
        getHalfExtentsWithMargin(): btVector3;
        getHalfExtentsWithoutMargin(): btVector3;
        getAabb(t: btTransform, aabbMin: btVector3, aabbMax: btVector3): void;
        calculateLocalInertia(mass: number, inertia: btVector3): void;
        localGetSupportingVertexWithoutMargin(vec: btVector3): btVector3;
        batchedUnitVectorGetSupportingVertexWithoutMargin(vectors: btVector3, supportVerticesOut: btVector3, numVectors: number): void;
        setMargin(margin: number): void;
        localGetSupportingVertex(vec: btVector3): btVector3;
        getUpAxis(): number;
        getAnisotropicRollingFrictionDirection(): btVector3;
        getRadius(): number;
        setLocalScaling(scaling: btVector3): void;
        getName(): string;
    }
    class btCylinderShapeX extends btCylinderShape {
        constructor(halfExtents: btVector3);
        localGetSupportingVertexWithoutMargin(vec: btVector3): btVector3;
        batchedUnitVectorGetSupportingVertexWithoutMargin(vectors: btVector3, supportVerticesOut: btVector3, numVectors: number): void;
        getName(): string;
        getRadius(): number;
    }
    class btCylinderShapeZ extends btCylinderShape {
        constructor(halfExtents: btVector3);
        localGetSupportingVertexWithoutMargin(vec: btVector3): btVector3;
        batchedUnitVectorGetSupportingVertexWithoutMargin(vectors: btVector3, supportVerticesOut: btVector3, numVectors: number): void;
        getName(): string;
        getRadius(): number;
    }
    class btSphereShape extends btConvexInternalShape {
        constructor(radius: number);
        localGetSupportingVertex(vec: btVector3): btVector3;
        localGetSupportingVertexWithoutMargin(vec: btVector3): btVector3;
        batchedUnitVectorGetSupportingVertexWithoutMargin(vectors: btVector3, supportVerticesOut: btVector3, numVectors: number): void;
        calculateLocalInertia(mass: number, inertia: btVector3): void;
        getAabb(t: btTransform, aabbMin: btVector3, aabbMax: btVector3): void;
        getRadius(): number;
        setUnscaledRadius(radius: number): void;
        getName(): string;
        setMargin(margin: number): void;
        getMargin(): number;
    }
    class btMultiSphereShape extends btConvexInternalAabbCachingShape {
        constructor(positions: btVector3, radi: ReadonlyArray<number>, numSpheres: number);
        calculateLocalInertia(mass: number, inertia: btVector3): void;
        localGetSupportingVertexWithoutMargin(vec: btVector3): btVector3;
        batchedUnitVectorGetSupportingVertexWithoutMargin(vectors: btVector3, supportVerticesOut: btVector3, numVectors: number): void;
        getSphereCount(): number;
        getSpherePosition(index: number): btVector3;
        getSphereRadius(index: number): number;
        getName(): string;
    }
    class btConeShape extends btConvexInternalShape {
        constructor(radius: number, height: number);
        localGetSupportingVertex(vec: btVector3): btVector3;
        localGetSupportingVertexWithoutMargin(vec: btVector3): btVector3;
        batchedUnitVectorGetSupportingVertexWithoutMargin(vectors: btVector3, supportVerticesOut: btVector3, numVectors: number): void;
        getRadius(): number;
        getHeight(): number;
        setRadius(radius: number): void;
        setHeight(height: number): void;
        calculateLocalInertia(mass: number, inertia: btVector3): void;
        getName(): string;
        setConeUpIndex(upIndex: number): void;
        getConeUpIndex(): number;
        getAnisotropicRollingFrictionDirection(): btVector3;
        setLocalScaling(scaling: btVector3): void;
    }
    class btConeShapeX extends btConeShape {
        constructor(radius: number, height: number);
        getAnisotropicRollingFrictionDirection(): btVector3;
        getName(): string;
    }
    class btConeShapeZ extends btConeShape {
        constructor(radius: number, height: number);
        getAnisotropicRollingFrictionDirection(): btVector3;
        getName(): string;
    }
    class btConvexHullShape extends btPolyhedralConvexAabbCachingShape {
        constructor(points?: ReadonlyArray<number>, numPoints?: number, stride?: number);
        addPoint(point: btVector3, recalculateLocalAABB?: boolean): void;
        getUnscaledPoints(): btVector3;
        getPoints(): btVector3;
        optimizeConvexHull(): void;
        getScaledPoint(i: number): btVector3;
        getNumPoints(): number;
        localGetSupportingVertex(vec: btVector3): btVector3;
        localGetSupportingVertexWithoutMargin(vec: btVector3): btVector3;
        batchedUnitVectorGetSupportingVertexWithoutMargin(vectors: btVector3, supportVerticesOut: btVector3, numVectors: number): void;
        project(trans: btTransform, dir: btVector3, minProj: number, maxProj: number, witnesPtMin: btVector3, witnesPtMax: btVector3): void;
        getName(): string;
        getNumVertices(): number;
        getNumEdges(): number;
        getEdge(i: number, pa: btVector3, pb: btVector3): void;
        getVertex(i: number, vtx: btVector3): void;
        getNumPlanes(): number;
        getPlane(planeNormal: btVector3, planeSupport: btVector3, i: number): void;
        isInside(pt: btVector3, tolerance: number): boolean;
        setLocalScaling(scaling: btVector3): void;
    }
    class btShapeHull {
        constructor(shape: btConvexShape);
        buildHull(margin: number, highres?: number): boolean;
        numTriangles(): number;
        numVertices(): number;
        numIndices(): number;
        getVertexPointer(): ReadonlyArray<btVector3>;
    }
    class btCompoundShapeChild {
        get_m_transform(): btTransform;
        set_m_transform(m_transform: btTransform): void;
        m_transform: btTransform;
        get_m_childShape(): btCollisionShape;
        set_m_childShape(m_childShape: btCollisionShape): void;
        m_childShape: btCollisionShape;
        get_m_childShapeType(): number;
        set_m_childShapeType(m_childShapeType: number): void;
        m_childShapeType: number;
        get_m_childMargin(): number;
        set_m_childMargin(m_childMargin: number): void;
        m_childMargin: number;
        get_m_node(): btDbvtNode;
        set_m_node(m_node: btDbvtNode): void;
        m_node: btDbvtNode;
    }
    class btCompoundShape extends btCollisionShape {
        constructor(enableDynamicAabbTree?: boolean, initialChildCapacity?: number);
        addChildShape(localTransform: btTransform, shape: btCollisionShape): void;
        removeChildShape(shape: btCollisionShape): void;
        removeChildShapeByIndex(childShapeindex: number): void;
        getNumChildShapes(): number;
        getChildShape(index: number): btCollisionShape;
        getChildTransform(index: number): btTransform;
        updateChildTransform(childIndex: number, newChildTransform: btTransform, shouldRecalculateLocalAabb?: boolean): void;
        getChildList(): ReadonlyArray<btCompoundShapeChild>;
        getAabb(t: btTransform, aabbMin: btVector3, aabbMax: btVector3): void;
        recalculateLocalAabb(): void;
        setLocalScaling(scaling: btVector3): void;
        getLocalScaling(): btVector3;
        calculateLocalInertia(mass: number, inertia: btVector3): void;
        setMargin(margin: number): void;
        getMargin(): number;
        getName(): string;
        getDynamicAabbTree(): btDbvt;
        createAabbTreeFromChildren(): void;
        calculatePrincipalAxisTransform(masses: ReadonlyArray<number>, principal: btTransform, inertia: btVector3): void;
        getUpdateRevision(): number;
    }
    class btInternalTriangleIndexCallback {
        internalProcessTriangleIndex(triangle: btVector3, partId: number, triangleIndex: number): void;
    }
    class btStridingMeshInterface {
        InternalProcessAllTriangles(callback: btInternalTriangleIndexCallback, aabbMin: btVector3, aabbMax: btVector3): void;
        setScaling(scaling: btVector3): void;
    }
    class btIndexedMesh {
        get_m_numTriangles(): number;
        set_m_numTriangles(m_numTriangles: number): void;
        m_numTriangles: number;
        get_m_triangleIndexStride(): number;
        set_m_triangleIndexStride(m_triangleIndexStride: number): void;
        m_triangleIndexStride: number;
        get_m_numVertices(): number;
        set_m_numVertices(m_numVertices: number): void;
        m_numVertices: number;
        get_m_vertexStride(): number;
        set_m_vertexStride(m_vertexStride: number): void;
        m_vertexStride: number;
        get_m_indexType(): PHY_ScalarType;
        set_m_indexType(m_indexType: PHY_ScalarType): void;
        m_indexType: PHY_ScalarType;
        get_m_vertexType(): PHY_ScalarType;
        set_m_vertexType(m_vertexType: PHY_ScalarType): void;
        m_vertexType: PHY_ScalarType;
    }
    class btIndexedMeshArray {
        size(): number;
        at(n: number): btIndexedMesh;
    }
    class btTriangleIndexVertexArray extends btStridingMeshInterface {
        constructor();
        constructor(numTriangles: number, triangleIndexBase: ReadonlyArray<number>, triangleIndexStride: number, numVertices: number, vertexBase: ReadonlyArray<number>, vertexStride: number);
        addIndexedMesh(mesh: btIndexedMesh, indexType?: PHY_ScalarType): void;
        unLockVertexBase(subpart: number): void;
        unLockReadOnlyVertexBase(subpart: number): void;
        getNumSubParts(): number;
        getIndexedMeshArray(): btIndexedMeshArray;
        preallocateVertices(numverts: number): void;
        preallocateIndices(numverts: number): void;
        hasPremadeAabb(): boolean;
        setPremadeAabb(aabbMin: btVector3, aabbMax: btVector3): void;
        getPremadeAabb(aabbMin: btVector3, aabbMax: btVector3): void;
    }
    class btTriangleMesh extends btTriangleIndexVertexArray {
        constructor(use32bitIndices?: boolean, use4componentVertices?: boolean);
        get_m_weldingThreshold(): number;
        set_m_weldingThreshold(m_weldingThreshold: number): void;
        m_weldingThreshold: number;
        getUse32bitIndices(): boolean;
        getUse4componentVertices(): boolean;
        addTriangle(vertex0: btVector3, vertex1: btVector3, vertex2: btVector3, removeDuplicateVertices?: boolean): void;
        addTriangleIndices(index1: number, index2: number, index3: number): void;
        preallocateVertices(numverts: number): void;
        preallocateIndices(numverts: number): void;
        findOrAddVertex(vertex: btVector3, removeDuplicateVertices: boolean): number;
        addIndex(index: number): void;
        getIndexedMeshArray(): btIndexedMeshArray;
    }
    type PHY_ScalarType = "PHY_FLOAT" | "PHY_DOUBLE" | "PHY_INTEGER" | "PHY_SHORT" | "PHY_FIXEDPOINT88" | "PHY_UCHAR";
    class btConcaveShape extends btCollisionShape {
        getMargin(): number;
        setMargin(collisionMargin: number): void;
    }
    class btEmptyShape extends btConcaveShape {
        constructor();
        getAabb(t: btTransform, aabbMin: btVector3, aabbMax: btVector3): void;
        setLocalScaling(scaling: btVector3): void;
        getLocalScaling(): btVector3;
    }
    class btStaticPlaneShape extends btConcaveShape {
        constructor(planeNormal: btVector3, planeConstant: number);
        getPlaneNormal(): btVector3;
        getPlaneConstant(): number;
    }
    class btTriangleMeshShape extends btConcaveShape {
    }
    class btTriangleInfoMap {
        constructor();
    }
    class btBvhTriangleMeshShape extends btTriangleMeshShape {
        constructor(meshInterface: btStridingMeshInterface, useQuantizedAabbCompression: boolean, buildBvh?: boolean);
        constructor(meshInterface: btStridingMeshInterface, useQuantizedAabbCompression: boolean, bvhAabbMin: btVector3, bvhAabbMax: btVector3, buildBvh?: boolean);
        generateInternalEdgeInfo(triangleInfoMap: btTriangleInfoMap): void;
    }
    class btHeightfieldTerrainShape extends btConcaveShape {
        constructor(heightStickWidth: number, heightStickLength: number, heightfieldData: unknown, heightScale: number, minHeight: number, maxHeight: number, upAxis: number, hdt: PHY_ScalarType, flipQuadEdges: boolean);
    }
    class btDefaultCollisionConstructionInfo {
        constructor();
    }
    class btDefaultCollisionConfiguration {
        constructor(info?: btDefaultCollisionConstructionInfo);
    }
    class btPersistentManifold {
        constructor();
        getBody0(): btCollisionObject;
        getBody1(): btCollisionObject;
        getNumContacts(): number;
        getContactPoint(index: number): btManifoldPoint;
    }
    class btDispatcher {
        getNumManifolds(): number;
        getManifoldByIndexInternal(index: number): btPersistentManifold;
    }
    class btCollisionDispatcher extends btDispatcher {
        constructor(conf: btDefaultCollisionConfiguration);
    }
    class btOverlappingPairCallback {
    }
    class btOverlappingPairCache {
        setInternalGhostPairCallback(ghostPairCallback: btOverlappingPairCallback): void;
        removeOverlappingPairsContainingProxy(proxy: btBroadphaseProxy, dispatcher: btDispatcher): void;
        getNumOverlappingPairs(): number;
    }
    class btAxisSweep3 {
        constructor(worldAabbMin: btVector3, worldAabbMax: btVector3, maxHandles?: number, pairCache?: btOverlappingPairCache, disableRaycastAccelerator?: boolean);
    }
    class btBroadphaseInterface {
        getOverlappingPairCache(): btOverlappingPairCache;
    }
    class btCollisionConfiguration {
    }
    class btDbvt {
    }
    class btDbvtBroadphase {
        constructor();
        getOverlappingPairCache(): btOverlappingPairCache;
    }
    class btBroadphaseProxy {
        get_m_collisionFilterGroup(): number;
        set_m_collisionFilterGroup(m_collisionFilterGroup: number): void;
        m_collisionFilterGroup: number;
        get_m_collisionFilterMask(): number;
        set_m_collisionFilterMask(m_collisionFilterMask: number): void;
        m_collisionFilterMask: number;
    }
    class btDbvtVolume {
    }
    class btDbvtNode {
        get_volume(): btDbvtVolume;
        set_volume(volume: btDbvtVolume): void;
        volume: btDbvtVolume;
        get_parent(): btDbvtNode;
        set_parent(parent: btDbvtNode): void;
        parent: btDbvtNode;
        isleaf(): boolean;
        isinternal(): boolean;
    }
    class btRigidBodyConstructionInfo {
        constructor(mass: number, motionState: btMotionState, collisionShape: btCollisionShape, localInertia?: btVector3);
        get_m_linearDamping(): number;
        set_m_linearDamping(m_linearDamping: number): void;
        m_linearDamping: number;
        get_m_angularDamping(): number;
        set_m_angularDamping(m_angularDamping: number): void;
        m_angularDamping: number;
        get_m_friction(): number;
        set_m_friction(m_friction: number): void;
        m_friction: number;
        get_m_rollingFriction(): number;
        set_m_rollingFriction(m_rollingFriction: number): void;
        m_rollingFriction: number;
        get_m_restitution(): number;
        set_m_restitution(m_restitution: number): void;
        m_restitution: number;
        get_m_linearSleepingThreshold(): number;
        set_m_linearSleepingThreshold(m_linearSleepingThreshold: number): void;
        m_linearSleepingThreshold: number;
        get_m_angularSleepingThreshold(): number;
        set_m_angularSleepingThreshold(m_angularSleepingThreshold: number): void;
        m_angularSleepingThreshold: number;
        get_m_additionalDamping(): boolean;
        set_m_additionalDamping(m_additionalDamping: boolean): void;
        m_additionalDamping: boolean;
        get_m_additionalDampingFactor(): number;
        set_m_additionalDampingFactor(m_additionalDampingFactor: number): void;
        m_additionalDampingFactor: number;
        get_m_additionalLinearDampingThresholdSqr(): number;
        set_m_additionalLinearDampingThresholdSqr(m_additionalLinearDampingThresholdSqr: number): void;
        m_additionalLinearDampingThresholdSqr: number;
        get_m_additionalAngularDampingThresholdSqr(): number;
        set_m_additionalAngularDampingThresholdSqr(m_additionalAngularDampingThresholdSqr: number): void;
        m_additionalAngularDampingThresholdSqr: number;
        get_m_additionalAngularDampingFactor(): number;
        set_m_additionalAngularDampingFactor(m_additionalAngularDampingFactor: number): void;
        m_additionalAngularDampingFactor: number;
    }
    class btRigidBody extends btCollisionObject {
        constructor(constructionInfo: btRigidBodyConstructionInfo);
        proceedToTransform(newTrans: btTransform): void;
        upcast(colObj: btCollisionObject): btRigidBody;
        predictIntegratedTransform(step: number, predictedTransform: btTransform): void;
        saveKinematicState(step: number): void;
        applyGravity(): void;
        clearGravity(): void;
        setGravity(acceleration: btVector3): void;
        getGravity(): btVector3;
        setDamping(lin_damping: number, ang_damping: number): void;
        getLinearDamping(): number;
        getAngularDamping(): number;
        getLinearSleepingThreshold(): number;
        getAngularSleepingThreshold(): number;
        applyDamping(timeStep: number): void;
        getCollisionShape(): btCollisionShape;
        setMassProps(mass: number, inertia: btVector3): void;
        getLinearFactor(): btVector3;
        setLinearFactor(linearFactor: btVector3): void;
        getInvMass(): number;
        getMass(): number;
        getInvInertiaTensorWorld(): btMatrix3x3;
        integrateVelocities(step: number): void;
        setCenterOfMassTransform(xform: btTransform): void;
        applyCentralForce(force: btVector3): void;
        getTotalForce(): btVector3;
        getTotalTorque(): btVector3;
        getInvInertiaDiagLocal(): btVector3;
        setInvInertiaDiagLocal(diagInvInertia: btVector3): void;
        setSleepingThresholds(linear: number, angular: number): void;
        applyTorque(torque: btVector3): void;
        applyForce(force: btVector3, rel_pos: btVector3): void;
        applyCentralImpulse(impulse: btVector3): void;
        applyTorqueImpulse(torque: btVector3): void;
        applyImpulse(impulse: btVector3, rel_pos: btVector3): void;
        applyPushImpulse(impulse: btVector3, rel_pos: btVector3): void;
        getPushVelocity(): btVector3;
        getTurnVelocity(): btVector3;
        setPushVelocity(v: btVector3): void;
        setTurnVelocity(v: btVector3): void;
        applyCentralPushImpulse(impulse: btVector3): void;
        applyTorqueTurnImpulse(torque: btVector3): void;
        clearForces(): void;
        updateInertiaTensor(): void;
        getCenterOfMassPosition(): btVector3;
        getOrientation(): btQuaternion;
        getCenterOfMassTransform(): btTransform;
        getLinearVelocity(): btVector3;
        getAngularVelocity(): btVector3;
        setLinearVelocity(lin_vel: btVector3): void;
        setAngularVelocity(ang_vel: btVector3): void;
        getVelocityInLocalPoint(rel_pos: btVector3): btVector3;
        getPushVelocityInLocalPoint(rel_pos: btVector3): btVector3;
        translate(v: btVector3): void;
        getAabb(aabbMin: btVector3, aabbMax: btVector3): void;
        computeImpulseDenominator(pos: btVector3, normal: btVector3): number;
        computeAngularImpulseDenominator(axis: btVector3): number;
        updateDeactivation(timeStep: number): void;
        wantsSleeping(): boolean;
        getBroadphaseProxy(): btBroadphaseProxy;
        setNewBroadphaseProxy(broadphaseProxy: btBroadphaseProxy): void;
        getMotionState(): btMotionState;
        setMotionState(motionState: btMotionState): void;
        setAngularFactor(angFac: btVector3): void;
        getAngularFactor(): btVector3;
        isInWorld(): boolean;
        addConstraintRef(c: btTypedConstraint): void;
        removeConstraintRef(c: btTypedConstraint): void;
        getConstraintRef(index: number): btTypedConstraint;
        getNumConstraintRefs(): number;
        setFlags(flags: number): void;
        getFlags(): number;
        computeGyroscopicImpulseImplicit_World(dt: number): btVector3;
        computeGyroscopicImpulseImplicit_Body(step: number): btVector3;
        computeGyroscopicForceExplicit(maxGyroscopicForce: number): btVector3;
        getLocalInertia(): btVector3;
    }
    class btSequentialImpulseConstraintSolver {
        constructor();
        reset(): void;
        setRandSeed(seed: number): void;
        getRandSeed(): number;
    }
    type btTypedConstraintType = "POINT2POINT_CONSTRAINT_TYPE" | "HINGE_CONSTRAINT_TYPE" | "CONETWIST_CONSTRAINT_TYPE" | "D6_CONSTRAINT_TYPE" | "SLIDER_CONSTRAINT_TYPE" | "CONTACT_CONSTRAINT_TYPE" | "D6_SPRING_CONSTRAINT_TYPE" | "GEAR_CONSTRAINT_TYPE" | "FIXED_CONSTRAINT_TYPE" | "D6_SPRING_2_CONSTRAINT_TYPE" | "MAX_CONSTRAINT_TYPE";
    type btConstraintParams = "BT_CONSTRAINT_ERP" | "BT_CONSTRAINT_STOP_ERP" | "BT_CONSTRAINT_CFM" | "BT_CONSTRAINT_STOP_CFM";
    class btJointFeedback {
        get_m_appliedForceBodyA(): btVector3;
        set_m_appliedForceBodyA(m_appliedForceBodyA: btVector3): void;
        m_appliedForceBodyA: btVector3;
        get_m_appliedTorqueBodyA(): btVector3;
        set_m_appliedTorqueBodyA(m_appliedTorqueBodyA: btVector3): void;
        m_appliedTorqueBodyA: btVector3;
        get_m_appliedForceBodyB(): btVector3;
        set_m_appliedForceBodyB(m_appliedForceBodyB: btVector3): void;
        m_appliedForceBodyB: btVector3;
        get_m_appliedTorqueBodyB(): btVector3;
        set_m_appliedTorqueBodyB(m_appliedTorqueBodyB: btVector3): void;
        m_appliedTorqueBodyB: btVector3;
    }
    class btTypedConstraint extends btTypedObject {
        getFixedBody(): btRigidBody;
        getOverrideNumSolverIterations(): number;
        setOverrideNumSolverIterations(overideNumIterations: number): void;
        getBreakingImpulseThreshold(): number;
        setBreakingImpulseThreshold(threshold: number): void;
        isEnabled(): boolean;
        setEnabled(enabled: boolean): void;
        getRigidBodyA(): btRigidBody;
        getRigidBodyB(): btRigidBody;
        getUserConstraintType(): number;
        setUserConstraintType(userConstraintType: number): void;
        setUserConstraintId(uid: number): void;
        getUserConstraintId(): number;
        setUserConstraintPtr(ptr: unknown): void;
        getUserConstraintPtr(): unknown;
        setJointFeedback(jointFeedback: btJointFeedback): void;
        getJointFeedback(): btJointFeedback;
        getUid(): number;
        needsFeedback(): boolean;
        enableFeedback(needsFeedback: boolean): void;
        getAppliedImpulse(): number;
        getConstraintType(): btTypedConstraintType;
        setParam(num: number, value: number, axis: number): void;
        getParam(num: number, axis: number): number;
    }
    class btConstraintSetting {
        constructor();
        get_m_tau(): number;
        set_m_tau(m_tau: number): void;
        m_tau: number;
        get_m_damping(): number;
        set_m_damping(m_damping: number): void;
        m_damping: number;
        get_m_impulseClamp(): number;
        set_m_impulseClamp(m_impulseClamp: number): void;
        m_impulseClamp: number;
    }
    type btPoint2PointFlags = "BT_P2P_FLAGS_ERP" | "BT_P2P_FLAGS_CFM";
    class btPoint2PointConstraint extends btTypedConstraint {
        constructor(rbA: btRigidBody, rbB: btRigidBody, pivotInA: btVector3, pivotInB: btVector3);
        constructor(rbA: btRigidBody, pivotInA: btVector3);
        get_m_setting(): btConstraintSetting;
        set_m_setting(m_setting: btConstraintSetting): void;
        m_setting: btConstraintSetting;
        buildJacobian(): void;
        updateRHS(timeStep: number): void;
        setPivotA(pivotA: btVector3): void;
        setPivotB(pivotB: btVector3): void;
        getPivotInA(): btVector3;
        getPivotInB(): btVector3;
        setParam(num: number, value: number, axis?: number): void;
        getParam(num: number, axis?: number): number;
        getFlags(): number;
    }
    class btAngularLimit {
        constructor();
        set(low: number, high: number, softness?: number, biasFactor?: number, relaxationFactor?: number): void;
        getSoftness(): number;
        getBiasFactor(): number;
        getRelaxationFactor(): number;
        getCorrection(): number;
        getSign(): number;
        getHalfRange(): number;
        isLimit(): boolean;
        fit(angle: ReadonlyArray<number>): void;
        getError(): number;
        getLow(): number;
        getHigh(): number;
    }
    type btHingeFlags = "BT_HINGE_FLAGS_CFM_STOP" | "BT_HINGE_FLAGS_ERP_STOP" | "BT_HINGE_FLAGS_CFM_NORM" | "BT_HINGE_FLAGS_ERP_NORM";
    class btHingeConstraint extends btTypedConstraint {
        constructor(rbA: btRigidBody, rbB: btRigidBody, pivotInA: btVector3, pivotInB: btVector3, axisInA: btVector3, axisInB: btVector3, useReferenceFrameA?: boolean);
        constructor(rbA: btRigidBody, rbB: btRigidBody, rbAFrame: btTransform, rbBFrame: btTransform, useReferenceFrameA?: boolean);
        constructor(rbA: btRigidBody, rbAFrame: btTransform, useReferenceFrameA?: boolean);
        buildJacobian(): void;
        updateRHS(timeStep: number): void;
        getRigidBodyA(): btRigidBody;
        getRigidBodyB(): btRigidBody;
        getFrameOffsetA(): btTransform;
        getFrameOffsetB(): btTransform;
        setFrames(frameA: btTransform, frameB: btTransform): void;
        setAngularOnly(angularOnly: boolean): void;
        enableAngularMotor(enableMotor: boolean, targetVelocity: number, maxMotorImpulse: number): void;
        enableMotor(enableMotor: boolean): void;
        setMaxMotorImpulse(maxMotorImpulse: number): void;
        setMotorTargetVelocity(motorTargetVelocity: number): void;
        setMotorTarget(targetAngle: number, dt: number): void;
        setLimit(low: number, high: number, softness: number, biasFactor: number, relaxationFactor?: number): void;
        getLimitSoftness(): number;
        getLimitBiasFactor(): number;
        getLimitRelaxationFactor(): number;
        setAxis(axisInA: btVector3): void;
        hasLimit(): boolean;
        getLowerLimit(): number;
        getUpperLimit(): number;
        getHingeAngle(): number;
        getHingeAngle(transA: btTransform, transB: btTransform): number;
        testLimit(transA: btTransform, transB: btTransform): void;
        getAFrame(): btTransform;
        getBFrame(): btTransform;
        getSolveLimit(): number;
        getLimitSign(): number;
        getAngularOnly(): boolean;
        getEnableAngularMotor(): boolean;
        getMotorTargetVelocity(): number;
        getMaxMotorImpulse(): number;
        getUseFrameOffset(): boolean;
        setUseFrameOffset(frameOffsetOnOff: boolean): void;
        getUseReferenceFrameA(): boolean;
        setUseReferenceFrameA(useReferenceFrameA: boolean): void;
        setParam(num: number, value: number, axis?: number): void;
        getParam(num: number, axis?: number): number;
        getFlags(): number;
    }
    type btConeTwistFlags = "BT_CONETWIST_FLAGS_LIN_CFM" | "BT_CONETWIST_FLAGS_LIN_ERP" | "BT_CONETWIST_FLAGS_ANG_CFM";
    class btConeTwistConstraint extends btTypedConstraint {
        constructor(rbA: btRigidBody, rbB: btRigidBody, rbAFrame: btTransform, rbBFrame: btTransform);
        constructor(rbA: btRigidBody, rbAFrame: btTransform);
        buildJacobian(): void;
        updateRHS(timeStep: number): void;
        getRigidBodyA(): btRigidBody;
        getRigidBodyB(): btRigidBody;
        setAngularOnly(angularOnly: boolean): void;
        getAngularOnly(): boolean;
        setLimit(limitIndex: number, limitValue: number): void;
        getLimit(limitIndex: number): number;
        setLimit(swingSpan1: number, swingSpan2: number, twistSpan: number, softness?: number, biasFactor?: number, relaxationFactor?: number): void;
        getAFrame(): btTransform;
        getBFrame(): btTransform;
        getSolveTwistLimit(): number;
        getSolveSwingLimit(): number;
        getTwistLimitSign(): number;
        calcAngleInfo(): void;
        calcAngleInfo2(transA: btTransform, transB: btTransform, invInertiaWorldA: btMatrix3x3, invInertiaWorldB: btMatrix3x3): void;
        getSwingSpan1(): number;
        getSwingSpan2(): number;
        getTwistSpan(): number;
        getLimitSoftness(): number;
        getBiasFactor(): number;
        getRelaxationFactor(): number;
        getTwistAngle(): number;
        isPastSwingLimit(): boolean;
        getDamping(): number;
        setDamping(damping: number): void;
        enableMotor(b: boolean): void;
        isMotorEnabled(): boolean;
        getMaxMotorImpulse(): number;
        isMaxMotorImpulseNormalized(): boolean;
        setMaxMotorImpulse(maxMotorImpulse: number): void;
        setMaxMotorImpulseNormalized(maxMotorImpulse: number): void;
        getFixThresh(): number;
        setFixThresh(fixThresh: number): void;
        setMotorTarget(q: btQuaternion): void;
        getMotorTarget(): btQuaternion;
        setMotorTargetInConstraintSpace(q: btQuaternion): void;
        GetPointForAngle(fAngleInRadians: number, fLength: number): btVector3;
        setParam(num: number, value: number, axis?: number): void;
        getFrameOffsetA(): btTransform;
        getFrameOffsetB(): btTransform;
        getParam(num: number, axis?: number): number;
        getFlags(): number;
    }
    class btRotationalLimitMotor {
        constructor();
        get_m_loLimit(): number;
        set_m_loLimit(m_loLimit: number): void;
        m_loLimit: number;
        get_m_hiLimit(): number;
        set_m_hiLimit(m_hiLimit: number): void;
        m_hiLimit: number;
        get_m_targetVelocity(): number;
        set_m_targetVelocity(m_targetVelocity: number): void;
        m_targetVelocity: number;
        get_m_maxMotorForce(): number;
        set_m_maxMotorForce(m_maxMotorForce: number): void;
        m_maxMotorForce: number;
        get_m_maxLimitForce(): number;
        set_m_maxLimitForce(m_maxLimitForce: number): void;
        m_maxLimitForce: number;
        get_m_damping(): number;
        set_m_damping(m_damping: number): void;
        m_damping: number;
        get_m_limitSoftness(): number;
        set_m_limitSoftness(m_limitSoftness: number): void;
        m_limitSoftness: number;
        get_m_normalCFM(): number;
        set_m_normalCFM(m_normalCFM: number): void;
        m_normalCFM: number;
        get_m_stopERP(): number;
        set_m_stopERP(m_stopERP: number): void;
        m_stopERP: number;
        get_m_stopCFM(): number;
        set_m_stopCFM(m_stopCFM: number): void;
        m_stopCFM: number;
        get_m_bounce(): number;
        set_m_bounce(m_bounce: number): void;
        m_bounce: number;
        get_m_enableMotor(): boolean;
        set_m_enableMotor(m_enableMotor: boolean): void;
        m_enableMotor: boolean;
        get_m_currentLimitError(): number;
        set_m_currentLimitError(m_currentLimitError: number): void;
        m_currentLimitError: number;
        get_m_currentPosition(): number;
        set_m_currentPosition(m_currentPosition: number): void;
        m_currentPosition: number;
        get_m_currentLimit(): number;
        set_m_currentLimit(m_currentLimit: number): void;
        m_currentLimit: number;
        get_m_accumulatedImpulse(): number;
        set_m_accumulatedImpulse(m_accumulatedImpulse: number): void;
        m_accumulatedImpulse: number;
        isLimited(): boolean;
        needApplyTorques(): boolean;
        testLimitValue(test_value: number): number;
        solveAngularLimits(timeStep: number, axis: btVector3, jacDiagABInv: number, body0: btRigidBody, body1: btRigidBody): number;
    }
    class btTranslationalLimitMotor {
        constructor();
        get_m_lowerLimit(): btVector3;
        set_m_lowerLimit(m_lowerLimit: btVector3): void;
        m_lowerLimit: btVector3;
        get_m_upperLimit(): btVector3;
        set_m_upperLimit(m_upperLimit: btVector3): void;
        m_upperLimit: btVector3;
        get_m_accumulatedImpulse(): btVector3;
        set_m_accumulatedImpulse(m_accumulatedImpulse: btVector3): void;
        m_accumulatedImpulse: btVector3;
        get_m_limitSoftness(): number;
        set_m_limitSoftness(m_limitSoftness: number): void;
        m_limitSoftness: number;
        get_m_damping(): number;
        set_m_damping(m_damping: number): void;
        m_damping: number;
        get_m_restitution(): number;
        set_m_restitution(m_restitution: number): void;
        m_restitution: number;
        get_m_normalCFM(): btVector3;
        set_m_normalCFM(m_normalCFM: btVector3): void;
        m_normalCFM: btVector3;
        get_m_stopERP(): btVector3;
        set_m_stopERP(m_stopERP: btVector3): void;
        m_stopERP: btVector3;
        get_m_stopCFM(): btVector3;
        set_m_stopCFM(m_stopCFM: btVector3): void;
        m_stopCFM: btVector3;
        get_m_enableMotor(): ReadonlyArray<boolean>;
        set_m_enableMotor(m_enableMotor: ReadonlyArray<boolean>): void;
        m_enableMotor: ReadonlyArray<boolean>;
        get_m_targetVelocity(): btVector3;
        set_m_targetVelocity(m_targetVelocity: btVector3): void;
        m_targetVelocity: btVector3;
        get_m_maxMotorForce(): btVector3;
        set_m_maxMotorForce(m_maxMotorForce: btVector3): void;
        m_maxMotorForce: btVector3;
        get_m_currentLimitError(): btVector3;
        set_m_currentLimitError(m_currentLimitError: btVector3): void;
        m_currentLimitError: btVector3;
        get_m_currentLinearDiff(): btVector3;
        set_m_currentLinearDiff(m_currentLinearDiff: btVector3): void;
        m_currentLinearDiff: btVector3;
        get_m_currentLimit(): ReadonlyArray<number>;
        set_m_currentLimit(m_currentLimit: ReadonlyArray<number>): void;
        m_currentLimit: ReadonlyArray<number>;
        isLimited(limitIndex: number): boolean;
        needApplyForce(limitIndex: number): boolean;
        testLimitValue(limitIndex: number, test_value: number): number;
        solveLinearAxis(timeStep: number, jacDiagABInv: number, body1: btRigidBody, pointInA: btVector3, body2: btRigidBody, pointInB: btVector3, limit_index: number, axis_normal_on_a: btVector3, anchorPos: btVector3): number;
    }
    type bt6DofFlags = "BT_6DOF_FLAGS_CFM_NORM" | "BT_6DOF_FLAGS_CFM_STOP" | "BT_6DOF_FLAGS_ERP_STOP";
    class btGeneric6DofConstraint extends btTypedConstraint {
        constructor(rbA: btRigidBody, rbB: btRigidBody, frameInA: btTransform, frameInB: btTransform, useLinearFrameReferenceFrameA: boolean);
        constructor(rbB: btRigidBody, frameInB: btTransform, useLinearFrameReferenceFrameB: boolean);
        calculateTransforms(transA: btTransform, transB: btTransform): void;
        calculateTransforms(): void;
        getCalculatedTransformA(): btTransform;
        getCalculatedTransformB(): btTransform;
        getFrameOffsetA(): btTransform;
        getFrameOffsetB(): btTransform;
        buildJacobian(): void;
        updateRHS(timeStep: number): void;
        getAxis(axis_index: number): btVector3;
        getAngle(axis_index: number): number;
        getRelativePivotPosition(axis_index: number): number;
        setFrames(frameA: btTransform, frameB: btTransform): void;
        setLinearLowerLimit(linearLower: btVector3): void;
        getLinearLowerLimit(linearLower: btVector3): void;
        setLinearUpperLimit(linearUpper: btVector3): void;
        getLinearUpperLimit(linearUpper: btVector3): void;
        setAngularLowerLimit(angularLower: btVector3): void;
        getAngularLowerLimit(angularLower: btVector3): void;
        setAngularUpperLimit(angularUpper: btVector3): void;
        getAngularUpperLimit(angularUpper: btVector3): void;
        getRotationalLimitMotor(index: number): btRotationalLimitMotor;
        getTranslationalLimitMotor(): btTranslationalLimitMotor;
        setLimit(axis: number, lo: number, hi: number): void;
        isLimited(limitIndex: number): boolean;
        calcAnchorPos(): void;
        getUseFrameOffset(): boolean;
        setUseFrameOffset(frameOffsetOnOff: boolean): void;
        getUseLinearReferenceFrameA(): boolean;
        setUseLinearReferenceFrameA(linearReferenceFrameA: boolean): void;
        setParam(num: number, value: number, axis?: number): void;
        getParam(num: number, axis?: number): number;
        setAxis(axis1: btVector3, axis2: btVector3): void;
        getFlags(): number;
    }
    type btSliderFlags = "BT_SLIDER_FLAGS_CFM_DIRLIN" | "BT_SLIDER_FLAGS_ERP_DIRLIN" | "BT_SLIDER_FLAGS_CFM_DIRANG" | "BT_SLIDER_FLAGS_ERP_DIRANG" | "BT_SLIDER_FLAGS_CFM_ORTLIN" | "BT_SLIDER_FLAGS_ERP_ORTLIN" | "BT_SLIDER_FLAGS_CFM_ORTANG" | "BT_SLIDER_FLAGS_ERP_ORTANG" | "BT_SLIDER_FLAGS_CFM_LIMLIN" | "BT_SLIDER_FLAGS_ERP_LIMLIN" | "BT_SLIDER_FLAGS_CFM_LIMANG" | "BT_SLIDER_FLAGS_ERP_LIMANG";
    class btSliderConstraint extends btTypedConstraint {
        constructor(rbA: btRigidBody, rbB: btRigidBody, frameInA: btTransform, frameInB: btTransform, useLinearReferenceFrameA: boolean);
        constructor(rbB: btRigidBody, frameInB: btTransform, useLinearReferenceFrameA: boolean);
        getRigidBodyA(): btRigidBody;
        getRigidBodyB(): btRigidBody;
        getCalculatedTransformA(): btTransform;
        getCalculatedTransformB(): btTransform;
        getFrameOffsetA(): btTransform;
        getFrameOffsetB(): btTransform;
        getLowerLinLimit(): number;
        setLowerLinLimit(lowerLimit: number): void;
        getUpperLinLimit(): number;
        setUpperLinLimit(upperLimit: number): void;
        getLowerAngLimit(): number;
        setLowerAngLimit(lowerAngLimit: number): void;
        getUpperAngLimit(): number;
        setUpperAngLimit(upperAngLimit: number): void;
        getUseLinearReferenceFrameA(): boolean;
        getSoftnessDirLin(): number;
        getRestitutionDirLin(): number;
        getDampingDirLin(): number;
        getSoftnessDirAng(): number;
        getRestitutionDirAng(): number;
        getDampingDirAng(): number;
        getSoftnessLimLin(): number;
        getRestitutionLimLin(): number;
        getDampingLimLin(): number;
        getSoftnessLimAng(): number;
        getRestitutionLimAng(): number;
        getDampingLimAng(): number;
        getSoftnessOrthoLin(): number;
        getRestitutionOrthoLin(): number;
        getDampingOrthoLin(): number;
        getSoftnessOrthoAng(): number;
        getRestitutionOrthoAng(): number;
        getDampingOrthoAng(): number;
        setSoftnessDirLin(softnessDirLin: number): void;
        setRestitutionDirLin(restitutionDirLin: number): void;
        setDampingDirLin(dampingDirLin: number): void;
        setSoftnessDirAng(softnessDirAng: number): void;
        setRestitutionDirAng(restitutionDirAng: number): void;
        setDampingDirAng(dampingDirAng: number): void;
        setSoftnessLimLin(softnessLimLin: number): void;
        setRestitutionLimLin(restitutionLimLin: number): void;
        setDampingLimLin(dampingLimLin: number): void;
        setSoftnessLimAng(softnessLimAng: number): void;
        setRestitutionLimAng(restitutionLimAng: number): void;
        setDampingLimAng(dampingLimAng: number): void;
        setSoftnessOrthoLin(softnessOrthoLin: number): void;
        setRestitutionOrthoLin(restitutionOrthoLin: number): void;
        setDampingOrthoLin(dampingOrthoLin: number): void;
        setSoftnessOrthoAng(softnessOrthoAng: number): void;
        setRestitutionOrthoAng(restitutionOrthoAng: number): void;
        setDampingOrthoAng(dampingOrthoAng: number): void;
        setPoweredLinMotor(onOff: boolean): void;
        getPoweredLinMotor(): boolean;
        setTargetLinMotorVelocity(targetLinMotorVelocity: number): void;
        getTargetLinMotorVelocity(): number;
        setMaxLinMotorForce(maxLinMotorForce: number): void;
        getMaxLinMotorForce(): number;
        setPoweredAngMotor(onOff: boolean): void;
        getPoweredAngMotor(): boolean;
        setTargetAngMotorVelocity(targetAngMotorVelocity: number): void;
        getTargetAngMotorVelocity(): number;
        setMaxAngMotorForce(maxAngMotorForce: number): void;
        getMaxAngMotorForce(): number;
        getLinearPos(): number;
        getAngularPos(): number;
        getUseFrameOffset(): boolean;
        setUseFrameOffset(frameOffsetOnOff: boolean): void;
        setFrames(frameA: btTransform, frameB: btTransform): void;
        setParam(num: number, value: number, axis?: number): void;
        getParam(num: number, axis?: number): number;
        getFlags(): number;
    }
    class btGeneric6DofSpringConstraint extends btGeneric6DofConstraint {
        constructor(rbA: btRigidBody, rbB: btRigidBody, frameInA: btTransform, frameInB: btTransform, useLinearFrameReferenceFrameA: boolean);
        constructor(rbB: btRigidBody, frameInB: btTransform, useLinearFrameReferenceFrameB: boolean);
        enableSpring(index: number, onOff: boolean): void;
        setStiffness(index: number, stiffness: number): void;
        setDamping(index: number, damping: number): void;
        setEquilibriumPoint(): void;
        setEquilibriumPoint(index: number): void;
        setEquilibriumPoint(index: number, val: number): void;
        isSpringEnabled(index: number): boolean;
        getStiffness(index: number): number;
        getDamping(index: number): number;
        getEquilibriumPoint(index: number): number;
        setAxis(axis1: btVector3, axis2: btVector3): void;
    }
    class btGearConstraint extends btTypedConstraint {
        constructor(rbA: btRigidBody, rbB: btRigidBody, axisInA: btVector3, axisInB: btVector3, ratio?: number);
        setAxisA(axisA: btVector3): void;
        setAxisB(axisB: btVector3): void;
        setRatio(ratio: number): void;
        getAxisA(): btVector3;
        getAxisB(): btVector3;
        getRatio(): number;
        setParam(num: number, value: number, axis?: number): void;
        getParam(num: number, axis?: number): number;
    }
    class btFixedConstraint extends btGeneric6DofSpring2Constraint {
        constructor(rbA: btRigidBody, rbB: btRigidBody, frameInA: btTransform, frameInB: btTransform);
    }
    type RotateOrder = "RO_XYZ" | "RO_XZY" | "RO_YXZ" | "RO_YZX" | "RO_ZXY" | "RO_ZYX";
    class btRotationalLimitMotor2 {
        constructor();
        get_m_loLimit(): number;
        set_m_loLimit(m_loLimit: number): void;
        m_loLimit: number;
        get_m_hiLimit(): number;
        set_m_hiLimit(m_hiLimit: number): void;
        m_hiLimit: number;
        get_m_bounce(): number;
        set_m_bounce(m_bounce: number): void;
        m_bounce: number;
        get_m_stopERP(): number;
        set_m_stopERP(m_stopERP: number): void;
        m_stopERP: number;
        get_m_stopCFM(): number;
        set_m_stopCFM(m_stopCFM: number): void;
        m_stopCFM: number;
        get_m_motorERP(): number;
        set_m_motorERP(m_motorERP: number): void;
        m_motorERP: number;
        get_m_motorCFM(): number;
        set_m_motorCFM(m_motorCFM: number): void;
        m_motorCFM: number;
        get_m_enableMotor(): boolean;
        set_m_enableMotor(m_enableMotor: boolean): void;
        m_enableMotor: boolean;
        get_m_targetVelocity(): number;
        set_m_targetVelocity(m_targetVelocity: number): void;
        m_targetVelocity: number;
        get_m_maxMotorForce(): number;
        set_m_maxMotorForce(m_maxMotorForce: number): void;
        m_maxMotorForce: number;
        get_m_servoMotor(): boolean;
        set_m_servoMotor(m_servoMotor: boolean): void;
        m_servoMotor: boolean;
        get_m_servoTarget(): number;
        set_m_servoTarget(m_servoTarget: number): void;
        m_servoTarget: number;
        get_m_enableSpring(): boolean;
        set_m_enableSpring(m_enableSpring: boolean): void;
        m_enableSpring: boolean;
        get_m_springStiffness(): number;
        set_m_springStiffness(m_springStiffness: number): void;
        m_springStiffness: number;
        get_m_springStiffnessLimited(): boolean;
        set_m_springStiffnessLimited(m_springStiffnessLimited: boolean): void;
        m_springStiffnessLimited: boolean;
        get_m_springDamping(): number;
        set_m_springDamping(m_springDamping: number): void;
        m_springDamping: number;
        get_m_springDampingLimited(): boolean;
        set_m_springDampingLimited(m_springDampingLimited: boolean): void;
        m_springDampingLimited: boolean;
        get_m_equilibriumPoint(): number;
        set_m_equilibriumPoint(m_equilibriumPoint: number): void;
        m_equilibriumPoint: number;
        get_m_currentLimitError(): number;
        set_m_currentLimitError(m_currentLimitError: number): void;
        m_currentLimitError: number;
        get_m_currentLimitErrorHi(): number;
        set_m_currentLimitErrorHi(m_currentLimitErrorHi: number): void;
        m_currentLimitErrorHi: number;
        get_m_currentPosition(): number;
        set_m_currentPosition(m_currentPosition: number): void;
        m_currentPosition: number;
        get_m_currentLimit(): number;
        set_m_currentLimit(m_currentLimit: number): void;
        m_currentLimit: number;
        isLimited(): boolean;
        testLimitValue(test_value: number): void;
    }
    class btTranslationalLimitMotor2 {
        constructor();
        get_m_lowerLimit(): btVector3;
        set_m_lowerLimit(m_lowerLimit: btVector3): void;
        m_lowerLimit: btVector3;
        get_m_upperLimit(): btVector3;
        set_m_upperLimit(m_upperLimit: btVector3): void;
        m_upperLimit: btVector3;
        get_m_bounce(): btVector3;
        set_m_bounce(m_bounce: btVector3): void;
        m_bounce: btVector3;
        get_m_stopERP(): btVector3;
        set_m_stopERP(m_stopERP: btVector3): void;
        m_stopERP: btVector3;
        get_m_stopCFM(): btVector3;
        set_m_stopCFM(m_stopCFM: btVector3): void;
        m_stopCFM: btVector3;
        get_m_motorERP(): btVector3;
        set_m_motorERP(m_motorERP: btVector3): void;
        m_motorERP: btVector3;
        get_m_motorCFM(): btVector3;
        set_m_motorCFM(m_motorCFM: btVector3): void;
        m_motorCFM: btVector3;
        get_m_enableMotor(): ReadonlyArray<boolean>;
        set_m_enableMotor(m_enableMotor: ReadonlyArray<boolean>): void;
        m_enableMotor: ReadonlyArray<boolean>;
        get_m_servoMotor(): ReadonlyArray<boolean>;
        set_m_servoMotor(m_servoMotor: ReadonlyArray<boolean>): void;
        m_servoMotor: ReadonlyArray<boolean>;
        get_m_enableSpring(): ReadonlyArray<boolean>;
        set_m_enableSpring(m_enableSpring: ReadonlyArray<boolean>): void;
        m_enableSpring: ReadonlyArray<boolean>;
        get_m_servoTarget(): btVector3;
        set_m_servoTarget(m_servoTarget: btVector3): void;
        m_servoTarget: btVector3;
        get_m_springStiffness(): btVector3;
        set_m_springStiffness(m_springStiffness: btVector3): void;
        m_springStiffness: btVector3;
        get_m_springStiffnessLimited(): ReadonlyArray<boolean>;
        set_m_springStiffnessLimited(m_springStiffnessLimited: ReadonlyArray<boolean>): void;
        m_springStiffnessLimited: ReadonlyArray<boolean>;
        get_m_springDamping(): btVector3;
        set_m_springDamping(m_springDamping: btVector3): void;
        m_springDamping: btVector3;
        get_m_springDampingLimited(): ReadonlyArray<boolean>;
        set_m_springDampingLimited(m_springDampingLimited: ReadonlyArray<boolean>): void;
        m_springDampingLimited: ReadonlyArray<boolean>;
        get_m_equilibriumPoint(): btVector3;
        set_m_equilibriumPoint(m_equilibriumPoint: btVector3): void;
        m_equilibriumPoint: btVector3;
        get_m_targetVelocity(): btVector3;
        set_m_targetVelocity(m_targetVelocity: btVector3): void;
        m_targetVelocity: btVector3;
        get_m_maxMotorForce(): btVector3;
        set_m_maxMotorForce(m_maxMotorForce: btVector3): void;
        m_maxMotorForce: btVector3;
        get_m_currentLimitError(): btVector3;
        set_m_currentLimitError(m_currentLimitError: btVector3): void;
        m_currentLimitError: btVector3;
        get_m_currentLimitErrorHi(): btVector3;
        set_m_currentLimitErrorHi(m_currentLimitErrorHi: btVector3): void;
        m_currentLimitErrorHi: btVector3;
        get_m_currentLinearDiff(): btVector3;
        set_m_currentLinearDiff(m_currentLinearDiff: btVector3): void;
        m_currentLinearDiff: btVector3;
        get_m_currentLimit(): ReadonlyArray<number>;
        set_m_currentLimit(m_currentLimit: ReadonlyArray<number>): void;
        m_currentLimit: ReadonlyArray<number>;
        isLimited(limitIndex: number): boolean;
        testLimitValue(limitIndex: number, test_value: number): void;
    }
    type bt6DofFlags2 = "BT_6DOF_FLAGS_CFM_STOP2" | "BT_6DOF_FLAGS_ERP_STOP2" | "BT_6DOF_FLAGS_CFM_MOTO2" | "BT_6DOF_FLAGS_ERP_MOTO2" | "BT_6DOF_FLAGS_USE_INFINITE_ERROR";
    class btGeneric6DofSpring2Constraint extends btTypedConstraint {
        constructor(rbA: btRigidBody, rbB: btRigidBody, frameInA: btTransform, frameInB: btTransform, rotOrder?: RotateOrder);
        constructor(rbB: btRigidBody, frameInB: btTransform, rotOrder?: RotateOrder);
        buildJacobian(): void;
        getRotationalLimitMotor(index: number): btRotationalLimitMotor2;
        getTranslationalLimitMotor(): btTranslationalLimitMotor2;
        calculateTransforms(transA: btTransform, transB: btTransform): void;
        calculateTransforms(): void;
        getCalculatedTransformA(): btTransform;
        getCalculatedTransformB(): btTransform;
        getFrameOffsetA(): btTransform;
        getFrameOffsetB(): btTransform;
        getAxis(axis_index: number): btVector3;
        getAngle(axis_index: number): number;
        getRelativePivotPosition(axis_index: number): number;
        setFrames(frameA: btTransform, frameB: btTransform): void;
        setLinearLowerLimit(linearLower: btVector3): void;
        getLinearLowerLimit(linearLower: btVector3): void;
        setLinearUpperLimit(linearUpper: btVector3): void;
        getLinearUpperLimit(linearUpper: btVector3): void;
        setAngularLowerLimit(angularLower: btVector3): void;
        setAngularLowerLimitReversed(angularLower: btVector3): void;
        getAngularLowerLimit(angularLower: btVector3): void;
        getAngularLowerLimitReversed(angularLower: btVector3): void;
        setAngularUpperLimit(angularUpper: btVector3): void;
        setAngularUpperLimitReversed(angularUpper: btVector3): void;
        getAngularUpperLimit(angularUpper: btVector3): void;
        getAngularUpperLimitReversed(angularUpper: btVector3): void;
        setLimit(axis: number, lo: number, hi: number): void;
        setLimitReversed(axis: number, lo: number, hi: number): void;
        isLimited(limitIndex: number): boolean;
        setRotationOrder(order: RotateOrder): void;
        getRotationOrder(): RotateOrder;
        setAxis(axis1: btVector3, axis2: btVector3): void;
        setBounce(index: number, bounce: number): void;
        enableMotor(index: number, onOff: boolean): void;
        setServo(index: number, onOff: boolean): void;
        setTargetVelocity(index: number, velocity: number): void;
        setServoTarget(index: number, target: number): void;
        setMaxMotorForce(index: number, force: number): void;
        enableSpring(index: number, onOff: boolean): void;
        setStiffness(index: number, stiffness: number): void;
        setDamping(index: number, damping: number): void;
        setEquilibriumPoint(): void;
        setEquilibriumPoint(index: number): void;
        setEquilibriumPoint(index: number, val: number): void;
        setParam(num: number, value: number, axis?: number): void;
        getParam(num: number, axis?: number): number;
        btGetMatrixElem(mat: btMatrix3x3, index: number): number;
        matrixToEulerXYZ(mat: btMatrix3x3, xyz: btVector3): boolean;
        matrixToEulerXZY(mat: btMatrix3x3, xyz: btVector3): boolean;
        matrixToEulerYXZ(mat: btMatrix3x3, xyz: btVector3): boolean;
        matrixToEulerYZX(mat: btMatrix3x3, xyz: btVector3): boolean;
        matrixToEulerZXY(mat: btMatrix3x3, xyz: btVector3): boolean;
        matrixToEulerZYX(mat: btMatrix3x3, xyz: btVector3): boolean;
    }
    class btHinge2Constraint extends btGeneric6DofSpring2Constraint {
        constructor(rbA: btRigidBody, rbB: btRigidBody, anchor: btVector3, axis1: btVector3, axis2: btVector3);
        getAnchor(): btVector3;
        getAnchor2(): btVector3;
        getAxis1(): btVector3;
        getAxis2(): btVector3;
        getAngle1(): number;
        getAngle2(): number;
        setUpperLimit(ang1max: number): void;
        setLowerLimit(ang1min: number): void;
    }
    class btConstraintSolver {
    }
    class btDispatcherInfo {
        get_m_timeStep(): number;
        set_m_timeStep(m_timeStep: number): void;
        m_timeStep: number;
        get_m_stepCount(): number;
        set_m_stepCount(m_stepCount: number): void;
        m_stepCount: number;
        get_m_dispatchFunc(): number;
        set_m_dispatchFunc(m_dispatchFunc: number): void;
        m_dispatchFunc: number;
        get_m_timeOfImpact(): number;
        set_m_timeOfImpact(m_timeOfImpact: number): void;
        m_timeOfImpact: number;
        get_m_useContinuous(): boolean;
        set_m_useContinuous(m_useContinuous: boolean): void;
        m_useContinuous: boolean;
        get_m_enableSatConvex(): boolean;
        set_m_enableSatConvex(m_enableSatConvex: boolean): void;
        m_enableSatConvex: boolean;
        get_m_enableSPU(): boolean;
        set_m_enableSPU(m_enableSPU: boolean): void;
        m_enableSPU: boolean;
        get_m_useEpa(): boolean;
        set_m_useEpa(m_useEpa: boolean): void;
        m_useEpa: boolean;
        get_m_allowedCcdPenetration(): number;
        set_m_allowedCcdPenetration(m_allowedCcdPenetration: number): void;
        m_allowedCcdPenetration: number;
        get_m_useConvexConservativeDistanceUtil(): boolean;
        set_m_useConvexConservativeDistanceUtil(m_useConvexConservativeDistanceUtil: boolean): void;
        m_useConvexConservativeDistanceUtil: boolean;
        get_m_convexConservativeDistanceThreshold(): number;
        set_m_convexConservativeDistanceThreshold(m_convexConservativeDistanceThreshold: number): void;
        m_convexConservativeDistanceThreshold: number;
    }
    class btCollisionWorld {
        getDispatcher(): btDispatcher;
        rayTest(rayFromWorld: btVector3, rayToWorld: btVector3, resultCallback: RayResultCallback): void;
        getPairCache(): btOverlappingPairCache;
        getDispatchInfo(): btDispatcherInfo;
        addCollisionObject(collisionObject: btCollisionObject, collisionFilterGroup?: number, collisionFilterMask?: number): void;
        removeCollisionObject(collisionObject: btCollisionObject): void;
        getBroadphase(): btBroadphaseInterface;
        convexSweepTest(castShape: btConvexShape, from: btTransform, to: btTransform, resultCallback: ConvexResultCallback, allowedCcdPenetration: number): void;
        contactPairTest(colObjA: btCollisionObject, colObjB: btCollisionObject, resultCallback: ContactResultCallback): void;
        contactTest(colObj: btCollisionObject, resultCallback: ContactResultCallback): void;
        setForceUpdateAllAabbs(forceUpdateAllAabbs: boolean): void;
        updateSingleAabb(colObj: btCollisionObject): void;
        setDebugDrawer(debugDrawer: btIDebugDraw): void;
        getDebugDrawer(): btIDebugDraw;
        debugDrawWorld(): void;
        debugDrawObject(worldTransform: btTransform, shape: btCollisionShape, color: btVector3): void;
    }
    class btContactSolverInfo {
        get_m_splitImpulse(): boolean;
        set_m_splitImpulse(m_splitImpulse: boolean): void;
        m_splitImpulse: boolean;
        get_m_splitImpulsePenetrationThreshold(): number;
        set_m_splitImpulsePenetrationThreshold(m_splitImpulsePenetrationThreshold: number): void;
        m_splitImpulsePenetrationThreshold: number;
        get_m_numIterations(): number;
        set_m_numIterations(m_numIterations: number): void;
        m_numIterations: number;
    }
    class btDynamicsWorld extends btCollisionWorld {
        addAction(action: btActionInterface): void;
        removeAction(action: btActionInterface): void;
        getSolverInfo(): btContactSolverInfo;
    }
    class btDiscreteDynamicsWorld extends btDynamicsWorld {
        constructor(dispatcher: btDispatcher, pairCache: btBroadphaseInterface, constraintSolver: btConstraintSolver, collisionConfiguration: btCollisionConfiguration);
        setGravity(gravity: btVector3): void;
        getGravity(): btVector3;
        addRigidBody(body: btRigidBody): void;
        addRigidBody(body: btRigidBody, group: number, mask: number): void;
        removeRigidBody(body: btRigidBody): void;
        addConstraint(constraint: btTypedConstraint, disableCollisionsBetweenLinkedBodies?: boolean): void;
        removeConstraint(constraint: btTypedConstraint): void;
        stepSimulation(timeStep: number, maxSubSteps?: number, fixedTimeStep?: number): number;
        setContactAddedCallback(funcpointer: number): void;
        setContactProcessedCallback(funcpointer: number): void;
        setContactDestroyedCallback(funcpointer: number): void;
    }
    class btVehicleTuning {
        constructor();
        get_m_suspensionStiffness(): number;
        set_m_suspensionStiffness(m_suspensionStiffness: number): void;
        m_suspensionStiffness: number;
        get_m_suspensionCompression(): number;
        set_m_suspensionCompression(m_suspensionCompression: number): void;
        m_suspensionCompression: number;
        get_m_suspensionDamping(): number;
        set_m_suspensionDamping(m_suspensionDamping: number): void;
        m_suspensionDamping: number;
        get_m_maxSuspensionTravelCm(): number;
        set_m_maxSuspensionTravelCm(m_maxSuspensionTravelCm: number): void;
        m_maxSuspensionTravelCm: number;
        get_m_frictionSlip(): number;
        set_m_frictionSlip(m_frictionSlip: number): void;
        m_frictionSlip: number;
        get_m_maxSuspensionForce(): number;
        set_m_maxSuspensionForce(m_maxSuspensionForce: number): void;
        m_maxSuspensionForce: number;
    }
    class btVehicleRaycasterResult {
        get_m_hitPointInWorld(): btVector3;
        set_m_hitPointInWorld(m_hitPointInWorld: btVector3): void;
        m_hitPointInWorld: btVector3;
        get_m_hitNormalInWorld(): btVector3;
        set_m_hitNormalInWorld(m_hitNormalInWorld: btVector3): void;
        m_hitNormalInWorld: btVector3;
        get_m_distFraction(): number;
        set_m_distFraction(m_distFraction: number): void;
        m_distFraction: number;
    }
    class btVehicleRaycaster {
        castRay(from: btVector3, to: btVector3, result: btVehicleRaycasterResult): void;
    }
    class btDefaultVehicleRaycaster extends btVehicleRaycaster {
        constructor(world: btDynamicsWorld);
    }
    class RaycastInfo {
        get_m_contactNormalWS(): btVector3;
        set_m_contactNormalWS(m_contactNormalWS: btVector3): void;
        m_contactNormalWS: btVector3;
        get_m_contactPointWS(): btVector3;
        set_m_contactPointWS(m_contactPointWS: btVector3): void;
        m_contactPointWS: btVector3;
        get_m_suspensionLength(): number;
        set_m_suspensionLength(m_suspensionLength: number): void;
        m_suspensionLength: number;
        get_m_hardPointWS(): btVector3;
        set_m_hardPointWS(m_hardPointWS: btVector3): void;
        m_hardPointWS: btVector3;
        get_m_wheelDirectionWS(): btVector3;
        set_m_wheelDirectionWS(m_wheelDirectionWS: btVector3): void;
        m_wheelDirectionWS: btVector3;
        get_m_wheelAxleWS(): btVector3;
        set_m_wheelAxleWS(m_wheelAxleWS: btVector3): void;
        m_wheelAxleWS: btVector3;
        get_m_isInContact(): boolean;
        set_m_isInContact(m_isInContact: boolean): void;
        m_isInContact: boolean;
        get_m_groundObject(): any;
        set_m_groundObject(m_groundObject: any): void;
        m_groundObject: any;
    }
    class btWheelInfoConstructionInfo {
        get_m_chassisConnectionCS(): btVector3;
        set_m_chassisConnectionCS(m_chassisConnectionCS: btVector3): void;
        m_chassisConnectionCS: btVector3;
        get_m_wheelDirectionCS(): btVector3;
        set_m_wheelDirectionCS(m_wheelDirectionCS: btVector3): void;
        m_wheelDirectionCS: btVector3;
        get_m_wheelAxleCS(): btVector3;
        set_m_wheelAxleCS(m_wheelAxleCS: btVector3): void;
        m_wheelAxleCS: btVector3;
        get_m_suspensionRestLength(): number;
        set_m_suspensionRestLength(m_suspensionRestLength: number): void;
        m_suspensionRestLength: number;
        get_m_maxSuspensionTravelCm(): number;
        set_m_maxSuspensionTravelCm(m_maxSuspensionTravelCm: number): void;
        m_maxSuspensionTravelCm: number;
        get_m_wheelRadius(): number;
        set_m_wheelRadius(m_wheelRadius: number): void;
        m_wheelRadius: number;
        get_m_suspensionStiffness(): number;
        set_m_suspensionStiffness(m_suspensionStiffness: number): void;
        m_suspensionStiffness: number;
        get_m_wheelsDampingCompression(): number;
        set_m_wheelsDampingCompression(m_wheelsDampingCompression: number): void;
        m_wheelsDampingCompression: number;
        get_m_wheelsDampingRelaxation(): number;
        set_m_wheelsDampingRelaxation(m_wheelsDampingRelaxation: number): void;
        m_wheelsDampingRelaxation: number;
        get_m_frictionSlip(): number;
        set_m_frictionSlip(m_frictionSlip: number): void;
        m_frictionSlip: number;
        get_m_maxSuspensionForce(): number;
        set_m_maxSuspensionForce(m_maxSuspensionForce: number): void;
        m_maxSuspensionForce: number;
        get_m_bIsFrontWheel(): boolean;
        set_m_bIsFrontWheel(m_bIsFrontWheel: boolean): void;
        m_bIsFrontWheel: boolean;
    }
    class btWheelInfo {
        get_m_suspensionStiffness(): number;
        set_m_suspensionStiffness(m_suspensionStiffness: number): void;
        m_suspensionStiffness: number;
        get_m_frictionSlip(): number;
        set_m_frictionSlip(m_frictionSlip: number): void;
        m_frictionSlip: number;
        get_m_engineForce(): number;
        set_m_engineForce(m_engineForce: number): void;
        m_engineForce: number;
        get_m_rollInfluence(): number;
        set_m_rollInfluence(m_rollInfluence: number): void;
        m_rollInfluence: number;
        get_m_suspensionRestLength1(): number;
        set_m_suspensionRestLength1(m_suspensionRestLength1: number): void;
        m_suspensionRestLength1: number;
        get_m_wheelsRadius(): number;
        set_m_wheelsRadius(m_wheelsRadius: number): void;
        m_wheelsRadius: number;
        get_m_wheelsDampingCompression(): number;
        set_m_wheelsDampingCompression(m_wheelsDampingCompression: number): void;
        m_wheelsDampingCompression: number;
        get_m_wheelsDampingRelaxation(): number;
        set_m_wheelsDampingRelaxation(m_wheelsDampingRelaxation: number): void;
        m_wheelsDampingRelaxation: number;
        get_m_steering(): number;
        set_m_steering(m_steering: number): void;
        m_steering: number;
        get_m_maxSuspensionForce(): number;
        set_m_maxSuspensionForce(m_maxSuspensionForce: number): void;
        m_maxSuspensionForce: number;
        get_m_maxSuspensionTravelCm(): number;
        set_m_maxSuspensionTravelCm(m_maxSuspensionTravelCm: number): void;
        m_maxSuspensionTravelCm: number;
        get_m_wheelsSuspensionForce(): number;
        set_m_wheelsSuspensionForce(m_wheelsSuspensionForce: number): void;
        m_wheelsSuspensionForce: number;
        get_m_bIsFrontWheel(): boolean;
        set_m_bIsFrontWheel(m_bIsFrontWheel: boolean): void;
        m_bIsFrontWheel: boolean;
        get_m_raycastInfo(): RaycastInfo;
        set_m_raycastInfo(m_raycastInfo: RaycastInfo): void;
        m_raycastInfo: RaycastInfo;
        get_m_chassisConnectionPointCS(): btVector3;
        set_m_chassisConnectionPointCS(m_chassisConnectionPointCS: btVector3): void;
        m_chassisConnectionPointCS: btVector3;
        constructor(ci: btWheelInfoConstructionInfo);
        getSuspensionRestLength(): number;
        updateWheel(chassis: btRigidBody, raycastInfo: RaycastInfo): void;
        get_m_worldTransform(): btTransform;
        set_m_worldTransform(m_worldTransform: btTransform): void;
        m_worldTransform: btTransform;
        get_m_wheelDirectionCS(): btVector3;
        set_m_wheelDirectionCS(m_wheelDirectionCS: btVector3): void;
        m_wheelDirectionCS: btVector3;
        get_m_wheelAxleCS(): btVector3;
        set_m_wheelAxleCS(m_wheelAxleCS: btVector3): void;
        m_wheelAxleCS: btVector3;
        get_m_rotation(): number;
        set_m_rotation(m_rotation: number): void;
        m_rotation: number;
        get_m_deltaRotation(): number;
        set_m_deltaRotation(m_deltaRotation: number): void;
        m_deltaRotation: number;
        get_m_brake(): number;
        set_m_brake(m_brake: number): void;
        m_brake: number;
        get_m_clippedInvContactDotSuspension(): number;
        set_m_clippedInvContactDotSuspension(m_clippedInvContactDotSuspension: number): void;
        m_clippedInvContactDotSuspension: number;
        get_m_suspensionRelativeVelocity(): number;
        set_m_suspensionRelativeVelocity(m_suspensionRelativeVelocity: number): void;
        m_suspensionRelativeVelocity: number;
        get_m_skidInfo(): number;
        set_m_skidInfo(m_skidInfo: number): void;
        m_skidInfo: number;
    }
    class btActionInterface {
        updateAction(collisionWorld: btCollisionWorld, deltaTimeStep: number): void;
    }
    class btRaycastVehicle extends btActionInterface {
        constructor(tuning: btVehicleTuning, chassis: btRigidBody, raycaster: btVehicleRaycaster);
        applyEngineForce(force: number, wheel: number): void;
        setSteeringValue(steering: number, wheel: number): void;
        getWheelTransformWS(wheelIndex: number): btTransform;
        updateWheelTransform(wheelIndex: number, interpolatedTransform: boolean): void;
        addWheel(connectionPointCS0: btVector3, wheelDirectionCS0: btVector3, wheelAxleCS: btVector3, suspensionRestLength: number, wheelRadius: number, tuning: btVehicleTuning, isFrontWheel: boolean): btWheelInfo;
        getNumWheels(): number;
        getRigidBody(): btRigidBody;
        getWheelInfo(index: number): btWheelInfo;
        setBrake(brake: number, wheelIndex: number): void;
        setCoordinateSystem(rightIndex: number, upIndex: number, forwardIndex: number): void;
        getCurrentSpeedKmHour(): number;
        getChassisWorldTransform(): btTransform;
        rayCast(wheel: btWheelInfo): number;
        updateVehicle(step: number): void;
        resetSuspension(): void;
        getSteeringValue(wheel: number): number;
        updateWheelTransformsWS(wheel: btWheelInfo, interpolatedTransform?: boolean): void;
        setPitchControl(pitch: number): void;
        updateSuspension(deltaTime: number): void;
        updateFriction(timeStep: number): void;
        getRightAxis(): number;
        getUpAxis(): number;
        getForwardAxis(): number;
        getForwardVector(): btVector3;
        getUserConstraintType(): number;
        setUserConstraintType(userConstraintType: number): void;
        setUserConstraintId(uid: number): void;
        getUserConstraintId(): number;
    }
    class btKinematicCharacterController extends btActionInterface {
        constructor(ghostObject: btPairCachingGhostObject, convexShape: btConvexShape, stepHeight: number, upAxis: btVector3);
        setUp(up: btVector3): void;
        setWalkDirection(walkDirection: btVector3): void;
        setVelocityForTimeInterval(velocity: btVector3, timeInterval: number): void;
        warp(origin: btVector3): void;
        preStep(collisionWorld: btCollisionWorld): void;
        playerStep(collisionWorld: btCollisionWorld, dt: number): void;
        setFallSpeed(fallSpeed: number): void;
        setJumpSpeed(jumpSpeed: number): void;
        setMaxJumpHeight(maxJumpHeight: number): void;
        canJump(): boolean;
        jump(): void;
        setGravity(gravity: btVector3): void;
        getGravity(): btVector3;
        setMaxSlope(slopeRadians: number): void;
        getMaxSlope(): number;
        getGhostObject(): btPairCachingGhostObject;
        setUseGhostSweepTest(useGhostObjectSweepTest: boolean): void;
        onGround(): boolean;
        setUpInterpolate(value: boolean): void;
    }
    class btGhostObject extends btCollisionObject {
        constructor();
        getNumOverlappingObjects(): number;
        getOverlappingObject(index: number): btCollisionObject;
    }
    class btPairCachingGhostObject extends btGhostObject {
        constructor();
    }
    class btGhostPairCallback {
        constructor();
    }
    class Vec3Long {
        constructor(x: number, y: number, z: number);
        X(): number;
        Y(): number;
        Z(): number;
    }
    class Vec3Real {
        constructor(x: number, y: number, z: number);
        X(): number;
        Y(): number;
        Z(): number;
    }
    class HACD {
        constructor();
        SetCompacityWeight(alpha: number): void;
        SetVolumeWeight(beta: number): void;
        SetConcavity(concavity: number): void;
        SetNClusters(nClusters: number): void;
        SetNVerticesPerCH(nVerticesPerCH: number): void;
        SetPoints(points: Vec3Real): void;
        SetNPoints(nPoints: number): void;
        SetTriangles(triangles: Vec3Long): void;
        SetNTriangles(nTriangles: number): void;
        Compute(fullCH?: boolean, exportDistPoints?: boolean): boolean;
        GetNClusters(): number;
        GetNPointsCH(numCH: number): number;
        GetNTrianglesCH(numCH: number): number;
        GetCH(c: number, points: Vec3Real, triangles: Vec3Long): number;
    }
    class VHACD {
        constructor();
        Compute(points: ReadonlyArray<number>, stridePoints: number, nPoints: number, triangles: ReadonlyArray<number>, strideTriangles: number, nTriangles: number, params: Parameters): boolean;
        GetNConvexHulls(): number;
        GetConvexHull(index: number, ch: ConvexHull): void;
        Cancel(): void;
        Clean(): void;
        Release(): void;
    }
    class Parameters {
        constructor();
        get_m_concavity(): number;
        set_m_concavity(m_concavity: number): void;
        m_concavity: number;
        get_m_alpha(): number;
        set_m_alpha(m_alpha: number): void;
        m_alpha: number;
        get_m_beta(): number;
        set_m_beta(m_beta: number): void;
        m_beta: number;
        get_m_gamma(): number;
        set_m_gamma(m_gamma: number): void;
        m_gamma: number;
        get_m_minVolumePerCH(): number;
        set_m_minVolumePerCH(m_minVolumePerCH: number): void;
        m_minVolumePerCH: number;
        get_m_resolution(): number;
        set_m_resolution(m_resolution: number): void;
        m_resolution: number;
        get_m_maxNumVerticesPerCH(): number;
        set_m_maxNumVerticesPerCH(m_maxNumVerticesPerCH: number): void;
        m_maxNumVerticesPerCH: number;
        get_m_depth(): number;
        set_m_depth(m_depth: number): void;
        m_depth: number;
        get_m_planeDownsampling(): number;
        set_m_planeDownsampling(m_planeDownsampling: number): void;
        m_planeDownsampling: number;
        get_m_convexhullDownsampling(): number;
        set_m_convexhullDownsampling(m_convexhullDownsampling: number): void;
        m_convexhullDownsampling: number;
        get_m_pca(): number;
        set_m_pca(m_pca: number): void;
        m_pca: number;
        get_m_mode(): number;
        set_m_mode(m_mode: number): void;
        m_mode: number;
        get_m_convexhullApproximation(): number;
        set_m_convexhullApproximation(m_convexhullApproximation: number): void;
        m_convexhullApproximation: number;
        get_m_oclAcceleration(): number;
        set_m_oclAcceleration(m_oclAcceleration: number): void;
        m_oclAcceleration: number;
    }
    class ConvexHull {
        constructor();
        get_m_points(): ReadonlyArray<number>;
        set_m_points(m_points: ReadonlyArray<number>): void;
        readonly m_points: ReadonlyArray<number>;
        get_m_triangles(): ReadonlyArray<number>;
        set_m_triangles(m_triangles: ReadonlyArray<number>): void;
        readonly m_triangles: ReadonlyArray<number>;
        get_m_nPoints(): number;
        set_m_nPoints(m_nPoints: number): void;
        readonly m_nPoints: number;
        get_m_nTriangles(): number;
        set_m_nTriangles(m_nTriangles: number): void;
        readonly m_nTriangles: number;
    }
    class btSoftBodyWorldInfo {
        constructor();
        get_air_density(): number;
        set_air_density(air_density: number): void;
        air_density: number;
        get_water_density(): number;
        set_water_density(water_density: number): void;
        water_density: number;
        get_water_offset(): number;
        set_water_offset(water_offset: number): void;
        water_offset: number;
        get_m_maxDisplacement(): number;
        set_m_maxDisplacement(m_maxDisplacement: number): void;
        m_maxDisplacement: number;
        get_water_normal(): btVector3;
        set_water_normal(water_normal: btVector3): void;
        water_normal: btVector3;
        get_m_broadphase(): btBroadphaseInterface;
        set_m_broadphase(m_broadphase: btBroadphaseInterface): void;
        m_broadphase: btBroadphaseInterface;
        get_m_dispatcher(): btDispatcher;
        set_m_dispatcher(m_dispatcher: btDispatcher): void;
        m_dispatcher: btDispatcher;
        get_m_gravity(): btVector3;
        set_m_gravity(m_gravity: btVector3): void;
        m_gravity: btVector3;
    }
    class Face {
        get_m_n(): ReadonlyArray<Node>;
        set_m_n(m_n: ReadonlyArray<Node>): void;
        m_n: ReadonlyArray<Node>;
        get_m_normal(): btVector3;
        set_m_normal(m_normal: btVector3): void;
        m_normal: btVector3;
        get_m_ra(): number;
        set_m_ra(m_ra: number): void;
        m_ra: number;
    }
    class tFaceArray {
        size(): number;
        at(n: number): Face;
    }
    class Node {
        get_m_x(): btVector3;
        set_m_x(m_x: btVector3): void;
        m_x: btVector3;
        get_m_q(): btVector3;
        set_m_q(m_q: btVector3): void;
        m_q: btVector3;
        get_m_v(): btVector3;
        set_m_v(m_v: btVector3): void;
        m_v: btVector3;
        get_m_f(): btVector3;
        set_m_f(m_f: btVector3): void;
        m_f: btVector3;
        get_m_n(): btVector3;
        set_m_n(m_n: btVector3): void;
        m_n: btVector3;
        get_m_im(): number;
        set_m_im(m_im: number): void;
        m_im: number;
        get_m_area(): number;
        set_m_area(m_area: number): void;
        m_area: number;
    }
    class tNodeArray {
        size(): number;
        at(n: number): Node;
    }
    class Material {
        get_m_kLST(): number;
        set_m_kLST(m_kLST: number): void;
        m_kLST: number;
        get_m_kAST(): number;
        set_m_kAST(m_kAST: number): void;
        m_kAST: number;
        get_m_kVST(): number;
        set_m_kVST(m_kVST: number): void;
        m_kVST: number;
        get_m_flags(): number;
        set_m_flags(m_flags: number): void;
        m_flags: number;
    }
    class tMaterialArray {
        size(): number;
        at(n: number): Material;
    }
    class Anchor {
        get_m_node(): Node;
        set_m_node(m_node: Node): void;
        m_node: Node;
        get_m_local(): btVector3;
        set_m_local(m_local: btVector3): void;
        m_local: btVector3;
        get_m_body(): btRigidBody;
        set_m_body(m_body: btRigidBody): void;
        m_body: btRigidBody;
        get_m_influence(): number;
        set_m_influence(m_influence: number): void;
        m_influence: number;
        get_m_c0(): btMatrix3x3;
        set_m_c0(m_c0: btMatrix3x3): void;
        m_c0: btMatrix3x3;
        get_m_c1(): btVector3;
        set_m_c1(m_c1: btVector3): void;
        m_c1: btVector3;
        get_m_c2(): number;
        set_m_c2(m_c2: number): void;
        m_c2: number;
    }
    class tAnchorArray {
        size(): number;
        at(n: number): Anchor;
        clear(): void;
        push_back(val: Anchor): void;
        pop_back(): void;
    }
    class Config {
        get_kVCF(): number;
        set_kVCF(kVCF: number): void;
        kVCF: number;
        get_kDP(): number;
        set_kDP(kDP: number): void;
        kDP: number;
        get_kDG(): number;
        set_kDG(kDG: number): void;
        kDG: number;
        get_kLF(): number;
        set_kLF(kLF: number): void;
        kLF: number;
        get_kPR(): number;
        set_kPR(kPR: number): void;
        kPR: number;
        get_kVC(): number;
        set_kVC(kVC: number): void;
        kVC: number;
        get_kDF(): number;
        set_kDF(kDF: number): void;
        kDF: number;
        get_kMT(): number;
        set_kMT(kMT: number): void;
        kMT: number;
        get_kCHR(): number;
        set_kCHR(kCHR: number): void;
        kCHR: number;
        get_kKHR(): number;
        set_kKHR(kKHR: number): void;
        kKHR: number;
        get_kSHR(): number;
        set_kSHR(kSHR: number): void;
        kSHR: number;
        get_kAHR(): number;
        set_kAHR(kAHR: number): void;
        kAHR: number;
        get_kSRHR_CL(): number;
        set_kSRHR_CL(kSRHR_CL: number): void;
        kSRHR_CL: number;
        get_kSKHR_CL(): number;
        set_kSKHR_CL(kSKHR_CL: number): void;
        kSKHR_CL: number;
        get_kSSHR_CL(): number;
        set_kSSHR_CL(kSSHR_CL: number): void;
        kSSHR_CL: number;
        get_kSR_SPLT_CL(): number;
        set_kSR_SPLT_CL(kSR_SPLT_CL: number): void;
        kSR_SPLT_CL: number;
        get_kSK_SPLT_CL(): number;
        set_kSK_SPLT_CL(kSK_SPLT_CL: number): void;
        kSK_SPLT_CL: number;
        get_kSS_SPLT_CL(): number;
        set_kSS_SPLT_CL(kSS_SPLT_CL: number): void;
        kSS_SPLT_CL: number;
        get_maxvolume(): number;
        set_maxvolume(maxvolume: number): void;
        maxvolume: number;
        get_timescale(): number;
        set_timescale(timescale: number): void;
        timescale: number;
        get_viterations(): number;
        set_viterations(viterations: number): void;
        viterations: number;
        get_piterations(): number;
        set_piterations(piterations: number): void;
        piterations: number;
        get_diterations(): number;
        set_diterations(diterations: number): void;
        diterations: number;
        get_citerations(): number;
        set_citerations(citerations: number): void;
        citerations: number;
        get_collisions(): number;
        set_collisions(collisions: number): void;
        collisions: number;
    }
    class btSoftBody extends btCollisionObject {
        constructor(worldInfo: btSoftBodyWorldInfo, node_count: number, x: btVector3, m: ReadonlyArray<number>);
        get_m_cfg(): Config;
        set_m_cfg(m_cfg: Config): void;
        m_cfg: Config;
        get_m_nodes(): tNodeArray;
        set_m_nodes(m_nodes: tNodeArray): void;
        m_nodes: tNodeArray;
        get_m_faces(): tFaceArray;
        set_m_faces(m_faces: tFaceArray): void;
        m_faces: tFaceArray;
        get_m_materials(): tMaterialArray;
        set_m_materials(m_materials: tMaterialArray): void;
        m_materials: tMaterialArray;
        get_m_anchors(): tAnchorArray;
        set_m_anchors(m_anchors: tAnchorArray): void;
        m_anchors: tAnchorArray;
        checkLink(node0: number, node1: number): boolean;
        checkFace(node0: number, node1: number, node2: number): boolean;
        appendMaterial(): Material;
        appendNode(x: btVector3, m: number): void;
        appendLink(node0: number, node1: number, mat: Material, bcheckexist: boolean): void;
        appendFace(node0: number, node1: number, node2: number, mat: Material): void;
        appendTetra(node0: number, node1: number, node2: number, node3: number, mat: Material): void;
        appendAnchor(node: number, body: btRigidBody, disableCollisionBetweenLinkedBodies: boolean, influence: number): void;
        addForce(force: btVector3): void;
        addForce(force: btVector3, node: number): void;
        addAeroForceToNode(windVelocity: btVector3, nodeIndex: number): void;
        getTotalMass(): number;
        setTotalMass(mass: number, fromfaces: boolean): void;
        setMass(node: number, mass: number): void;
        transform(trs: btTransform): void;
        translate(trs: btVector3): void;
        rotate(rot: btQuaternion): void;
        scale(scl: btVector3): void;
        generateClusters(k: number, maxiterations?: number): number;
        generateBendingConstraints(distance: number, mat: Material): number;
        upcast(colObj: btCollisionObject): btSoftBody;
        getRestLengthScale(): number;
        setRestLengthScale(restLength: number): void;
    }
    class btSoftBodyRigidBodyCollisionConfiguration extends btDefaultCollisionConfiguration {
        constructor(info?: btDefaultCollisionConstructionInfo);
    }
    class btSoftBodySolver {
    }
    class btDefaultSoftBodySolver extends btSoftBodySolver {
        constructor();
    }
    class btSoftBodyArray {
        size(): number;
        at(n: number): btSoftBody;
    }
    class btSoftRigidDynamicsWorld extends btDiscreteDynamicsWorld {
        constructor(dispatcher: btDispatcher, pairCache: btBroadphaseInterface, constraintSolver: btConstraintSolver, collisionConfiguration: btCollisionConfiguration, softBodySolver: btSoftBodySolver);
        addSoftBody(body: btSoftBody, collisionFilterGroup: number, collisionFilterMask: number): void;
        removeSoftBody(body: btSoftBody): void;
        removeCollisionObject(collisionObject: btCollisionObject): void;
        getWorldInfo(): btSoftBodyWorldInfo;
        getSoftBodyArray(): btSoftBodyArray;
    }
    class btSoftBodyHelpers {
        constructor();
        CreateRope(worldInfo: btSoftBodyWorldInfo, from: btVector3, to: btVector3, res: number, fixeds: number): btSoftBody;
        CreatePatch(worldInfo: btSoftBodyWorldInfo, corner00: btVector3, corner10: btVector3, corner01: btVector3, corner11: btVector3, resx: number, resy: number, fixeds: number, gendiags: boolean, perturbation?: number): btSoftBody;
        CreatePatchUV(worldInfo: btSoftBodyWorldInfo, corner00: btVector3, corner10: btVector3, corner01: btVector3, corner11: btVector3, resx: number, resy: number, fixeds: number, gendiags: boolean, tex_coords?: ReadonlyArray<number>): btSoftBody;
        CalculateUV(resx: number, resy: number, ix: number, iy: number, id: number): number;
        CreateEllipsoid(worldInfo: btSoftBodyWorldInfo, center: btVector3, radius: btVector3, res: number): btSoftBody;
        CreateFromTriMesh(worldInfo: btSoftBodyWorldInfo, vertices: ReadonlyArray<number>, triangles: ReadonlyArray<number>, ntriangles: number, randomizeConstraints?: boolean): btSoftBody;
        CreateFromConvexHull(worldInfo: btSoftBodyWorldInfo, vertices: btVector3, nvertices: number, randomizeConstraints?: boolean): btSoftBody;
    }
}