import test from 'node:test'
import assert from 'node:assert/strict'
import { Vector3, Matrix, Quaternion } from '@babylonjs/core/Maths/math.vector.js'
import { ShapeCastResult } from '@babylonjs/core/Physics/shapeCastResult.js'
import RaycastVehicle from '../src/physics/raycastVehicle.js'

const createMockBody = ({ filterMembershipMask = 2, hitDistanceByWheel = [0.6, 0.6] } = {}) => {
    const appliedForces = []
    const raycastQueries = []
    const identityMatrix = Matrix.Identity()

    const physicsEngine = {
        gravity: new Vector3(0, -9.81, 0),
        raycastCalls: 0,
        raycastToRef(_from, _to, result, query) {
            const callIndex = this.raycastCalls++
            raycastQueries.push(query ? { ...query } : undefined)
            const dist = hitDistanceByWheel[callIndex]
            if (dist === undefined || dist === null) {
                result.reset(_from, _to)
                return
            }
            result.reset(_from, _to)
            result.setHitData({ x: 0, y: 1, z: 0 }, { x: _from.x, y: _from.y - dist, z: _from.z })
            result.setHitDistance(dist)
        }
    }

    const body = {
        _physicsEngine: physicsEngine,
        shape: { filterMembershipMask },
        transformNode: {
            position: new Vector3(0, 1, 0),
            rotationQuaternion: new Quaternion(),
            computeWorldMatrix() {},
            getWorldMatrix() {
                return identityMatrix
            }
        },
        getLinearVelocity() {
            return new Vector3(0, 0, 0)
        },
        getAngularVelocity() {
            return new Vector3(0, 0, 0)
        },
        setAngularVelocity() {},
        getGravityFactor() {
            return 1
        },
        applyForce(force, point) {
            appliedForces.push({
                force: force.clone(),
                point: point.clone()
            })
        }
    }

    return { body, appliedForces, raycastQueries }
}

const createMockWheel = ({ x, suspensionRestLength = 0.6 }) => ({
    positionLocal: new Vector3(x, 0, 0),
    positionWorld: new Vector3(x, 1, 0),
    suspensionAxisLocal: new Vector3(0, -1, 0),
    suspensionAxisWorld: new Vector3(0, -1, 0),
    axleAxisLocal: new Vector3(1, 0, 0),
    forwardAxisLocal: new Vector3(0, 0, 1),
    suspensionRestLength,
    springRate: 0,
    damperRate: 0,
    radius: 0.2,
    steering: 0,
    rotation: 0,
    inContact: false,
    hitDistance: suspensionRestLength,
    hitPoint: new Vector3(),
    hitNormal: new Vector3(0, 1, 0),
    previousSuspensionCompressionMeters: 0,
    suspensionCompressionMeters: 0,
    normalLoadN: 0,
    longitudinalVelocityMps: 0,
    lateralVelocityMps: 0,
    slipAngleRad: 0,
    slipRatio: 0,
    angularVelocityRadPerSec: 0,
    visualAngularVelocity: 0,
    longitudinalForceN: 0,
    lateralForceN: 0,
    rawLongitudinalForceN: 0,
    rawLateralForceN: 0,
    combinedForceScale: 0,
    combinedForceUtilization: 0,
    frictionLimitN: 0,
    surfaceFriction: 1,
    tireModel: 'brush',
    longitudinalStiffnessPerLoad: 12,
    corneringStiffnessPerLoad: 10,
    longitudinalShape: 1.5,
    lateralShape: 1.3,
    longitudinalGripRatio: 1,
    lateralGripRatio: 1,
    canBrake: true,
    brakeTorqueNm: 0,
    maxBrakeTorqueNm: 3200,
    driveTorqueNm: 0,
    rollingResistanceCoefficient: 0,
    angularDampingNmPerRadPerSec: 0,
    wheelInertiaKgM2: 1.2,
    transform: {
        position: new Vector3(),
        rotationQuaternion: new Quaternion(),
        computeWorldMatrix() {},
        getWorldMatrix() {
            return Matrix.Identity()
        }
    }
})

test('anti-roll bar pushes more-compressed side up and pulls less-compressed side down', () => {
    // Wheel 0 (left, x=-1) hitDistance=0.2 => compression=0.4 (more compressed / low side)
    // Wheel 1 (right, x=+1) hitDistance=0.5 => compression=0.1 (less compressed / high side)
    const { body, appliedForces } = createMockBody({ hitDistanceByWheel: [0.2, 0.5] })
    const vehicle = new RaycastVehicle(body, {})
    vehicle.predictiveLookAheadSeconds = 0

    const leftWheel = createMockWheel({ x: -1 })
    const rightWheel = createMockWheel({ x: 1 })
    vehicle.addWheel(leftWheel)
    vehicle.addWheel(rightWheel)
    vehicle.addAntiRollAxle({ wheelA: 0, wheelB: 1, force: 1000 })

    vehicle.update(1 / 60)

    // springRate=0, damperRate=0 => suspensionForce=0, so the 2 anti-roll forces are the last applyForce calls per wheel
    const antiRollLeft = appliedForces.filter((f) => Math.abs(f.point.x - -1) < 1e-6).at(-1)
    const antiRollRight = appliedForces.filter((f) => Math.abs(f.point.x - 1) < 1e-6).at(-1)

    assert.ok(antiRollLeft, 'left wheel should receive anti-roll force')
    assert.ok(antiRollRight, 'right wheel should receive anti-roll force')
    assert.ok(antiRollLeft.force.y > 0, `more-compressed wheel should be pushed up (+Y), got ${antiRollLeft.force.y}`)
    assert.ok(antiRollRight.force.y < 0, `less-compressed wheel should be pulled down (-Y), got ${antiRollRight.force.y}`)
    assert.ok(Math.abs(antiRollLeft.force.y + antiRollRight.force.y) < 1e-6, 'anti-roll force pair should balance to zero')
})

test('raycastToRef query excludes vehicle chassis filterMembershipMask', () => {
    const { body, raycastQueries } = createMockBody({ filterMembershipMask: 2, hitDistanceByWheel: [0.4] })
    const vehicle = new RaycastVehicle(body, {})
    vehicle.predictiveLookAheadSeconds = 0
    vehicle.addWheel(createMockWheel({ x: 0 }))

    vehicle.update(1 / 60)

    assert.equal(raycastQueries.length, 1)
    assert.ok(raycastQueries[0], 'raycastToRef should receive a query object')
    assert.equal(raycastQueries[0].collideWith & 2, 0, 'chassis mask bit (2) must be excluded from collideWith')
    assert.equal(raycastQueries[0].collideWith & 1, 1, 'world mask bit (1) must remain included in collideWith')
})

test('cylinder shapeCast maps hitFraction to hitDistance, ignores chassis body, and applies bump stop', () => {
    const shapeCastQueries = []
    const mockShape = { id: 'mock-cylinder' }
    const { body } = createMockBody({ filterMembershipMask: 2 })
    body._physicsEngine.getPhysicsPlugin = () => ({
        shapeCast(query, _inputResult, hitResult) {
            shapeCastQueries.push({
                shape: query.shape,
                ignoreBody: query.ignoreBody,
                shouldHitTriggers: query.shouldHitTriggers,
                startY: query.startPosition.y,
                endY: query.endPosition.y
            })
            hitResult.reset()
            // hitFraction = 0.1 -> castTravel = 0.6 - 0.2 = 0.4 -> centerOffset = 0.04
            // hitDistance = 0.04 + 0.2 = 0.24 -> compression = 0.6 - 0.24 = 0.36
            // bumpStopStart = 0.4 * 0.85 = 0.34 -> excess = 0.02
            hitResult.setHitData({ x: 0, y: 1, z: 0 }, { x: 0, y: 0.76, z: 0 })
            hitResult.setHitFraction(0.1)
        }
    })

    const vehicle = new RaycastVehicle(body, {})
    vehicle.predictiveLookAheadSeconds = 0

    const wheel = createMockWheel({ x: 0, suspensionRestLength: 0.6 })
    wheel.radius = 0.2
    wheel.springRate = 10000
    wheel.damperRate = 0
    wheel.bumpStopStartRatio = 0.85
    wheel.bumpStopRate = 100000
    wheel.shapeCastInputResult = new ShapeCastResult()
    wheel.shapeCastHitResult = new ShapeCastResult()
    wheel.getOrCreateCollisionShape = () => mockShape
    vehicle.addWheel(wheel)

    vehicle.update(1 / 60)

    assert.equal(shapeCastQueries.length, 1)
    assert.equal(shapeCastQueries[0].shape, mockShape)
    assert.equal(shapeCastQueries[0].ignoreBody, body)
    assert.equal(shapeCastQueries[0].shouldHitTriggers, false)
    assert.ok(Math.abs(shapeCastQueries[0].startY - 0) < 1e-6)
    assert.ok(Math.abs(shapeCastQueries[0].endY - -0.4) < 1e-6)
    assert.ok(Math.abs(wheel.hitDistance - 0.24) < 1e-6)
    assert.ok(Math.abs(wheel.suspensionCompressionMeters - 0.36) < 1e-6)
    // Main spring: 10000 * 0.36 = 3600 N; bump stop: 100000 * (0.36 - 0.34) = 2000 N -> total 5600 N
    assert.ok(Math.abs(wheel.normalLoadN - 5600) < 1e-4, `expected 5600N with bump stop, got ${wheel.normalLoadN}`)
    // Wheel visual center should be 0.04m below mount (y = 0 - 0.04 = -0.04)
    assert.ok(Math.abs(wheel.transform.position.y - -0.04) < 1e-6)
})
