import { Vector3, Quaternion } from '@babylonjs/core/Maths/math.vector.js'
import { Axis } from '@babylonjs/core/Maths/math.axis.js'
import { PhysicsRaycastResult } from '@babylonjs/core/Physics/physicsRaycastResult.js'
import {
    getBodyVelocityAtPoint,
    clampNumber,
    assertPositiveNumber,
    computeSuspensionForce
} from '../utils/utils.js'

const tmp1 = new Vector3()
const tmp2 = new Vector3()
const tmp3 = new Vector3()
const tmpq1 = new Quaternion()
const upAxisLocal = new Vector3(0, 1, 0)
const rightAxisLocal = new Vector3(1, 0, 0)
const forwardAxisLocal = Vector3.Cross(upAxisLocal, rightAxisLocal)
forwardAxisLocal.normalize()
rightAxisLocal.normalize()

const raycastResult = new PhysicsRaycastResult()

class RaycastVehicle {
    constructor(body, scene) {
        this.body = body
        this.scene = scene
        this.physicsEngine = body._physicsEngine
        this.wheels = []
        this.predictiveLookAheadSeconds = 0.5
        this.predictionRatio = 0.6
        this.nWheelsOnGround = 0
        this.speed = 0
        this.antiRollAxles = []
    }

    addWheel(wheel) {
        this.wheels.push(wheel)
    }

    removeWheel(wheel, index) {
        if (index !== undefined) {
            this.wheels.splice(index, 1)
            return
        }

        this.wheels.splice(this.wheels.indexOf(wheel), 1)
    }

    addAntiRollAxle(axle) {
        this.antiRollAxles.push(axle)
    }

    removeAntiRollAxle(axle, index) {
        if (index !== undefined) {
            this.antiRollAxles.splice(index, 1)
            return
        }

        this.antiRollAxles.splice(this.antiRollAxles.indexOf(axle), 1)
    }

    updateWheelTransform(wheel) {
        Vector3.TransformCoordinatesToRef(wheel.positionLocal, this.body.transformNode.getWorldMatrix(), wheel.positionWorld)
        Vector3.TransformNormalToRef(wheel.suspensionAxisLocal, this.body.transformNode.getWorldMatrix(), wheel.suspensionAxisWorld)
    }

    updateVehicleSpeed() {
        Vector3.TransformNormalToRef(this.body.getLinearVelocity(), this.body.transformNode.getWorldMatrix().clone().invert(), tmp1)
        this.speed = tmp1.z
    }

    updateWheelSteering(wheel) {
        Quaternion.RotationAxisToRef(wheel.suspensionAxisLocal.negateToRef(tmp1), wheel.steering, tmpq1)
        this.body.transformNode.rotationQuaternion.multiplyToRef(tmpq1, wheel.transform.rotationQuaternion)
        wheel.transform.rotationQuaternion.normalize()
        wheel.transform.computeWorldMatrix(true)
    }

    updateWheelRaycast(wheel) {
        tmp1.copyFrom(wheel.suspensionAxisWorld).scaleInPlace(wheel.suspensionRestLength).addInPlace(wheel.positionWorld)
        this.physicsEngine.raycastToRef(wheel.positionWorld, tmp1, raycastResult)

        if (!raycastResult.hasHit) {
            wheel.inContact = false
            wheel.hitDistance = wheel.suspensionRestLength
            return
        }

        wheel.hitPoint.copyFrom(raycastResult.hitPointWorld)
        wheel.hitNormal.copyFrom(raycastResult.hitNormalWorld)
        wheel.hitDistance = raycastResult.hitDistance
        wheel.inContact = true
        this.nWheelsOnGround++
    }

    updateWheelSuspension(wheel, dtSeconds) {
        if (!wheel.inContact) {
            wheel.previousSuspensionCompressionMeters = 0
            wheel.suspensionCompressionMeters = 0
            return
        }

        const previousCompressionMeters = wheel.suspensionCompressionMeters
        const compressionMeters = clampNumber(
            wheel.suspensionRestLength - wheel.hitDistance,
            0,
            wheel.suspensionRestLength
        )
        const compressionVelocityMps = (compressionMeters - previousCompressionMeters) / dtSeconds
        wheel.previousSuspensionCompressionMeters = previousCompressionMeters
        wheel.suspensionCompressionMeters = compressionMeters

        // Linear spring-damper suspension in SI units:
        // springForce = k * x, damperForce = c * xDot, total normal force is never negative.
        const { suspensionForce } = computeSuspensionForce(
            wheel.springRate,
            compressionMeters,
            wheel.damperRate,
            compressionVelocityMps
        )

        const suspensionForceVector = Vector3.TransformNormalToRef(
            wheel.suspensionAxisLocal.negateToRef(tmp1),
            this.body.transformNode.getWorldMatrix(),
            tmp1
        ).scaleInPlace(suspensionForce)

        this.body.applyForce(suspensionForceVector, wheel.hitPoint)
    }

    updateWheelSideForce(wheel, dtSeconds) {
        if (!wheel.inContact) {
            return
        }

        const tireWorldVel = getBodyVelocityAtPoint(this.body, wheel.positionWorld)
        const steeringDir = Vector3.TransformNormalToRef(wheel.axleAxisLocal, wheel.transform.getWorldMatrix(), tmp1)
        const steeringVel = Vector3.Dot(steeringDir, tireWorldVel)
        const desiredVelChange = -steeringVel
        const desiredAccel = desiredVelChange / dtSeconds

        this.body.applyForce(
            // Temporary arcade lateral force gain, not a brush tire model.
            steeringDir.scaleInPlace(wheel.sideForce * desiredAccel),
            Vector3.LerpToRef(wheel.hitPoint, wheel.positionWorld, wheel.sideForcePositionRatio, tmp2)
        )
    }

    updateWheelForce(wheel) {
        if (!wheel.inContact || wheel.force === 0) {
            return
        }

        const forwardDirectionWorld = Vector3.TransformNormalToRef(
            wheel.forwardAxisLocal,
            wheel.transform.getWorldMatrix(),
            tmp1
        ).scaleInPlace(wheel.force)

        this.body.applyForce(forwardDirectionWorld, tmp2.copyFrom(wheel.hitPoint))
    }

    updateWheelRotation(wheel, dtSeconds) {
        const wheelPointVelocity = getBodyVelocityAtPoint(this.body, wheel.positionWorld)
        const forwardDirectionWorld = Vector3.TransformNormalToRef(
            wheel.forwardAxisLocal,
            wheel.transform.getWorldMatrix(),
            tmp1
        )

        // Temporary no-slip visual approximation until real wheel angular dynamics exist:
        // angularSpeed(rad/s) ~= linearSpeed(m/s) / radius(m)
        wheel.visualAngularVelocity = Vector3.Dot(forwardDirectionWorld, wheelPointVelocity) / wheel.radius
        wheel.rotation += wheel.visualAngularVelocity * dtSeconds

        Quaternion.RotationAxisToRef(wheel.axleAxisLocal, wheel.rotation, tmpq1)
        wheel.transform.rotationQuaternion.multiplyToRef(tmpq1, wheel.transform.rotationQuaternion)
        wheel.transform.rotationQuaternion.normalize()
    }

    updateWheelTransformPosition(wheel) {
        wheel.transform.position.copyFrom(wheel.positionWorld)
        wheel.transform.position.addInPlace(wheel.suspensionAxisWorld.scale(wheel.hitDistance - wheel.radius))
    }

    updateVehiclePredictiveLanding(dtSeconds) {
        if (this.predictiveLookAheadSeconds < 0) {
            throw new Error(`vehicle.predictiveLookAheadSeconds must be >= 0. Received ${this.predictiveLookAheadSeconds}`)
        }

        if (this.nWheelsOnGround > 0 || this.predictiveLookAheadSeconds === 0) {
            return
        }

        const position = this.body.transformNode.position
        const gravity = tmp1.copyFrom(this.physicsEngine.gravity).scaleInPlace(this.body.getGravityFactor())
        const predictTime = this.predictiveLookAheadSeconds
        const linearVelocity = this.body.getLinearVelocity()

        if (linearVelocity.lengthSquared() === 0) {
            return
        }

        const predictedPosition = tmp2.copyFrom(linearVelocity).scaleInPlace(predictTime)
        predictedPosition.addInPlace(gravity.scaleInPlace(0.5 * predictTime * predictTime))
        predictedPosition.addInPlace(position)

        this.physicsEngine.raycastToRef(position, predictedPosition, raycastResult)

        if (!raycastResult.hasHit) {
            return
        }

        const currentUp = Vector3.TransformNormalToRef(Axis.Y, this.body.transformNode.getWorldMatrix(), tmp1)
        const landingNormal = raycastResult.hitNormalWorld
        const rotationDifference = Vector3.CrossToRef(currentUp, landingNormal, tmp2)
        const predictedAngularVelocity = rotationDifference.scaleToRef(1 / predictTime, tmp3)

        this.body.setAngularVelocity(
            Vector3.LerpToRef(this.body.getAngularVelocity(), predictedAngularVelocity, this.predictionRatio, tmp1)
        )
    }

    update(dtSeconds) {
        assertPositiveNumber(dtSeconds, 'vehicle.update(dtSeconds)')

        this.body.transformNode.computeWorldMatrix(true)
        this.nWheelsOnGround = 0
        this.updateVehicleSpeed()

        this.wheels.forEach((wheel) => {
            this.updateWheelTransform(wheel)
            this.updateWheelSteering(wheel)
            this.updateWheelRaycast(wheel)
            this.updateWheelSuspension(wheel, dtSeconds)
            this.updateWheelForce(wheel)
            this.updateWheelSideForce(wheel, dtSeconds)
            this.updateWheelTransformPosition(wheel)
            this.updateWheelRotation(wheel, dtSeconds)
        })

        this.updateVehiclePredictiveLanding(dtSeconds)

        this.antiRollAxles.forEach((axle) => {
            const wheelA = this.wheels[axle.wheelA]
            const wheelB = this.wheels[axle.wheelB]
            if (!wheelA || !wheelB) {
                return
            }
            if (!wheelA.inContact && !wheelB.inContact) {
                return
            }

            const wheelOrder =
                wheelA.suspensionCompressionMeters >= wheelB.suspensionCompressionMeters ? [wheelA, wheelB] : [wheelB, wheelA]
            const maxCompressionMeters = (wheelA.suspensionRestLength + wheelB.suspensionRestLength) / 2
            const compressionDifference =
                wheelOrder[0].suspensionCompressionMeters - wheelOrder[1].suspensionCompressionMeters
            const compressionRatio = Math.min(compressionDifference, maxCompressionMeters) / maxCompressionMeters

            const antiRollForce = tmp1.copyFrom(wheelOrder[0].suspensionAxisWorld).scaleInPlace(axle.force * compressionRatio)
            this.body.applyForce(antiRollForce, wheelOrder[0].positionWorld)

            antiRollForce.copyFrom(wheelOrder[1].suspensionAxisWorld).negateInPlace().scaleInPlace(axle.force * compressionRatio)
            this.body.applyForce(antiRollForce, wheelOrder[1].positionWorld)
        })
    }
}

export default RaycastVehicle
