import { Vector3, Quaternion } from '@babylonjs/core/Maths/math.vector.js'
import { Axis } from '@babylonjs/core/Maths/math.axis.js'
import { PhysicsRaycastResult } from '@babylonjs/core/Physics/physicsRaycastResult.js'
import {
    getBodyVelocityAtPoint,
    clampNumber,
    assertPositiveNumber,
    computeSuspensionForce
} from '../utils/utils.js'
import { computeTireForces } from './tireModel.js'
import { DEFAULT_AERODYNAMICS_CONFIG, computeAerodynamicForces } from './aerodynamics.js'

const tmp1 = new Vector3()
const tmp2 = new Vector3()
const tmp3 = new Vector3()
const tmp4 = new Vector3()
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

        this.minSlipVelocityDenominatorMps = 1
        this.maxSlipRatioMagnitude = 3
        this.maxSlipAngleMagnitudeRad = Math.PI / 2
        this.angularVelocityStopThresholdRadPerSec = 0.5
        this.raycastCollideWithMask = undefined
        this._raycastQuery = { collideWith: ~0 }
        this.aerodynamicsConfig = { ...DEFAULT_AERODYNAMICS_CONFIG }
        this.aeroTelemetry = {
            dragForceN: 0,
            totalDownforceN: 0,
            frontDownforceN: 0,
            rearDownforceN: 0
        }
        this._shapeCastQuery = {
            shape: null,
            rotation: new Quaternion(),
            startPosition: new Vector3(),
            endPosition: new Vector3(),
            shouldHitTriggers: false,
            ignoreBody: this.body
        }
    }

    getRaycastQuery() {
        if (this.raycastCollideWithMask !== undefined) {
            this._raycastQuery.collideWith = this.raycastCollideWithMask
            return this._raycastQuery
        }

        const chassisMembershipMask = this.body?.shape?.filterMembershipMask
        if (typeof chassisMembershipMask === 'number' && chassisMembershipMask !== 0 && chassisMembershipMask !== ~0) {
            this._raycastQuery.collideWith = ~chassisMembershipMask
            return this._raycastQuery
        }

        return undefined
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

    resetWheelTireState(wheel) {
        wheel.normalLoadN = 0
        wheel.longitudinalVelocityMps = 0
        wheel.lateralVelocityMps = 0
        wheel.slipAngleRad = 0
        wheel.slipRatio = 0
        wheel.longitudinalForceN = 0
        wheel.lateralForceN = 0
        wheel.rawLongitudinalForceN = 0
        wheel.rawLateralForceN = 0
        wheel.combinedForceScale = 0
        wheel.combinedForceUtilization = 0
        wheel.frictionLimitN = 0
    }

    updateWheelRaycast(wheel) {
        const physicsPlugin = this.physicsEngine.getPhysicsPlugin?.()
        const wheelShape = physicsPlugin?.shapeCast ? wheel.getOrCreateCollisionShape?.(this.scene) : null

        if (wheelShape) {
            const castTravel = Math.max(wheel.suspensionRestLength - wheel.radius, 0)
            const query = this._shapeCastQuery
            query.shape = wheelShape
            query.rotation.copyFrom(wheel.transform.rotationQuaternion)
            query.startPosition.copyFrom(wheel.positionWorld)
            query.endPosition
                .copyFrom(wheel.suspensionAxisWorld)
                .scaleInPlace(castTravel)
                .addInPlace(wheel.positionWorld)
            query.shouldHitTriggers = false
            query.ignoreBody = this.body

            physicsPlugin.shapeCast(query, wheel.shapeCastInputResult, wheel.shapeCastHitResult)

            const hitResult = wheel.shapeCastHitResult
            if (!hitResult.hasHit) {
                wheel.inContact = false
                wheel.hitDistance = wheel.suspensionRestLength
                wheel.hitPoint.copyFrom(wheel.positionWorld)
                wheel.hitNormal.copyFrom(wheel.suspensionAxisWorld).negateInPlace().normalize()
                return
            }

            wheel.hitPoint.copyFrom(hitResult.hitPoint)
            wheel.hitNormal.copyFrom(hitResult.hitNormal)
            wheel.hitDistance = hitResult.hitFraction * castTravel + wheel.radius
            wheel.inContact = true
            this.nWheelsOnGround++
            return
        }

        tmp1.copyFrom(wheel.suspensionAxisWorld).scaleInPlace(wheel.suspensionRestLength).addInPlace(wheel.positionWorld)
        this.physicsEngine.raycastToRef(wheel.positionWorld, tmp1, raycastResult, this.getRaycastQuery())

        if (!raycastResult.hasHit) {
            wheel.inContact = false
            wheel.hitDistance = wheel.suspensionRestLength
            wheel.hitPoint.copyFrom(wheel.positionWorld)
            wheel.hitNormal.copyFrom(wheel.suspensionAxisWorld).negateInPlace().normalize()
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
            wheel.normalLoadN = 0
            return
        }

        const maxCompressionMeters = Math.max(wheel.suspensionRestLength - wheel.radius, 0)
        const previousCompressionMeters = wheel.suspensionCompressionMeters
        const compressionMeters = clampNumber(
            wheel.suspensionRestLength - wheel.hitDistance,
            0,
            maxCompressionMeters
        )
        const compressionVelocityMps = (compressionMeters - previousCompressionMeters) / dtSeconds
        wheel.previousSuspensionCompressionMeters = previousCompressionMeters
        wheel.suspensionCompressionMeters = compressionMeters

        const { suspensionForce } = computeSuspensionForce(
            wheel.springRate,
            compressionMeters,
            wheel.damperRate,
            compressionVelocityMps
        )
        const bumpStopStartCompression = maxCompressionMeters * clampNumber(wheel.bumpStopStartRatio ?? 0.85, 0, 1)
        const bumpStopExcess = Math.max(0, compressionMeters - bumpStopStartCompression)
        const bumpStopForce = bumpStopExcess * (wheel.bumpStopRate ?? 0)
        const totalSuspensionForce = suspensionForce + bumpStopForce
        wheel.normalLoadN = totalSuspensionForce

        const suspensionForceVector = Vector3.TransformNormalToRef(
            wheel.suspensionAxisLocal.negateToRef(tmp1),
            this.body.transformNode.getWorldMatrix(),
            tmp1
        ).scaleInPlace(totalSuspensionForce)

        this.body.applyForce(suspensionForceVector, wheel.hitPoint)
    }

    computeWheelContactFrame(wheel) {
        const forwardRaw = Vector3.TransformNormalToRef(wheel.forwardAxisLocal, wheel.transform.getWorldMatrix(), tmp1)
        if (!wheel.inContact) {
            wheel.contactForwardWorld = forwardRaw.normalizeToRef(wheel.contactForwardWorld ?? new Vector3())
            wheel.contactLateralWorld = Vector3.TransformNormalToRef(
                wheel.axleAxisLocal,
                wheel.transform.getWorldMatrix(),
                wheel.contactLateralWorld ?? new Vector3()
            ).normalize()
            return
        }

        const contactNormal = wheel.hitNormal.normalizeToRef(tmp2)
        const lateralRaw = Vector3.TransformNormalToRef(wheel.axleAxisLocal, wheel.transform.getWorldMatrix(), tmp3)

        const lateralProjection = lateralRaw.subtractToRef(
            contactNormal.scaleToRef(Vector3.Dot(lateralRaw, contactNormal), tmp4),
            wheel.contactLateralWorld ?? new Vector3()
        )

        if (lateralProjection.lengthSquared() < 1e-8) {
            Vector3.CrossToRef(contactNormal, forwardRaw, lateralProjection)
        }

        lateralProjection.normalize()
        const forwardTangent = Vector3.CrossToRef(lateralProjection, contactNormal, wheel.contactForwardWorld ?? new Vector3())
        if (forwardTangent.lengthSquared() < 1e-8) {
            forwardRaw.subtractToRef(contactNormal.scaleToRef(Vector3.Dot(forwardRaw, contactNormal), tmp4), forwardTangent)
        }
        forwardTangent.normalize()

        if (Vector3.Dot(forwardTangent, forwardRaw) < 0) {
            forwardTangent.scaleInPlace(-1)
            lateralProjection.scaleInPlace(-1)
        }

        wheel.contactForwardWorld = forwardTangent
        wheel.contactLateralWorld = lateralProjection
    }

    updateWheelTireKinematics(wheel) {
        this.computeWheelContactFrame(wheel)

        const contactPoint = wheel.inContact ? wheel.hitPoint : wheel.positionWorld
        const tireWorldVel = getBodyVelocityAtPoint(this.body, contactPoint)

        wheel.longitudinalVelocityMps = Vector3.Dot(wheel.contactForwardWorld, tireWorldVel)
        wheel.lateralVelocityMps = Vector3.Dot(wheel.contactLateralWorld, tireWorldVel)

        const speedDenominator = Math.max(Math.abs(wheel.longitudinalVelocityMps), this.minSlipVelocityDenominatorMps)
        const wheelSurfaceSpeedMps = wheel.angularVelocityRadPerSec * wheel.radius

        // Sign convention:
        // - slipAngle > 0 when the contact patch velocity has a +lateral component
        // - slipRatio > 0 when wheel surface speed is faster than ground speed (driving slip)
        wheel.slipAngleRad = clampNumber(
            Math.atan2(wheel.lateralVelocityMps, speedDenominator),
            -this.maxSlipAngleMagnitudeRad,
            this.maxSlipAngleMagnitudeRad
        )
        wheel.slipRatio = clampNumber(
            (wheelSurfaceSpeedMps - wheel.longitudinalVelocityMps) / speedDenominator,
            -this.maxSlipRatioMagnitude,
            this.maxSlipRatioMagnitude
        )
    }

    updateWheelTireForces(wheel) {
        if (!wheel.inContact || wheel.normalLoadN <= 0) {
            wheel.longitudinalForceN = 0
            wheel.lateralForceN = 0
            wheel.rawLongitudinalForceN = 0
            wheel.rawLateralForceN = 0
            wheel.frictionLimitN = 0
            wheel.combinedForceScale = 0
            wheel.combinedForceUtilization = 0
            return
        }

        const tireForces = computeTireForces({
            model: wheel.tireModel,
            normalLoadN: wheel.normalLoadN,
            slipAngleRad: wheel.slipAngleRad,
            slipRatio: wheel.slipRatio,
            surfaceFriction: wheel.surfaceFriction,
            longitudinalStiffnessPerLoad: wheel.longitudinalStiffnessPerLoad,
            corneringStiffnessPerLoad: wheel.corneringStiffnessPerLoad,
            longitudinalShape: wheel.longitudinalShape,
            lateralShape: wheel.lateralShape,
            longitudinalGripRatio: wheel.longitudinalGripRatio,
            lateralGripRatio: wheel.lateralGripRatio,
            pacejkaLongitudinal: wheel.pacejkaLongitudinal,
            pacejkaLateral: wheel.pacejkaLateral
        })

        wheel.rawLongitudinalForceN = tireForces.rawLongitudinalForceN
        wheel.rawLateralForceN = tireForces.rawLateralForceN
        wheel.longitudinalForceN = tireForces.longitudinalForceN
        wheel.lateralForceN = tireForces.lateralForceN
        wheel.frictionLimitN = tireForces.frictionLimitN
        wheel.combinedForceScale = tireForces.combinedForceScale
        wheel.combinedForceUtilization = tireForces.combinedForceUtilization

        const tireForceWorld = wheel.contactForwardWorld
            .scaleToRef(wheel.longitudinalForceN, tmp1)
            .addInPlace(wheel.contactLateralWorld.scaleToRef(wheel.lateralForceN, tmp2))

        this.body.applyForce(tireForceWorld, wheel.hitPoint)
    }

    updateWheelAngularDynamics(wheel, dtSeconds) {
        const angularDirectionSource =
            Math.abs(wheel.angularVelocityRadPerSec) > this.angularVelocityStopThresholdRadPerSec
                ? wheel.angularVelocityRadPerSec
                : wheel.longitudinalVelocityMps / wheel.radius
        const spinDirection = Math.sign(angularDirectionSource)

        const brakeTorqueNm = wheel.canBrake ? Math.min(Math.abs(wheel.brakeTorqueNm), wheel.maxBrakeTorqueNm) : 0
        const brakeTorqueOpposingSpinNm = spinDirection === 0 ? 0 : brakeTorqueNm * spinDirection
        const rollingResistanceTorqueNm =
            wheel.inContact && spinDirection !== 0
                ? wheel.rollingResistanceCoefficient * wheel.normalLoadN * wheel.radius * spinDirection
                : 0
        // The contact patch force generates an equal-and-opposite torque on the wheel.
        // Positive Fx drives the chassis forward, so it subtracts from the wheel's spin torque.
        const groundReactionTorqueNm = wheel.inContact ? wheel.longitudinalForceN * wheel.radius : 0
        const angularDampingTorqueNm = wheel.angularDampingNmPerRadPerSec * wheel.angularVelocityRadPerSec

        const netTorqueNm =
            wheel.driveTorqueNm -
            brakeTorqueOpposingSpinNm -
            groundReactionTorqueNm -
            rollingResistanceTorqueNm -
            angularDampingTorqueNm

        const angularAccelerationRadPerSec2 = netTorqueNm / wheel.wheelInertiaKgM2
        let nextAngularVelocityRadPerSec = wheel.angularVelocityRadPerSec + angularAccelerationRadPerSec2 * dtSeconds

        if (spinDirection !== 0 && Math.sign(nextAngularVelocityRadPerSec) !== spinDirection && Math.abs(brakeTorqueNm) > 0) {
            nextAngularVelocityRadPerSec = 0
        }

        wheel.angularVelocityRadPerSec = nextAngularVelocityRadPerSec
        wheel.visualAngularVelocity = wheel.angularVelocityRadPerSec
    }

    updateWheelRotation(wheel, dtSeconds) {
        wheel.rotation += wheel.angularVelocityRadPerSec * dtSeconds

        Quaternion.RotationAxisToRef(wheel.axleAxisLocal, wheel.rotation, tmpq1)
        wheel.transform.rotationQuaternion.multiplyToRef(tmpq1, wheel.transform.rotationQuaternion)
        wheel.transform.rotationQuaternion.normalize()
    }

    updateWheelTransformPosition(wheel) {
        const maxTravel = Math.max(wheel.suspensionRestLength - wheel.radius, 0)
        const wheelOffset = clampNumber(wheel.hitDistance - wheel.radius, 0, maxTravel)
        wheel.transform.position.copyFrom(wheel.positionWorld)
        wheel.transform.position.addInPlace(wheel.suspensionAxisWorld.scale(wheelOffset))
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

        this.physicsEngine.raycastToRef(position, predictedPosition, raycastResult, this.getRaycastQuery())

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

    getWheelTelemetrySnapshot() {
        return this.wheels.map((wheel, index) => ({
            index,
            inContact: wheel.inContact,
            normalLoadN: wheel.normalLoadN,
            longitudinalVelocityMps: wheel.longitudinalVelocityMps,
            lateralVelocityMps: wheel.lateralVelocityMps,
            slipAngleRad: wheel.slipAngleRad,
            slipRatio: wheel.slipRatio,
            angularVelocityRadPerSec: wheel.angularVelocityRadPerSec,
            longitudinalForceN: wheel.longitudinalForceN,
            lateralForceN: wheel.lateralForceN,
            rawLongitudinalForceN: wheel.rawLongitudinalForceN,
            rawLateralForceN: wheel.rawLateralForceN,
            combinedForceScale: wheel.combinedForceScale,
            combinedForceUtilization: wheel.combinedForceUtilization,
            frictionLimitN: wheel.frictionLimitN,
            surfaceFriction: wheel.surfaceFriction,
            tireModel: wheel.tireModel
        }))
    }

    setAerodynamicsConfig(config) {
        this.aerodynamicsConfig = { ...DEFAULT_AERODYNAMICS_CONFIG, ...config }
    }

    updateAerodynamics() {
        const aero = computeAerodynamicForces({
            forwardSpeedMps: this.speed,
            linearVelocityWorld: this.body.getLinearVelocity(),
            config: this.aerodynamicsConfig
        })

        this.aeroTelemetry.dragForceN = aero.dragForceN
        this.aeroTelemetry.totalDownforceN = aero.totalDownforceN
        this.aeroTelemetry.frontDownforceN = aero.frontDownforceN
        this.aeroTelemetry.rearDownforceN = aero.rearDownforceN

        if (!this.aerodynamicsConfig.enabled) {
            return
        }

        const worldMatrix = this.body.transformNode.getWorldMatrix()
        if (aero.dragForceN > 0) {
            tmp1.set(aero.dragForceWorld.x, aero.dragForceWorld.y, aero.dragForceWorld.z)
            this.body.applyForce(tmp1, this.body.transformNode.position)
        }

        if (aero.totalDownforceN > 0) {
            const chassisDownWorld = Vector3.TransformNormalToRef(Axis.Y, worldMatrix, tmp1).negateInPlace().normalize()

            if (aero.frontDownforceN > 0) {
                const frontPoint = Vector3.TransformCoordinatesToRef(
                    tmp3.set(0, 0, this.aerodynamicsConfig.frontAxleLocalZ),
                    worldMatrix,
                    tmp2
                )
                this.body.applyForce(chassisDownWorld.scaleToRef(aero.frontDownforceN, tmp4), frontPoint)
            }

            if (aero.rearDownforceN > 0) {
                const rearPoint = Vector3.TransformCoordinatesToRef(
                    tmp3.set(0, 0, this.aerodynamicsConfig.rearAxleLocalZ),
                    worldMatrix,
                    tmp2
                )
                this.body.applyForce(chassisDownWorld.scaleToRef(aero.rearDownforceN, tmp4), rearPoint)
            }
        }
    }

    update(dtSeconds) {
        assertPositiveNumber(dtSeconds, 'vehicle.update(dtSeconds)')

        this.body.transformNode.computeWorldMatrix(true)
        this.nWheelsOnGround = 0
        this.updateVehicleSpeed()
        this.updateAerodynamics()

        this.wheels.forEach((wheel) => {
            this.updateWheelTransform(wheel)
            this.updateWheelSteering(wheel)
            this.updateWheelRaycast(wheel)
            this.updateWheelSuspension(wheel, dtSeconds)
            this.updateWheelTireKinematics(wheel)
            this.updateWheelTireForces(wheel)
            this.updateWheelAngularDynamics(wheel, dtSeconds)
            this.updateWheelTransformPosition(wheel)
            this.updateWheelRotation(wheel, dtSeconds)

            if (!wheel.inContact) {
                wheel.normalLoadN = 0
            }
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

            const antiRollForce = tmp1
                .copyFrom(wheelOrder[0].suspensionAxisWorld)
                .negateInPlace()
                .scaleInPlace(axle.force * compressionRatio)
            this.body.applyForce(antiRollForce, wheelOrder[0].positionWorld)

            antiRollForce.copyFrom(wheelOrder[1].suspensionAxisWorld).scaleInPlace(axle.force * compressionRatio)
            this.body.applyForce(antiRollForce, wheelOrder[1].positionWorld)
        })
    }
}

export default RaycastVehicle
