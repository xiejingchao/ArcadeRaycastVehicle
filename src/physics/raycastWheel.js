import { Vector3, Quaternion } from '@babylonjs/core/Maths/math.vector.js'
import { TransformNode } from '@babylonjs/core/Meshes/transformNode.js'
import { assertNonNegativeNumber, assertPositiveNumber } from '../utils/utils.js'

class RaycastWheel{
    constructor(options){
		if (options.springRate !== undefined && options.suspensionForce !== undefined) {
			throw new Error('Use either wheel.springRate or legacy wheel.suspensionForce, not both.')
		}
		if (options.damperRate !== undefined && options.suspensionDamping !== undefined) {
			throw new Error('Use either wheel.damperRate or legacy wheel.suspensionDamping, not both.')
		}

        this.positionLocal = options.positionLocal.clone()
        this.positionWorld = options.positionLocal.clone()
        this.suspensionAxisLocal = options.suspensionAxisLocal.clone()
		this.suspensionAxisWorld = this.suspensionAxisLocal.clone()
		this.axleAxisLocal = options.axleAxisLocal.clone()
		this.forwardAxisLocal = options.forwardAxisLocal.clone()
		this.sideForce = options.sideForce ?? 40
		this.sideForcePositionRatio = options.sideForcePositionRatio ?? 0.1
        this.radius = options.radius ?? 0.2
		this.suspensionRestLength = options.suspensionRestLength ?? 0.5
		this.springRate = options.springRate ?? options.suspensionForce ?? 15000
		this.damperRate = options.damperRate ?? ((options.suspensionDamping ?? 0.1) * this.springRate)
		this.surfaceFriction = options.surfaceFriction ?? 1
		this.longitudinalStiffnessPerLoad = options.longitudinalStiffnessPerLoad ?? 12
		this.corneringStiffnessPerLoad = options.corneringStiffnessPerLoad ?? 10
		this.longitudinalShape = options.longitudinalShape ?? 1.6
		this.lateralShape = options.lateralShape ?? 1.6
		this.longitudinalGripRatio = options.longitudinalGripRatio ?? 1
		this.lateralGripRatio = options.lateralGripRatio ?? 1
		this.wheelInertiaKgM2 = options.wheelInertiaKgM2 ?? 1.2
		this.maxBrakeTorqueNm = options.maxBrakeTorqueNm ?? 3200
		this.angularDampingNmPerRadPerSec = options.angularDampingNmPerRadPerSec ?? 0.3
		this.rollingResistanceCoefficient = options.rollingResistanceCoefficient ?? 0.015
		this.isDriven = options.isDriven ?? false
		this.isSteerable = options.isSteerable ?? false
		this.canBrake = options.canBrake ?? true
		this.suspensionCompressionMeters = 0
		this.previousSuspensionCompressionMeters = 0
		this.hitDistance = 0
		this.hitNormal = new Vector3()
		this.hitPoint = new Vector3()
		this.inContact = false
		this.normalLoadN = 0
		
        this.steering = 0
		this.rotation = 0
		this.visualAngularVelocity = 0
		this.angularVelocityRadPerSec = options.angularVelocityRadPerSec ?? 0
		this.driveTorqueNm = 0
		this.brakeTorqueNm = 0
        this.force = 0

		// Runtime tire telemetry:
		// - longitudinal axis: +forward along wheel.forwardAxisLocal
		// - lateral axis: +right along wheel.axleAxisLocal
		// - slip angle > 0 means contact patch velocity has +lateral component
		// - positive brakeTorqueNm value means brake input magnitude (opposing current spin direction)
		this.longitudinalVelocityMps = 0
		this.lateralVelocityMps = 0
		this.slipAngleRad = 0
		this.slipRatio = 0
		this.longitudinalForceN = 0
		this.lateralForceN = 0
		this.rawLongitudinalForceN = 0
		this.rawLateralForceN = 0
		this.combinedForceScale = 0
		this.combinedForceUtilization = 0
		this.frictionLimitN = 0

		assertPositiveNumber(this.radius, 'wheel.radius')
		assertPositiveNumber(this.suspensionRestLength, 'wheel.suspensionRestLength')
		assertPositiveNumber(this.springRate, 'wheel.springRate')
		assertNonNegativeNumber(this.damperRate, 'wheel.damperRate')
		assertPositiveNumber(this.surfaceFriction, 'wheel.surfaceFriction')
		assertNonNegativeNumber(this.longitudinalStiffnessPerLoad, 'wheel.longitudinalStiffnessPerLoad')
		assertNonNegativeNumber(this.corneringStiffnessPerLoad, 'wheel.corneringStiffnessPerLoad')
		assertPositiveNumber(this.longitudinalShape, 'wheel.longitudinalShape')
		assertPositiveNumber(this.lateralShape, 'wheel.lateralShape')
		assertPositiveNumber(this.longitudinalGripRatio, 'wheel.longitudinalGripRatio')
		assertPositiveNumber(this.lateralGripRatio, 'wheel.lateralGripRatio')
		assertPositiveNumber(this.wheelInertiaKgM2, 'wheel.wheelInertiaKgM2')
		assertNonNegativeNumber(this.maxBrakeTorqueNm, 'wheel.maxBrakeTorqueNm')
		assertNonNegativeNumber(this.angularDampingNmPerRadPerSec, 'wheel.angularDampingNmPerRadPerSec')
		assertNonNegativeNumber(this.rollingResistanceCoefficient, 'wheel.rollingResistanceCoefficient')

        this.transform = new TransformNode("WheelTransform")
        this.transform.rotationQuaternion = new Quaternion()
    }
}

export default RaycastWheel