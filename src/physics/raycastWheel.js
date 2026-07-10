import { Vector3, Quaternion } from '@babylonjs/core/Maths/math.vector.js'
import { TransformNode } from '@babylonjs/core/Meshes/transformNode.js'
import { assertNonNegativeNumber, assertPositiveNumber } from '../utils/utils.js'

class RaycastWheel{
    constructor(options){
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
		this.suspensionCompressionMeters = 0
		this.previousSuspensionCompressionMeters = 0
		this.hitDistance = 0
		this.hitNormal = new Vector3()
		this.hitPoint = new Vector3()
		this.inContact = false
		
        this.steering = 0
		this.rotation = 0
		this.visualAngularVelocity = 0
        this.force = 0

		assertPositiveNumber(this.radius, 'wheel.radius')
		assertPositiveNumber(this.suspensionRestLength, 'wheel.suspensionRestLength')
		assertPositiveNumber(this.springRate, 'wheel.springRate')
		assertNonNegativeNumber(this.damperRate, 'wheel.damperRate')

        this.transform = new TransformNode("WheelTransform")
        this.transform.rotationQuaternion = new Quaternion()
    }
}

export default RaycastWheel