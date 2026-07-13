import { Vector3 } from '@babylonjs/core/Maths/math.vector.js'

const getBodyVelocityAtPoint = (body, point) => {
    const r = point.subtract(body.transformNode.position)
    const angularVelocity = body.getAngularVelocity()
    const res = Vector3.Cross(angularVelocity, r)
    const velocity = body.getLinearVelocity()
    res.addInPlace(velocity)
    return res;
}

const clampNumber = (num, a, b) => Math.max(Math.min(num, Math.max(a, b)), Math.min(a, b));
const lerp = (x, y, a) => x * (1 - a) + y * a;
const moveTowards = (current, target, maxDelta) => {
    if (maxDelta < 0) {
        throw new Error(`maxDelta must be >= 0. Received ${maxDelta}`)
    }

    const delta = target - current
    if (Math.abs(delta) <= maxDelta) {
        return target
    }

    return current + Math.sign(delta) * maxDelta
}

const assertPositiveNumber = (value, name) => {
    if (!Number.isFinite(value) || value <= 0) {
        throw new Error(`${name} must be > 0. Received ${value}`)
    }
}

const assertNonNegativeNumber = (value, name) => {
    if (!Number.isFinite(value) || value < 0) {
        throw new Error(`${name} must be >= 0. Received ${value}`)
    }
}

const computeSuspensionForce = (springRate, compressionMeters, damperRate, compressionVelocityMps) => {
    const springForce = springRate * compressionMeters
    const damperForce = damperRate * compressionVelocityMps

    return {
        springForce,
        damperForce,
        suspensionForce: Math.max(0, springForce + damperForce)
    }
}

export {
    getBodyVelocityAtPoint,
    clampNumber,
    lerp,
    moveTowards,
    assertPositiveNumber,
    assertNonNegativeNumber,
    computeSuspensionForce
}