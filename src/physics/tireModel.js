import { clampNumber } from '../utils/utils.js'

const EPSILON = 1e-6

const assertFiniteNumber = (value, name) => {
    if (!Number.isFinite(value)) {
        throw new Error(`${name} must be finite. Received ${value}`)
    }
}

const assertPositiveFiniteNumber = (value, name) => {
    assertFiniteNumber(value, name)
    if (value <= 0) {
        throw new Error(`${name} must be > 0. Received ${value}`)
    }
}

const applyFrictionEllipseLimit = ({
    fxN,
    fyN,
    frictionLimitN,
    longitudinalGripRatio = 1,
    lateralGripRatio = 1
}) => {
    assertFiniteNumber(fxN, 'tire.fxN')
    assertFiniteNumber(fyN, 'tire.fyN')
    assertPositiveFiniteNumber(frictionLimitN, 'tire.frictionLimitN')
    assertPositiveFiniteNumber(longitudinalGripRatio, 'tire.longitudinalGripRatio')
    assertPositiveFiniteNumber(lateralGripRatio, 'tire.lateralGripRatio')

    const fxLimit = Math.max(frictionLimitN * longitudinalGripRatio, EPSILON)
    const fyLimit = Math.max(frictionLimitN * lateralGripRatio, EPSILON)
    const utilization = Math.sqrt((fxN * fxN) / (fxLimit * fxLimit) + (fyN * fyN) / (fyLimit * fyLimit))
    const scale = utilization > 1 ? 1 / utilization : 1

    return {
        scale,
        utilization,
        longitudinalForceN: fxN * scale,
        lateralForceN: fyN * scale
    }
}

const computeBrushTireForces = ({
    normalLoadN,
    slipAngleRad,
    slipRatio,
    surfaceFriction = 1,
    longitudinalStiffnessPerLoad = 12,
    corneringStiffnessPerLoad = 10,
    longitudinalShape = 1.6,
    lateralShape = 1.6,
    longitudinalGripRatio = 1,
    lateralGripRatio = 1
}) => {
    assertFiniteNumber(normalLoadN, 'tire.normalLoadN')
    assertFiniteNumber(slipAngleRad, 'tire.slipAngleRad')
    assertFiniteNumber(slipRatio, 'tire.slipRatio')
    assertFiniteNumber(surfaceFriction, 'tire.surfaceFriction')
    assertFiniteNumber(longitudinalStiffnessPerLoad, 'tire.longitudinalStiffnessPerLoad')
    assertFiniteNumber(corneringStiffnessPerLoad, 'tire.corneringStiffnessPerLoad')
    assertPositiveFiniteNumber(longitudinalShape, 'tire.longitudinalShape')
    assertPositiveFiniteNumber(lateralShape, 'tire.lateralShape')
    assertPositiveFiniteNumber(longitudinalGripRatio, 'tire.longitudinalGripRatio')
    assertPositiveFiniteNumber(lateralGripRatio, 'tire.lateralGripRatio')

    if (normalLoadN <= 0 || surfaceFriction <= 0) {
        return {
            rawLongitudinalForceN: 0,
            rawLateralForceN: 0,
            longitudinalForceN: 0,
            lateralForceN: 0,
            frictionLimitN: 0,
            combinedForceScale: 0,
            combinedForceUtilization: 0
        }
    }

    // Load-sensitive simplified brush model approximation:
    // Kx ~= (longitudinalStiffnessPerLoad * Fz), Ky ~= (corneringStiffnessPerLoad * Fz).
    // Raw force is soft-saturated with tanh, then jointly limited by friction ellipse.
    const clampedSlipRatio = clampNumber(slipRatio, -3, 3)
    const clampedSlipAngleRad = clampNumber(slipAngleRad, -Math.PI / 2, Math.PI / 2)
    const frictionLimitN = surfaceFriction * normalLoadN
    const longitudinalStiffnessN = Math.max(0, longitudinalStiffnessPerLoad) * normalLoadN
    const lateralStiffnessN = Math.max(0, corneringStiffnessPerLoad) * normalLoadN

    const linearLongitudinalForceN = longitudinalStiffnessN * clampedSlipRatio
    const linearLateralForceN = -lateralStiffnessN * clampedSlipAngleRad
    const rawLongitudinalForceN =
        frictionLimitN *
        Math.tanh((linearLongitudinalForceN / Math.max(frictionLimitN, EPSILON)) * longitudinalShape)
    const rawLateralForceN =
        frictionLimitN * Math.tanh((linearLateralForceN / Math.max(frictionLimitN, EPSILON)) * lateralShape)

    const limited = applyFrictionEllipseLimit({
        fxN: rawLongitudinalForceN,
        fyN: rawLateralForceN,
        frictionLimitN,
        longitudinalGripRatio,
        lateralGripRatio
    })

    return {
        rawLongitudinalForceN,
        rawLateralForceN,
        longitudinalForceN: limited.longitudinalForceN,
        lateralForceN: limited.lateralForceN,
        frictionLimitN,
        combinedForceScale: limited.scale,
        combinedForceUtilization: limited.utilization
    }
}

export { computeBrushTireForces, applyFrictionEllipseLimit }
