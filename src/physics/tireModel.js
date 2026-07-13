import { clampNumber } from '../utils/utils.js'

const EPSILON = 1e-6
const DEFAULT_TIRE_MODEL = 'pacejkaForgiving'
const DEFAULT_TIRE_PRESET = 'balanced'

const TIRE_MODEL_PRESETS = {
    grip: {
        surfaceFriction: 1.08,
        longitudinalGripRatio: 1.02,
        lateralGripRatio: 1.08,
        pacejkaLongitudinal: {
            peakSlip: 0.11,
            stiffnessFactorB: 16,
            shapeFactorC: 1.3,
            curvatureFactorE: 0.75,
            peakGripRatio: 0.99,
            postPeakSlip: 0.12,
            postPeakTransition: 0.2,
            postPeakRetention: 0.9
        },
        pacejkaLateral: {
            peakSlip: 0.135,
            stiffnessFactorB: 13,
            shapeFactorC: 1.28,
            curvatureFactorE: 0.72,
            peakGripRatio: 1,
            postPeakSlip: 0.14,
            postPeakTransition: 0.24,
            postPeakRetention: 0.88
        }
    },
    balanced: {
        surfaceFriction: 1.05,
        longitudinalGripRatio: 1,
        lateralGripRatio: 1,
        pacejkaLongitudinal: {
            peakSlip: 0.12,
            stiffnessFactorB: 14,
            shapeFactorC: 1.26,
            curvatureFactorE: 0.7,
            peakGripRatio: 0.98,
            postPeakSlip: 0.13,
            postPeakTransition: 0.28,
            postPeakRetention: 0.93
        },
        pacejkaLateral: {
            peakSlip: 0.15,
            stiffnessFactorB: 11,
            shapeFactorC: 1.24,
            curvatureFactorE: 0.68,
            peakGripRatio: 0.98,
            postPeakSlip: 0.155,
            postPeakTransition: 0.28,
            postPeakRetention: 0.92
        }
    },
    driftFriendly: {
        surfaceFriction: 1.02,
        longitudinalGripRatio: 1,
        lateralGripRatio: 0.94,
        pacejkaLongitudinal: {
            peakSlip: 0.1,
            stiffnessFactorB: 12,
            shapeFactorC: 1.2,
            curvatureFactorE: 0.62,
            peakGripRatio: 0.96,
            postPeakSlip: 0.105,
            postPeakTransition: 0.32,
            postPeakRetention: 0.96
        },
        pacejkaLateral: {
            peakSlip: 0.12,
            stiffnessFactorB: 9,
            shapeFactorC: 1.18,
            curvatureFactorE: 0.58,
            peakGripRatio: 0.94,
            postPeakSlip: 0.125,
            postPeakTransition: 0.34,
            postPeakRetention: 0.97
        }
    }
}

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

const smoothStep01 = (value) => {
    const t = clampNumber(value, 0, 1)
    return t * t * (3 - 2 * t)
}

const computeMagicFormulaBase = (slipInput, stiffnessFactorB, shapeFactorC, curvatureFactorE, peakForceN) =>
    peakForceN *
    Math.sin(
        shapeFactorC *
            Math.atan(
                stiffnessFactorB * slipInput -
                    curvatureFactorE * (stiffnessFactorB * slipInput - Math.atan(stiffnessFactorB * slipInput))
            )
    )

const deriveMagicFormulaB = ({ peakSlip, shapeFactorC, curvatureFactorE, stiffnessFactorB }) => {
    // A preset can override B directly when a hand-tuned stiffness works better than the semantic estimate.
    if (Number.isFinite(stiffnessFactorB) && stiffnessFactorB > 0) {
        return stiffnessFactorB
    }

    const peakSlipMagnitude = Math.max(Math.abs(peakSlip), EPSILON)
    const curvatureScale = Math.max(0.25, 1 - curvatureFactorE)
    return Math.tan(Math.PI / (2 * Math.max(shapeFactorC, 1.01))) / (peakSlipMagnitude * curvatureScale)
}

const computeForgivingMagicFormulaForce = ({
    slipInput,
    frictionLimitN,
    peakSlip,
    shapeFactorC = 1.25,
    curvatureFactorE = 0.7,
    peakGripRatio = 1,
    stiffnessFactorB,
    postPeakSlip = peakSlip,
    postPeakTransition = Math.max(Math.abs(peakSlip), 0.1),
    postPeakRetention = 0.92
}) => {
    assertFiniteNumber(slipInput, 'tire.slipInput')
    assertPositiveFiniteNumber(frictionLimitN, 'tire.frictionLimitN')
    assertPositiveFiniteNumber(shapeFactorC, 'tire.shapeFactorC')
    assertFiniteNumber(curvatureFactorE, 'tire.curvatureFactorE')
    assertPositiveFiniteNumber(peakGripRatio, 'tire.peakGripRatio')
    assertPositiveFiniteNumber(Math.max(Math.abs(peakSlip), EPSILON), 'tire.peakSlip')
    assertPositiveFiniteNumber(Math.max(Math.abs(postPeakTransition), EPSILON), 'tire.postPeakTransition')
    assertPositiveFiniteNumber(postPeakRetention, 'tire.postPeakRetention')

    const signedSlip = Number.isFinite(slipInput) ? slipInput : 0
    const slipMagnitude = Math.abs(signedSlip)
    const effectivePeakForceN = frictionLimitN * peakGripRatio
    const effectiveStiffnessFactorB = deriveMagicFormulaB({
        peakSlip,
        shapeFactorC,
        curvatureFactorE,
        stiffnessFactorB
    })
    const rawForceN = computeMagicFormulaBase(
        signedSlip,
        effectiveStiffnessFactorB,
        shapeFactorC,
        curvatureFactorE,
        effectivePeakForceN
    )

    // The post-peak blend keeps the force on a broad, controllable plateau.
    // It starts after the chosen peak region and eases toward a retained force with a smoothstep
    // so the drift window widens without introducing a grip cliff or a force discontinuity.
    const postPeakStart = Math.max(Math.abs(postPeakSlip), Math.abs(peakSlip))
    if (slipMagnitude <= postPeakStart) {
        return rawForceN
    }

    const transitionMagnitude = Math.max(Math.abs(postPeakTransition), EPSILON)
    const peakForceAtStartN = Math.abs(
        computeMagicFormulaBase(
            (Math.sign(signedSlip) || 1) * postPeakStart,
            effectiveStiffnessFactorB,
            shapeFactorC,
            curvatureFactorE,
            effectivePeakForceN
        )
    )
    const retainedForceMagnitudeN = Math.min(peakForceAtStartN, effectivePeakForceN * postPeakRetention)
    const blend = smoothStep01((slipMagnitude - postPeakStart) / transitionMagnitude)
    const retainedForceN = Math.sign(signedSlip) * retainedForceMagnitudeN

    return rawForceN * (1 - blend) + retainedForceN * blend
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

const createZeroForceResult = () => ({
    rawLongitudinalForceN: 0,
    rawLateralForceN: 0,
    longitudinalForceN: 0,
    lateralForceN: 0,
    frictionLimitN: 0,
    combinedForceScale: 0,
    combinedForceUtilization: 0
})

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
        return createZeroForceResult()
    }

    // Brush fallback: force grows linearly with load-sensitive stiffness, then saturates smoothly.
    // Raw Fx/Fy are still sent through the same combined-slip ellipse as the Pacejka path.
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

const computePacejkaTireForces = ({
    normalLoadN,
    slipAngleRad,
    slipRatio,
    surfaceFriction = 1,
    longitudinalGripRatio = 1,
    lateralGripRatio = 1,
    pacejkaLongitudinal = {},
    pacejkaLateral = {}
}) => {
    assertFiniteNumber(normalLoadN, 'tire.normalLoadN')
    assertFiniteNumber(slipAngleRad, 'tire.slipAngleRad')
    assertFiniteNumber(slipRatio, 'tire.slipRatio')
    assertFiniteNumber(surfaceFriction, 'tire.surfaceFriction')
    assertPositiveFiniteNumber(longitudinalGripRatio, 'tire.longitudinalGripRatio')
    assertPositiveFiniteNumber(lateralGripRatio, 'tire.lateralGripRatio')

    if (normalLoadN <= 0 || surfaceFriction <= 0) {
        return createZeroForceResult()
    }

    const clampedSlipRatio = clampNumber(slipRatio, -3, 3)
    const clampedSlipAngleRad = clampNumber(slipAngleRad, -Math.PI / 2, Math.PI / 2)

    // D scales with available grip. Here D = mu * Fz * peakGripRatio, so load and surface friction
    // move the whole curve while B/C/E and the post-peak controls shape how quickly it builds and how
    // forgiving it remains beyond the peak.
    const frictionLimitN = surfaceFriction * normalLoadN
    const rawLongitudinalForceN = computeForgivingMagicFormulaForce({
        slipInput: clampedSlipRatio,
        frictionLimitN,
        ...pacejkaLongitudinal
    })
    const rawLateralForceN = computeForgivingMagicFormulaForce({
        // Positive slip angle means the patch velocity points to wheel-right, so the tire force must
        // point left to resist that motion.
        slipInput: -clampedSlipAngleRad,
        frictionLimitN,
        ...pacejkaLateral
    })

    // Raw Fx/Fy are the single-axis curve outputs. The ellipse is the only combined-slip limiter so the
    // final force stays inside mu * Fz without reintroducing per-axis hard clamps or discontinuities.
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

const computeTireForces = ({ model = DEFAULT_TIRE_MODEL, ...options }) => {
    if (model === 'brush') {
        return computeBrushTireForces(options)
    }

    if (model === 'pacejka' || model === DEFAULT_TIRE_MODEL) {
        return computePacejkaTireForces(options)
    }

    throw new Error(`Unknown tire model "${model}"`)
}

export {
    DEFAULT_TIRE_MODEL,
    DEFAULT_TIRE_PRESET,
    TIRE_MODEL_PRESETS,
    computeBrushTireForces,
    computePacejkaTireForces,
    computeTireForces,
    applyFrictionEllipseLimit
}
