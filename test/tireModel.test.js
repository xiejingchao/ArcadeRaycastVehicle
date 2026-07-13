import test from 'node:test'
import assert from 'node:assert/strict'
import {
    DEFAULT_TIRE_MODEL,
    DEFAULT_TIRE_PRESET,
    TIRE_MODEL_PRESETS,
    computeBrushTireForces,
    computeTireForces
} from '../src/physics/tireModel.js'

const POST_PEAK_MAX_VARIATION_RATIO = 0.18
const MAX_LONGITUDINAL_DISCONTINUITY_N = 1300
const MAX_LATERAL_DISCONTINUITY_N = 500

const basePacejkaInput = {
    model: DEFAULT_TIRE_MODEL,
    normalLoadN: 3000,
    surfaceFriction: TIRE_MODEL_PRESETS[DEFAULT_TIRE_PRESET].surfaceFriction,
    longitudinalGripRatio: TIRE_MODEL_PRESETS[DEFAULT_TIRE_PRESET].longitudinalGripRatio,
    lateralGripRatio: TIRE_MODEL_PRESETS[DEFAULT_TIRE_PRESET].lateralGripRatio,
    pacejkaLongitudinal: TIRE_MODEL_PRESETS[DEFAULT_TIRE_PRESET].pacejkaLongitudinal,
    pacejkaLateral: TIRE_MODEL_PRESETS[DEFAULT_TIRE_PRESET].pacejkaLateral
}

test('returns zero force with zero normal load or zero friction', () => {
    const zeroLoad = computeTireForces({
        ...basePacejkaInput,
        normalLoadN: 0,
        slipAngleRad: 0.2,
        slipRatio: 0.3
    })
    const zeroFriction = computeTireForces({
        ...basePacejkaInput,
        surfaceFriction: 0,
        slipAngleRad: 0.2,
        slipRatio: 0.3
    })

    ;[zeroLoad, zeroFriction].forEach((result) => {
        assert.equal(result.longitudinalForceN, 0)
        assert.equal(result.lateralForceN, 0)
        assert.equal(result.frictionLimitN, 0)
    })
})

test('returns near-zero force at zero slip inputs', () => {
    const result = computeTireForces({
        ...basePacejkaInput,
        slipAngleRad: 0,
        slipRatio: 0
    })

    assert.ok(Math.abs(result.longitudinalForceN) < 1e-9)
    assert.ok(Math.abs(result.lateralForceN) < 1e-9)
})

test('brush model remains available as an explicit fallback', () => {
    const result = computeBrushTireForces({
        normalLoadN: 3000,
        surfaceFriction: 1,
        slipAngleRad: 0.1,
        slipRatio: 0.15
    })

    assert.ok(result.longitudinalForceN > 0)
    assert.ok(result.lateralForceN < 0)
})

test('longitudinal and lateral force signs follow slip direction conventions', () => {
    const positiveSlip = computeTireForces({
        ...basePacejkaInput,
        slipAngleRad: 0.1,
        slipRatio: 0.2
    })
    const negativeSlip = computeTireForces({
        ...basePacejkaInput,
        slipAngleRad: -0.1,
        slipRatio: -0.2
    })

    assert.ok(positiveSlip.longitudinalForceN > 0)
    assert.ok(positiveSlip.lateralForceN < 0)
    assert.ok(negativeSlip.longitudinalForceN < 0)
    assert.ok(negativeSlip.lateralForceN > 0)
})

test('balanced pacejka curve builds progressively from zero toward the peak', () => {
    const forceAtSmallSlip = Math.abs(
        computeTireForces({
            ...basePacejkaInput,
            slipAngleRad: 0,
            slipRatio: 0.02
        }).rawLongitudinalForceN
    )
    const forceAtMediumSlip = Math.abs(
        computeTireForces({
            ...basePacejkaInput,
            slipAngleRad: 0,
            slipRatio: 0.06
        }).rawLongitudinalForceN
    )
    const forceNearPeak = Math.abs(
        computeTireForces({
            ...basePacejkaInput,
            slipAngleRad: 0,
            slipRatio: 0.12
        }).rawLongitudinalForceN
    )

    assert.ok(forceAtSmallSlip > 0)
    assert.ok(forceAtMediumSlip > forceAtSmallSlip)
    assert.ok(forceNearPeak > forceAtMediumSlip)
    assert.ok(forceNearPeak < basePacejkaInput.normalLoadN * basePacejkaInput.surfaceFriction * 1.05)
})

test('post-peak retention avoids a large drop after the balanced preset peak region', () => {
    const samples = [0.1, 0.12, 0.16, 0.22, 0.32].map((slipRatio) =>
        Math.abs(
            computeTireForces({
                ...basePacejkaInput,
                slipAngleRad: 0,
                slipRatio
            }).rawLongitudinalForceN
        )
    )
    const peakForce = Math.max(...samples)
    const postPeakMinimum = Math.min(...samples.slice(2))

    assert.ok(postPeakMinimum >= peakForce * 0.8)
    assert.ok(Math.abs(samples[2] - samples[3]) <= peakForce * POST_PEAK_MAX_VARIATION_RATIO)
    assert.ok(Math.abs(samples[3] - samples[4]) <= peakForce * POST_PEAK_MAX_VARIATION_RATIO)
})

test('all presets keep a usable post-peak plateau without a force cliff', () => {
    Object.entries(TIRE_MODEL_PRESETS).forEach(([presetName, preset]) => {
        const samples = [0.12, 0.18, 0.26, 0.36].map((slipRatio) =>
            Math.abs(
                computeTireForces({
                    model: DEFAULT_TIRE_MODEL,
                    normalLoadN: 3200,
                    surfaceFriction: preset.surfaceFriction,
                    longitudinalGripRatio: preset.longitudinalGripRatio,
                    lateralGripRatio: preset.lateralGripRatio,
                    pacejkaLongitudinal: preset.pacejkaLongitudinal,
                    pacejkaLateral: preset.pacejkaLateral,
                    slipAngleRad: 0,
                    slipRatio
                }).rawLongitudinalForceN
            )
        )
        const peakForce = Math.max(...samples)
        const postPeakMinimum = Math.min(...samples.slice(1))

        assert.ok(postPeakMinimum >= peakForce * 0.75, presetName)
    })
})

test('pacejka outputs remain finite and continuous over common slip ranges', () => {
    let previousLongitudinal = null
    let previousLateral = null

    for (let index = -40; index <= 40; index++) {
        const slipRatio = index * 0.025
        const slipAngleRad = index * 0.01
        const result = computeTireForces({
            ...basePacejkaInput,
            slipAngleRad,
            slipRatio
        })

        assert.ok(Number.isFinite(result.rawLongitudinalForceN))
        assert.ok(Number.isFinite(result.rawLateralForceN))
        assert.ok(Number.isFinite(result.longitudinalForceN))
        assert.ok(Number.isFinite(result.lateralForceN))

        if (previousLongitudinal !== null) {
            assert.ok(Math.abs(result.rawLongitudinalForceN - previousLongitudinal) < MAX_LONGITUDINAL_DISCONTINUITY_N)
            assert.ok(Math.abs(result.rawLateralForceN - previousLateral) < MAX_LATERAL_DISCONTINUITY_N)
        }

        previousLongitudinal = result.rawLongitudinalForceN
        previousLateral = result.rawLateralForceN
    }
})

test('combined slip limit does not exceed friction ellipse', () => {
    const result = computeTireForces({
        ...basePacejkaInput,
        normalLoadN: 4000,
        surfaceFriction: 1,
        longitudinalGripRatio: 0.9,
        lateralGripRatio: 1.1,
        slipAngleRad: 0.4,
        slipRatio: 1.2
    })

    const normalized = Math.sqrt(
        (result.longitudinalForceN / (result.frictionLimitN * 0.9)) ** 2 +
            (result.lateralForceN / (result.frictionLimitN * 1.1)) ** 2
    )
    assert.ok(Number.isFinite(normalized))
    assert.ok(normalized <= 1 + 1e-9)
    assert.ok(result.combinedForceScale > 0)
    assert.ok(result.combinedForceScale <= 1)
})

test('all tire presets produce finite usable forces', () => {
    Object.entries(TIRE_MODEL_PRESETS).forEach(([presetName, preset]) => {
        const result = computeTireForces({
            model: DEFAULT_TIRE_MODEL,
            normalLoadN: 3200,
            surfaceFriction: preset.surfaceFriction,
            longitudinalGripRatio: preset.longitudinalGripRatio,
            lateralGripRatio: preset.lateralGripRatio,
            pacejkaLongitudinal: preset.pacejkaLongitudinal,
            pacejkaLateral: preset.pacejkaLateral,
            slipAngleRad: 0.12,
            slipRatio: 0.18
        })

        assert.ok(Number.isFinite(result.rawLongitudinalForceN), presetName)
        assert.ok(Number.isFinite(result.rawLateralForceN), presetName)
        assert.ok(Number.isFinite(result.longitudinalForceN), presetName)
        assert.ok(Number.isFinite(result.lateralForceN), presetName)
        assert.ok(Math.abs(result.longitudinalForceN) > 100, presetName)
        assert.ok(Math.abs(result.lateralForceN) > 100, presetName)
    })
})
