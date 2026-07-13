import test from 'node:test'
import assert from 'node:assert/strict'
import { computeBrushTireForces } from '../src/physics/tireModel.js'

test('returns zero force with zero normal load', () => {
    const result = computeBrushTireForces({
        normalLoadN: 0,
        slipAngleRad: 0.2,
        slipRatio: 0.3
    })

    assert.equal(result.longitudinalForceN, 0)
    assert.equal(result.lateralForceN, 0)
    assert.equal(result.frictionLimitN, 0)
})

test('returns near-zero force at zero slip inputs', () => {
    const result = computeBrushTireForces({
        normalLoadN: 3000,
        slipAngleRad: 0,
        slipRatio: 0
    })

    assert.ok(Math.abs(result.longitudinalForceN) < 1e-9)
    assert.ok(Math.abs(result.lateralForceN) < 1e-9)
})

test('longitudinal and lateral force signs follow slip direction conventions', () => {
    const positiveSlip = computeBrushTireForces({
        normalLoadN: 3000,
        slipAngleRad: 0.1,
        slipRatio: 0.2
    })
    const negativeSlip = computeBrushTireForces({
        normalLoadN: 3000,
        slipAngleRad: -0.1,
        slipRatio: -0.2
    })

    assert.ok(positiveSlip.longitudinalForceN > 0)
    assert.ok(positiveSlip.lateralForceN < 0)
    assert.ok(negativeSlip.longitudinalForceN < 0)
    assert.ok(negativeSlip.lateralForceN > 0)
})

test('combined slip limit does not exceed friction ellipse', () => {
    const result = computeBrushTireForces({
        normalLoadN: 4000,
        slipAngleRad: 0.4,
        slipRatio: 1.2,
        surfaceFriction: 1,
        longitudinalGripRatio: 0.9,
        lateralGripRatio: 1.1
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
