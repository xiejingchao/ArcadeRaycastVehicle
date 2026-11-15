import test from 'node:test'
import assert from 'node:assert/strict'
import {
    DEFAULT_DRIVETRAIN_CONFIG,
    DrivetrainController,
    computeLsdSplit,
    evaluateEngineTorqueNm
} from '../src/physics/drivetrain.js'

const makeWheels = () =>
    [0, 1, 2, 3].map(() => ({
        isDriven: true,
        canBrake: true,
        maxBrakeTorqueNm: 3200,
        angularVelocityRadPerSec: 0,
        slipRatio: 0,
        inContact: true,
        driveTorqueNm: 0,
        brakeTorqueNm: 0
    }))

test('evaluateEngineTorqueNm peaks around peakTorqueRpm and stays within bounds', () => {
    const idleTorque = evaluateEngineTorqueNm(DEFAULT_DRIVETRAIN_CONFIG.idleRpm)
    const peakTorque = evaluateEngineTorqueNm(DEFAULT_DRIVETRAIN_CONFIG.peakTorqueRpm)
    const redlineTorque = evaluateEngineTorqueNm(DEFAULT_DRIVETRAIN_CONFIG.maxRpm)

    assert.ok(peakTorque > idleTorque)
    assert.ok(peakTorque >= redlineTorque)
    assert.ok(Math.abs(peakTorque - DEFAULT_DRIVETRAIN_CONFIG.maxEngineTorqueNm) < 1e-6)
})

test('computeLsdSplit transfers torque toward slower-spinning side while conserving total torque', () => {
    const [torqueA, torqueB] = computeLsdSplit(1000, 30, 5, 0.5, 0.6, 8)
    assert.ok(Math.abs(torqueA + torqueB - 1000) < 1e-6)
    assert.ok(torqueB > torqueA, `slower side should receive more torque: A=${torqueA}, B=${torqueB}`)
    assert.ok(torqueA >= 0 && torqueB >= 0)
})

test('DrivetrainController upshifts, limits wheelspin with TCS, and modulates brakes with ABS', () => {
    const controller = new DrivetrainController()
    assert.equal(controller.absEnabled, true)
    assert.equal(controller.tcsEnabled, true)

    const wheels = makeWheels()
    // High wheel omega => high RPM => upshift from gear 1 to 2
    wheels.forEach((w) => {
        w.angularVelocityRadPerSec = 58
        w.slipRatio = 0.45
    })
    let accelState
    for (let i = 0; i < 8; i++) {
        accelState = controller.step({
            dtSeconds: 0.1,
            vehicleSpeedMps: 12,
            longitudinalInput: 1,
            wheels
        })
    }
    assert.equal(accelState.gear, 2)
    assert.equal(accelState.tcsActive, true)
    assert.ok(wheels[0].driveTorqueNm > 0)

    // Disabling TCS disables torque cut
    controller.tcsEnabled = false
    const accelNoTcs = controller.step({
        dtSeconds: 0.05,
        vehicleSpeedMps: 12,
        longitudinalInput: 1,
        wheels
    })
    assert.equal(accelNoTcs.tcsActive, false)

    // Braking at speed with high negative slip triggers ABS pulse modulation
    wheels.forEach((w) => {
        w.slipRatio = -0.35
    })
    const brakeState = controller.step({
        dtSeconds: 0.05,
        vehicleSpeedMps: 15,
        longitudinalInput: -1,
        wheels
    })
    assert.equal(brakeState.absActive, true)
    assert.ok(wheels[0].brakeTorqueNm < wheels[0].maxBrakeTorqueNm)

    // Disabling ABS applies full brake torque
    controller.absEnabled = false
    const brakeNoAbs = controller.step({
        dtSeconds: 0.05,
        vehicleSpeedMps: 15,
        longitudinalInput: -1,
        wheels
    })
    assert.equal(brakeNoAbs.absActive, false)
    assert.equal(wheels[0].brakeTorqueNm, wheels[0].maxBrakeTorqueNm)
})
