import test from 'node:test'
import assert from 'node:assert/strict'
import { KeyboardInputProcessor } from '../src/physics/inputSmoothing.js'

test('throttle and brake pedals ramp smoothly instead of jumping to 100%', () => {
    const processor = new KeyboardInputProcessor()

    const tap = processor.step({
        dtSeconds: 0.1,
        forwardPressed: true,
        backwardPressed: false,
        leftPressed: false,
        rightPressed: false,
        vehicleSpeedMps: 0
    })
    assert.ok(tap.throttlePedal > 0.2 && tap.throttlePedal < 0.4, `expected partial throttle after 0.1s, got ${tap.throttlePedal}`)

    let full = tap
    for (let i = 0; i < 6; i++) {
        full = processor.step({
            dtSeconds: 0.05,
            forwardPressed: true,
            backwardPressed: false,
            leftPressed: false,
            rightPressed: false,
            vehicleSpeedMps: 10
        })
    }
    assert.equal(full.throttlePedal, 1)

    const brakeTap = processor.step({
        dtSeconds: 0.08,
        forwardPressed: false,
        backwardPressed: true,
        leftPressed: false,
        rightPressed: false,
        vehicleSpeedMps: 15
    })
    assert.ok(brakeTap.brakePedal > 0.25 && brakeTap.brakePedal < 0.5, `expected partial brake after 0.08s, got ${brakeTap.brakePedal}`)
    assert.ok(brakeTap.longitudinalInput < 0)
})

test('steering narrows at high speed, ramps progressively, and counter-steers faster', () => {
    const processor = new KeyboardInputProcessor()

    const initial = processor.step({
        dtSeconds: 0.05,
        forwardPressed: false,
        backwardPressed: false,
        leftPressed: false,
        rightPressed: true,
        vehicleSpeedMps: 0
    })
    const initialDelta = initial.steerAngleRad

    const held = processor.step({
        dtSeconds: 0.2,
        forwardPressed: false,
        backwardPressed: false,
        leftPressed: false,
        rightPressed: true,
        vehicleSpeedMps: 0
    })
    const heldAvgRate = (held.steerAngleRad - initialDelta) / 0.2
    const initialRate = initialDelta / 0.05
    assert.ok(heldAvgRate > initialRate, `expected progressive rate to exceed initial rate: ${heldAvgRate} vs ${initialRate}`)

    const counter = processor.step({
        dtSeconds: 0.05,
        forwardPressed: false,
        backwardPressed: false,
        leftPressed: true,
        rightPressed: false,
        vehicleSpeedMps: 0
    })
    const counterRate = Math.abs(counter.steerAngleRad - held.steerAngleRad) / 0.05
    assert.ok(counterRate > initialRate * 2, `expected fast counter-steer rate, got ${counterRate} vs ${initialRate}`)

    const highSpeed = processor.step({
        dtSeconds: 0.05,
        forwardPressed: true,
        backwardPressed: false,
        leftPressed: false,
        rightPressed: true,
        vehicleSpeedMps: 35
    })
    assert.ok(highSpeed.maxSteerBySpeed <= 0.1301)
})
