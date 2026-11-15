import test from 'node:test'
import assert from 'node:assert/strict'
import { Vector3, Matrix, Quaternion } from '@babylonjs/core/Maths/math.vector.js'
import RaycastVehicle from '../src/physics/raycastVehicle.js'
import { DEFAULT_AERODYNAMICS_CONFIG, computeAerodynamicForces } from '../src/physics/aerodynamics.js'

test('computeAerodynamicForces computes quadratic drag opposite velocity and front/rear downforce', () => {
    const speedMps = 40
    const result = computeAerodynamicForces({
        forwardSpeedMps: speedMps,
        linearVelocityWorld: { x: 0, y: 0, z: speedMps },
        config: DEFAULT_AERODYNAMICS_CONFIG
    })

    const expectedDrag =
        0.5 *
        DEFAULT_AERODYNAMICS_CONFIG.airDensityKgPerM3 *
        DEFAULT_AERODYNAMICS_CONFIG.dragCoefficient *
        DEFAULT_AERODYNAMICS_CONFIG.frontalAreaM2 *
        speedMps *
        speedMps
    const expectedDownforce =
        0.5 *
        DEFAULT_AERODYNAMICS_CONFIG.airDensityKgPerM3 *
        DEFAULT_AERODYNAMICS_CONFIG.downforceCoefficient *
        DEFAULT_AERODYNAMICS_CONFIG.frontalAreaM2 *
        speedMps *
        speedMps

    assert.ok(Math.abs(result.dragForceN - expectedDrag) < 1e-6)
    assert.ok(Math.abs(result.dragForceWorld.z + expectedDrag) < 1e-6)
    assert.ok(Math.abs(result.totalDownforceN - expectedDownforce) < 1e-6)
    assert.ok(
        Math.abs(result.frontDownforceN - expectedDownforce * DEFAULT_AERODYNAMICS_CONFIG.frontDownforceShare) < 1e-6
    )
    assert.ok(Math.abs(result.frontDownforceN + result.rearDownforceN - result.totalDownforceN) < 1e-6)
})

test('computeAerodynamicForces avoids lift in reverse and guards non-finite inputs', () => {
    const reversing = computeAerodynamicForces({
        forwardSpeedMps: -20,
        linearVelocityWorld: { x: 0, y: 0, z: -20 }
    })
    assert.equal(reversing.totalDownforceN, 0)
    assert.ok(reversing.dragForceWorld.z > 0)

    const nonFinite = computeAerodynamicForces({
        forwardSpeedMps: Number.NaN,
        linearVelocityWorld: { x: Number.NaN, y: Number.POSITIVE_INFINITY, z: 0 }
    })
    assert.equal(nonFinite.dragForceN, 0)
    assert.equal(nonFinite.totalDownforceN, 0)
})

test('RaycastVehicle applies aerodynamic drag and front/rear downforce during update', () => {
    const appliedForces = []
    const identityMatrix = Matrix.Identity()
    const body = {
        _physicsEngine: {
            gravity: new Vector3(0, -9.81, 0),
            raycastToRef(_from, _to, result) {
                result.reset(_from, _to)
            }
        },
        transformNode: {
            position: new Vector3(0, 1, 0),
            rotationQuaternion: new Quaternion(),
            computeWorldMatrix() {},
            getWorldMatrix() {
                return identityMatrix
            }
        },
        getLinearVelocity() {
            return new Vector3(0, 0, 30)
        },
        getAngularVelocity() {
            return new Vector3(0, 0, 0)
        },
        setAngularVelocity() {},
        getGravityFactor() {
            return 1
        },
        applyForce(force, point) {
            appliedForces.push({ force: force.clone(), point: point.clone() })
        }
    }

    const vehicle = new RaycastVehicle(body, {})
    vehicle.predictiveLookAheadSeconds = 0
    vehicle.update(1 / 60)

    assert.ok(vehicle.aeroTelemetry.dragForceN > 0)
    assert.ok(vehicle.aeroTelemetry.totalDownforceN > 0)
    assert.equal(appliedForces.length, 3)
    assert.ok(appliedForces[0].force.z < 0, 'drag force should oppose +Z motion')
    assert.ok(appliedForces[1].force.y < 0 && appliedForces[1].point.z > 0, 'front downforce should act downward at front axle')
    assert.ok(appliedForces[2].force.y < 0 && appliedForces[2].point.z < 0, 'rear downforce should act downward at rear axle')
})
