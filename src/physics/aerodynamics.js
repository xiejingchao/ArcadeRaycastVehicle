import { clampNumber } from '../utils/utils.js'

export const DEFAULT_AERODYNAMICS_CONFIG = {
    enabled: true,
    airDensityKgPerM3: 1.225,
    dragCoefficient: 0.34,
    frontalAreaM2: 2.1,
    downforceCoefficient: 0.65,
    frontDownforceShare: 0.42,
    frontAxleLocalZ: 0.8,
    rearAxleLocalZ: -0.7,
    maxSpeedForAeroMps: 120
}

export const computeAerodynamicForces = ({ forwardSpeedMps = 0, linearVelocityWorld = { x: 0, y: 0, z: 0 }, config = {} } = {}) => {
    const cfg = { ...DEFAULT_AERODYNAMICS_CONFIG, ...config }
    if (!cfg.enabled) {
        return {
            dragForceWorld: { x: 0, y: 0, z: 0 },
            dragForceN: 0,
            totalDownforceN: 0,
            frontDownforceN: 0,
            rearDownforceN: 0
        }
    }

    const rho = Number.isFinite(cfg.airDensityKgPerM3) ? Math.max(0, cfg.airDensityKgPerM3) : 0
    const cd = Number.isFinite(cfg.dragCoefficient) ? Math.max(0, cfg.dragCoefficient) : 0
    const cl = Number.isFinite(cfg.downforceCoefficient) ? Math.max(0, cfg.downforceCoefficient) : 0
    const area = Number.isFinite(cfg.frontalAreaM2) ? Math.max(0, cfg.frontalAreaM2) : 0
    const maxSpeed = Number.isFinite(cfg.maxSpeedForAeroMps) ? Math.max(0, cfg.maxSpeedForAeroMps) : 120

    const vx = Number.isFinite(linearVelocityWorld?.x) ? linearVelocityWorld.x : 0
    const vy = Number.isFinite(linearVelocityWorld?.y) ? linearVelocityWorld.y : 0
    const vz = Number.isFinite(linearVelocityWorld?.z) ? linearVelocityWorld.z : 0
    const rawSpeed3d = Math.hypot(vx, vy, vz)

    let dragForceWorld = { x: 0, y: 0, z: 0 }
    let dragForceN = 0
    if (rawSpeed3d > 1e-6 && maxSpeed > 0) {
        const clampedSpeed3d = Math.min(rawSpeed3d, maxSpeed)
        const invRaw = 1 / rawSpeed3d
        dragForceN = 0.5 * rho * cd * area * clampedSpeed3d * clampedSpeed3d
        dragForceWorld = {
            x: -vx * invRaw * dragForceN,
            y: -vy * invRaw * dragForceN,
            z: -vz * invRaw * dragForceN
        }
    }

    const safeForwardSpeed = Number.isFinite(forwardSpeedMps) ? clampNumber(Math.max(0, forwardSpeedMps), 0, maxSpeed) : 0
    const totalDownforceN = 0.5 * rho * cl * area * safeForwardSpeed * safeForwardSpeed
    const frontShare = Number.isFinite(cfg.frontDownforceShare) ? clampNumber(cfg.frontDownforceShare, 0, 1) : 0.42
    const frontDownforceN = totalDownforceN * frontShare
    const rearDownforceN = totalDownforceN - frontDownforceN

    return {
        dragForceWorld,
        dragForceN,
        totalDownforceN,
        frontDownforceN,
        rearDownforceN
    }
}
