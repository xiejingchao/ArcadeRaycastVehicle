import { clampNumber, moveTowards } from '../utils/utils.js'

export const DEFAULT_INPUT_SMOOTHING_CONFIG = {
    throttleRiseSeconds: 0.35,
    throttleReleaseSeconds: 0.1,
    brakeRiseSeconds: 0.22,
    brakeReleaseSeconds: 0.08,
    directionChangeSpeedMps: 1.2,
    maxSteerAngleLowSpeedRad: 0.52,
    maxSteerAngleHighSpeedRad: 0.13,
    speedForMinSteerMps: 35,
    steerInitialRateRadPerSec: 1.1,
    steerMaxRateRadPerSec: 2.6,
    steerRampSeconds: 0.28,
    steerReturnRateRadPerSec: 4.2,
    counterSteerRateMultiplier: 2.2,
    slipStabilizeGain: 0.25,
    slipStabilizeMaxRad: 0.06,
    slipStabilizeMinSpeedMps: 5
}

export class KeyboardInputProcessor {
    constructor(config = {}) {
        this.config = { ...DEFAULT_INPUT_SMOOTHING_CONFIG, ...config }
        this.throttlePedal = 0
        this.brakePedal = 0
        this.steerAngleRad = 0
        this.steerHoldSeconds = 0
        this.lastSteerSign = 0
    }

    reset() {
        this.throttlePedal = 0
        this.brakePedal = 0
        this.steerAngleRad = 0
        this.steerHoldSeconds = 0
        this.lastSteerSign = 0
    }

    step({ dtSeconds, forwardPressed, backwardPressed, leftPressed, rightPressed, vehicleSpeedMps, averageSlipAngleRad = 0 }) {
        const cfg = this.config
        const speedAbs = Math.abs(vehicleSpeedMps)
        const rawLongitudinal = (forwardPressed ? 1 : 0) + (backwardPressed ? -1 : 0)

        const isBraking =
            rawLongitudinal !== 0 &&
            speedAbs > cfg.directionChangeSpeedMps &&
            Math.sign(vehicleSpeedMps) !== Math.sign(rawLongitudinal)

        const targetThrottle = !isBraking && rawLongitudinal !== 0 ? 1 : 0
        const targetBrake = isBraking ? 1 : 0

        const throttleRate =
            targetThrottle > this.throttlePedal
                ? 1 / Math.max(cfg.throttleRiseSeconds, 1e-3)
                : 1 / Math.max(cfg.throttleReleaseSeconds, 1e-3)
        const brakeRate =
            targetBrake > this.brakePedal
                ? 1 / Math.max(cfg.brakeRiseSeconds, 1e-3)
                : 1 / Math.max(cfg.brakeReleaseSeconds, 1e-3)

        this.throttlePedal = moveTowards(this.throttlePedal, targetThrottle, throttleRate * dtSeconds)
        this.brakePedal = moveTowards(this.brakePedal, targetBrake, brakeRate * dtSeconds)

        let longitudinalInput = 0
        if (this.brakePedal > 0.001 && speedAbs > cfg.directionChangeSpeedMps) {
            longitudinalInput = -Math.sign(vehicleSpeedMps) * this.brakePedal
        } else if (this.throttlePedal > 0.001) {
            const driveDir = rawLongitudinal !== 0 ? Math.sign(rawLongitudinal) : vehicleSpeedMps >= -cfg.directionChangeSpeedMps ? 1 : -1
            longitudinalInput = driveDir * this.throttlePedal
        }

        const steerInput = (rightPressed ? 1 : 0) + (leftPressed ? -1 : 0)
        const steerSpeedRatio = clampNumber(speedAbs / Math.max(cfg.speedForMinSteerMps, 1e-3), 0, 1)
        const maxSteerBySpeed =
            cfg.maxSteerAngleLowSpeedRad +
            (cfg.maxSteerAngleHighSpeedRad - cfg.maxSteerAngleLowSpeedRad) * steerSpeedRatio

        if (steerInput !== 0 && steerInput === this.lastSteerSign) {
            this.steerHoldSeconds += dtSeconds
        } else {
            this.steerHoldSeconds = 0
        }
        this.lastSteerSign = steerInput

        let targetSteer = steerInput * maxSteerBySpeed
        if (steerInput === 0 && speedAbs >= cfg.slipStabilizeMinSpeedMps && Number.isFinite(averageSlipAngleRad)) {
            targetSteer = clampNumber(
                -averageSlipAngleRad * cfg.slipStabilizeGain,
                -cfg.slipStabilizeMaxRad,
                cfg.slipStabilizeMaxRad
            )
            targetSteer = clampNumber(targetSteer, -maxSteerBySpeed, maxSteerBySpeed)
        }

        const holdProgress = clampNumber(this.steerHoldSeconds / Math.max(cfg.steerRampSeconds, 1e-3), 0, 1)
        const activeSteerRate =
            cfg.steerInitialRateRadPerSec + (cfg.steerMaxRateRadPerSec - cfg.steerInitialRateRadPerSec) * holdProgress

        let steerRate = steerInput === 0 ? cfg.steerReturnRateRadPerSec : activeSteerRate
        if (steerInput !== 0 && this.steerAngleRad !== 0 && Math.sign(targetSteer) !== Math.sign(this.steerAngleRad)) {
            steerRate *= cfg.counterSteerRateMultiplier
        }

        this.steerAngleRad = moveTowards(this.steerAngleRad, targetSteer, steerRate * dtSeconds)
        this.steerAngleRad = clampNumber(this.steerAngleRad, -maxSteerBySpeed, maxSteerBySpeed)

        return {
            throttlePedal: this.throttlePedal,
            brakePedal: this.brakePedal,
            longitudinalInput,
            steerAngleRad: this.steerAngleRad,
            maxSteerBySpeed
        }
    }
}
