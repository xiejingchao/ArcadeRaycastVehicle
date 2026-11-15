import { clampNumber, moveTowards } from '../utils/utils.js'

const RAD_PER_SEC_TO_RPM = 60 / (2 * Math.PI)

export const DEFAULT_DRIVETRAIN_CONFIG = {
    idleRpm: 900,
    maxRpm: 7000,
    peakTorqueRpm: 4500,
    maxEngineTorqueNm: 280,
    gearRatios: [3.4, 2.2, 1.55, 1.15, 0.88],
    reverseGearRatio: -3.2,
    finalDriveRatio: 3.6,
    upshiftRpm: 6200,
    downshiftRpm: 2400,
    shiftCooldownSeconds: 0.28,
    centerFrontSplit: 0.4,
    centerLockFactor: 0.45,
    axleLockFactor: 0.55,
    lsdSlipScaleRadPerSec: 8,
    absSlipThreshold: 0.16,
    absMinSpeedMps: 1.5,
    absPulseHz: 15,
    tcsSlipThreshold: 0.18,
    tcsMinTorqueScale: 0.15
}

export const evaluateEngineTorqueNm = (rpm, config = DEFAULT_DRIVETRAIN_CONFIG) => {
    const clampedRpm = clampNumber(rpm, config.idleRpm, config.maxRpm)
    const normalized = (clampedRpm - config.idleRpm) / Math.max(config.maxRpm - config.idleRpm, 1)
    const peakNormalized = (config.peakTorqueRpm - config.idleRpm) / Math.max(config.maxRpm - config.idleRpm, 1)

    let curveFactor
    if (normalized <= peakNormalized) {
        const rise = normalized / Math.max(peakNormalized, 1e-6)
        curveFactor = 0.62 + 0.38 * Math.sin(rise * Math.PI * 0.5)
    } else {
        const fall = (normalized - peakNormalized) / Math.max(1 - peakNormalized, 1e-6)
        curveFactor = 1 - 0.28 * fall * fall
    }

    return config.maxEngineTorqueNm * clampNumber(curveFactor, 0.5, 1)
}

export const computeLsdSplit = (
    totalTorqueNm,
    omegaA,
    omegaB,
    baseShareA = 0.5,
    lockFactor = 0.5,
    slipScaleRadPerSec = 8
) => {
    if (totalTorqueNm === 0 || !Number.isFinite(totalTorqueNm)) {
        return [0, 0]
    }

    const baseA = totalTorqueNm * clampNumber(baseShareA, 0, 1)
    const sign = Math.sign(totalTorqueNm)
    // Transfer torque away from the faster-spinning side toward the slower/grippier side.
    const transfer =
        Math.tanh((omegaA - omegaB) / Math.max(slipScaleRadPerSec, 1e-3)) *
        clampNumber(lockFactor, 0, 1) *
        Math.abs(totalTorqueNm) *
        0.5

    let torqueA = baseA - sign * transfer
    if (sign > 0) {
        torqueA = clampNumber(torqueA, 0, totalTorqueNm)
    } else {
        torqueA = clampNumber(torqueA, totalTorqueNm, 0)
    }
    const torqueB = totalTorqueNm - torqueA
    return [torqueA, torqueB]
}

export class DrivetrainController {
    constructor(config = {}) {
        this.config = { ...DEFAULT_DRIVETRAIN_CONFIG, ...config }
        this.gear = 1
        this.rpm = this.config.idleRpm
        this.absEnabled = true
        this.tcsEnabled = true
        this.absActive = false
        this.tcsActive = false
        this.shiftCooldownRemaining = 0
        this.pulseTimeSeconds = 0
        this.throttleOpenRatio = 0
        this.brakeOpenRatio = 0
    }

    getTotalGearRatio(gear = this.gear) {
        if (gear === -1) {
            return this.config.reverseGearRatio * this.config.finalDriveRatio
        }
        const index = clampNumber(gear - 1, 0, this.config.gearRatios.length - 1)
        return this.config.gearRatios[index] * this.config.finalDriveRatio
    }

    step({ dtSeconds, vehicleSpeedMps, longitudinalInput, wheels, directionChangeSpeedMps = 1.2 }) {
        const cfg = this.config
        this.pulseTimeSeconds += dtSeconds
        this.shiftCooldownRemaining = Math.max(0, this.shiftCooldownRemaining - dtSeconds)

        const speedAbs = Math.abs(vehicleSpeedMps)
        const isBraking =
            longitudinalInput !== 0 &&
            speedAbs > directionChangeSpeedMps &&
            Math.sign(vehicleSpeedMps) !== Math.sign(longitudinalInput)

        const throttleInput = isBraking ? 0 : Math.abs(longitudinalInput)
        const brakeInput = isBraking ? Math.abs(longitudinalInput) : 0
        this.throttleOpenRatio = throttleInput
        this.brakeOpenRatio = brakeInput

        const previousGear = this.gear

        if (!isBraking && longitudinalInput < 0 && speedAbs <= directionChangeSpeedMps) {
            this.gear = -1
        } else if (!isBraking && longitudinalInput >= 0 && this.gear === -1 && vehicleSpeedMps >= -directionChangeSpeedMps) {
            this.gear = 1
        }

        const drivenWheels = wheels.filter((w) => w.isDriven)
        const avgDrivenOmega =
            drivenWheels.length > 0
                ? drivenWheels.reduce((sum, w) => sum + w.angularVelocityRadPerSec, 0) / drivenWheels.length
                : 0

        const activeRatio = this.getTotalGearRatio(this.gear)
        const wheelDrivenRpm = Math.abs(avgDrivenOmega * activeRatio) * RAD_PER_SEC_TO_RPM
        const launchRpmFloor = cfg.idleRpm + throttleInput * 1700
        const targetRpm = clampNumber(Math.max(wheelDrivenRpm, launchRpmFloor), cfg.idleRpm, cfg.maxRpm)
        this.rpm = moveTowards(this.rpm, targetRpm, 9500 * dtSeconds)

        if (this.gear > 0 && this.shiftCooldownRemaining <= 0) {
            if (
                throttleInput > 0.1 &&
                this.rpm >= cfg.upshiftRpm &&
                this.gear < cfg.gearRatios.length
            ) {
                this.gear += 1
                this.shiftCooldownRemaining = cfg.shiftCooldownSeconds
            } else if (this.rpm <= cfg.downshiftRpm && this.gear > 1) {
                this.gear -= 1
                this.shiftCooldownRemaining = cfg.shiftCooldownSeconds
            }
        }

        const shiftOccurred = this.gear !== previousGear
        const engineTorqueNm = throttleInput * evaluateEngineTorqueNm(this.rpm, cfg)
        const totalWheelDriveTorqueNm = engineTorqueNm * this.getTotalGearRatio(this.gear)

        // Wheel layout: [0]=RL, [1]=RR, [2]=FL, [3]=FR
        const rearWheels = [wheels[0], wheels[1]].filter(Boolean)
        const frontWheels = [wheels[2], wheels[3]].filter(Boolean)

        const avgFrontOmega =
            frontWheels.length > 0
                ? frontWheels.reduce((s, w) => s + w.angularVelocityRadPerSec, 0) / frontWheels.length
                : 0
        const avgRearOmega =
            rearWheels.length > 0
                ? rearWheels.reduce((s, w) => s + w.angularVelocityRadPerSec, 0) / rearWheels.length
                : 0

        const [frontAxleTorqueNm, rearAxleTorqueNm] = computeLsdSplit(
            totalWheelDriveTorqueNm,
            avgFrontOmega,
            avgRearOmega,
            cfg.centerFrontSplit,
            cfg.centerLockFactor,
            cfg.lsdSlipScaleRadPerSec
        )

        const [rearLeftTorqueNm, rearRightTorqueNm] =
            rearWheels.length === 2
                ? computeLsdSplit(
                      rearAxleTorqueNm,
                      rearWheels[0].angularVelocityRadPerSec,
                      rearWheels[1].angularVelocityRadPerSec,
                      0.5,
                      cfg.axleLockFactor,
                      cfg.lsdSlipScaleRadPerSec
                  )
                : [rearAxleTorqueNm, 0]

        const [frontLeftTorqueNm, frontRightTorqueNm] =
            frontWheels.length === 2
                ? computeLsdSplit(
                      frontAxleTorqueNm,
                      frontWheels[0].angularVelocityRadPerSec,
                      frontWheels[1].angularVelocityRadPerSec,
                      0.5,
                      cfg.axleLockFactor,
                      cfg.lsdSlipScaleRadPerSec
                  )
                : [frontAxleTorqueNm, 0]

        const rawDriveByWheel = [rearLeftTorqueNm, rearRightTorqueNm, frontLeftTorqueNm, frontRightTorqueNm]

        let tcsActive = false
        let absActive = false
        const absPulseFactor = 0.45 + 0.25 * Math.sin(this.pulseTimeSeconds * cfg.absPulseHz * Math.PI * 2)

        wheels.forEach((wheel, index) => {
            let driveTorqueNm = wheel.isDriven ? (rawDriveByWheel[index] ?? 0) : 0
            if (this.tcsEnabled && wheel.isDriven && throttleInput > 0 && wheel.inContact) {
                const excessSlip = Math.abs(wheel.slipRatio) - cfg.tcsSlipThreshold
                if (excessSlip > 0) {
                    const tcsScale = clampNumber(1 - excessSlip * 2.5, cfg.tcsMinTorqueScale, 1)
                    driveTorqueNm *= tcsScale
                    tcsActive = true
                }
            }

            let brakeTorqueNm = wheel.canBrake ? brakeInput * wheel.maxBrakeTorqueNm : 0
            if (
                this.absEnabled &&
                wheel.canBrake &&
                brakeInput > 0 &&
                wheel.inContact &&
                speedAbs > cfg.absMinSpeedMps &&
                Math.abs(wheel.slipRatio) > cfg.absSlipThreshold
            ) {
                brakeTorqueNm *= absPulseFactor
                absActive = true
            }

            wheel.driveTorqueNm = driveTorqueNm
            wheel.brakeTorqueNm = brakeTorqueNm
        })

        this.tcsActive = tcsActive
        this.absActive = absActive

        return {
            gear: this.gear,
            rpm: this.rpm,
            throttleOpenRatio: this.throttleOpenRatio,
            brakeOpenRatio: this.brakeOpenRatio,
            absActive: this.absActive,
            tcsActive: this.tcsActive,
            shiftOccurred
        }
    }
}
