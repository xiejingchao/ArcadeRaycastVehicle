import { Vector3, Quaternion } from '@babylonjs/core/Maths/math.vector.js'
import { Axis, Space } from '@babylonjs/core/Maths/math.axis.js'
import { MeshBuilder } from '@babylonjs/core/Meshes/meshBuilder.js'
import { Engine } from '@babylonjs/core/Engines/engine.js'
import { Scene } from '@babylonjs/core/scene.js'
import '@babylonjs/core/Physics/physicsEngineComponent'

import { FreeCamera } from '@babylonjs/core/Cameras/freeCamera.js'
import { FollowCamera } from '@babylonjs/core/Cameras/followCamera.js'
import { HemisphericLight } from '@babylonjs/core/Lights/hemisphericLight.js'

import { KeyboardEventTypes } from '@babylonjs/core/Events/keyboardEvents.js'
import { HavokPlugin } from '@babylonjs/core/Physics/v2/Plugins/havokPlugin.js'
import { PhysicsShapeConvexHull, PhysicsShapeMesh } from '@babylonjs/core/Physics/v2/physicsShape.js'
import { PhysicsBody } from '@babylonjs/core/Physics/v2/physicsBody.js'

import '@babylonjs/core/Materials/Textures/Loaders/envTextureLoader.js'
import { CubeTexture } from '@babylonjs/core/Materials/Textures/cubeTexture.js'

import { PhysicsMotionType } from '@babylonjs/core/Physics/v2/IPhysicsEnginePlugin.js'

import { SceneLoader } from '@babylonjs/core/Loading/sceneLoader.js'
import '@babylonjs/loaders/glTF'

import HavokPhysics from '@babylonjs/havok'
import RaycastVehicle from './physics/raycastVehicle.js'
import RaycastWheel from './physics/raycastWheel.js'
import { DrivetrainController } from './physics/drivetrain.js'
import { KeyboardInputProcessor } from './physics/inputSmoothing.js'
import { clampNumber, moveTowards } from './utils/utils.js'
import {
    DEFAULT_TIRE_MODEL,
    DEFAULT_TIRE_PRESET,
    TIRE_MODEL_PRESETS,
    computeTireForces
} from './physics/tireModel.js'

import { Sound } from '@babylonjs/core/Audio/sound.js'
import '@babylonjs/core/Audio/audioEngine.js'
import '@babylonjs/core/Audio/audioSceneComponent.js'

import { Animation } from '@babylonjs/core/Animations/animation.js'

const tmp1 = new Vector3()
const tmp2 = new Vector3()
const RAD_TO_DEG = 180 / Math.PI

const createOverlayElement = (tagName, style) => {
    const element = document.createElement(tagName)
    Object.assign(element.style, style)
    document.body.appendChild(element)
    return element
}

const drawChartFrame = ({ context, x, y, width, height, title, xLabel, yLabel, limitLabel, forceLimitN, yMaxN }) => {
    context.strokeStyle = 'rgba(180, 245, 200, 0.3)'
    context.strokeRect(x, y, width, height)

    context.fillStyle = '#d8fdd8'
    context.fillText(title, x, y - 10)
    context.fillText(xLabel, x + width - 70, y + height + 18)
    context.save()
    context.translate(x - 24, y + height * 0.5)
    context.rotate(-Math.PI / 2)
    context.fillText(yLabel, 0, 0)
    context.restore()

    const zeroY = y + height * 0.5
    context.strokeStyle = 'rgba(180, 245, 200, 0.18)'
    context.beginPath()
    context.moveTo(x, zeroY)
    context.lineTo(x + width, zeroY)
    context.stroke()

    const drawReferenceLine = (forceN) => {
        const lineY = zeroY - (forceN / yMaxN) * (height * 0.5)
        context.beginPath()
        context.moveTo(x, lineY)
        context.lineTo(x + width, lineY)
        context.stroke()
    }

    context.strokeStyle = 'rgba(255, 197, 105, 0.24)'
    drawReferenceLine(forceLimitN)
    drawReferenceLine(-forceLimitN)
    context.fillStyle = '#ffc569'
    context.fillText(limitLabel, x + 8, y + 14)
}

const drawCurvePath = ({
    context,
    samples,
    chartX,
    chartY,
    chartWidth,
    chartHeight,
    xMin,
    xMax,
    yMaxN,
    color,
    forceKey
}) => {
    context.strokeStyle = color
    context.beginPath()
    samples.forEach((sample, index) => {
        const xRatio = (sample.input - xMin) / Math.max(xMax - xMin, 1e-6)
        const yRatio = sample[forceKey] / yMaxN
        const px = chartX + xRatio * chartWidth
        const py = chartY + chartHeight * 0.5 - yRatio * (chartHeight * 0.5)
        if (index === 0) {
            context.moveTo(px, py)
            return
        }
        context.lineTo(px, py)
    })
    context.stroke()
}

const drawCurveWorkPoint = ({
    context,
    chartX,
    chartY,
    chartWidth,
    chartHeight,
    xMin,
    xMax,
    yMaxN,
    inputValue,
    rawForceN,
    finalForceN,
    rawColor,
    finalColor
}) => {
    const xRatio = clampNumber((inputValue - xMin) / Math.max(xMax - xMin, 1e-6), 0, 1)
    const px = chartX + xRatio * chartWidth
    const rawY = chartY + chartHeight * 0.5 - clampNumber(rawForceN / yMaxN, -1.25, 1.25) * (chartHeight * 0.5)
    const finalY = chartY + chartHeight * 0.5 - clampNumber(finalForceN / yMaxN, -1.25, 1.25) * (chartHeight * 0.5)

    context.strokeStyle = 'rgba(255,255,255,0.18)'
    context.beginPath()
    context.moveTo(px, chartY)
    context.lineTo(px, chartY + chartHeight)
    context.stroke()

    context.fillStyle = rawColor
    context.beginPath()
    context.arc(px, rawY, 4, 0, Math.PI * 2)
    context.fill()

    context.fillStyle = finalColor
    context.beginPath()
    context.arc(px, finalY, 4, 0, Math.PI * 2)
    context.fill()
}

const buildPureAxisCurveSamples = ({
    tireModel,
    tireConfig,
    normalLoadN,
    surfaceFriction,
    longitudinalGripRatio,
    lateralGripRatio,
    slipRatioRange,
    slipAngleRangeDeg,
    sampleCount
}) => {
    const longitudinal = []
    const lateral = []

    for (let index = 0; index <= sampleCount; index++) {
        const ratio = index / sampleCount
        const slipRatio = slipRatioRange[0] + (slipRatioRange[1] - slipRatioRange[0]) * ratio
        const longitudinalResult = computeTireForces({
            model: tireModel,
            normalLoadN,
            slipAngleRad: 0,
            slipRatio,
            surfaceFriction,
            longitudinalGripRatio,
            lateralGripRatio,
            pacejkaLongitudinal: tireConfig.pacejkaLongitudinal,
            pacejkaLateral: tireConfig.pacejkaLateral,
            longitudinalStiffnessPerLoad: tireConfig.longitudinalStiffnessPerLoad,
            corneringStiffnessPerLoad: tireConfig.corneringStiffnessPerLoad,
            longitudinalShape: tireConfig.longitudinalShape,
            lateralShape: tireConfig.lateralShape
        })
        longitudinal.push({
            input: slipRatio,
            rawForceN: longitudinalResult.rawLongitudinalForceN
        })

        const slipAngleDeg = slipAngleRangeDeg[0] + (slipAngleRangeDeg[1] - slipAngleRangeDeg[0]) * ratio
        const lateralResult = computeTireForces({
            model: tireModel,
            normalLoadN,
            slipAngleRad: (slipAngleDeg / RAD_TO_DEG),
            slipRatio: 0,
            surfaceFriction,
            longitudinalGripRatio,
            lateralGripRatio,
            pacejkaLongitudinal: tireConfig.pacejkaLongitudinal,
            pacejkaLateral: tireConfig.pacejkaLateral,
            longitudinalStiffnessPerLoad: tireConfig.longitudinalStiffnessPerLoad,
            corneringStiffnessPerLoad: tireConfig.corneringStiffnessPerLoad,
            longitudinalShape: tireConfig.longitudinalShape,
            lateralShape: tireConfig.lateralShape
        })
        lateral.push({
            input: slipAngleDeg,
            rawForceN: lateralResult.rawLateralForceN
        })
    }

    return { longitudinal, lateral }
}

const drawTireCurveDebugger = ({ context, curveDebugger, curveSamples, wheelTelemetry }) => {
    const canvasWidth = context.canvas.width
    const canvasHeight = context.canvas.height
    context.clearRect(0, 0, canvasWidth, canvasHeight)
    context.fillStyle = 'rgba(0, 0, 0, 0.82)'
    context.fillRect(0, 0, canvasWidth, canvasHeight)
    context.font = '12px monospace'

    const chartWidth = 320
    const chartHeight = 130
    const chartGap = 36
    const chartX = 22
    const topY = 34
    const bottomY = topY + chartHeight + chartGap

    const longitudinalLimitN = curveDebugger.surfaceFriction * curveDebugger.normalLoadN * curveDebugger.longitudinalGripRatio
    const lateralLimitN = curveDebugger.surfaceFriction * curveDebugger.normalLoadN * curveDebugger.lateralGripRatio
    const longitudinalYMaxN = longitudinalLimitN * 1.15
    const lateralYMaxN = lateralLimitN * 1.15
    const debuggerHeader = [
        'tire curve debugger',
        `model=${curveDebugger.tireModel}`,
        `preset=${curveDebugger.presetName}`,
        `drawFz=${curveDebugger.normalLoadN.toFixed(0)}N`,
        `drawMu=${curveDebugger.surfaceFriction.toFixed(2)}`,
        `wheel=${curveDebugger.wheelIndex}`
    ].join(' ')

    context.fillStyle = '#d8fdd8'
    context.fillText(debuggerHeader, chartX, 16)
    context.fillText('cyan=pure-axis curve  orange=raw live point  green=final live point', chartX, canvasHeight - 12)

    drawChartFrame({
        context,
        x: chartX,
        y: topY,
        width: chartWidth,
        height: chartHeight,
        title: 'Fx vs slip ratio (alpha = 0)',
        xLabel: 'kappa',
        yLabel: 'Fx [N]',
        limitLabel: `±mu*Fz*gx = ${longitudinalLimitN.toFixed(0)}N`,
        forceLimitN: longitudinalLimitN,
        yMaxN: longitudinalYMaxN
    })
    drawCurvePath({
        context,
        samples: curveSamples.longitudinal,
        chartX,
        chartY: topY,
        chartWidth,
        chartHeight,
        xMin: curveDebugger.slipRatioRange[0],
        xMax: curveDebugger.slipRatioRange[1],
        yMaxN: longitudinalYMaxN,
        color: '#67d5ff',
        forceKey: 'rawForceN'
    })
    if (wheelTelemetry) {
        drawCurveWorkPoint({
            context,
            chartX,
            chartY: topY,
            chartWidth,
            chartHeight,
            xMin: curveDebugger.slipRatioRange[0],
            xMax: curveDebugger.slipRatioRange[1],
            yMaxN: longitudinalYMaxN,
            inputValue: wheelTelemetry.slipRatio,
            rawForceN: wheelTelemetry.rawLongitudinalForceN,
            finalForceN: wheelTelemetry.longitudinalForceN,
            rawColor: '#ffb14a',
            finalColor: '#72f0a7'
        })
    }

    drawChartFrame({
        context,
        x: chartX,
        y: bottomY,
        width: chartWidth,
        height: chartHeight,
        title: 'Fy vs slip angle (kappa = 0)',
        xLabel: 'alpha [deg]',
        yLabel: 'Fy [N]',
        limitLabel: `±mu*Fz*gy = ${lateralLimitN.toFixed(0)}N`,
        forceLimitN: lateralLimitN,
        yMaxN: lateralYMaxN
    })
    drawCurvePath({
        context,
        samples: curveSamples.lateral,
        chartX,
        chartY: bottomY,
        chartWidth,
        chartHeight,
        xMin: curveDebugger.slipAngleRangeDeg[0],
        xMax: curveDebugger.slipAngleRangeDeg[1],
        yMaxN: lateralYMaxN,
        color: '#67d5ff',
        forceKey: 'rawForceN'
    })
    if (wheelTelemetry) {
        drawCurveWorkPoint({
            context,
            chartX,
            chartY: bottomY,
            chartWidth,
            chartHeight,
            xMin: curveDebugger.slipAngleRangeDeg[0],
            xMax: curveDebugger.slipAngleRangeDeg[1],
            yMaxN: lateralYMaxN,
            inputValue: wheelTelemetry.slipAngleRad * RAD_TO_DEG,
            rawForceN: wheelTelemetry.rawLateralForceN,
            finalForceN: wheelTelemetry.lateralForceN,
            rawColor: '#ffb14a',
            finalColor: '#72f0a7'
        })
    }
}

const shapeAxisInput = (input, deadZone, exponent) => {
    const magnitude = Math.abs(input)
    if (magnitude <= deadZone) {
        return 0
    }

    const normalized = (magnitude - deadZone) / Math.max(1 - deadZone, 1e-6)
    return Math.sign(input) * Math.pow(clampNumber(normalized, 0, 1), exponent)
}

const init = async () => {
    const canvas = document.getElementById('renderCanvas')
    const simulationConfig = {
        fixedTimeStepSeconds: 1 / 120,
        maxPhysicsStepsPerFrame: 8
    }
    const telemetryConfig = {
        enabled: false,
        curveDebugger: {
            enabled: false,
            wheelIndex: 2,
            normalLoadN: 3000,
            surfaceFriction: null,
            slipRatioRange: [-0.35, 0.35],
            slipAngleRangeDeg: [-20, 20],
            sampleCount: 72
        }
    }
    const engine = new Engine(canvas, true, {
        deterministicLockstep: true,
        lockstepMaxSteps: simulationConfig.maxPhysicsStepsPerFrame,
        timeStep: simulationConfig.fixedTimeStepSeconds,
        audioEngine: true
    })
    const scene = new Scene(engine)
    Scene.MaxDeltaTime = simulationConfig.fixedTimeStepSeconds * simulationConfig.maxPhysicsStepsPerFrame * 1000

    const camera = new FreeCamera('camera1', new Vector3(0, 5, -10), scene)
    camera.setTarget(Vector3.Zero())
    camera.attachControl(canvas, true)
    const light = new HemisphericLight('light', new Vector3(0, 1, 0), scene)
    light.intensity = 0.7

    const hdrTexture = CubeTexture.CreateFromPrefilteredData('environmentSpecular.env', scene)
    scene.environmentTexture = hdrTexture

    const animations = await Animation.ParseFromFileAsync('Vehicle', 'curves/acceleration.json')
    const accelerationCurve = animations[0]

    const levelContainer = await SceneLoader.LoadAssetContainerAsync('./models/', 'ds01.gltf', scene)
    const levelFiles = levelContainer.instantiateModelsToScene((name) => name, true)

    const vehicleContainer = await SceneLoader.LoadAssetContainerAsync('./models/', 'vehicleNS.gltf', scene)
    const vehicleFiles = vehicleContainer.instantiateModelsToScene((name) => name, true)
    const vehicleMesh = vehicleFiles.rootNodes[0].getChildren()[0]
    vehicleMesh.scaling.set(1.2, 1.2, 1.2)
    const wheelMesh = vehicleFiles.rootNodes[0].getChildren()[1]
    // Raw Wheel mesh radius is ~0.165m and width is ~0.109m:
    // scale (1.4, 1.21, 1.21) matches physical radius 0.2m and width ~0.15m cleanly without arch clipping.
    wheelMesh.scaling.set(1.4, 1.21, 1.21)
    wheelMesh.isVisible = false

    const HK = await HavokPhysics()
    const gravityVector = new Vector3(0, -9.81, 0)
    const physicsPlugin = new HavokPlugin(false, HK)
    scene.enablePhysics(gravityVector, physicsPlugin)
    const physicsEngine = scene.getPhysicsEngine()
    physicsEngine.setTimeStep(simulationConfig.fixedTimeStepSeconds)
    physicsEngine.setSubTimeStep(0)

    const levelRootNode = levelFiles.rootNodes[0]
    levelRootNode.scaling.set(0.3, 0.3, 0.3)
    levelRootNode.computeWorldMatrix(true)

    const getMeshesByNameIncluded = (meshes, names) => {
        const result = []
        meshes.forEach((mesh) => {
            names.forEach((name) => {
                if (mesh.name.includes(name)) result.push(mesh)
            })
        })
        return result
    }
    const levelChildren = levelRootNode.getChildren()
    const staticGeom = getMeshesByNameIncluded(levelChildren, ['ground', 'side', 'pipe', 'ramp', 'rail', 'platform'])
    staticGeom.forEach((mesh) => {
        mesh.computeWorldMatrix(true)
        const meshPhysicsShape = new PhysicsShapeMesh(mesh, scene)
        const meshPhysicsBody = new PhysicsBody(mesh, PhysicsMotionType.STATIC, false, scene)
        meshPhysicsBody.shape = meshPhysicsShape
        meshPhysicsShape.filterMembershipMask = 1
    })
    const dynamicGeom = getMeshesByNameIncluded(levelChildren, ['barrel', 'Box', 'cone'])
    dynamicGeom.forEach((mesh) => {
        mesh.computeWorldMatrix(true)
        const meshPhysicsShape = new PhysicsShapeConvexHull(mesh, scene)
        const meshPhysicsBody = new PhysicsBody(mesh, PhysicsMotionType.DYNAMIC, true, scene)
        meshPhysicsBody.shape = meshPhysicsShape
        meshPhysicsBody.setMassProperties({
            mass: 10
        })
        meshPhysicsShape.filterMembershipMask = 1
    })

    const chassisMesh = MeshBuilder.CreateBox('Chassis', { width: 1, height: 0.4, depth: 2 })
    vehicleMesh.setParent(chassisMesh)
    chassisMesh.position.y = 5
    chassisMesh.position.x = 0
    chassisMesh.rotationQuaternion = new Quaternion()
    chassisMesh.visibility = 0

    const chassisPhysicsShape = new PhysicsShapeConvexHull(chassisMesh, scene)

    const chassisPhysicsBody = new PhysicsBody(chassisMesh, PhysicsMotionType.DYNAMIC, false, scene)
    chassisPhysicsBody.shape = chassisPhysicsShape
    chassisPhysicsBody.setMassProperties({
        mass: 1200,
        centerOfMass: new Vector3(0, -0.5, 0)
    })
    chassisPhysicsShape.filterMembershipMask = 2

    const followCamera = new FollowCamera('FollowCam', new Vector3(-6, 0, 0), scene)
    followCamera.heightOffset = 1
    followCamera.radius = 5
    followCamera.rotationOffset = 180
    followCamera.cameraAcceleration = 0.08
    followCamera.maxCameraSpeed = 30
    followCamera.lockedTarget = chassisMesh
    scene.activeCamera = followCamera

    const vehicle = new RaycastVehicle(chassisPhysicsBody, scene)
    const vehiclePhysicsConfig = {
        predictiveLookAheadSeconds: 0.5,
        predictionRatio: 0.8,
        maxVehicleSpeedMps: 60,
        numberOfGears: 5,
        wheelBaseMeters: 1.5,
        drivetrain: {
            maxDriveTorqueNm: 2600,
            brakeFromDirectionChangeSpeedMps: 1.2
        },
        steering: {
            maxSteerAngleLowSpeedRad: 0.52,
            maxSteerAngleHighSpeedRad: 0.13,
            speedForMinSteerMps: 35,
            steerRateRadiansPerSecond: 2.6,
            steerReturnRateRadiansPerSecond: 4.2,
            deadZone: 0,
            inputExponent: 1
        },
        yawAssist: {
            enabled: true,
            strength: 0.45,
            activationSpeedMps: 8,
            fullAssistSpeedMps: 20,
            minGroundedWheels: 3,
            maxAssistTorqueNm: 1800,
            yawRateGain: 850,
            slipAngleGain: 1300,
            slipAngleThresholdRad: 0.08
        },
        tire: {
            model: DEFAULT_TIRE_MODEL,
            presetName: DEFAULT_TIRE_PRESET
        },
        aerodynamics: {
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
    }
    vehicle.predictiveLookAheadSeconds = vehiclePhysicsConfig.predictiveLookAheadSeconds
    vehicle.predictionRatio = vehiclePhysicsConfig.predictionRatio
    vehicle.setAerodynamicsConfig(vehiclePhysicsConfig.aerodynamics)

    const selectedTirePreset = TIRE_MODEL_PRESETS[vehiclePhysicsConfig.tire.presetName] ?? TIRE_MODEL_PRESETS[DEFAULT_TIRE_PRESET]
    const selectedTireConfig = {
        tireModel: vehiclePhysicsConfig.tire.model,
        surfaceFriction: selectedTirePreset.surfaceFriction,
        longitudinalGripRatio: selectedTirePreset.longitudinalGripRatio,
        lateralGripRatio: selectedTirePreset.lateralGripRatio,
        pacejkaLongitudinal: { ...selectedTirePreset.pacejkaLongitudinal },
        pacejkaLateral: { ...selectedTirePreset.pacejkaLateral }
    }
    const curveDebuggerConfig = {
        ...telemetryConfig.curveDebugger,
        tireModel: selectedTireConfig.tireModel,
        presetName: vehiclePhysicsConfig.tire.presetName,
        // The debugger may draw against a different mu than gameplay for comparison.
        // This override only affects the render-time reference curves; setting it to 0 deliberately
        // draws a zero-grip reference without changing the gameplay tire forces.
        surfaceFriction: telemetryConfig.curveDebugger.surfaceFriction ?? selectedTireConfig.surfaceFriction,
        longitudinalGripRatio: selectedTireConfig.longitudinalGripRatio,
        lateralGripRatio: selectedTireConfig.lateralGripRatio
    }
    // These samples are a render-time reference for the HUD only.
    // They call the same tire-model module as gameplay, but they do not feed back into the fixed-step update.
    // If you change the tire config in code, rebuild/restart so the debugger regenerates against the new setup.
    const curveSamples = curveDebuggerConfig.enabled
        ? buildPureAxisCurveSamples({
              tireModel: selectedTireConfig.tireModel,
              tireConfig: selectedTireConfig,
              normalLoadN: curveDebuggerConfig.normalLoadN,
              surfaceFriction: curveDebuggerConfig.surfaceFriction,
              longitudinalGripRatio: curveDebuggerConfig.longitudinalGripRatio,
              lateralGripRatio: curveDebuggerConfig.lateralGripRatio,
              slipRatioRange: curveDebuggerConfig.slipRatioRange,
              slipAngleRangeDeg: curveDebuggerConfig.slipAngleRangeDeg,
              sampleCount: curveDebuggerConfig.sampleCount
          })
        : null

    const wheelBaseConfig = {
        suspensionRestLength: 0.6,
        springRate: 30000,
        damperRate: 4500,
        suspensionAxisLocal: new Vector3(0, -1, 0),
        axleAxisLocal: new Vector3(1, 0, 0),
        forwardAxisLocal: new Vector3(0, 0, 1),
        sideForcePositionRatio: 0.1,
        sideForce: 40,
        radius: 0.2,
        width: 0.15,
        bumpStopStartRatio: 0.85,
        bumpStopRate: 150000,
        wheelInertiaKgM2: 1.2,
        maxBrakeTorqueNm: 3200,
        angularDampingNmPerRadPerSec: 0.5,
        rollingResistanceCoefficient: 0.015,
        tireModel: selectedTireConfig.tireModel,
        surfaceFriction: selectedTireConfig.surfaceFriction,
        longitudinalStiffnessPerLoad: 11.5,
        corneringStiffnessPerLoad: 9.5,
        longitudinalShape: 1.5,
        lateralShape: 1.7,
        longitudinalGripRatio: selectedTireConfig.longitudinalGripRatio,
        lateralGripRatio: selectedTireConfig.lateralGripRatio,
        pacejkaLongitudinal: selectedTireConfig.pacejkaLongitudinal,
        pacejkaLateral: selectedTireConfig.pacejkaLateral
    }

    vehicle.addWheel(
        new RaycastWheel({
            ...wheelBaseConfig,
            positionLocal: new Vector3(0.52, 0, -0.7),
            isDriven: true,
            isSteerable: false,
            canBrake: true
        })
    )

    vehicle.addWheel(
        new RaycastWheel({
            ...wheelBaseConfig,
            positionLocal: new Vector3(-0.52, 0, -0.7),
            isDriven: true,
            isSteerable: false,
            canBrake: true
        })
    )

    vehicle.addWheel(
        new RaycastWheel({
            ...wheelBaseConfig,
            positionLocal: new Vector3(-0.52, 0, 0.8),
            isDriven: true,
            isSteerable: true,
            canBrake: true
        })
    )

    vehicle.addWheel(
        new RaycastWheel({
            ...wheelBaseConfig,
            positionLocal: new Vector3(0.52, 0, 0.8),
            isDriven: true,
            isSteerable: true,
            canBrake: true
        })
    )

    vehicle.addAntiRollAxle({ wheelA: 0, wheelB: 1, force: 10000 })
    vehicle.addAntiRollAxle({ wheelA: 2, wheelB: 3, force: 10000 })

    const wheelMeshes = [wheelMesh.createInstance(0), wheelMesh.createInstance(1), wheelMesh.createInstance(2), wheelMesh.createInstance(3)]
    wheelMeshes.forEach((wheelModel) => {
        wheelModel.parent = null
    })

    if (Engine.audioEngine) {
        Engine.audioEngine.useCustomUnlockedButton = true
    }

    let engineLowPassFilter = null
    const setupEngineFilter = () => {
        try {
            const audioCtx = Engine.audioEngine?.audioContext
            const destNode = scene.mainSoundTrack?.getSoundTrackAudioNode?.() ?? Engine.audioEngine?.masterGain
            if (!audioCtx || !destNode || engineLowPassFilter) {
                return
            }
            engineLowPassFilter = audioCtx.createBiquadFilter()
            engineLowPassFilter.type = 'lowpass'
            engineLowPassFilter.frequency.value = 280
            engineLowPassFilter.Q.value = 0.7
            engineLowPassFilter.connect(destNode)
            revSound.connectToSoundTrackAudioNode(engineLowPassFilter)
        } catch {
            engineLowPassFilter = null
        }
    }

    const revSound = new Sound('rev', '/sounds/med_on.wav', scene, setupEngineFilter, {
        loop: true,
        autoplay: true,
        playbackRate: 0.55,
        volume: 0.04
    })

    const shiftSound = new Sound('shift', '/sounds/shift_1.wav', scene, null, {
        loop: false,
        autoplay: false,
        playbackRate: 1,
        volume: 0.65
    })

    const unlockAudio = () => {
        if (Engine.audioEngine && !Engine.audioEngine.unlocked) {
            Engine.audioEngine.unlock()
        }
        setupEngineFilter()
        if (revSound.isReady() && !revSound.isPlaying) {
            revSound.play()
        }
    }
    window.addEventListener('pointerdown', unlockAudio)
    window.addEventListener('keydown', unlockAudio)

    let soundMuted = false
    const soundToggleBtn = createOverlayElement('button', {
        position: 'absolute',
        left: '12px',
        top: '12px',
        padding: '6px 10px',
        borderRadius: '6px',
        border: '1px solid rgba(255, 255, 255, 0.16)',
        background: 'rgba(10, 14, 22, 0.72)',
        color: '#e8eef7',
        font: '12px/1.2 monospace',
        cursor: 'pointer',
        zIndex: '10'
    })
    soundToggleBtn.type = 'button'
    soundToggleBtn.tabIndex = -1
    const syncSoundToggleLabel = () => {
        soundToggleBtn.textContent = soundMuted ? '🔇 Sound: OFF' : '🔊 Sound: ON'
    }
    syncSoundToggleLabel()
    soundToggleBtn.addEventListener('click', () => {
        unlockAudio()
        soundMuted = !soundMuted
        Engine.audioEngine?.setGlobalVolume(soundMuted ? 0 : 1)
        syncSoundToggleLabel()
        soundToggleBtn.blur()
    })

    const controls = {
        forward: false,
        backward: false,
        left: false,
        right: false
    }

    const inputProcessor = new KeyboardInputProcessor({
        directionChangeSpeedMps: vehiclePhysicsConfig.drivetrain.brakeFromDirectionChangeSpeedMps,
        maxSteerAngleLowSpeedRad: vehiclePhysicsConfig.steering.maxSteerAngleLowSpeedRad,
        maxSteerAngleHighSpeedRad: vehiclePhysicsConfig.steering.maxSteerAngleHighSpeedRad,
        speedForMinSteerMps: vehiclePhysicsConfig.steering.speedForMinSteerMps
    })

    const resetControls = () => {
        controls.forward = false
        controls.backward = false
        controls.left = false
        controls.right = false
        inputProcessor.reset()
    }

    scene.onKeyboardObservable.add((kbInfo) => {
        const key = kbInfo.event.key?.toLowerCase()
        switch (kbInfo.type) {
            case KeyboardEventTypes.KEYDOWN:
                if (key === 'w') controls.forward = true
                if (key === 's') controls.backward = true
                if (key === 'a') controls.left = true
                if (key === 'd') controls.right = true
                break
            case KeyboardEventTypes.KEYUP:
                if (key === 'w') controls.forward = false
                if (key === 's') controls.backward = false
                if (key === 'a') controls.left = false
                if (key === 'd') controls.right = false
                break
        }
    })

    window.addEventListener('blur', resetControls)
    document.addEventListener('visibilitychange', () => {
        if (document.hidden) {
            resetControls()
            Engine.audioEngine?.setGlobalVolume(0)
        } else {
            Engine.audioEngine?.setGlobalVolume(soundMuted ? 0 : 1)
        }
    })

    const telemetryHud = telemetryConfig.enabled
        ? createOverlayElement('pre', {
              position: 'absolute',
              left: '8px',
              top: '8px',
              padding: '8px',
              margin: '0',
              background: 'rgba(0,0,0,0.6)',
              color: '#d8fdd8',
              font: '12px/1.3 monospace',
              pointerEvents: 'none',
              whiteSpace: 'pre'
          })
        : null
    const curveDebuggerCanvas = curveDebuggerConfig.enabled
        ? createOverlayElement('canvas', {
              position: 'absolute',
              left: '8px',
              bottom: '8px',
              width: '364px',
              height: '340px',
              border: '1px solid rgba(216, 253, 216, 0.18)',
              background: 'transparent',
              pointerEvents: 'none'
          })
        : null
    const curveDebuggerContext = curveDebuggerCanvas ? curveDebuggerCanvas.getContext('2d') : null
    if (curveDebuggerCanvas) {
        curveDebuggerCanvas.width = 364
        curveDebuggerCanvas.height = 340
    }

    const drivetrain = new DrivetrainController()

    const debugHud = createOverlayElement('div', {
        position: 'absolute',
        right: '12px',
        top: '12px',
        width: '248px',
        padding: '10px 12px',
        borderRadius: '8px',
        background: 'rgba(10, 14, 22, 0.72)',
        border: '1px solid rgba(255, 255, 255, 0.14)',
        color: '#e8eef7',
        font: '12px/1.4 monospace',
        userSelect: 'none',
        zIndex: '10'
    })
    debugHud.innerHTML = `
        <div style="display:flex;justify-content:space-between;align-items:baseline;margin-bottom:6px;">
            <span id="hudSpeed" style="font-size:15px;font-weight:700;">0 km/h (0.0 m/s)</span>
            <span id="hudGear" style="color:#72f0a7;font-weight:700;">G1</span>
        </div>
        <div style="margin-bottom:6px;">
            <div style="display:flex;justify-content:space-between;"><span>RPM</span><span id="hudRpm">900</span></div>
            <div style="height:6px;background:rgba(255,255,255,0.12);border-radius:3px;overflow:hidden;">
                <div id="hudRpmBar" style="height:100%;width:0%;background:#67d5ff;"></div>
            </div>
        </div>
        <div style="margin-bottom:6px;">
            <div style="display:flex;justify-content:space-between;"><span>Steer</span><span id="hudSteer">0.0°</span></div>
            <div style="position:relative;height:6px;background:rgba(255,255,255,0.12);border-radius:3px;overflow:hidden;">
                <div style="position:absolute;left:50%;top:0;bottom:0;width:1px;background:rgba(255,255,255,0.35);"></div>
                <div id="hudSteerBar" style="position:absolute;top:0;bottom:0;left:50%;width:0%;background:#ffd166;"></div>
            </div>
        </div>
        <div style="margin-bottom:6px;">
            <div style="display:flex;justify-content:space-between;"><span>Throttle</span><span id="hudThrottle">0%</span></div>
            <div style="height:6px;background:rgba(255,255,255,0.12);border-radius:3px;overflow:hidden;">
                <div id="hudThrottleBar" style="height:100%;width:0%;background:#72f0a7;"></div>
            </div>
        </div>
        <div style="margin-bottom:8px;">
            <div style="display:flex;justify-content:space-between;"><span>Brake</span><span id="hudBrake">0%</span></div>
            <div style="height:6px;background:rgba(255,255,255,0.12);border-radius:3px;overflow:hidden;">
                <div id="hudBrakeBar" style="height:100%;width:0%;background:#ff6b6b;"></div>
            </div>
        </div>
        <div style="display:flex;justify-content:space-between;margin-bottom:8px;color:#b7c4d8;">
            <span>Aero</span><span id="hudAero">Drag 0N | DF 0N</span>
        </div>
        <div style="display:flex;gap:6px;">
            <button id="hudAbsBtn" type="button" tabindex="-1" style="flex:1;padding:4px 6px;border-radius:4px;border:1px solid rgba(255,255,255,0.2);background:#1f3a2e;color:#72f0a7;font:inherit;cursor:pointer;">ABS: ON</button>
            <button id="hudTcsBtn" type="button" tabindex="-1" style="flex:1;padding:4px 6px;border-radius:4px;border:1px solid rgba(255,255,255,0.2);background:#1f3a2e;color:#72f0a7;font:inherit;cursor:pointer;">TCS: ON</button>
        </div>
    `
    const hudSpeedEl = debugHud.querySelector('#hudSpeed')
    const hudGearEl = debugHud.querySelector('#hudGear')
    const hudRpmEl = debugHud.querySelector('#hudRpm')
    const hudRpmBarEl = debugHud.querySelector('#hudRpmBar')
    const hudSteerEl = debugHud.querySelector('#hudSteer')
    const hudSteerBarEl = debugHud.querySelector('#hudSteerBar')
    const hudThrottleEl = debugHud.querySelector('#hudThrottle')
    const hudThrottleBarEl = debugHud.querySelector('#hudThrottleBar')
    const hudBrakeEl = debugHud.querySelector('#hudBrake')
    const hudBrakeBarEl = debugHud.querySelector('#hudBrakeBar')
    const hudAeroEl = debugHud.querySelector('#hudAero')
    const hudAbsBtn = debugHud.querySelector('#hudAbsBtn')
    const hudTcsBtn = debugHud.querySelector('#hudTcsBtn')

    const syncAssistButtonStyle = (button, label, enabled, active) => {
        button.textContent = active ? `${label}: ACTIVE` : `${label}: ${enabled ? 'ON' : 'OFF'}`
        if (!enabled) {
            button.style.background = '#2a2a32'
            button.style.color = '#9aa4b2'
            return
        }
        button.style.background = active ? '#5c4314' : '#1f3a2e'
        button.style.color = active ? '#ffd166' : '#72f0a7'
    }

    hudAbsBtn.addEventListener('click', () => {
        drivetrain.absEnabled = !drivetrain.absEnabled
        syncAssistButtonStyle(hudAbsBtn, 'ABS', drivetrain.absEnabled, false)
        hudAbsBtn.blur()
    })
    hudTcsBtn.addEventListener('click', () => {
        drivetrain.tcsEnabled = !drivetrain.tcsEnabled
        syncAssistButtonStyle(hudTcsBtn, 'TCS', drivetrain.tcsEnabled, false)
        hudTcsBtn.blur()
    })

    let steerValue = 0

    revSound.play()
    scene.onBeforePhysicsObservable.add(() => {
        if (!Number.isFinite(chassisMesh.position.y) || chassisMesh.position.y < -15) {
            chassisMesh.position.set(0, 2, 0)
            chassisMesh.rotationQuaternion.set(0, 0, 0, 1)
            chassisPhysicsBody.setLinearVelocity(Vector3.Zero())
            chassisPhysicsBody.setAngularVelocity(Vector3.Zero())
            inputProcessor.reset()
        }

        const dtSeconds = simulationConfig.fixedTimeStepSeconds
        const speedAbs = Math.abs(vehicle.speed)

        const steerableContactWheelsForInput = vehicle.wheels.filter((wheel) => wheel.isSteerable && wheel.inContact)
        const avgSteerableSlipAngleRad =
            steerableContactWheelsForInput.length > 0
                ? steerableContactWheelsForInput.reduce((acc, wheel) => acc + wheel.slipAngleRad, 0) /
                  steerableContactWheelsForInput.length
                : 0

        const smoothedInput = inputProcessor.step({
            dtSeconds,
            forwardPressed: controls.forward,
            backwardPressed: controls.backward,
            leftPressed: controls.left,
            rightPressed: controls.right,
            vehicleSpeedMps: vehicle.speed,
            averageSlipAngleRad: avgSteerableSlipAngleRad
        })
        steerValue = smoothedInput.steerAngleRad

        vehicle.wheels.forEach((wheel) => {
            wheel.steering = wheel.isSteerable ? steerValue : 0
            wheel.force = 0
        })

        const drivetrainState = drivetrain.step({
            dtSeconds,
            vehicleSpeedMps: vehicle.speed,
            longitudinalInput: smoothedInput.longitudinalInput,
            wheels: vehicle.wheels,
            directionChangeSpeedMps: vehiclePhysicsConfig.drivetrain.brakeFromDirectionChangeSpeedMps
        })
        if (drivetrainState.shiftOccurred && Engine.audioEngine?.unlocked) {
            shiftSound.play()
        }

        vehicle.update(dtSeconds)

        if (
            vehiclePhysicsConfig.yawAssist.enabled &&
            speedAbs >= vehiclePhysicsConfig.yawAssist.activationSpeedMps &&
            vehicle.nWheelsOnGround >= vehiclePhysicsConfig.yawAssist.minGroundedWheels
        ) {
            const chassisUp = Vector3.TransformNormalToRef(Axis.Y, chassisMesh.getWorldMatrix(), tmp1).normalize()
            const actualYawRateRadPerSec = Vector3.Dot(chassisPhysicsBody.getAngularVelocity(), chassisUp)
            const targetYawRateRadPerSec =
                vehiclePhysicsConfig.wheelBaseMeters > 0.01
                    ? (vehicle.speed / vehiclePhysicsConfig.wheelBaseMeters) * Math.tan(steerValue)
                    : 0

            const steerableContactWheels = vehicle.wheels.filter((wheel) => wheel.isSteerable && wheel.inContact)
            const averageSlipAngleRad =
                steerableContactWheels.length > 0
                    ? steerableContactWheels.reduce((acc, wheel) => acc + wheel.slipAngleRad, 0) / steerableContactWheels.length
                    : 0

            const yawRateError = targetYawRateRadPerSec - actualYawRateRadPerSec
            const slipAssistError = Math.abs(averageSlipAngleRad) - vehiclePhysicsConfig.yawAssist.slipAngleThresholdRad
            const slipAssistTerm =
                slipAssistError > 0 ? -averageSlipAngleRad * vehiclePhysicsConfig.yawAssist.slipAngleGain : 0

            const speedAssistScale = clampNumber(
                (speedAbs - vehiclePhysicsConfig.yawAssist.activationSpeedMps) /
                    Math.max(
                        vehiclePhysicsConfig.yawAssist.fullAssistSpeedMps - vehiclePhysicsConfig.yawAssist.activationSpeedMps,
                        1e-6
                    ),
                0,
                1
            )
            const groundedAssistScale = clampNumber(vehicle.nWheelsOnGround / vehicle.wheels.length, 0, 1)
            const yawAssistTorqueNm = clampNumber(
                (yawRateError * vehiclePhysicsConfig.yawAssist.yawRateGain + slipAssistTerm) *
                    vehiclePhysicsConfig.yawAssist.strength *
                    speedAssistScale *
                    groundedAssistScale,
                -vehiclePhysicsConfig.yawAssist.maxAssistTorqueNm,
                vehiclePhysicsConfig.yawAssist.maxAssistTorqueNm
            )

            if (yawAssistTorqueNm !== 0 && Number.isFinite(yawAssistTorqueNm)) {
                chassisPhysicsBody.applyTorque(chassisUp.scaleToRef(yawAssistTorqueNm, tmp2))
            }
        }
    })

    scene.onBeforeRenderObservable.add(() => {
        vehicle.wheels.forEach((wheel, index) => {
            if (!wheelMeshes[index]) return
            const wheelModel = wheelMeshes[index]
            wheelModel.position.copyFrom(wheel.transform.position)
            wheelModel.rotationQuaternion.copyFrom(wheel.transform.rotationQuaternion)
            if (index == 0 || index == 3) wheelModel.rotate(Axis.Y, Math.PI, Space.LOCAL)
        })

        const telemetry = telemetryHud || curveDebuggerContext ? vehicle.getWheelTelemetrySnapshot() : null

        if (telemetryHud && telemetry) {
            const totalNormalLoadN = telemetry.reduce((sum, wheel) => sum + wheel.normalLoadN, 0)
            const lines = [
                `speed=${vehicle.speed.toFixed(2)}m/s grounded=${vehicle.nWheelsOnGround} FzSum=${totalNormalLoadN.toFixed(0)}N tire=${selectedTireConfig.tireModel}/${vehiclePhysicsConfig.tire.presetName}`
            ]
            telemetry.forEach((wheel) => {
                lines.push(
                    `W${wheel.index} ${wheel.inContact ? 'G' : 'A'} Fz=${wheel.normalLoadN.toFixed(0)}N α=${(
                        wheel.slipAngleRad * RAD_TO_DEG
                    ).toFixed(1)}° κ=${wheel.slipRatio.toFixed(2)} Fx=${wheel.longitudinalForceN.toFixed(0)} Fy=${
                        wheel.lateralForceN
                    .toFixed(0)} ω=${wheel.angularVelocityRadPerSec.toFixed(1)} scale=${wheel.combinedForceScale.toFixed(2)}`
                )
            })
            telemetryHud.textContent = lines.join('\n')
        }

        if (curveDebuggerContext && telemetry) {
            const selectedWheelTelemetry = telemetry[curveDebuggerConfig.wheelIndex] ?? null
            drawTireCurveDebugger({
                context: curveDebuggerContext,
                curveDebugger: curveDebuggerConfig,
                curveSamples,
                wheelTelemetry: selectedWheelTelemetry
            })
        }

        const rpmRatio = clampNumber(
            (drivetrain.rpm - drivetrain.config.idleRpm) /
                Math.max(drivetrain.config.maxRpm - drivetrain.config.idleRpm, 1),
            0,
            1
        )
        try {
            const safeRate = 0.55 + 1.15 * rpmRatio
            const safeVol = 0.04 + 0.14 * drivetrain.throttleOpenRatio + 0.08 * rpmRatio
            if (Number.isFinite(safeRate)) {
                revSound.setPlaybackRate(safeRate)
            }
            if (Number.isFinite(safeVol)) {
                revSound.setVolume(safeVol)
            }
            if (engineLowPassFilter && Number.isFinite(rpmRatio)) {
                engineLowPassFilter.frequency.value =
                    280 + 2120 * Math.max(drivetrain.throttleOpenRatio, rpmRatio * 0.8)
            }
        } catch {
            // Audio updates must never interrupt 3D rendering
        }

        const speedMps = vehicle.speed
        const speedKmh = speedMps * 3.6
        hudSpeedEl.textContent = `${Math.abs(speedKmh).toFixed(0)} km/h (${speedMps.toFixed(1)} m/s)`
        hudGearEl.textContent = drivetrain.gear === -1 ? 'R' : `G${drivetrain.gear}`
        hudRpmEl.textContent = `${Math.round(drivetrain.rpm)}`
        hudRpmBarEl.style.width = `${(rpmRatio * 100).toFixed(1)}%`

        const steerDeg = steerValue * RAD_TO_DEG
        hudSteerEl.textContent = `${steerDeg >= 0 ? '+' : ''}${steerDeg.toFixed(1)}°`
        const steerHalfPct = clampNumber(
            (Math.abs(steerValue) / Math.max(vehiclePhysicsConfig.steering.maxSteerAngleLowSpeedRad, 1e-4)) * 50,
            0,
            50
        )
        hudSteerBarEl.style.width = `${steerHalfPct.toFixed(1)}%`
        hudSteerBarEl.style.left = steerValue >= 0 ? '50%' : `${(50 - steerHalfPct).toFixed(1)}%`

        const throttlePct = Math.round(drivetrain.throttleOpenRatio * 100)
        const brakePct = Math.round(drivetrain.brakeOpenRatio * 100)
        hudThrottleEl.textContent = `${throttlePct}%`
        hudThrottleBarEl.style.width = `${throttlePct}%`
        hudBrakeEl.textContent = `${brakePct}%`
        hudBrakeBarEl.style.width = `${brakePct}%`
        hudAeroEl.textContent = `Drag ${Math.round(vehicle.aeroTelemetry.dragForceN)}N | DF ${Math.round(
            vehicle.aeroTelemetry.totalDownforceN
        )}N`

        syncAssistButtonStyle(hudAbsBtn, 'ABS', drivetrain.absEnabled, drivetrain.absActive)
        syncAssistButtonStyle(hudTcsBtn, 'TCS', drivetrain.tcsEnabled, drivetrain.tcsActive)
    })

    engine.runRenderLoop(() => {
        scene.render()
    })

    window.addEventListener('resize', () => {
        engine.resize()
    })
}

init()
