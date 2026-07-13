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

import HavokPhysics from '@babylonjs/havok'
import RaycastVehicle from './physics/raycastVehicle.js'
import RaycastWheel from './physics/raycastWheel.js'
import { clampNumber, moveTowards } from './utils/utils.js'
import {
    DEFAULT_TIRE_MODEL,
    DEFAULT_TIRE_PRESET,
    TIRE_MODEL_PRESETS,
    computeTireForces
} from './physics/tireModel.js'

import { Sound } from '@babylonjs/core/Audio/sound.js'
import '@babylonjs/core/Audio/audioSceneComponent.js'

import { Animation } from '@babylonjs/core/Animations/animation.js'

const tmp1 = new Vector3()
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
        timeStep: simulationConfig.fixedTimeStepSeconds
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
    wheelMesh.scaling.set(2.4, 1.6, 1.6)

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
            maxSteerAngleLowSpeedRad: 0.6,
            maxSteerAngleHighSpeedRad: 0.24,
            speedForMinSteerMps: 35,
            steerRateRadiansPerSecond: 2.7,
            steerReturnRateRadiansPerSecond: 4.8,
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
        }
    }
    vehicle.predictiveLookAheadSeconds = vehiclePhysicsConfig.predictiveLookAheadSeconds
    vehicle.predictionRatio = vehiclePhysicsConfig.predictionRatio

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
        // The debugger may draw against a different mu than gameplay for comparison,
        // but this override only affects the render-time reference curves.
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
            positionLocal: new Vector3(0.49, 0, -0.7),
            isDriven: false,
            isSteerable: false,
            canBrake: true
        })
    )

    vehicle.addWheel(
        new RaycastWheel({
            ...wheelBaseConfig,
            positionLocal: new Vector3(-0.49, 0, -0.7),
            isDriven: false,
            isSteerable: false,
            canBrake: true
        })
    )

    vehicle.addWheel(
        new RaycastWheel({
            ...wheelBaseConfig,
            positionLocal: new Vector3(-0.49, 0, 0.8),
            isDriven: true,
            isSteerable: true,
            canBrake: true
        })
    )

    vehicle.addWheel(
        new RaycastWheel({
            ...wheelBaseConfig,
            positionLocal: new Vector3(0.49, 0, 0.8),
            isDriven: true,
            isSteerable: true,
            canBrake: true
        })
    )

    vehicle.addAntiRollAxle({ wheelA: 0, wheelB: 1, force: 10000 })
    vehicle.addAntiRollAxle({ wheelA: 2, wheelB: 3, force: 10000 })

    const wheelMeshes = [wheelMesh.createInstance(0), wheelMesh.createInstance(1), wheelMesh.createInstance(2), wheelMesh.createInstance(3)]

    const revSound = new Sound('rev', '/sounds/med_on.wav', scene, null, {
        loop: true,
        autoplay: true,
        playbackRate: 1,
        volume: 0.3
    })

    const shiftSound = new Sound('shift', '/sounds/shift_1.wav', scene, null, {
        loop: false,
        autoplay: false,
        playbackRate: 1,
        volume: 0.8
    })

    const controls = {
        forward: false,
        backward: false,
        left: false,
        right: false
    }

    scene.onKeyboardObservable.add((kbInfo) => {
        switch (kbInfo.type) {
            case KeyboardEventTypes.KEYDOWN:
                if (kbInfo.event.key == 'w') controls.forward = true
                if (kbInfo.event.key == 's') controls.backward = true
                if (kbInfo.event.key == 'a') controls.left = true
                if (kbInfo.event.key == 'd') controls.right = true
                break
            case KeyboardEventTypes.KEYUP:
                if (kbInfo.event.key == 'w') controls.forward = false
                if (kbInfo.event.key == 's') controls.backward = false
                if (kbInfo.event.key == 'a') controls.left = false
                if (kbInfo.event.key == 'd') controls.right = false
                break
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

    let currentGear = 0
    let steerValue = 0

    revSound.play()
    scene.onBeforePhysicsObservable.add(() => {
        const dtSeconds = simulationConfig.fixedTimeStepSeconds

        const longitudinalInput = (controls.forward ? 1 : 0) + (controls.backward ? -1 : 0)
        const steerInput = (controls.right ? 1 : 0) + (controls.left ? -1 : 0)
        const speedAbs = Math.abs(vehicle.speed)

        const maxSteerBySpeed =
            vehiclePhysicsConfig.steering.maxSteerAngleLowSpeedRad +
            (vehiclePhysicsConfig.steering.maxSteerAngleHighSpeedRad -
                vehiclePhysicsConfig.steering.maxSteerAngleLowSpeedRad) *
                clampNumber(speedAbs / vehiclePhysicsConfig.steering.speedForMinSteerMps, 0, 1)

        const shapedSteerInput = shapeAxisInput(
            steerInput,
            vehiclePhysicsConfig.steering.deadZone,
            vehiclePhysicsConfig.steering.inputExponent
        )
        const targetSteer = shapedSteerInput * maxSteerBySpeed
        const steerRate =
            shapedSteerInput === 0
                ? vehiclePhysicsConfig.steering.steerReturnRateRadiansPerSecond
                : vehiclePhysicsConfig.steering.steerRateRadiansPerSecond
        steerValue = moveTowards(steerValue, targetSteer, steerRate * dtSeconds)
        steerValue = clampNumber(steerValue, -maxSteerBySpeed, maxSteerBySpeed)

        const speedForDriveCurve = Math.min(speedAbs, vehiclePhysicsConfig.maxVehicleSpeedMps)
        const speedProgress = (speedForDriveCurve / vehiclePhysicsConfig.maxVehicleSpeedMps) * 100
        const driveCurveScale = accelerationCurve.evaluate(speedProgress)
        const isDirectionChangeBrake =
            longitudinalInput !== 0 &&
            speedAbs > vehiclePhysicsConfig.drivetrain.brakeFromDirectionChangeSpeedMps &&
            Math.sign(vehicle.speed) !== Math.sign(longitudinalInput)

        const totalDriveTorqueNm = isDirectionChangeBrake
            ? 0
            : longitudinalInput * driveCurveScale * vehiclePhysicsConfig.drivetrain.maxDriveTorqueNm
        const brakeInput = isDirectionChangeBrake ? Math.abs(longitudinalInput) : 0

        const drivenWheels = vehicle.wheels.filter((wheel) => wheel.isDriven)
        const driveTorquePerWheelNm = drivenWheels.length > 0 ? totalDriveTorqueNm / drivenWheels.length : 0

        vehicle.wheels.forEach((wheel) => {
            wheel.steering = wheel.isSteerable ? steerValue : 0
            wheel.driveTorqueNm = wheel.isDriven ? driveTorquePerWheelNm : 0
            wheel.brakeTorqueNm = wheel.canBrake ? brakeInput * wheel.maxBrakeTorqueNm : 0
            wheel.force = 0
        })

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
                chassisPhysicsBody.applyTorque(chassisUp.scale(yawAssistTorqueNm))
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

        const maxSpeedSound = Math.min(Math.abs(vehicle.speed), vehiclePhysicsConfig.maxVehicleSpeedMps)
        const gearProgression = (maxSpeedSound * vehiclePhysicsConfig.numberOfGears) / vehiclePhysicsConfig.maxVehicleSpeedMps
        const currentGearNumber = Math.floor(gearProgression)
        const gearRatio = gearProgression - currentGearNumber
        revSound.setPlaybackRate(currentGearNumber / vehiclePhysicsConfig.numberOfGears + 1.2 * gearRatio)
        if (currentGear !== currentGearNumber) {
            currentGear = currentGearNumber
            shiftSound.play()
        }
    })

    engine.runRenderLoop(() => {
        scene.render()
    })

    window.addEventListener('resize', () => {
        engine.resize()
    })
}

init()
