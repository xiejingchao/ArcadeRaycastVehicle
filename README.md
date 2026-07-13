A very simplified RaycastVehicle using Babylon.js and HavokPhysics

Live test:
https://raggardk.github.io/

## Physics conventions

- Fixed physics timestep: **1/120 s**.
- Babylon/Havok stepping uses Babylon Engine deterministic lockstep (`timeStep = 1/120`, `lockstepMaxSteps = 8`) so Havok and the custom vehicle forces advance together from `scene.onBeforePhysicsObservable`.
- Maximum accumulated simulation time per render frame: **8 substeps = 66.7 ms**.
- Units follow SI conventions: length `m`, time `s`, mass `kg`, speed `m/s`, acceleration `m/s²`, force `N`, torque `N·m`, angle `rad`, angular speed `rad/s`, spring rate `N/m`, damper rate `N·s/m`.

## Tire model and runtime state

- The default main tire curve is now **`pacejkaForgiving`**, a simplified Magic Formula tuned for a wider controllable region and a soft post-peak plateau.
- The previous **`brush`** (`tanh` saturation) model is still available as a fallback/regression reference.
- Wheel runtime telemetry tracks:
  - `normalLoadN (Fz)`
  - `longitudinalVelocityMps`, `lateralVelocityMps`
  - `slipAngleRad`, `slipRatio`
  - `angularVelocityRadPerSec`
  - `rawLongitudinalForceN/rawLateralForceN` before combined-slip limiting
  - `longitudinalForceN/lateralForceN` after combined-slip limiting
  - `combinedForceScale`, `combinedForceUtilization`, `frictionLimitN`
- Sign conventions:
  - Longitudinal `+` is wheel forward (`forwardAxisLocal`)
  - Lateral `+` is wheel right (`axleAxisLocal`)
  - Positive slip ratio means the wheel surface speed is faster than ground speed, so driving slip produces positive `Fx`
  - Positive slip angle means the contact-patch velocity has a positive/right lateral component, so the tire responds with negative `Fy`

## Forgiving Pacejka / Magic Formula

- Pure-axis force uses the simplified Magic Formula form:

  `F(x) = D * sin(C * atan(Bx - E * (Bx - atan(Bx))))`

- The implementation uses **semantic per-axis tuning** in `src/physics/tireModel.js`:
  - `peakSlip` — target slip location for the main peak region (`kappa` for longitudinal, `rad` for lateral)
  - `shapeFactorC` — how rounded or sharp the main peak feels
  - `curvatureFactorE` — how strongly the curve bends near the peak
  - `stiffnessFactorB` — optional direct override for initial build-up stiffness
  - `peakGripRatio` — scales `D`; with `D = mu * Fz * peakGripRatio`
  - `postPeakSlip` — where the forgiving post-peak treatment starts
  - `postPeakTransition` — width of the smooth blend toward the retained plateau
  - `postPeakRetention` — retained fraction of peak force after the blend
- The post-peak blend uses a smoothstep easing so the curve widens into a stable plateau instead of dropping off a cliff.

## Combined slip / friction ellipse

- Raw `Fx/Fy` are computed first from the selected single-axis model.
- Final force is then limited once by a friction ellipse so total grip stays inside `mu * Fz` continuously.
- Telemetry exposes both raw and final values so curve-debugger overlays can show where combined slip is reducing the final force.

## Tire presets

`src/index.js` selects the active tire setup via `vehiclePhysicsConfig.tire`:

- `grip`
  - Higher lateral peak/support
  - Less slide-friendly after peak
- `balanced` (**default**)
  - Wide peak platform
  - Small, recoverable drift window
- `driftFriendly`
  - Easier to enter slip
  - Higher post-peak retention for sustained controllable sliding

These presets only change tire-curve and grip-ratio parameters; they do not rely on stronger yaw-assist tricks.

## Wheel angular dynamics

- Wheel visual spin comes from wheel angular dynamics, not direct vehicle speed.
- Per wheel:
  - `wheelInertiaKgM2`
  - `driveTorqueNm`
  - `brakeTorqueNm` / `maxBrakeTorqueNm`
  - `rollingResistanceCoefficient`
  - `angularDampingNmPerRadPerSec`
- Integrated each fixed step:
  - `netTorque = driveTorque - brakeTorque - groundReactionTorque - rollingResistance - angularDamping`
  - `omega += (netTorque / inertia) * dt`

## Steering controller and yaw assist

- Steering remains fixed-step and configurable:
  - speed-sensitive steering angle cap
  - rate-limited steering in/out
  - dead zone and input exponent
- Yaw assist remains optional and mild:
  - `vehiclePhysicsConfig.yawAssist.enabled`
  - only active at speed with enough grounded wheels
  - applies bounded yaw torque only

## Telemetry HUD and tire curve debugger

- Lightweight HTML overlays live in `src/index.js`.
- Basic telemetry HUD:
  - toggle with `telemetryConfig.enabled`
  - updates during render only, so it does not affect fixed-step physics
- Tire curve debugger:
  - toggle with `telemetryConfig.curveDebugger.enabled`
  - configure `wheelIndex`, drawing `normalLoadN`, drawing `surfaceFriction`, and input ranges
  - renders two pure-axis charts:
    - `Fx` vs `slipRatio` at `slipAngle = 0`
    - `Fy` vs `slipAngle` (deg) at `slipRatio = 0`
  - overlays the selected wheel’s live raw and final working points from telemetry
- The debugger samples the same tire-model module as gameplay, but it is still a **render-time visualization**: it does not feed values back into the vehicle simulation.

## Parameters to tune

- Drivetrain: `maxDriveTorqueNm`, direction-change brake threshold
- Steering: low/high speed max angle, speed range, steer/return rates, dead zone, exponent
- Yaw assist: `enabled`, `strength`, speed window, grounded-wheel threshold, gains, max torque
- Tire model selection/preset: `vehiclePhysicsConfig.tire.model`, `vehiclePhysicsConfig.tire.presetName`
- Tire curves: preset values in `src/physics/tireModel.js` and debugger settings in `src/index.js`

## Tests and validation

- Build: `npm run build`
- Tire model tests: `npm test`

## Known limitations

- Single-ray wheel contact only (no multi-ray / cylinder contact patch)
- No full Pacejka 2002/2012 parameter set or manufacturer data fitting
- No tire temperature, wear, carcass deflection, ABS, or TCS
- No multi-surface friction map system yet
- The forgiving Pacejka defaults are gameplay-oriented, not measured real-tire data
