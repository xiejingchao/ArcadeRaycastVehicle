A very simplified RaycastVehicle using Babylon.js and HavokPhysics

Live test:
https://raggardk.github.io/

## Physics conventions

- Fixed physics timestep: **1/120 s**.
- Babylon/Havok stepping uses Babylon Engine deterministic lockstep (`timeStep = 1/120`, `lockstepMaxSteps = 8`) so the engine-owned accumulator advances both Havok and custom vehicle forces in sync on `scene.onBeforePhysicsObservable`.
- Maximum accumulated simulation time per render frame: **8 substeps = 66.7 ms**.
- Units follow SI conventions: length `m`, time `s`, mass `kg`, speed `m/s`, acceleration `m/s²`, force `N`, torque `N·m`, angle `rad`, angular speed `rad/s`, spring rate `N/m`, damper rate `N·s/m`.

## Tire model and wheel runtime state (steps 2-6)

- Each wheel now tracks telemetry/state:
  - `normalLoadN (Fz)`
  - `longitudinalVelocityMps`, `lateralVelocityMps` (contact-point velocity decomposition)
  - `slipAngleRad`, `slipRatio`
  - `angularVelocityRadPerSec`
  - `rawLongitudinalForceN/rawLateralForceN` (before combined-slip limit)
  - `longitudinalForceN/lateralForceN` (after combined-slip limit)
  - `combinedForceScale`, `surfaceFriction`
- Sign conventions:
  - Longitudinal `+` is wheel forward axis (`forwardAxisLocal`)
  - Lateral `+` is wheel right axis (`axleAxisLocal`)
  - Positive slip angle means contact-point lateral velocity has positive/right component
  - Positive brake input is a magnitude; applied torque always opposes current wheel spin direction
- Suspension remains linear spring-damper; its output is the wheel normal load `Fz` used by tire force calculations.

## Simplified brush tire + friction ellipse

- Pure tire-force computation is in `src/physics/tireModel.js` (no Babylon dependency).
- Simplified load-sensitive brush-style approximation:
  - `Kx ≈ longitudinalStiffnessPerLoad * Fz`
  - `Ky ≈ corneringStiffnessPerLoad * Fz`
  - Raw force soft saturation uses `tanh(...)` shaped by `longitudinalShape/lateralShape`.
- Combined-slip constraint uses friction ellipse scaling so the final `Fx/Fy` respects total grip (`mu * Fz`) jointly, not per-axis hard-clamp.
- Wheel contact force is applied on the contact tangent plane defined by hit normal.

## Wheel angular dynamics

- Wheel visual spin now comes from wheel angular dynamics, not direct vehicle speed.
- Per wheel (SI):
  - `wheelInertiaKgM2`
  - `driveTorqueNm`
  - `brakeTorqueNm` / `maxBrakeTorqueNm`
  - `rollingResistanceCoefficient`
  - `angularDampingNmPerRadPerSec`
- Integrated each fixed step:
  - `netTorque = driveTorque - brakeTorque - groundReactionTorque - rollingResistance - angularDamping`
  - `omega += (netTorque / inertia) * dt`
- Demo drivetrain remains front-wheel-drive (`isDriven` on front wheels).

## Steering controller and yaw assist

- Steering is fixed-step and configurable:
  - speed-sensitive steering angle cap
  - rate-limited steering in/out (per second units)
  - dead zone and optional exponent curve
- Yaw assist is configurable and mild:
  - off switch: `vehiclePhysicsConfig.yawAssist.enabled`
  - only active at higher speed with enough grounded wheels
  - applies bounded yaw torque (no direct velocity/angular velocity override for assist)

## Telemetry HUD

- Lightweight HTML overlay HUD is available in `src/index.js`:
  - toggle with `telemetryConfig.enabled`
  - updates in render callback only (does not affect fixed-step physics)
  - shows per-wheel contact state, `Fz`, slip angle (deg), slip ratio, `Fx/Fy`, wheel speed, combined scale

## Parameters to tune (defaults in `src/index.js`)

- Drivetrain: `maxDriveTorqueNm`, direction-change brake threshold.
- Steering: low/high speed max angle, speed range, steer/return rates, dead zone, exponent.
- Yaw assist: `enabled`, `strength`, speed window, grounded-wheel threshold, gains, max torque.
- Tire: `surfaceFriction`, longitudinal/cornering stiffness-per-load, shape factors, ellipse axis ratios.
- Wheel spin damping: `rollingResistanceCoefficient`, `angularDampingNmPerRadPerSec`.

## Tests and validation

- Build: `npm run build`
- Tire model tests: `npm test`

## Known limitations

- Single-ray wheel contact only (no multi-ray/cylinder wheel contact patch).
- No full engine/RPM/gearbox/differential model yet.
- No ABS/TCS, tire thermal/wear modeling, or per-material friction map system.
