A very simplified RaycastVehicle using Babylon.js and HavokPhysics

Live test:
https://raggardk.github.io/

## Physics conventions

- Fixed physics timestep: **1/120 s**.
- Babylon/Havok stepping uses Babylon Engine deterministic lockstep (`timeStep = 1/120`, `lockstepMaxSteps = 8`) so the engine-owned accumulator advances both Havok and custom vehicle forces in sync on `scene.onBeforePhysicsObservable`.
- Maximum accumulated simulation time per render frame: **8 substeps = 66.7 ms**. Longer frame deltas are clamped to avoid spiral-of-death after tab switches or stalls.
- Units follow SI conventions: length `m`, time `s`, mass `kg`, speed `m/s`, acceleration `m/s²`, force `N`, torque `N·m`, angle `rad`, angular speed `rad/s`, spring rate `N/m`, damper rate `N·s/m`.
- Suspension uses a linear spring-damper model: `springForce = springRate * compressionMeters`, `damperForce = damperRate * compressionVelocityMps`, and total suspension normal force is clamped to be non-negative.
- Wheel visual spin is still a temporary no-slip approximation: `angularSpeed ≈ linearSpeed / radius`. Real wheel angular dynamics, slip ratio, slip angle, brush tire forces, and friction ellipse are **not** implemented yet.
- The current lateral `sideForce` is still an arcade placeholder, kept FPS-independent by evaluating it only inside fixed physics steps.

### Known limitations

- Tire telemetry interfaces for `Fz`, slip angle, and slip ratio are not exposed yet, but the fixed-step loop and SI-named spring/damper parameters are now in place for that next step.
- The lateral tire model is still a simplified arcade approximation rather than a brush model.
