Ball-Balancing Robot – Mathematical Overview
===========================================

This document summarizes the equations used across the project and explains why each approach was chosen over alternatives. File paths are workspace-relative.

Coordinate Frames and Symbols
-----------------------------
- Platform attitude is represented by a unit normal vector **n** = (nx, ny, nz); for small tilts nx ≈ pitch and ny ≈ roll in radians.
- Servo angles θi are measured from the vertical axis and correspond to the three evenly spaced base joints at 0°, 120°, and 240°.
- Length terms: lp (top triangle half-edge), l1 (upper link), l2 (lower link), lb (base triangle half-edge), h (platform height). These match the arguments stored inside `robotKinematics.py`.

3-RRS Kinematics
----------------
File: `robotKinematics.py`

- Height limits: `compute_maxh` and `compute_minh` apply Pythagoras to determine the vertical travel the 3-RRS mechanism can achieve:
  - maxh = √((l1 + l2)² − (lp − lb)²)
  - minh is piecewise, folding whichever link is longer over the base triangle until the remaining triangle degenerates.
- Platform pose (`solve_top`): given a desired unit normal (a,b,c) and height h, the three top attachment points A1–A3 are computed in closed form. Each point is obtained by projecting the normal onto the corresponding edge direction, normalizing by √(4c² + (a ± √3 b)²) to avoid singularities. The optional `invert` flag mirrors the assembly.
- Middle joints (`solve_middle`): for each leg, the code builds quadratic coefficients p,q,r,s,t describing the circle-line intersections between link lengths l1/l2 and the triangle geometry. Discriminants (s² − 4rt) are enforced to stay non-negative; the physically reachable branch is chosen by the sign of the square root. When `invert` is set the z-component of each Ci is flipped.
- Servo angles: after C1–C3 are known, servo elevations follow directly:
  - θ1 = π/2 − atan2(√(C1x² + C1y²) − lb, C1z)
  - θ2 = atan2(C2z, C2x − lb)
  - θ3 mirrors θ1.
- Spherical commands (`solve_inverse_kinematics_spherical`): user inputs θ (tilt) and φ (heading) are converted to a unit vector (a,b,c) = (sin θ cos φ, sin θ sin φ, cos θ). The solver clamps θ to the safe bound `maxtheta`—computed via `max_theta`, which binary-searches the largest tilt that keeps both link-length constraints satisfied to within 1e−3 and all discriminants ≥ 0—before running the full inverse kinematics.

Why this model: The project now relies on the full 3-RRS geometry everywhere, so the “legacy” solver has become the default implementation. Using the exact linkage math prevents infeasible servo commands, keeps the legs parallelogram-constrained, and produces mechanically consistent neutral angles without iterative numerical solvers.

Attitude Transformations
------------------------
Files: `main.py`, `PID.py`

- Offsets and flips: raw pitch/roll from `MPU6050.read()` are debiased via `PITCH_OFFSET`/`ROLL_OFFSET` and optionally negated per axis (to accommodate inverted sensor mounting).
- Rotation or calibration matrix: if at least two `CAL_*` vectors are supplied, the code normalizes the measured direction vectors, builds VᵀV and TᵀV, analytically inverts the 2×2 matrix, and forms a best-fit transform M = (TᵀV)(VᵀV)⁻¹ that maps raw (pitch, roll) to desired directions (default targets 300°, 60°, 180°). Otherwise a simple axis rotation R(α) constructed from `AXIS_ROT_DEG` is applied: (pitch', roll') = R(α) · (pitch, roll).
- Optional EMA/deadband: after PID calculation the script can exponentially smooth the corrections via `OUTPUT_EMA_ALPHA` (ema ← α·ema + (1−α)·corr) and suppress small corrections inside `OUTPUT_DEADBAND` to reduce micro-jitter.

Why these transforms: Mounting tolerances and PCB alignment errors are dominated by linear effects; solving the 2×2 least-squares matrix captures both scale and skew. When that isn’t available, a rotation plus sign flips keep runtime costs negligible while still aligning the IMU frame with the servo frame.

PID Control Laws
----------------
Files: `main.py` (`AxisPID`), `PID.py` (`PIDcontroller`)

Core form: u = kp·e + ki·∫e dt + kd·de/dt.

- `main.py:AxisPID`: Minimal PID per axis with stored integral and derivative state. The derivative term uses the most recent dt (bounded below by 1 ms), and outputs are clamped to ±`max_out` to match `MAX_TILT_DEG`. The pair of controllers produces corrections (corr_x, corr_y) that are subsequently turned into spherical commands: θ = min(max_tilt, √(corr_x² + corr_y²)), φ = (atan2(corr_y, corr_x) in degrees + 360) mod 360.
- `PID.py:PIDcontroller`: PID per axis plus exponential smoothing of the PID output: filtered = α·pid + (1−α)·prev. Magnitude is mapped either linearly (β·r) or through tanh(β·r) to soften large commands; φ = atan2(y,x) normalized to [0,360).

Why PID: The plant is slow and approximately linear near upright, so PID suffices without model-based control. Output clamps and saturation-aware integral prevent windup during large disturbances; the optional tanh mapping reduces aggressive swings for large errors without retuning gains.

IMU Sensor Fusion
-----------------
File: `imu.py`

- Gyro bias: average N samples of raw rates (converted by gyro_scale) to estimate bias terms (gyro_bias_x/gyro_bias_y).
- Time step: dt = max(1e−3, t_now − t_prev) to avoid spikes.
- Accelerometer angles: pitch_acc = atan2(ax, √(ay² + az²)), roll_acc = atan2(ay, √(ax² + az²)) (degrees).
- Gyro integration: pitch_gyro = pitch_prev + gy·dt; roll_gyro = roll_prev + gx·dt after bias removal.
- Complementary filter: pitch = α_pitch·pitch_gyro + (1 − α_pitch)·pitch_acc; roll similarly with α_roll.

Why complementary filtering: It combines low-frequency stability of the accelerometer (less drift) with high-frequency responsiveness of the gyro (less noise) at negligible cost, suitable for microcontroller-class hardware. Bias calibration up front reduces steady-state drift without needing a full Kalman filter.

Servo Limits and Neutral Blending
---------------------------------
File: `controller.py`

- Angle clamp: `clamp(value, 19°, 90°)` ensures commanded servo angles stay within safe mechanical limits before applying any offsets (s1 subtracts an additional 4° after clamping to match its horn orientation).
- Neutral motions: `initialize()` drives the robot to a known neutral pose (54° per servo → down to 19° → flat pose solved by `Goto_time_spherical`) so that offsets and kinematic assumptions all start from the same reference.
- Per-servo offsets: once manually calibrated, small constant adjustments inside `set_motor_angles` keep the plate level without altering the kinematics.

Why these safeguards: Hard limits protect hardware, the neutral sequence ensures each boot starts from a reproducible pose, and static offsets correct build tolerances without introducing bias in the math stack.
