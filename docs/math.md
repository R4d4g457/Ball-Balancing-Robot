Ball-Balancing Robot – Mathematical Overview
===========================================

This document summarizes the equations used across the project and explains why each approach was chosen over alternatives. File paths are workspace-relative.

Coordinate Frames and Symbols
-----------------------------
- Platform attitude is represented by a unit normal vector **n** = (nx, ny, nz). For small tilts, nx ≈ pitch, ny ≈ roll in radians.
- Servo angles are measured from the vertical axis; θi denotes the angle of leg i away from vertical.
- Base joints sit at 0°, 120°, and 240° around the z-axis; bx/by denote their unit circle coordinates for each leg.
- Length parameters (legacy model): lp (top triangle half-edge), l1 (upper link), l2 (lower link), lb (base triangle half-edge), h (platform height).

Kinematics (Simplified Approximation)
-------------------------------------
File: `robotKinematics.py`

Forward kinematics: estimate the platform normal from servo angles by averaging leg directions:
- Convert servo angles: ti = radians(θi).
- Average leg direction projections: nx = (sin t1 + sin t2 + sin t3) / 3, ny = (cos t1 + cos t2 + cos t3) / 3.
- Enforce unit length: nz = sqrt(max(0, 1 − nx² − ny²)).

Inverse kinematics: map a desired normal to servo angles with a closed form per leg:
- For each base angle βi ∈ {0°,120°,240°}, bx = cos βi, by = sin βi.
- Compute cos θi = (R1 − R2 (bx·nx + by·ny)) / (L·nz), clamp to [−1,1], then θi = acos(cos θi).

Why this model: It trades exact linkage geometry for speed and numerical stability on-device. The approximation treats each leg as a spherical joint with an effective lever arm (R1,R2,L), avoiding iterative solvers and coping better with IMU noise. When higher fidelity is needed, the legacy solver is used instead.

Tilt command to servos: `tilt_to_servos(tilt_x_deg, tilt_y_deg)` converts pitch/roll (deg) into a unit normal via nx = sin(tx), ny = sin(ty), nz = sqrt(1 − nx² − ny²) with a floor to avoid divide-by-zero, then calls inverse kinematics. The small-angle sin() mapping keeps tilt commands linear near zero while respecting the unit-normal constraint.

Kinematics (Full 3-RRS Geometry)
--------------------------------
File: `robotKinematics_legacy.py`

Height limits: derived by Pythagoras to find feasible vertical travel:
- maxh = sqrt((l1 + l2)² − (lp − lb)²)
- minh = piecewise sqrt of remaining triangle after folding one link over the other.

Platform pose (solve_top): given unit normal (a,b,c) and height h, computes top triangle points A1–A3 by projecting the normal onto each edge direction; expressions use closed forms to avoid trigonometric iteration.

Middle joints (solve_middle): solves for C1–C3 by intersecting circles/lines from each upper and lower link:
- Intermediate variables p,q,r,s,t encode the quadratic solution coefficients; discriminants (s² − 4rt) are checked for feasibility.
- Solutions select the physically reachable branch (sign choices); invert mode flips z to model mirrored assembly.

Servo angles: θ1 = π/2 − atan2(√(C1x² + C1y²) − lb, C1z); θ2 = atan2(C2z, C2x − lb); θ3 symmetric to θ1. These convert 3D joint positions into required servo elevations.

Maximum tilt search (max_theta): binary-searches θ that keeps all quadratic discriminants non-negative and link lengths satisfied at a given h. Validity checks ensure both links match their prescribed lengths to within tolerance.

Why this model: It captures the true parallelogram constraints of the 3-RRS mechanism, ensuring collision-free, feasible poses. A binary search over tilt avoids singularities and provides a safe envelope for UI sliders and IMU-driven commands.

Attitude Transformations
------------------------
Files: `controller.py`, `main_legacy_imu.py`, `PID.py`

- Axis rotation: (pitch', roll') = R(α) · (pitch, roll), where R is a 2×2 rotation by `axis_rotation_deg` to align IMU axes with hardware geometry.
- Inversions: optional sign flips per axis to handle sensor mounting differences.
- Gains and offsets: affine scaling and subtraction of static biases before control.
- Calibration matrix (legacy IMU mode): solves for a best-fit 2×2 matrix M using at least two measured direction vectors v and desired targets t. It forms VᵀV and TᵀV, inverts the 2×2 VᵀV analytically, and computes M = (TᵀV)(VᵀV)⁻¹. This linear least-squares fit compensates for axis skew and scale mismatch beyond a simple rotation.

Why these transforms: Simple rotation/flip covers most mounting errors with low latency. The optional least-squares matrix offers a more expressive calibration without resorting to nonlinear optimization, keeping runtime negligible on the Pi.

PID Control Laws
----------------
Files: `controller.py` (PID), `main_legacy_imu.py` (AxisPID), `PID.py` (PIDcontroller)

Core form: u = kp·e + ki·∫e dt + kd·de/dt.

- `controller.py:PID`: Adds integral decay (integral ← integral·(1 − decay·dt)) to bleed bias, clamps output to ±max_out, and applies basic anti-windup by accepting integral updates only when unsaturated. Two independent controllers regulate pitch and roll corrections.
- `main_legacy_imu.py:AxisPID`: Minimal PID with clamped output for each axis; corrections (corr_x, corr_y) are converted to spherical tilt: θ = min(max_tilt, √(corr_x² + corr_y²)), φ = atan2(corr_y, corr_x) wrapped to [0,360). This isolates magnitude and heading for the kinematics solver.
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
Files: `controller.py`, `controller_legacy.py`

- Angle clamp: clamp(value, 19°, 89°) (legacy up to 90°) keeps motion within safe mechanical limits.
- Neutral blending: when `neutral_blend < 1`, commanded angle = neutral + (command − neutral)·blend; useful to soften motion and reuse a calibrated neutral pose.
- Per-servo offsets: add small biases after clamping/blending to mechanically level the plate.

Why these safeguards: Hard limits protect hardware; blending offers a smooth tuning knob between a safe neutral and full-range commands; offsets correct assembly tolerances without altering the core kinematics.
