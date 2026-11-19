# Kinodynamic Swerve Simulation and Control

This repository contains Python notebooks for image-based obstacle mapping and a simulated force-allocation benchmark, plus Java/WPILib source for geometric path planning, trajectory tracking, disturbance injection, and visualization. It is an engineering simulation workspace, not a complete deployable robot project.

> **Validation scope:** No result in this repository is a measurement from a physical robot. The Monte Carlo figures below are outputs saved by the Python 8-state RK4 benchmark. The Java `Kinematic Tester` is a separate WPILib simulation harness whose `Field2d`/NetworkTables telemetry can be inspected with AdvantageScope; it does not run that RK4 benchmark or generate the reported table.

## 1. Engineering problem

The implementation addresses four development constraints:

1. **Limited access to the physical robot.** Mapping, route generation, impact response, and collision handling need repeatable simulation exercises before hardware deployment.
2. **Ambiguous disturbance signals.** A single-wheel traction-loss event can resemble an external disturbance in a torque/current-derived signal. The benchmark therefore tests a temporal-consistency rule for distinguishing its synthetic slip and impact events. It does not implement or compare a conventional disturbance observer (DOB) against hardware sensor data.
3. **Allocation cost and actuator limits.** A convergence-based QP allocator has data-dependent iteration count. The notebook compares it with a fixed-four-wheel allocation routine under the benchmark's motor-voltage, current, and friction limits. The timing comparison is specific to these Python implementations and this machine; it is not a real-time guarantee.
4. **Map geometry errors.** Pixel contours and coordinate conversion can yield offset or fragmented obstacle boundaries. The mapping and interactive calibration tools support visual review and limited repair before the geometry is consumed by collision and planning code.

## 2. Architecture and implemented methods

```mermaid
flowchart LR
    IMG[Field image and obstacle reference data] --> CV[OpenCV mapping notebook]
    CV --> MAP[Line-segment map]
    MAP --> GUI[Pygame interactive calibration]
    GUI --> JAVA[Java obstacle loader]
    JAVA --> PLAN[Expanded obstacle boxes and orthogonal visibility graph]
    PLAN --> PATH[Dijkstra, path cleanup, quadratic Bezier corners]
    PATH --> PROFILE[Curvature and acceleration speed profile]
    PROFILE --> FOLLOW[Local-inertia Ghost Follower]
    FOLLOW --> WPI[WPILib simulation harness]
    WPI --> NT[Field2d and NetworkTables]
    NT --> AS[AdvantageScope visualization]

    subgraph Offline benchmark
        MC[Seeded scenarios] --> RK[Python 8-state, 1 ms RK4 plant]
        RK --> O1[Fixed-four-wheel O(1) allocator]
        RK --> QP[ADMM constrained-QP baseline]
        O1 --> OUT[Scenario-specific metrics]
        QP --> OUT
    end
```

The two simulation paths are separate: the Python RK4 benchmark evaluates force allocation on a translational plant, while the Java/WPILib program exercises map-based route planning and visual telemetry.

### Module A — Vision mapping and interactive geometry calibration

`Python opencv obstacle mapping.ipynb` uses mouse-selected BGR color samples to segment obstacles. It crops the field using an HSV threshold for the bright field boundary, applies a 5 × 5 morphological opening and 45 × 45 closing to the sampled-color mask, filters small contours, and approximates the remaining contours as polygons. A selected reference obstacle is matched to obstacle records using geometric proportions. If symmetric candidates remain, the operator can select the alliance side (`B` or `R`).

For reference width and height in meters \(w_m,h_m\), and detected pixel dimensions \(w_p,h_p\), the notebook uses the uniform scale

$$
s = \frac{1}{2}\left(\frac{w_m}{w_p}+\frac{h_m}{h_p}\right)
\quad [\mathrm{m/pixel}].
$$

It maps the cropped image coordinates to meters as \(x_m=sx_p\) and \(y_m=s(H_p-y_p)\), reversing the image's vertical axis. Each polygon edge is written as a line-segment record. The notebook clones the selected input map to `map_obstacles_2.txt` before appending mapped segments; inspect this file before using it.

#### Interactive final calibration

`Obstacle final calibration code.ipynb` provides a separate Pygame pass for residual group offsets and small gaps between mapped line fragments:

- A click samples nearby pixels of similar color (within a 90-pixel neighborhood). The tool finds the nearest mapped segment, identifies its obstacle group, compares the sampled-color centroid with the rendered group center, converts that pixel delta to meters using the detected image bounds and 16.54 m × 8.07 m field dimensions, and translates the group's segment coordinates.
- This is a centroid-based, image-resolution-dependent translation, not a guarantee of sub-pixel accuracy.
- Press **R** to run `repair_line_continuity`. It considers only horizontal and vertical segments within the same group and adjusts nearby perpendicular or collinear endpoints using a 0.50 m tolerance. Diagonal segments are not modified. This heuristic does not guarantee that every outline is a closed polygon; inspect the result.
- Press **S** or click **SAVE [S]** to write the result. The tool normally creates a sibling file ending in `_calibrated.txt`; if it cannot create that copy, its fallback is to use the selected input file itself.

![Interactive obstacle calibration GUI](Obstacle%20Calibration%20GUI.png)

![Earlier final obstacle calibration view](Final%20obstacle%20calibration%20tweaks.png)

The calibration and mapping stages are illustrated below.

![OpenCV color selection on the field image](Mapping%20Vision%20part%201.png)

![Obstacle reference and scale calibration](Mapping%20Vision%20part%202.png)

![Obstacle contour extraction and mapping](Mapping%20Vision%20part%203.png)

![Rebuilt field obstacle perimeter, view 1](FRC%20Rebuilt%20Obstacle%20map%20perimeter%20%23%201.png)

![Rebuilt field obstacle perimeter, view 2](FRC%20Rebuilt%20Obstacle%20map%20perimeter%20%232.png)

![Rebuilt field obstacle perimeter, view 3](FRC%20Rebuilt%20Obstacle%20map%20perimeter%20%233.png)

![Rebuilt field obstacle perimeter, view 4](FRC%20rebuilt%20Obstacle%20map%20perimeter%20%23%204.png)

### Module B — Geometric route planning and kinematic profiling (Java/WPILib)

`Kinematic Tester.java` loads grouped line records and obstacle bounds, then plans between waypoints:

1. **Obstacle expansion and route graph.** Each obstacle box is expanded by half the robot's diagonal, \(\sqrt{L_r^2+W_r^2}/2\), plus the configured safety margin. Candidate x/y coordinates around expanded boxes form an orthogonal visibility graph. Adjacent nodes on each row or column are connected only when the segment is obstacle-free. Dijkstra's algorithm returns the shortest graph path. The graph search is not claimed to be \(O(N)\).
2. **Path cleanup and corner smoothing.** Duplicate nodes are removed. A micro-step shorter than 0.18 m is removed only when the direct replacement segment is clear; segment-versus-box visibility uses Liang–Barsky clipping. Turns are trimmed and sampled as quadratic Bézier curves (10 subdivisions per curve, with the trim distance capped at 0.55 m).
3. **Speed and heading profile.** `perfilamientoFisicoON` runs in \(O(N)\) over the sampled path. It applies a 2.8 m/s speed cap, a 4.0 m/s² acceleration/deceleration forward-backward pass, and a discrete curvature limit \(\sqrt{0.95g/\kappa}\) when \(\kappa>0.1\ \mathrm{m}^{-1}\). The 0.95 factor is a fixed implementation constant, not a dynamically estimated tire coefficient. Heading follows the shortest angular change linearly with cumulative path distance. The resulting poses receive a robot-footprint collision check.

### Module C — Local-inertia follower and impact/slip logic

The Java `LocalInertiaGhostFollower` estimates a stopping point from the previous commanded velocity, using \(a_{\max}=9\ \mathrm{m/s^2}\) (and a separate angular limit). It compares this point with the current reference sample and scales the reference clock by \(\max(0,1-1.5\|e\|)\). It combines attenuated tangent feedforward with position feedback and tracks the reference heading. `GhostPose` and `StoppingPoint` are published for visualization. This is the implemented heuristic; it is not a general stability proof.

The Python benchmark implements a distinct fixed-four-wheel force allocator. It distributes planar force according to \(w_i=N_i/\sum_k N_k\), projects each wheel force onto \(\|\mathbf{F}_i\|_2\leq\mu N_i\), and applies a motor limit based on bus voltage, back-EMF, and a 120 A current ceiling. For a fixed four-wheel platform its loops have a fixed number of iterations and do not iterate to solver convergence.

The benchmark's causal filter retro-projects a torque-change time:

$$
t_{0,i}=t-\frac{|\tau_i|}{|d\tau_i/dt|}.
$$

It flags an event when at least three modules are active and the standard deviation of the four projected times is at most 0.8 ms. The benchmark's activity threshold is \(|d\tau_i/dt|>80\ \mathrm{N\,m/s}\); inactive modules are assigned the current time. Inputs are synthetic torque measurements with noise and a modeled single-wheel torque drop during slip, not readings from physical current or acoustic sensors. The benchmark tests this defined scenario; it does not establish a general classifier for all slip and impact events.

The Java simulation contains a separate simplified filter: its derivative threshold is 120, and a detected event adjusts only the forward command, capped at ±0.8 m/s. It does not run the Python force-allocation routine.

## 3. Simulation and benchmark results

### Python 8-state RK4 benchmark

The Monte Carlo notebook uses

$$
\mathbf{x}=[x,\ y,\ v_x,\ v_y,\ \omega_{w,0},\ \omega_{w,1},\ \omega_{w,2},\ \omega_{w,3}]
$$

with a 1 ms step and classical fourth-order Runge–Kutta integration. The state contains planar translation and four wheel angular speeds; it has no chassis heading/yaw state. The chassis translation includes viscous damping \(-b\mathbf{v}\), with \(b=35\ \mathrm{N\,s/m}\). The actuator model limits current to 120 A and reduces available torque with back-EMF. The benchmark also introduces a one-step force-command transmission delay.

Each 3.5 s trial uses a 70 kg nominal mass varied uniformly by ±5%, a 320–380 N impact at 85–95 degrees from 1.80–1.83 s, and a front-left-wheel slip interval from 0.80–0.85 s. The torque signal includes 0.15 N·m Gaussian noise. A fixed NumPy random seed (`42`) makes the scenario sequence deterministic, although wall-clock timings can vary by machine and runtime.

The saved notebook output for \(N=50\) trials reports mean ± its normal-approximation 95% confidence interval (computed as \(1.96s/\sqrt{N}\)):

| Metric as evaluated by the notebook | Fixed-four-wheel O(1) | ADMM constrained-QP |
|---|---:|---:|
| Mean allocation latency (µs) | 41.11 ± 1.25 | 20.17 ± 0.57 |
| Maximum \(|y|\) in impact window, 1.80–2.10 s (mm) | 33.19 ± 0.46 | 40.90 ± 0.22 |
| Full-trial lateral \(y\) RMSE over 3.5 s (mm) | 18.22 ± 0.23 | 27.96 ± 0.08 |
| Slip-window false-positive rate (%) | 10.64 ± 1.21 | 100.00 ± 0.00 |

The notebook labels the third metric “Full Trajectory RMSE,” but computes \(\sqrt{\mathrm{mean}(y^2)}\); it is a one-axis lateral-position RMSE, not a 2D path RMSE. The slip false-positive definitions are also implementation-specific: the O(1) column counts causal-filter triggers, while the QP column counts a lateral allocated-force condition above 20 N. These figures should not be generalized beyond this simulated scenario.

In this scenario, the O(1) implementation reports 18.8% lower peak lateral drift and 34.8% lower lateral RMSE than the QP baseline, while its slip-window false-positive rate is 89.36 percentage points lower. These are descriptive, scenario-specific differences in the notebook's generated trials, not estimates of hardware performance.

The O(1) Python routine's measured mean latency is higher than this QP baseline. The routine uses Python-level loops over four modules; the baseline uses a pre-factored small matrix and NumPy operations, and performs up to 40 ADMM iterations with an early residual stop. That implementation-level difference is consistent with the measured result; asymptotic complexity alone does not determine Python wall-clock latency. At the reported means, both values are below a 1 ms period (about 24× and 50×, respectively), but this is a notebook microbenchmark, not a worst-case execution-time or hardware real-time result.

![Monte Carlo validation run, console output](Monte%20Carlo%20validation%20Part%201.png)

![Monte Carlo controller benchmark results](Monte%20Carlo%20Validation%20Part%202.png)

### Separate Java/WPILib simulation and AdvantageScope telemetry

The Java `Kinematic Tester` uses WPILib's `simulationPeriodic` loop and a simplified command-plus-disturbance pose model; it is not the 8-state RK4 plant above. Its elapsed step is taken from FPGA time, with a 20 ms fallback for invalid intervals. Injected disturbances decay exponentially and are integrated into the simulated pose. No published Monte Carlo metrics in this README are calculated from this Java harness.

Raw joystick button **3** (`C`) toggles randomized routes. The code samples start/end poses with randomized headings, attempts route planning, and injects three external impacts at approximately 15%, 50%, and 80% route progress. During an injected external impact, the simulation bypasses its wall-collision stop branch; this prevents the scripted event itself from being treated as an obstacle collision, but is not evidence of hardware impact recovery. Telemetry includes the robot pose, estimated poses, trajectory, `GhostPose`, `StoppingPoint`, and impact markers.

![AdvantageScope field and simulation telemetry](Different%20Field.png)

#### Kinematic Tester video

<video controls preload="metadata" width="100%" src="./Kinematic%20Tester%20Video.mp4">
  Your browser does not support embedded video. [Open the Kinematic Tester video](Kinematic%20Tester%20Video.mp4).
</video>

[Open the Kinematic Tester video](Kinematic%20Tester%20Video.mp4)

#### Collision and impact testing video

<video controls preload="metadata" width="100%" src="./Collisions%20Testing.mp4">
  Your browser does not support embedded video. [Open the collision testing video](Collisions%20Testing.mp4).
</video>

[Open the collision testing video](Collisions%20Testing.mp4)

## 4. Repository structure and reproducibility

### Repository contents

| File | Purpose |
|---|---|
| `Python opencv obstacle mapping.ipynb` | OpenCV field cropping, sampled-color contour mapping, scale calibration, and line-segment export. |
| `Obstacle final calibration code.ipynb` | Pygame group-offset alignment and optional continuity repair. |
| `Deterministic O(1) Kinodynamic Force Allocation.ipynb` | Python RK4 model, O(1) allocation, ADMM baseline, Monte Carlo run, plots, and saved results. |
| `Kinematic Tester.java` | WPILib route planning, local follower, simplified impact simulation, collision checks, and telemetry. Integrate/rename as `Robot.java` in a compatible host project. |
| `Advantage Scope Obstacle Mapping.java` | Additional Java field/obstacle mapping source. |
| `map_obstacles_calibrated.txt` and `map_obstacles_calibrated.txt` | Example obstacle-map records. Review map paths and field labels before use. |
| `Mapping Vision part 1.png`–`Mapping Vision part 3.png` | Mapping workflow figures. |
| `FRC Rebuilt Obstacle map perimeter # 1.png`–`# 4.png` | Rendered perimeter examples. |
| `Obstacle Calibration GUI.png` | Interactive calibration screenshot. |
| `Monte Carlo validation Part 1.png`, `Monte Carlo Validation Part 2.png` | Saved benchmark output and plots. |
| `Different Field.png` | Field/telemetry visualization. |
| `Kinematic Tester Video.mp4`, `Collisions Testing.mp4` | Simulation recordings. |

### Reproducible setup

#### Python notebooks

From the project directory, install the notebook dependencies:

```bash
python -m pip install jupyter opencv-python numpy pygame pandas matplotlib
jupyter notebook
```

`tqdm` is optional; the benchmark notebook includes a fallback progress display.

1. Open `Python opencv obstacle mapping.ipynb`. Update its Windows input paths near the final cell (`img_path`, `nav_path`, and `txt_path`). Select obstacle colors, press **P**, select a reference obstacle and press **P** again, resolve an ambiguous alliance match with **B** or **R**, then select further obstacles and press **P** to map them. Press **Esc** when finished. Review `map_obstacles_2.txt`.
2. Open `Obstacle final calibration code.ipynb` and run its cell. Select the image folder and obstacle text file in the GUI, check the competition year and image name settings in the source/configuration, then load the map. Click obstacle groups to align them; press **R** only when a continuity repair is appropriate, then press **S** and inspect the saved file. Back up the input map before calibration, because the tool can fall back to writing to it if it cannot create the calibrated copy.
3. Run all cells in `Deterministic O(1) Kinodynamic Force Allocation.ipynb` to reproduce the 50-trial benchmark and plots. The measured timings depend on the host computer.

#### WPILib and AdvantageScope

The repository does not include a GradleRIO project, wrapper, or complete robot deployment configuration. In a compatible WPILib project, integrate the Java source under `frc.robot` (the public `Robot` class must be in `Robot.java`), configure the obstacle path in `DEFAULT_OBSTACLES_PATH`, and launch the host project's simulation:

```powershell
.\gradlew.bat simulateJava
```

Connect AdvantageScope to the NetworkTables instance configured by the host project to view the field and telemetry. The Java simulation and the Python RK4 benchmark are separate validation tools; neither substitutes for testing on a physical chassis.
