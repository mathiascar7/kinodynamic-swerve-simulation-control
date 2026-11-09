# Stochastic Swerve Navigation and Disturbance-Rejection Simulation

A simulation and validation workspace for autonomous swerve-drive navigation, calibrated field-obstacle mapping, and local disturbance rejection without continuous access to the physical robot.

## 1. Project Origin & Core Problem

Autonomous navigation development depends on repeatable access to a complete robot. That access is not always available during development, making it difficult to model and validate trajectory tracking, obstacle geometry, and responses to impacts or wheel slip before deploying to hardware.

This project addresses that constraint with a virtual workflow for field mapping, trajectory simulation, and controller stress testing. **AdvantageScope** is the primary environment for 3D simulation, `Field2d` trajectory visualization, and high-frequency closed-loop telemetry analysis.

The figures and recordings below are embedded in this page so the workflow can be followed without leaving the README.

## 2. System Architecture

The system is organized around four interacting modules. The navigation and controller architecture described here is under active development.

### Vision-based obstacle mapping

A Python pipeline using OpenCV processes a field image, extracts obstacle contours, calibrates image scale against known obstacle geometry, and exports obstacle boundaries as line segments in `map_obstacles_calibrated.txt`. The map is used by the simulation to represent field obstacles.

For a detected contour with pixel width \(w_p\) and height \(h_p\), and a matching reference obstacle of physical dimensions \(w_m\) and \(h_m\), the uniform image scale is computed as:

$$
s = \frac{1}{2}\left(\frac{w_m}{w_p}+\frac{h_m}{h_p}\right)
\quad [\mathrm{m/pixel}]
$$

Pixel coordinates are converted to field coordinates by scaling and reversing the image's vertical axis:

$$
x_m = s x_p,\qquad y_m = s(H_p-y_p)
$$

where \(H_p\) is the cropped field image height. A non-vertical segment between \((x_1,y_1)\) and \((x_2,y_2)\) is represented by:

$$
y=mx+b,\qquad
m=\frac{y_2-y_1}{x_2-x_1},\qquad b=y_1-mx_1
$$

Vertical and horizontal segments are represented directly rather than dividing by zero.

#### Mapping workflow

1. **Inspect the field image and identify obstacle colors.** The notebook uses OpenCV to isolate selected color regions and detect obstacle contours.

   ![OpenCV color selection on the field image](Mapping%20Vision%20part%201.png)

2. **Calibrate from a known obstacle.** Select the reference obstacle and confirm it with `P`; the notebook compares its geometric proportions with the obstacle data and computes a uniform meters-per-pixel scale. If there are multiple geometrically similar candidates, the notebook asks for the field/alliance selection (`B` or `R`) to disambiguate them.

   ![Obstacle reference and scale calibration](Mapping%20Vision%20part%202.png)

3. **Map additional obstacles.** Select another obstacle of the calibrated type and press `P` to extract and convert its perimeter to field coordinates.

   ![Obstacle contour extraction and mapping](Mapping%20Vision%20part%203.png)

4. **Review the generated geometry.** The calibrated line-segment records are written to the obstacle map file. The repository includes examples of the resulting field perimeter:

   ![Rebuilt field obstacle perimeter, view 1](FRC%20Rebuilt%20Obstacle%20map%20perimeter%20%23%201.png)

   ![Rebuilt field obstacle perimeter, view 2](FRC%20Rebuilt%20Obstacle%20map%20perimeter%20%232.png)

   ![Rebuilt field obstacle perimeter, view 3](FRC%20Rebuilt%20Obstacle%20map%20perimeter%20%233.png)

   ![Rebuilt field obstacle perimeter, view 4](FRC%20rebuilt%20Obstacle%20map%20perimeter%20%23%204.png)

5. **Load the map in simulation.** The notebook writes its mapped geometry to `map_obstacles_2.txt` beside the input map. The Java source's default path is `C:\Downloads\map_obstacles_calibrated.txt`; point it to the generated file (or copy the reviewed output to the expected path) before starting the simulation.

### Local disturbance-rejection control

The robot-side Java/WPILib local control loop is designed for \(O(1)\) work per update at a fixed four-wheel count. It targets rapid rejection of impact disturbances and wheel-traction failures (slip), while accounting for dynamic motor load transfer.

### Global field-centric localization and guidance

A predictive inertial **Ghost Follower** operates in \(SE(2)\), comparing the robot pose with a Bézier reference path and smoothly realigning the robot's simulated field pose. The field-frame position and heading error can be expressed as:

$$
\mathbf{e}_{xy} =
\begin{bmatrix}x_r-x\\y_r-y\end{bmatrix},
\qquad
e_\theta=\operatorname{wrap}_{[-\pi,\pi)}(\theta_r-\theta)
$$

where \((x,y,\theta)\) is the robot pose and \((x_r,y_r,\theta_r)\) is the reference pose. A cubic Bézier segment is parameterized by:

$$
\mathbf{B}(u)=(1-u)^3\mathbf{P}_0
+3(1-u)^2u\mathbf{P}_1
+3(1-u)u^2\mathbf{P}_2
+u^3\mathbf{P}_3,\qquad 0\leq u\leq1
$$

### Dynamic simulation

A native numerical simulator models the robot's motion and supports closed-loop trajectory and disturbance experiments. Its integration method and load-transfer model are described below.

## 3. Physics & Simulation Engine

The dynamic plant is advanced with the classical fourth-order Runge–Kutta method (RK4). For state \(\mathbf{x}\), dynamics \(\dot{\mathbf{x}}=f(t,\mathbf{x})\), and simulation step \(h\):

$$
\begin{aligned}
k_1 &= f(t_n,\mathbf{x}_n)\\
k_2 &= f\left(t_n+\frac{h}{2},\mathbf{x}_n+\frac{h}{2}k_1\right)\\
k_3 &= f\left(t_n+\frac{h}{2},\mathbf{x}_n+\frac{h}{2}k_2\right)\\
k_4 &= f(t_n+h,\mathbf{x}_n+hk_3)\\
\mathbf{x}_{n+1} &= \mathbf{x}_n+\frac{h}{6}(k_1+2k_2+2k_3+k_4)
\end{aligned}
$$

The controller's force limits account for acceleration-induced load transfer. For mass \(m\), gravity \(g\), center-of-gravity height \(h_c\), longitudinal and lateral accelerations \(a_x,a_y\), and geometry parameters \(L,W\), the model uses:

$$
N_0=\frac{mg}{4},\qquad
\Delta_x=\frac{ma_xh_c}{2L},\qquad
\Delta_y=\frac{ma_yh_c}{2W}
$$

The four corner loads are computed in the model's corner order as:

$$
\begin{bmatrix}N_1\\N_2\\N_3\\N_4\end{bmatrix}
=
\begin{bmatrix}
\max(1\,\mathrm{N},N_0-\Delta_x+\Delta_y)\\
\max(1\,\mathrm{N},N_0-\Delta_x-\Delta_y)\\
\max(1\,\mathrm{N},N_0+\Delta_x+\Delta_y)\\
\max(1\,\mathrm{N},N_0+\Delta_x-\Delta_y)
\end{bmatrix}
$$

Each wheel's ground-force magnitude is then limited by its Coulomb friction bound:

$$
\|\mathbf{F}_i\|_2 \leq \mu N_i
$$

where \(\mu\) is the friction coefficient. This makes the traction limit responsive to the simulated dynamic weight distribution rather than treating all four wheels as equally loaded under acceleration.

## 4. Automated Stochastic Benchmarking

The simulation includes an automated stochastic test loop for stressing trajectory tracking with injected mechanical impacts and measuring the resulting path deviation. In the WPILib simulation, use the configured **C** control to start or stop the randomized test loop; the current Java code maps this control to raw joystick button 3.

The companion Monte Carlo benchmark runs \(N=50\) trials and reports peak impact drift, full-trajectory RMSE, controller latency, and false-positive response under wheel slip. The notebook also compares the proposed \(O(1)\) controller with an ADMM constrained-QP baseline. Reported values (mean \(\pm\) 95% confidence interval) are:

| Metric | \(O(1)\) controller | ADMM QP baseline |
|---|---:|---:|
| Mean computation latency (\(\mu s\)) | \(41.11 \pm 1.25\) | \(20.17 \pm 0.57\) |
| Peak impact spatial drift (mm) | \(33.19 \pm 0.46\) | \(40.90 \pm 0.22\) |
| Full-trajectory RMSE (mm) | \(18.22 \pm 0.23\) | \(27.96 \pm 0.08\) |
| False-positive rate under wheel slip (%) | \(10.64 \pm 1.21\) | \(100.00 \pm 0.00\) |

These are the results recorded by the benchmark notebook; they are measurements from the described simulation, not hardware test results.

![Monte Carlo validation run, console output](Monte%20Carlo%20validation%20Part%201.png)

![Monte Carlo controller benchmark results](Monte%20Carlo%20Validation%20Part%202.png)

### Kinematic Tester and simulation videos

The Kinematic Tester is the Java/WPILib simulation entry point included in this repository. It combines the local \(O(1)\) controller, the Ghost Follower, simulated field/obstacle geometry, impact injection, and telemetry output. Start the WPILib simulation, connect AdvantageScope to the telemetry source configured by the host robot project, and use the configured **C** control (raw joystick button 3 in the current Java source) to toggle the stochastic test loop. The displayed field, ghost pose, robot pose, and impact markers support visual inspection while telemetry is recorded.

![AdvantageScope field and simulation telemetry](Different%20Field.png)

The following recordings are embedded here. If a browser or GitHub renderer does not play repository-hosted MP4 inline, use the video link directly below the player.

#### Kinematic Tester

<video controls preload="metadata" width="100%" src="./Kinematic%20Tester%20Video.mp4">
  Your browser does not support embedded video. [Open the Kinematic Tester video](Kinematic%20Tester%20Video.mp4).
</video>

[Open the Kinematic Tester video](Kinematic%20Tester%20Video.mp4)

#### Collision and impact testing

<video controls preload="metadata" width="100%" src="./Collisions%20Testing.mp4">
  Your browser does not support embedded video. [Open the collision testing video](Collisions%20Testing.mp4).
</video>

[Open the collision testing video](Collisions%20Testing.mp4)

### End-to-end workflow

1. Prepare a field image and the reference obstacle geometry.
2. Run the OpenCV notebook to select colors, calibrate the image, map obstacle perimeters, and inspect the generated line records.
3. Point the Java simulation at the calibrated obstacle map.
4. Run the Java source inside a WPILib/GradleRIO robot project and connect AdvantageScope to that project's telemetry.
5. Exercise the path and local controller in simulation; toggle the randomized impact loop with the configured **C** control.
6. Run the companion Monte Carlo benchmark notebook for the \(N=50\) randomized comparison and inspect its reported metrics and plots.

## 5. Setup & Installation

### Clone the repository

```bash
git clone <repository-url>
cd <repository-directory>
```

### Run the obstacle-mapping notebook

The mapping workflow is provided as a Jupyter notebook, not as a standalone Python script. It imports OpenCV and NumPy; install them with Jupyter in the Python environment you intend to use:

```bash
python -m pip install jupyter opencv-python numpy
jupyter notebook
```

Open [Python opencv obstacle mapping.ipynb](Python%20opencv%20obstacle%20mapping.ipynb), configure its image, field-data, and obstacle-map paths for your local inputs, and run the notebook cells. The notebook's path examples are machine-specific; update them before running. Review the calibrated obstacle output before using it in simulation.

### Run the WPILib simulation

The repository contains Java source, but does not include a complete GradleRIO/WPILib robot project or Gradle wrapper. To run it:

1. Open or create the WPILib robot project that will host the simulation.
2. Add and integrate the repository's Java simulation/control source into that project.
3. From the **WPILib project root**, launch the simulation with the project's Gradle wrapper:

   ```powershell
   .\gradlew.bat simulateJava
   ```

4. Open AdvantageScope and connect it to the telemetry source configured by the WPILib project to inspect the 2D/3D field view and telemetry.

This repository alone is not a directly runnable WPILib project; its Java source requires the surrounding WPILib/GradleRIO project configuration.
