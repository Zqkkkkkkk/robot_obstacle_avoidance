# Robot Obstacle Avoidance

This repository is a standalone Python project for robot obstacle avoidance research and coursework. It is structured as a small simulation lab rather than a one-off script, so you can extend it into a graduation project, benchmark multiple methods, and export reproducible experiment results.

## Included methods

- A* grid search for global path planning
- Artificial Potential Field (APF) for continuous local obstacle avoidance
- Dynamic Window Approach (DWA) for velocity-space trajectory optimization
- Dynamic obstacle playback and scenario benchmarking for motion-aware evaluation

These methods cover three common mathematical perspectives:

- Graph search: minimize cumulative path cost
- Potential theory: combine attractive and repulsive fields
- Constrained optimization in control space: evaluate reachable velocity commands over a short horizon

## Core models

The repository uses a 2D planar mobile robot with a unicycle motion model:

```text
x_{k+1} = x_k + v cos(theta_k) dt
y_{k+1} = y_k + v sin(theta_k) dt
theta_{k+1} = theta_k + omega dt
```

### A* cost model

```text
f(n) = g(n) + h(n)
```

- `g(n)`: accumulated travel cost from the start node
- `h(n)`: Euclidean heuristic to the goal

### Artificial Potential Field

```text
F(q) = F_att(q) + sum(F_rep_i(q))
F_att(q) = k_att (q_goal - q)
F_rep(q) = k_rep (1/d(q) - 1/Q*) / d(q)^2
```

The repulsive term is only active inside the obstacle influence radius `Q*`.

### Dynamic Window Approach

DWA samples admissible velocity commands inside the dynamic window:

```text
V_d = [v - a_max dt, v + a_max dt] x [omega - alpha_max dt, omega + alpha_max dt]
```

Each candidate trajectory is scored using heading quality, clearance, and forward speed.

## Repository layout

```text
robot_obstacle_avoidance/
├── pyproject.toml
├── README.md
├── requirements.txt
├── examples/
├── src/
│   └── robot_obstacle_avoidance/
│       ├── __init__.py
│       ├── __main__.py
│       ├── cli.py
│       ├── environment.py
│       ├── geometry.py
│       ├── metrics.py
│       ├── models.py
│       ├── scenarios.py
│       ├── simulation.py
│       └── algorithms/
│           ├── __init__.py
│           ├── a_star.py
│           ├── apf.py
│           ├── base.py
│           └── dwa.py
└── tests/
```

## Quick start

### 1. Create or activate a virtual environment

```powershell
python -m venv .venv
.venv\Scripts\Activate.ps1
```

### 2. Install the project

```powershell
pip install -r requirements.txt
```

### 3. Run a simulation

```powershell
python -m robot_obstacle_avoidance --algorithm astar --scenario slalom
python -m robot_obstacle_avoidance --algorithm apf --scenario narrow_passage
python -m robot_obstacle_avoidance --algorithm dwa --scenario open_field --json
python -m robot_obstacle_avoidance --algorithm astar --scenario open_field --save-plot examples/astar_open_field.png
python -m robot_obstacle_avoidance --scenario slalom --compare-all --save-plot examples/slalom_comparison.png
python -m robot_obstacle_avoidance --algorithm astar --scenario open_field --save-animation examples/astar_open_field.gif
python -m robot_obstacle_avoidance --scenario dynamic_crossing --compare-all --save-animation examples/dynamic_crossing_compare.gif
```

### 4. Run tests

```powershell
python -m unittest discover -s tests -v
```

### 5. Launch the web visualization

```powershell
robot-avoidance-web
```

or

```powershell
python -m robot_obstacle_avoidance.webapp --host 127.0.0.1 --port 8000
```

## Built-in scenarios

- `open_field`: low obstacle density, useful for smoke tests
- `slalom`: alternating obstacles that require repeated heading corrections
- `narrow_passage`: a harder environment with a tight central corridor
- `dynamic_crossing`: moving obstacles cross the lane and expose the advantage of motion-aware local planning

## Visualization

The command-line tool can generate static figures suitable for reports or coursework:

```powershell
python -m robot_obstacle_avoidance --algorithm astar --scenario open_field --plot
python -m robot_obstacle_avoidance --algorithm dwa --scenario slalom --save-plot results/dwa_slalom.png
python -m robot_obstacle_avoidance --scenario narrow_passage --compare-all --save-plot results/narrow_passage_compare.png
python -m robot_obstacle_avoidance --algorithm apf --scenario slalom --save-animation results/apf_slalom.gif
python -m robot_obstacle_avoidance --scenario open_field --compare-all --save-animation results/open_field_compare.gif
```

- `--plot`: open an interactive matplotlib window
- `--save-plot`: save a PNG figure to disk
- `--compare-all`: overlay A*, APF, and DWA trajectories in one figure
- `--animate`: open an animated playback window
- `--save-animation`: save a GIF animation to disk
- `--fps`: control the playback and export frame rate

The generated figure includes:

- scenario boundary and circular obstacles
- start and goal markers
- trajectory geometry
- key metrics such as success flag, path length, travel time, and minimum clearance

The generated animation shows the robot trajectory growing over time and is suitable for demos, presentations, and appendices.

In the `dynamic_crossing` scenario, DWA predicts obstacle motion across its short planning horizon. This gives it a clear advantage over the other two methods, which do not explicitly optimize over future moving-obstacle states.

## Web visualization

The repository also includes a browser-based control panel for interactive playback.

Features:

- choose a scenario and either a single algorithm or compare-all mode
- replay trajectories directly in the browser without generating files first
- inspect metrics for each algorithm in a live dashboard
- adjust playback speed and jump to any simulation step with a timeline slider
- place custom circle or rectangle obstacles directly on the canvas
- remove individual custom obstacles or clear the scenario-specific obstacle set
- view richer mixed obstacle fields with both circular and block-style geometry
- inspect moving obstacle playback in the browser and compare how each planner reacts

Main routes:

- `/`: interactive dashboard
- `/api/meta`: available algorithms and scenarios
- `/api/simulate`: run one algorithm with query parameters such as `algorithm=astar&scenario=open_field`
- `/api/compare`: run all algorithms on one scenario

## Cloud deployment

This repository is prepared for long-lived deployment on a real cloud platform.

Included deployment assets:

- `render.yaml`: Render web service configuration
- `Procfile`: generic process definition for Procfile-compatible platforms
- `Dockerfile`: container image for any Docker-based platform
- `DEPLOYMENT.md`: concise deployment instructions

Recommended route:

1. Push the `robot_obstacle_avoidance` repository to GitHub.
2. Create a new Render web service or blueprint from that repository.
3. Let Render run the included build and start commands.
4. Open `/healthz` after deployment to confirm the service is healthy.

The production service uses `waitress` instead of Flask's development server.

## Extension directions

- Add hybrid A* plus DWA tracking
- Add control barrier function or model predictive control
- Export CSV logs and draw trajectories
- Introduce dynamic obstacles and probabilistic occupancy maps
- Compare path smoothness, clearance, time, and failure rate across repeated trials
