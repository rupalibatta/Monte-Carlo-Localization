# Monte Carlo Localization

A Python simulation of **Monte Carlo Localization (MCL)** that estimates a robot's 2D position from velocity commands and noisy simulated lidar measurements. Developed for **Caltech ME 133b**, the project inlcudes a simulation as well as multiple experiments on localization accuracy, computational cost, particle diversity, and recovery after sudden displacement.

The robot moves through a bounded map containing four drifting polygonal obstacles. A particle filter keeps guessing its location, updates them using range measurements, and attempts to recover when the robot is teleported to an unknown position.

## Project outline

1. **Build a dynamic environment:** define polygonal obstacles, animate their motion, and check whether positions are in free space.
2. **Simulate robot motion and sensing:** apply noisy velocity commands and generate four-direction lidar scans through raycasting.
3. **Implement the particle filter:** initialize position hypotheses, propagate them, compute measurement likelihoods, and resample.
4. **Maintain exploration:** add local roughening and random global particles to reduce collapse and support relocation recovery.
5. **Visualize localization:** display the robot, particle cloud, and estimated position in an interactive WASD-controlled simulation.
6. **Evaluate design choices:** sweep particle count, likelihood noise, and roughening; compare resampling methods and recovery strategies.

## The localization problem

The filter receives a control input $u_t=(v_x,v_y)$ and a noisy four-beam range measurement $z_t$. It estimates $(x,y)$ without using the simulated robot's true position. Ground truth is available separately for visualization and evaluation.

| Component | Implementation |
| --- | --- |
| State | Position $(x,y)$; no orientation state |
| Map | Rectangle $[0,10] \times [0,12]$ with four polygonal obstacles |
| Obstacle motion | Circular translations with obstacle-specific radius, frequency, and phase |
| Control | Velocity along the map's horizontal and vertical axes |
| Sensor | Four fixed, axis-aligned lidar beams: right, up, left, down |
| Initial belief | Particles sampled uniformly in current free space |
| Failure scenario | Robot teleported to a random free position at update index 60 |

The filter uses the **current, known obstacle geometry** for both collision checks and predicted scans. This is localization in a supplied dynamic map, not simultaneous localization and mapping (SLAM).

## How the particle filter works

Each particle stores a candidate position and a weight:

$$
p_i=(x_i,y_i,w_i), \qquad \sum_{i=1}^{N}w_i=1.
$$

The particle population approximates the robot's position belief and can represent several possible locations at once.

### 1. Motion prediction

For the robot and each particle, the motion model applies the velocity command and independent Gaussian noise:

$$
x_t=x_{t-1}+v_x\Delta t+n_x, \qquad
y_t=y_{t-1}+v_y\Delta t+n_y,
\qquad n_x,n_y\sim\mathcal{N}(0,\sigma_{\mathrm{bot}}^2).
$$

If a proposed endpoint lies outside the map or intersects an obstacle, the previous position is retained. The implementation checks endpoints rather than the full swept path.

### 2. Simulated lidar

`raycast_distance` marches each ray in increments of `0.05` until it encounters an obstacle, exits the map, or reaches the maximum range of `7.5`. The robot's measurements include Gaussian noise and are clipped to the sensor range. Predicted scans at particle positions are generated without added noise.

### 3. Measurement weighting

A particle receives a higher likelihood when its predicted scan resembles the observed scan. The measurement model assumes independent Gaussian errors across the four beams:

$$
w_i\propto p(z_t\mid x_i,y_i)
=\prod_{j=1}^{4}\frac{1}{\sqrt{2\pi\sigma^2}}
\exp\left(-\frac{(z_{t,j}-\hat z_{i,j})^2}{2\sigma^2}\right).
$$

Particles outside current free space receive zero weight. The batch implementation evaluates the summed squared residual in the exponent and omits the Gaussian normalization constant, which is common to all particles for a fixed $\sigma$.

### 4. Resampling and diversity

The default **systematic, or low-variance, resampler** uses one random offset and evenly spaced samples through the cumulative weight distribution. Selected particles receive equal weights of $1/N$.

Two mechanisms modify the resampled population:

- **Roughening:** add Gaussian position jitter to copied particles. If the jittered position is invalid, retain the source position.
- **Random particle injection:** independently replace each output particle with a uniformly sampled free-space hypothesis with probability `0.1`. This gives the filter opportunities to discover a new location after teleportation; the actual injected fraction varies by update.

If all likelihoods collapse to zero, the filter reinitializes the population uniformly in free space. Resampling occurs at every filter update.

### 5. Position estimate and diagnostics

The estimated position is the arithmetic mean of the **resampled** particle positions. Effective sample size (ESS) is computed from normalized weights **before** resampling:

$$
N_{\mathrm{eff}}=\frac{1}{\sum_i w_i^2}.
$$

ESS measures weight concentration, not spatial spread or localization accuracy. A particle cloud concentrated at an incorrect location can still have high ESS.

## Repository contents

| File | Purpose |
| --- | --- |
| [`particle_filter_mcl.ipynb`](particle_filter_mcl.ipynb) | Environment, motion and sensor models, interactive filter, and reconstructed batch experiments |
| [`ME133b_MCL_report.pdf`](ME133b_MCL_report.pdf) | Final report with methodology, original experiment tables, figures, and discussion |
| [`requirements.txt`](requirements.txt) | NumPy, pandas, Matplotlib, Shapely, and Jupyter dependencies |
| `.gitignore` | Ignore rules for local development files |

The notebook is organized into two main parts: the interactive implementation followed by **Reconstructed batch experiments**. Experiment outputs are generated when those cells are run.

## Setup

From the repository directory:

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
jupyter lab
```

On Windows, activate the environment with `.venv\Scripts\activate` instead. Open `particle_filter_mcl.ipynb` in JupyterLab.

### Run the interactive simulation

1. Select a Matplotlib backend that supports a native interactive window. On macOS, the notebook's first cell contains `%matplotlib osx`.
2. Run the implementation cells in order through the keyboard and timer callbacks, stopping before **Reconstructed batch experiments**.
3. Click the plot window to give it keyboard focus.
4. Use **W / A / S / D** to move up / left / down / right. Press **Q** or **Escape** to close the simulation.

The plot displays a **green diamond** for the true position, a **red cross** for the estimate, and the particle cloud. The initial interactive particle count is `25`.

Obstacles advance on a `100 ms` timer. Robot motion, measurements, and filter updates occur on WASD keypresses. The teleport happens at zero-based update index `60`, corresponding to the 61st movement keypress.

The inline backend can display static plots but does not provide the native keyboard interaction used by this demo. On other systems, select an available interactive backend such as `%matplotlib qt` if its GUI dependencies are installed.

### Run the batch experiments

1. Comment out `%matplotlib osx` in the first cell and use `%matplotlib inline` or a headless backend.
2. Run the imports and environment/model definitions above the interactive demo.
3. Skip the interactive launch cells beginning with `visual = Visualization()` and the following callback/timer cell.
4. Run **Reconstructed batch experiments** and its subsequent cells in order.

Each trial uses **120 updates**, a fixed reconstructed velocity sequence, and a teleport at index **60**. Each experiment configuration uses **five seeded trials**. Large particle counts require more computation because predicted scans are raycast separately for every particle.

## Parameters

| Parameter | Interactive default | Batch baseline | Role |
| --- | --- | --- | --- |
| Particle count | `25` | `100` | Number of position hypotheses |
| Lidar likelihood standard deviation | `1.0` | `1.0` | How strongly scan discrepancies are penalized |
| Actual lidar noise standard deviation | `1.0` | `1.0` | Noise added to simulated robot scans |
| Roughening standard deviation | `0.1` | `0.1` | Position jitter after resampling |
| Random injection probability | `0.1` | `0.1` | Chance of replacing each particle with a global hypothesis |
| Motion noise standard deviation | `0.01` | `0.01` | Position noise applied per update |
| Time increment | `0.1` | `0.1` | Converts velocity commands to displacement |
| Maximum lidar range | `7.5` | `7.5` | Sensor range cap |
| Ray-marching increment | `0.05` | `0.05` | Spatial resolution of predicted ranges |

Change the interactive constants before rerunning their dependent function definitions. For batch runs, configure `ExperimentConfig`; `run_experiment` returns per-update data, and `run_trials` returns combined trajectories and per-trial summaries.

## Experiments and evaluation

Localization error is the Euclidean distance between the estimated and true positions, measured in map coordinate units. Pre-teleport error covers indices `0–59`; post-teleport error covers `60–119`.

| Experiment | Configurations | Evaluation |
| --- | --- | --- |
| Particle count | `10, 25, 50, 100, 200, 500` | Mean error before/after teleport and average time per update |
| Likelihood noise | $\sigma=0.5,1,2,3$; `100` particles | Pre-teleport error while actual sensor noise stays at `1.0` |
| Roughening | `0, 0.05, 0.1, 0.2`; `25` particles | Mean error and mean pre-resampling ESS over the full trial |
| Resampling rule | Systematic vs. multinomial; `25` particles | Error before/after teleport and example trajectories |
| Global recovery | Injection probability `0` vs. `0.1`; `25` particles | Post-teleport error and recovery trajectories |

Particle-count and likelihood-noise summaries include across-trial standard deviations. The error-versus-step plots show the first seeded trial for each selected configuration rather than an average trajectory. Timing measures the complete batch trial divided by its number of updates, including measurement simulation and bookkeeping.

### Generated outputs

Running the experiment cells creates five summary files in `mcl_experiment_results/`:

- `particle_count_summary.csv`
- `lidar_noise_summary.csv`
- `roughening_summary.csv`
- `resampling_summary.csv`
- `random_injection_summary.csv`

Seven PNG plots are saved in `mcl_experiment_results/figures/`: particle-count error trajectories, before/after error by particle count, compute cost, likelihood-noise sensitivity, ESS versus roughening, resampling comparison, and kidnapping recovery.

### Findings

Improved accuracy when increasing very small particle populations, followed by diminishing returns as computational cost rises. Sensitivity to the likelihood-noise parameter, benefits from moderate roughening, and lower post-teleport error with systematic resampling than with multinomial resampling in the tested setup.

## Reproducibility and limitations

The original batch-analysis driver is no longer available. The notebook's experiment layer was reconstructed from the surviving implementation and final report. It follows the report's parameter grids, metrics, trial counts, and figure types, but the original control sequence and random seeds were not recoverable. New numerical results may therefore differ from the report.

Additional modeling limits:

- The robot is a point with two translational degrees of freedom. Heading, footprint, and rotational motion are not modeled.
- Obstacle positions are supplied exactly to the filter. Unknown obstacle motion and map uncertainty are not evaluated.
- Lidar uses four axis-aligned beams and discrete ray marching. The Gaussian likelihood approximates the clipped measurement distribution.
- Collision checks evaluate proposed endpoints; moving obstacles can also overtake stationary positions.
- A mean estimate can fall between distinct belief modes, and globally injected particles can shift that mean.
- Recovery is evaluated through post-teleport error, without a formal recovery-time threshold or success-rate metric.

## Authors

**Rupali Batta and Felipe Cruz**  
California Institute of Technology, ME 133b.