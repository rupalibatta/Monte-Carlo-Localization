# Monte Carlo Localization

A Python simulation of **Monte Carlo Localization (MCL)** for **Caltech ME 133b**. We implemented a particle filter to estimate a robot's 2D position from noisy motion and simulated lidar readings, then tested how well it tracked the robot in a map with moving obstacles and recovered after a sudden displacement.

## Project outline

1. Simulate a robot moving through a bounded map with four drifting polygonal obstacles.
2. Generate noisy velocity updates and four-direction lidar measurements using raycasting.
3. Estimate the robot's position with particle prediction, measurement weighting, and systematic resampling.
4. Add particle roughening and random particle injection to maintain diversity and recover after teleportation.
5. Compare localization error, computation time, and recovery across particle-filter settings.

## Localization and particle filtering

The robot moves in a 2D map, while a set of particles represents possible positions. At each update, particles move according to the velocity command plus Gaussian noise. Four simulated lidar beams (right, up, left, down) measure distances to obstacles or map boundaries. Particles whose predicted scans best match the observed scan receive higher weights:

$$
w_i \propto \exp\left(-\frac{1}{2\sigma^2}\sum_{j=1}^{4}(z_j-\hat z_{i,j})^2\right).
$$

The filter uses **systematic resampling** to favor higher-weight particles, adds small position perturbations (*roughening*), and replaces a fraction of particles with random free-space samples. The estimated position is the mean of the resampled particles. The simulation displays the true position, estimate, and particle cloud, and supports keyboard-controlled movement.

## Experiments and results

We varied particle count, lidar likelihood noise, roughening, and resampling method, and compared performance with and without random particle injection. Each reconstructed batch configuration uses five seeded trials of 120 updates; the robot is teleported at update 60 to test recovery.

Increasing the number of particles improved localization at small population sizes, with diminishing returns and higher computation cost. Moderate roughening helped preserve particle diversity, while global particle injection improved recovery after teleportation. In the tested setup, systematic resampling produced lower post-teleport error than multinomial resampling.

The filter assumes the current obstacle map is known, so this is **localization, not SLAM**. The robot is modeled as a point with no heading, and sensing is limited to four simulated lidar beams. The batch experiment code was reconstructed from the surviving implementation and report; rerun results may differ from the original experiments.

## Repository contents

| File | Contents |
| --- | --- |
| [`particle_filter_mcl.ipynb`](particle_filter_mcl.ipynb) | Interactive simulation, particle filter, and reconstructed experiments |
| [`ME133b_MCL_report.pdf`](ME133b_MCL_report.pdf) | Original methodology, experiments, and results |
| [`requirements.txt`](requirements.txt) | Python dependencies |
| `.gitignore` | Local-file exclusions |

## Authors

**Rupali Batta and Felipe Cruz**  
California Institute of Technology, **ME 133b**
