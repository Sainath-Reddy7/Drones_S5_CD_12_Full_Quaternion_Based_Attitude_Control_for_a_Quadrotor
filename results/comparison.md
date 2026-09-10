# Cross-simulator comparison -- paper scenarios on all stacks

Same frozen controller (Pq=20, Pw=4, +/-4 N*m), same references, same
noise model everywhere; see FRP.md section 3. Lower is better except
duration. settle_* use the noise-aware band (max(5%, 2*noise)) on the
0.5 s-smoothed wrap-aware Euler error.

| simulator | scenario | RMS att. err [deg] | max att. err [deg] | settle phi [s] | settle theta [s] | settle psi [s] | torque sat. [-] | drift [m] | duration [s] |
|---|---|---|---|---|---|---|---|---|---|
| ardupilot | flip | 31.25 | 91.13 | 3.24 | 0.00 | 0.00 | 0.36 | 1.81 | 4.98 |
| ardupilot | flip | 41.65 | 179.64 | 2.48 | 2.22 | 0.00 | 0.38 | 6.15 | 4.98 |
| ardupilot | flip | 30.94 | 87.94 | 2.94 | 0.00 | 0.00 | 0.33 | 1.92 | 4.98 |
| ardupilot | sine | 14.51 | 34.03 | 0.00 | 0.00 | 0.00 | 0.03 | 15.27 | 14.98 |
| ardupilot | sine | 15.00 | 34.82 | 0.00 | 0.00 | 0.00 | 0.02 | 15.33 | 14.98 |
| ardupilot | sine | 14.49 | 33.03 | 0.00 | 0.00 | 0.00 | 0.03 | 15.32 | 14.98 |
| ardupilot | step | 46.25 | 113.16 | 13.98 | 9.98 | 5.98 | 0.59 | 65.63 | 14.98 |
| ardupilot | step | 48.45 | 138.08 | 13.98 | 9.98 | 5.98 | 0.59 | 70.18 | 14.98 |
| ardupilot | step | 47.36 | 160.43 | 13.98 | 9.98 | 5.98 | 0.61 | 68.42 | 14.98 |
| gazebo | flip | 144.82 | 179.70 | 4.98 | 0.00 | 4.98 | 0.86 | 4.42 | 4.98 |
| gazebo | flip | 144.65 | 179.94 | 4.98 | 0.00 | 4.98 | 0.87 | 4.77 | 4.98 |
| gazebo | flip | 145.09 | 179.79 | 4.98 | 0.00 | 4.98 | 0.88 | 6.86 | 4.98 |
| gazebo | sine | 17.33 | 39.68 | 11.78 | 7.64 | 0.00 | 0.03 | 34.22 | 14.98 |
| gazebo | sine | 17.38 | 40.80 | 9.28 | 5.16 | 0.50 | 0.03 | 39.12 | 14.98 |
| gazebo | sine | 17.33 | 42.42 | 12.36 | 4.92 | 0.00 | 0.04 | 36.05 | 14.98 |
| gazebo | step | 130.39 | 179.56 | 13.98 | 9.98 | 5.98 | 0.88 | 44.02 | 14.98 |
| gazebo | step | 117.69 | 179.73 | 13.98 | 9.98 | 5.98 | 0.88 | 52.70 | 14.98 |
| gazebo | step | 122.35 | 179.88 | 13.98 | 9.98 | 5.98 | 0.88 | 51.23 | 14.98 |
| gym_pybullet | flip | 115.19 | 179.88 | 5.00 | 5.00 | 5.00 | 0.79 | 28.19 | 5.00 |
| gym_pybullet | flip | 113.82 | 179.88 | 5.00 | 4.47 | 5.00 | 0.79 | 29.11 | 5.00 |
| gym_pybullet | flip | 115.72 | 179.91 | 5.00 | 4.70 | 5.00 | 0.79 | 29.05 | 5.00 |
| gym_pybullet | sine | 17.52 | 41.17 | 12.47 | 8.65 | 5.85 | 0.01 | 105.39 | 15.00 |
| gym_pybullet | step | 17.68 | 70.74 | 13.97 | 0.87 | 6.00 | 0.02 | 1275.92 | 15.00 |
| gym_pybullet | step | 17.58 | 71.84 | 14.00 | 0.83 | 6.00 | 0.02 | 1276.38 | 15.00 |
| mujoco | flip | 44.39 | 91.91 | 2.88 | 0.00 | 0.00 | 0.01 | 54.40 | 5.00 |
| mujoco | flip | 44.55 | 91.32 | 2.97 | 0.00 | 0.00 | 0.01 | 54.43 | 5.00 |
| mujoco | flip | 44.51 | 90.91 | 2.95 | 0.00 | 0.00 | 0.01 | 54.52 | 5.00 |
| mujoco | sine | 17.61 | 41.10 | 12.47 | 8.65 | 5.82 | 0.01 | 105.66 | 15.00 |
| mujoco | step | 17.71 | 71.18 | 13.97 | 0.86 | 6.00 | 0.02 | 1864.86 | 15.00 |
| mujoco | step | 17.66 | 72.92 | 14.00 | 0.83 | 6.00 | 0.02 | 1875.99 | 15.00 |
