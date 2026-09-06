# Cross-simulator comparison -- paper scenarios on all stacks

Same frozen controller (Pq=20, Pw=4, +/-4 N*m), same references, same
noise model everywhere; see FRP.md section 3. Lower is better except
duration. settle_* use the noise-aware band (max(5%, 2*noise)) on the
0.5 s-smoothed wrap-aware Euler error.

| simulator | scenario | RMS att. err [deg] | max att. err [deg] | settle phi [s] | settle theta [s] | settle psi [s] | torque sat. [-] | drift [m] | duration [s] |
|---|---|---|---|---|---|---|---|---|---|
| gym_pybullet | flip | 115.19 | 179.88 | 5.00 | 5.00 | 5.00 | 0.79 | 28.19 | 5.00 |
| gym_pybullet | flip | 109.34 | 179.88 | 5.00 | 5.00 | 5.00 | 0.60 | 36.21 | 5.00 |
| gym_pybullet | flip | 133.97 | 179.98 | 5.00 | 3.88 | 5.00 | 0.79 | 24.95 | 5.00 |
| gym_pybullet | sine | 17.52 | 41.17 | 12.47 | 8.65 | 5.85 | 0.01 | 105.39 | 15.00 |
| gym_pybullet | step | 17.68 | 70.74 | 13.97 | 0.87 | 6.00 | 0.02 | 1275.92 | 15.00 |
| gym_pybullet | step | 17.58 | 71.84 | 14.00 | 0.83 | 6.00 | 0.02 | 1276.38 | 15.00 |
| mujoco | flip | 44.39 | 91.91 | 2.88 | 0.00 | 0.00 | 0.01 | 54.40 | 5.00 |
| mujoco | flip | 44.55 | 91.32 | 2.97 | 0.00 | 0.00 | 0.01 | 54.43 | 5.00 |
| mujoco | flip | 44.51 | 90.91 | 2.95 | 0.00 | 0.00 | 0.01 | 54.52 | 5.00 |
| mujoco | sine | 17.61 | 41.10 | 12.47 | 8.65 | 5.82 | 0.01 | 105.66 | 15.00 |
| mujoco | step | 17.71 | 71.18 | 13.97 | 0.86 | 6.00 | 0.02 | 1864.86 | 15.00 |
| mujoco | step | 17.66 | 72.92 | 14.00 | 0.83 | 6.00 | 0.02 | 1875.99 | 15.00 |
