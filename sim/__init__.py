"""Cross-simulator deployment of the paper's quaternion controller (FRP.md).

    sim/common/       frozen controller bridge + scenarios + telemetry
    sim/gym_pybullet/ gym-pybullet-drones adapter
    sim/mujoco/       MuJoCo adapter (MJCF model + runner)
    sim/gazebo/       Gazebo adapter (WSL2: world + SDF + controller node)
    sim/ardupilot/    ArduPilot SITL adapter (WSL2: pymavlink bridge)
"""
