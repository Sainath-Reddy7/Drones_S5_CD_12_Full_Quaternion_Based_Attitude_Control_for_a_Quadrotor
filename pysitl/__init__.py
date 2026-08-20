"""pysitl — a PX4/Gazebo-style Software-In-The-Loop simulator, in pure Python.

Wraps the Fresk & Nikolakopoulos (ECC 2013) quaternion attitude controller from
the sibling ``quat_sitl`` package in a full 6-DOF quadrotor flight-dynamics
simulation with a PX4-shaped architecture: a uORB-style message bus, a
multi-rate lockstep scheduler, an arming/flight-mode commander, control
allocation, structured logging, and a browser ground station.

The *content* is paper-faithful (eq. 18 rotational plant, the unmodified P^2
controller, uniform +/-0.1 measurement noise, no state estimator, the paper's
own inertia); the *architecture* is what reproduces the PX4 SITL workflow.
"""

__version__ = "0.1.0"
