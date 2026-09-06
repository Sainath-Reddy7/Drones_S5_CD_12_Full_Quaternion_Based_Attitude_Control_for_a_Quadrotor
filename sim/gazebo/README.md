# Gazebo adapter — paper attitude scenarios in Gazebo physics

Architecture (FRP.md section 3): **Gazebo is the physics engine**; ArduPilot
SITL sits between Gazebo and our controller exactly as it would on a real
Gazebo testbed — the `ardupilot_gazebo` plugin feeds IMU/GPS into the flight
stack and applies its motor commands to the model, while our frozen paper
controller (`sim/ardupilot/bridge_node.py`) flies it over MAVLink from
outside. Nothing about the controller changes between this stack and the
ArduPilot-native run; only the physics does.

```
   paper controller (this repo, unmodified gains)
              │  MAVLink: SET_ATTITUDE_TARGET @ 50 Hz
              ▼
   ArduPilot SITL  ── ardupilot_gazebo plugin ──▶  Gazebo (gz-sim) physics
              ▲                                        │
              └────────── IMU / pose / state ──────────┘
```

Everything here targets **Ubuntu 24.04 under WSL2** (Gazebo does not run
natively on Windows). One-time setup, then three commands per run.

## 1. One-time setup (inside WSL2)

```bash
sudo apt update && sudo apt install cmake build-essential git python3-pip

# --- Gazebo Harmonic ---
sudo apt-get update
sudo apt-get install -y lsb-release gnupg curl
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs.osrfoundation.org.gazebo.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs.osrfoundation.org.gazebo.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update && sudo apt-get install -y gz-harmonic

# --- ArduPilot ---
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git ~/ardupilot
cd ~/ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
./waf configure --board sitl && ./waf copter

# --- ardupilot_gazebo plugin ---
git clone https://github.com/ArduPilot/ardupilot_gazebo.git ~/ardupilot_gazebo
cd ~/ardupilot_gazebo
cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j4

# --- this repo + bridge deps ---
git clone https://github.com/bodapatisaikrishna/Drones_S5_CD_12_Full_Quaternion_Based_Attitude_control_for_a_Quadrotor.git ~/drones
python3 -m pip install -r ~/drones/sim/ardupilot/requirements.txt
```

Put the environment in every shell (or `~/.bashrc`):

```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build
export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:$HOME/drones/sim/gazebo/worlds
```

## 2. Per-run (three terminals, all inside WSL2)

```bash
# T1: Gazebo with the paper attitude world
cd ~/drones && gz sim -v4 -r --iterations 0 sim/gazebo/worlds/paper_attitude.world

# T2: ArduPilot SITL bound to Gazebo (frame name from the ardupilot_gazebo
#     docs; check `sim_vehicle.py -v ArduCopter -f list | grep gazebo` if the
#     default does not match your checkout)
cd ~/ardupilot && Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris \
    --no-mavproxy -L home --model JSON

# T3: the paper controller
cd ~/drones && python3 -m sim.ardupilot.bridge_node --scenario step --connection tcp:127.0.0.1:5760
```

Repeat with `--scenario sine` and `--scenario flip`. Logs and figures land in
`results/sim_gazebo/` — for bookkeeping clarity, rename the directory or the
CSV after each run so `sim/compare.py` attributes rows to the right stack:

```bash
mv results/sim_ardupilot results/sim_gazebo   # after a Gazebo-backed run
```

## 3. Headless (no GUI) runs

WSL2 without WSLg: run Gazebo headless with `--headless-rendering` or simply
omit the GUI: `gz sim -s -r -v4 ...` (server-only). The bridge only needs
MAVLink, so the benchmark runs unchanged.

## Notes

- **Vehicle:** the stock ardupilot_gazebo `iris` is their maintained,
  ArduPilot-tuned quad. A parameter-matched variant of OUR paper vehicle
  (0.2 kg, paper inertia) as an SDF is future work; the frozen-controller
  comparison stands either way, and the finding is expected to match
  FRP.md section 7 (outer-loop behavior on a production stack).
- **Why ArduPilot is in the loop:** Gazebo provides no standard way to inject
  raw body torques on an arbitrary model from Python; going through the
  flight stack is how real Gazebo testbeds fly. The identical bridge powers
  the ArduPilot-native item (`sim/ardupilot/`), so the only variable between
  the two evaluation items is the physics engine itself.
