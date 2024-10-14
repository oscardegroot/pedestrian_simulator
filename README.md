[![ROS1 - Build](https://github.com/oscardegroot/ros_tools/actions/workflows/ros1.yml/badge.svg)](https://github.com/oscardegroot/ros_tools/actions/workflows/ros1.yml)

# Pedestrian Simulator
This repository implements a pedestrian simulator based on the social forces implementation of [`libpedsim`](https://github.com/chgloor/pedsim). It features 

- Pedestrian spawn/goal locations encoded in `xml` format
- Static obstacles that the pedestrians react to can be configured
- Support for uncertain pedestrian dynamics (Gaussian or Mixture of Gaussian)
- Support for ROS/ROS2


> **Disclaimer:** This is an internal package used for research purposes. The implementation was not designed to share and has not been documented in detail. Please use with care.

## Installation
Clone dependencies from `catkin_ws/src`
```bash
git clone https://github.com/oscardegroot/ros_tools.git
git clone https://github.com/oscardegroot/pedsim_original.git
git clone https://github.com/oscardegroot/asr_rapidxml.git
```

Clone this repo
```bash
git clone https://github.com/oscardegroot/pedestrian_simulator.git
```

> **Note:** To use ROS2, run `python3 switch_to_ros.py 2` in the base folder of `pedestrian_simulator` and `ros_tools`.

Install dependencies from `catkin_ws`

```bash
rosdep install --from-paths src --ignore-src -r -y
```

Build with

```bash
catkin build pedestrian_simulator
```


## Usage
Pedestrian scenarios are in `pedestrian_simulator/scenarios/`.

To launch, use 

```bash
roslaunch pedestrian_simulator ros1_simulation.launch pedestrian_scenario:=random_social/8_corridor.xml
```

**Example Simulation:**

<img src="https://imgur.com/61i3M78.gif" width="80%">

---

## License
This project is licensed under the Apache 2.0 license - see the LICENSE file for details.