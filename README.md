# Efficient Trajectory Optimization for Robot Motion Planning -- Examples
Examples of efficient trajectory optimization for robot motion planning 

## Dependency

* [chebfun](http://www.chebfun.org/) - Numericaltool for Chebyshev function
* [CasADi](https://github.com/casadi/casadi/wiki) - Symbolic tool for automatic differentiation

## Usage

Run MainDemo.m and follow instructions. 

### Available Demos

* [0] 2D scara robot, time optimal motion with kinematics constraints

* [1] 2D scara robot, obstacle avoidance with kinematics constraints

* [2] 2D scara robot, obstacle avoidance with dynamics constraints

* [3] 2D wafer handling robot, obscatle avoidance with kinematics constraints, description refer to "Trajectory planning for robot manipulators considering kinematic constraints using probabilistic roadmap approach." Xiaowen Yu etc., 2017, or "Intelligent Control and Planning for Industrial Robots." Yu Zhao, 2018.

* [4] 6-axis robot case, using FANUC M20iA model from [ARTE](http://arvc.umh.es/arte/index_en.html), implemented robot kinematics, inverse dynamics (recursive Newton Euler method, or rNE), and forward dynamics (articulated body algorithm, or ABA) for planning calculation. Examples coming soon...

Besides, a quadrotor demo can be found in [another git repo](https://github.com/yzhao334/Flipping-Test.git)

### Videos

[2D scara robot planning](https://youtu.be/Up3LHq3DUD0)

[3D 6-axis robot planning](https://youtu.be/EZmLXtO3C2E)

### Reference
* Zhao, Yu, Hsien-Chung Lin, and Masayoshi Tomizuka. "Efficient trajectory optimization for robot motion planning." 2018 15th International Conference on Control, Automation, Robotics and Vision (ICARCV). IEEE, 2018.
* Zhao, Yu. Intelligent Control and Planning for Industrial Robots. Diss. UC Berkeley, 2018.
* Yu, Xiaowen, et al. "Trajectory planning for robot manipulators considering kinematic constraints using probabilistic roadmap approach." Journal of Dynamic Systems, Measurement, and Control 139.2 (2017).
