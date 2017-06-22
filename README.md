This branch stores the GitHub pages site for Headtracker.

# headtracker
Headtracker is a state estimation system for tracking the orientation of a
virtual reality headset. The system is based on an Extended Kalman Filter with
representive measurement models of a 3-axis gyroscope, accelerometer, and
magnetometer. Results and additional information are available at 
[VR Headtracker](https://mbanderson.github.io/headtracker/).

# Usage
[ekf.m](https://github.com/mbanderson/headtracker/blob/master/src/tracker/Filter/ekf.m) is the main driver function. The function
inputs determine the measurement model configuration. [ekf.m](https://github.com/mbanderson/headtracker/blob/master/src/tracker/Filter/ekf.m) 
relies heavily on [HeadDynamicsModel.m](https://github.com/mbanderson/headtracker/blob/master/src/tracker/Dynamics/HeadDynamicsModel.m), 
which generates the true head motion and generates sensor measurements.

# Directories
[tracker](https://github.com/mbanderson/headtracker/tree/master/src/tracker) contains the core state estimator and measurement 
models. [data](https://github.com/mbanderson/headtracker/tree/master/src/data) contains data collection and analysis scripts, used 
to obtain realistic sensor noise parameters. [accelerometer](https://github.com/mbanderson/headtracker/tree/master/src/accelerometer) 
implements an accelerometer constellation system used to compute angular rate
measurements.