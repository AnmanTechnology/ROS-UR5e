# ROS-UR5e
ROS package for UR5 robot e-series.


## Install dependencies

    $ cd ~/tt_ws
    $ git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git src/Universal_Robots_ROS_Driver
    $ git clone -b calibration_devel https://github.com/fmauch/universal_robot.git src/fmauch_universal_robot
    $ rosdep install --from-paths src --ignore-src -r -y

## Installing a URCap on a e-Series robot

https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/install_urcap_e_series.md