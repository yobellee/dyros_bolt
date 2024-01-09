# Dyros Bolt
controller for bolt

-----------------------------------------

# Installation
### 0. prerequisities
  * This project was developed with Ubuntu 20.04 & 18.04, ROS Noetic & ROS Melodic
  * Realrobot mode uses ODrive board V3.6
  * [libdwbc](https://github.com/saga0619/libdwbc) package
  * mscl download [MSCL](https://github.com/LORD-MicroStrain/MSCL/releases/download/v65.0.0/c++-mscl_65.0.0_amd64.deb) 
    ```sh
        sudo dpkg -i c++-mscl_65.0.0_amd64.deb  
    ```
### 1. clone repository
```sh
cd catkin_ws/src
git clone https://github.com/yongarry/dyros_bolt
```

### 2. mujoco_ros_sim()
```sh
cd catkin_ws/src
git clone https://github.com/saga0619/mujoco_ros_sim
```

### 3. IMU sensor (for real robot)
```sh
cd catkin_ws/src
git clone https://github.com/yongarry/bolt_imu
```

### 4. GUI
```sh
sudo apt install qtbase5-private-dev libqt5x11extras5*
cd catkin_ws/src
git clone https://github.com/yongarry/dyros_bolt_gui
```
