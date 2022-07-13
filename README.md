# ROS wrapper for ORB-SLAM3

A ROS wrapper for [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3). The main idea is to use the ORB-SLAM3 as a standalone library and interface with it instead of putting everything in one package.

- **Pros**:
  - Easy to update the ORB-SLAM3 library (currently in [V0.3 Beta](https://github.com/UZ-SLAMLab/ORB_SLAM3#orb-slam3)).
  - Easy to plug in different variants (and there are many) that are not built for ROS (hopefully).
- **Cons**:
  - Might be more difficult to spot bugs.
  - Development involves more steps (1. Make changes in ORB-SLAM3 library -> 2. Build ORB-SLAM3 -> 3. Change the ROS-wrapper if necessary -> 4. Test).
  - Might break when dependencies or upstream changes.


## Current known limitations
Only works for RGBD node.

# Installation
First install ORB-SLAM3 normally with all of its dependencies (any location is fine) then install this package in a ```catkin build``` environment.

## 1. ORB-SLAM3 (original or other variants)

- Install the [prerequisites](https://github.com/UZ-SLAMLab/ORB_SLAM3#2-prerequisites).
- Build and install ORB-SLAM3. Any location is fine (default directory that I use later on is the home folder `~`):
```
cd ~
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git ORB_SLAM3
cd ORB_SLAM3
chmod +x build.sh
./build.sh
```

- Make sure that **`libORB_SLAM3.so`** is created in the *ORB_SLAM3/lib* folder. If not, check the issue list from the [original repo](https://github.com/UZ-SLAMLab/ORB_SLAM3/issues) and rebuild the package.

## 2. ORB_SLAM3_ros (this package)

- Clone the package. Note that it should be a `catkin build` workspace.
```
cd ~/catkin_ws/src/ # Or the name of your workspace
git clone https://github.com/joeaortiz/ORB_SLAM3_ros.git
```

- Open `CMakeLists.txt` and change the directory that leads to ORB-SLAM3 library at the beginning of the file (default is home folder).
```
cd ~/catkin_ws/src/ORB_SLAM3_ros/
nano CMakeLists.txt

# Change this to your installation of ORB-SLAM3. Default is ~/
set(ORB_SLAM3_DIR
   $ENV{HOME}/ORB_SLAM3
)
```

- Build the package normally.
```
cd ~/catkin_ws/
catkin_make
```

- Unzip the `ORBvoc.txt` file in the `config` folder in this package. Alternatively, you can change the `voc_file` param in the launch file to point to the right folder.
```
cd ~/catkin_ws/src/ORB_SLAM3_ros/config
tar -xf ORBvoc.txt.tar.gz
```

- If everything works fine, you can now try the different launch files in the `launch` folder.

## 3. How to run

Example with the realsense camera. By default the ORB-SLAM3 is run in RGBD mode.
First install the realsense ros wrapper with `sudo apt-get install ros-$ROS_DISTRO-realsense2-camera`, then you can launch ORB-SLAM3 with:
```
roslaunch orb_slam3_ros_wrapper run_realsense.launch
```

# Published topic
When running in RGBD mode, the topic `/frames` is published which is a custom message containing synchronised:
- `rgb` of type [`sensor_msg/Image`](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html)
- `depth` of type [`sensor_msg/Image`](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html)
- `pose` of type [`geometry_msgs/Pose`](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Pose.html)
