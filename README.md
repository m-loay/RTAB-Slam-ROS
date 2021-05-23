# RTAB-Slam-ROS
In this project you will create a 2D occupancy grid and 3D octomap from a simulated environment using your own robot with the RTAB-Map package.

<table style="width:100%">
  <tr>
    <th><p>
           <img src="images/3D_map.png"
            alt="3D map" width="400" height="200"></a>
           <br>3D Map
        </p>
    </th>
    <th><p>
           <img src="images/2D_map.png"
            alt="2D map" width="200" height="200"></a>
           <br>2D Map
      </p>
    </th>
  </tr>
  <tr>
    <th><p>
           <img src="images/occupancy_grid.png"
            alt="occupancy grid" width="400" height="200"></a>
           <br>Occupancy Grid
      </p>
    </th>
    <th><p>
           <img src="images/features.png"
            alt="features" width="200" height="200"></a>
           <br>Detected Features
      </p>
    </th>
  </tr>
</table>

## Project tree:

* [my_robot/](./src/my_robot)
  * [config/](./src/my_robot/config)
  * [launch/](./src/my_robot/launch)
    * [localization.launch](./src/my_robot/launch/localization.launch)
    * [localization_run.launch](./src/my_robot/launch/localization_run.launch)
    * [mapping.launch](./src/my_robot/launch/mapping.launch)
    * [mapping_run.launch](./src/my_robot/launch/mapping_run.launch)
    * [robot_description.launch](./src/my_robot/launch/robot_description.launch)
    * [world.launch](./src/my_robot/launch/world.launch)
  * [meshes/](./src/my_robot/meshes)
    * [hokuyo.dae](./src/my_robot/meshes/hokuyo.dae)
    * [rtabmap.yaml](./src/my_robot/meshes/rtabmap.yaml)
  * [urdf/](./src/my_robot/urdf)
    * [my_robot.gazebo](./src/my_robot/urdf/my_robot.gazebo)
    * [my_robot.xacro](./src/my_robot/urdf/my_robot.xacro)
  * [worlds/](./src/my_robot/worlds)
    * [myroboworld.world](./src/my_robot/worlds/myroboworld.world)
    * [timsWorld.world](./src/my_robot/worlds/timsWorld.world)
  * [CMakeLists.txt](./src/my_robot/CMakeLists.txt)
  * [package.xml](./src/my_robot/package.xml)
* [CMakeLists.txt](./src/CMakeLists.txt)

## Project Description

In this project, I [RTAB-Map Pacakge](http://wiki.ros.org/rtabmap_ros) to perform slam in a created envionrment and robot built in Gazebo:

1. Update [my_robot.xacro](./src/my_robot/urdf/my_robot.xacro):  
Update `my_robot.xacro` to support RGB-D camera.

2. Update [my_robot.gazebo](./src/my_robot/urdf/my_robot.gazebo):  
Update `my_robot.gazebo` to support RGB-D camera plugin to publish the correct topics.

3. Create mapping launch file to utilize `RTAB-Map Pacakge`  [mapping.launch](./src/my_robot/launch/mapping.launch):  
This launch file create a node for RTAP-map to create map and this launch file is included in [mapping_run.launch](./src/my_robot/launch/mapping_run.launch) to run both of map node and the world.

4. Create localization launch file to utilize `RTAB-Map Pacakge`  [localization_run.launch](./src/my_robot/launch/localization_run.launch):  
This launch file create a node for RTAP-map to perform localization and this launch file is included in [localization_run.launch](./src/my_robot/launch/localization_run.launch).

## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)

* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

* ROS  >= (Kinetic/Melodic/Noetic)
  * For all platform and OS [Click here for installation instructions](http://wiki.ros.org/ROS/Installation)

## Basic Build Instructions

1. Clone this repo.
2. Inside cloned folder `catkin_make`
3. Then source the workspace: `source devel/setup.bash`
4. Launch the world and robot
    - Run the world & mapping through this launch file:
    `roslaunch my_robot mapping_run.launch`

    - Run the teleoperation keyboard:
    `rosrun teleop_twist_keyboard teleop_twist_keyboard.py`

    - Generate the ouput from database viwer:
    `rtabmap-databaseViewer ~/.ros/rtabmap.db`
