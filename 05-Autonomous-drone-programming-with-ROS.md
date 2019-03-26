# [Why ROS](#why-ros)

ROS, or Robotic Operating System, is an open source stack for robotics.
It runs on multiple OSes, but we'll cover installing ROS-kinetic on Ubuntu* ([OS installation described here](90-(References)-OS-user-Installation)).

After that, you're in familiar territory. You can run ROS commands as you're used to. Intel Aero is using 4 ROS nodes of interest:
* communicate with the flight controller using the [MAVROS node](http://wiki.ros.org/mavros). MAVROS is designed to work with MAVLINK compatible flight controllers, like Intel Aero Flight Controller with PX4 or Arudpilot stacks.
* interact with the Intel Real Sense R200 camera using [RealSense node](http://wiki.ros.org/RealSense).
* as an alternative to the MAVROS node, you can access the IMU with a [dedicated node imu_driver](https://github.com/intel/imu_driver). MAVROS will give you access to the entire MAVLink protocol (IMU included), imu_driver will give you access to the IMU only in a simpler way.
* in addition to the Intel Real Sense node, you can use the (Intel Aero camera node)[https://github.com/intel/camera_driver].


A good intro on MAVROS with Gazebo simulator is kindly provided by [University of California - Merced course UCM-ME190](https://github.com/UCM-ME190/MavRos-takeoff-n-land)

# [Installation](#installation)

```bash
sudo add-apt-repository http://packages.ros.org/ros/ubuntu
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 0xB01FA116
sudo apt-get update

sudo apt -y install ros-kinetic-desktop-full ros-kinetic-rqt python-rosinstall ros-kinetic-realsense-camera ros-kinetic-mavros ros-kinetic-web-video-server ros-kinetic-visp-tracker ros-kinetic-visp-camera-calibration ros-kinetic-vision-visp ros-kinetic-vision-opencv ros-kinetic-video-stream-opencv ros-kinetic-uvc-camera ros-kinetic-usb-cam ros-kinetic-test-mavros ros-kinetic-rviz-visual-tools ros-kinetic-rostopic ros-kinetic-roslaunch python-rosinstall python-rosinstall-generator python-wstool build-essential ros-kinetic-pyros python-rosdep

sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo geographiclib-get-geoids egm96-5
```
# [Launching](#launching)

```bash
roscore &
roscd realsense_camera
roslaunch realsense_camera r200_nodelet_rgbd.launch &
rosrun mavros mavros_node _fcu_url:=tcp://127.0.0.1:5760 _system_id:=2 &
```

I launched core, RealSense and mavros in 3 terminals to see the logs:

![](https://raw.githubusercontent.com/guermonprez/intel-aero-documents/master/doc_photos/ros_terms.png)

If you ask for the list of topics, with `rostopic list`, you'll get a long list including:
```
/camera/color/camera_info
/camera/color/image_raw/compressed
/camera/depth/image_raw/compressed
/camera/ir/image_raw/compressed
/camera/ir2/image_raw/compressed
/mavlink/from
/mavlink/to
/mavros/actuator_control
/mavros/adsb/send
/mavros/adsb/vehicle
/mavros/altitude
/mavros/battery
```
it means you have both RealSense and MAVlink topics available.

# [Visualization with rviz](#visualization-with-rviz)
After you launched ROS, use rossun rviz to see the RealSense output:
```
roscd realsense_camera
rosrun rviz rviz -d rviz/realsense_rgbd_pointcloud.rviz
```
you can also try realsenseRvizConfiguration2.rviz and realsenseRvizConfiguration3.rviz.

A point cloud is the sum of 3D and depth data, both coming from the R200 camera.
In this screenshot, you can see the 2 chairs from the camera point of vue, but also from the side to show you it's in 3D.

![](https://raw.githubusercontent.com/guermonprez/intel-aero-documents/master/doc_photos/ros_pointcloud.png)

# [SLAM with RTAB](#slam-with-RTAB)
A SLAM library is provided by [RTAB-Map](http://wiki.ros.org/rtabmap_ros).
```
roscd realsense_camera
roslaunch realsense_camera r200_nodelet_rgbd.launch &
roslaunch rtabmap_ros rgbd_mapping.launch rtabmap_args:="--delete_db_on_start" depth_registered_topic:=/camera/depth_registered/sw_registered/image_rect_raw
```

When you move, odometry is computed visually and a larger 3D model of the room is built. If the screen goes red, it means the odometry is lost and the model building process is stopped.

![](https://raw.githubusercontent.com/guermonprez/intel-aero-documents/master/doc_photos/ros_slam.png)

# [Computer Vision with VISP](#computer-vision-with-visp)

You have access to the camera and depth data, you're free to develop your own code.
If you want to go higher, use OpenCV.
And if you need a very high level computer vision library, use VISP. With VISP, you can perform tasks like model based tracking.

[Demo videos are available on their video feed](https://www.youtube.com/user/VispTeam).




