# Scale Truck Control

[![Video Label](http://img.youtube.com/vi/wKmWD8BPldw/0.jpg)](https://youtu.be/wKmWD8BPldw?t=0s)

## I. Hardware

* High-level Controller: Nvidia Jetson AGX Xavier 8GB
* Low-level Controller: Teensy 4.1 (ARM Cortex-M7)
* USB Camera: ELP-USBFHD04H-BL180
* Lidar: RPLidar A3

## II. Software

High-level Controller
* Jetpack 5.1.2, which is based on Ubuntu 20.04 LTS
* OpenCV: 4.4.0 version - include options (GPU, CUDA, CUDNN)
* ROS Noetic Ninjemys

Low-level Controller
* ros-noetic-rosserial-arduino

## III. Demonstration

Demonstration of a platoon with 3 trucks: https://www.youtube.com/watch?v=wKmWD8BPldw

* Intro: 0:00
* Scenario 1: 0:35
* Scenario 2: 1:36
* Case Study: Camera Failure. 2:31
* Emergency Stop: 2:45

## IV. Installation

### Step 1: Install Jetpack 5.1.2 (ubuntu 20.04 LTS)

If not done already, flash the Jetson with Nvidia Jetpack 5.1.2: https://developer.nvidia.com/embedded/jetpack

If the SDK components like CUDA were not installed automatically by Nvidia SDK Manager, then run the following upon logging into the Jetson:
```
sudo apt update
sudo apt upgrade
sudo apt install nvidia-jetpack
```

See [the Nvidia Jetpack installation instructions](https://docs.nvidia.com/jetson/jetpack/install-jetpack/index.html#install-jetpack) for more details.

### Step 2: Build OpenCV 4.4.0 from Source

Run the following commands to install OpenCV's build dependencies:
```
sudo apt update
sudo apt upgrade

sudo apt install -y build-essential cmake
sudo apt install -y pkg-config
sudo apt install -y libjpeg-dev libtiff5-dev libpng-dev
sudo apt install -y ffmpeg libavcodec-dev libavformat-dev libswscale-dev libxvidcore-dev libx264-dev libxine2-dev
sudo apt install -y libv4l-dev v4l-utils
sudo apt install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev 
sudo apt install -y libgtk-3-dev
sudo apt install -y mesa-utils libgl1-mesa-dri libgtkgl2.0-dev libgtkglext1-dev
sudo apt install -y libatlas-base-dev gfortran libeigen3-dev
sudo apt install -y python3-dev python3-numpy
```

Clone OpenCV 4.4.0 and OpenCV's extra modules:
```
mkdir OpenCV && cd OpenCV
git clone -b 4.4.0 https://github.com/opencv/opencv
git clone -b 4.4.0 https://github.com/opencv/opencv_contrib
mkdir opencv/build && cd opencv/build
```

Build OpenCV 4.4.0:
```
cmake -D CMAKE_BUILD_TYPE=RELEASE \
  -D CMAKE_INSTALL_PREFIX=/usr/local \
  -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
  -D WITH_OPENCL=OFF \
  -D WITH_CUDA=ON \
  -D CUDA_ARCH_PTX="" \
  -D WITH_CUDNN=ON \
  -D WITH_CUBLAS=ON \
  -D ENABLE_FAST_MATH=ON \
  -D CUDA_FAST_MATH=ON \
  -D OPENCV_DNN_CUDA=ON \
  -D ENABLE_NEON=ON \
  -D WITH_QT=OFF \
  -D WITH_OPENMP=ON \
  -D WITH_OPENGL=ON \
  -D BUILD_TIFF=ON \
  -D WITH_FFMPEG=ON \
  -D WITH_GSTREAMER=ON \
  -D WITH_TBB=ON \
  -D BUILD_TBB=ON \
  -D BUILD_TESTS=OFF \
  -D WITH_V4L=ON \
  -D WITH_LIBV4L=ON \
  -D OPENCV_ENABLE_NONFREE=ON \
  -D INSTALL_C_EXAMPLES=OFF \
  -D INSTALL_PYTHON_EXAMPLES=OFF \
  -D BUILD_NEW_PYTHON_SUPPORT=ON \
  -D BUILD_opencv_python3=TRUE \
  -D BUILD_EXAMPLES=OFF \
  ..
sudo make install -j10
```

Edit the pkg-config file for OpenCV:
```
sudo vim /usr/lib/pkgconfig/opencv.pc
```

And add the following:
```
prefix=/usr/local
exec_prefix=${prefix}libdir=${exec_prefix}/lib/aarch64-linux-gnu
includedir_old=${prefix}/include/opencv4/opencv
includedir_new=${prefix}/include/opencv4

Name: OpenCV
Description: Open Source Computer Vision Library
Version: 4.4.0
Libs: -L${exec_prefix}/lib/aarch64-linux-gnu -lopencv_dnn -lopencv_gapi -lopencv_highgui -lopencv_ml -lopencv_objdetect -lopencv_photo -lopencv_stitching -lopencv_video -lopencv_calib3d -lopencv_features2d -lopencv_flann -lopencv_videoio -lopencv_imgcodecs -lopencv_imgproc -lopencv_core
Libs.private: -ldl -lm -lpthread -lrt
Cflags: -I${includedir_old} -I${includedir_new}
```

### Step 3: Install ROS

Add the ROS repositories to `/etc/apt/sources.list.d/`:
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

Add apt keys:
```
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

Update to fetch the APT repository and install ROS Noetic:
```
sudo apt update 
sudo apt install ros-noetic-desktop-full
```

ROS Noetic provides `setup.bash`, a script which configures the current shell's environment. Do one or both of the following:
```
# To update the environment on startup for every future shell session:
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# To update the environment for the current session:
source /opt/ros/noetic/setup.bash
```

### Step 4: Create a Catkin Workspace

Make a directory for the catkin workspace and run catkin_make:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```

### Step 5: (Optional) Install Pip and PyTorch

If you would like to do Pytorch development on the Jetson, complete this step.

Install pip for Python 3.8:
```
sudo apt install python3-pip
```

[Install PyTorch](https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/index.html#prereqs-install):
```
pip3 install --no-cache https://developer.download.nvidia.cn/compute/redist/jp/v511/pytorch/torch-2.0.0+nv23.05-cp38-cp38-linux_aarch64.whl
```

[Build Torchvision from source and install](https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048):
```
sudo apt install libjpeg-dev zlib1g-dev libpython3-dev libopenblas-dev libavcodec-dev libavformat-dev libswscale-dev
git clone --branch v0.15.1 https://github.com/pytorch/vision torchvision
cd torchvision
export BUILD_VERSION=0.15.0
python3 setup.py install --user
```

### Step 6: Clone scale_truck_control and Dependencies

Enter `~/catkin_ws/src`, which was created in Step 5, then clone scale_truck_control and each of its dependencies:
```
cd ~/catkin_ws/src
```
* [scale_truck_control](https://github.com/SuneelFreimuth/scale_truck_control.git)
    ```
    git clone https://github.com/SuneelFreimuth/scale_truck_control.git 
    ```
* [scale_truck_control_lane_detector](https://github.com/SuneelFreimuth/scale_truck_control_lane_detector.git)
    ```
    git clone https://github.com/SuneelFreimuth/scale_truck_control_lane_detector.git 
    ```
* [scale_truck_control_msgs](https://github.com/SuneelFreimuth/scale_truck_control_msgs.git)
    ```
    git clone https://github.com/SuneelFreimuth/scale_truck_control_msgs.git 
    ```
* [common_msgs](https://github.com/ros/common_msgs/)
    ```
    git clone https://github.com/ros/common_msgs.git
    ```
* [usb_cam](https://github.com/ros-drivers/usb_cam/)
    ```
    git clone https://github.com/ros-drivers/usb_cam.git
    ```
* [ros_rplidar](https://github.com/robopeak/rplidar_ros/)
    ```
    git clone https://github.com/robopeak/rplidar_ros.git
    ```
* [obstacle_detector](https://github.com/tysik/obstacle_detector)
    ```
    git clone https://github.com/tysik/obstacle_detector.git
    ```
* [laser_filters](https://github.com/ros-perception/laser_filters/)
    ```
    git clone -b noetic-devel https://github.com/ros-perception/laser_filters.git 
    ```
* [vision_opencv](https://github.com/ros-perception/vision_opencv/) (vision_opencv, image_geometry, cv_bridge)
    ```
    git clone -b noetic https://github.com/ros-perception/vision_opencv.git
    ```

If you installed Pytorch and cloned SuneelFreimuth/scale_truck_control_lane_detector, install its dependencies:
```
pip3 install -r ~/catkin_ws/src/scale_truck_control_lane_detector/requirements.txt
```

Install rosserial-arduino for ROS Noetic for communication with the low-level controller:
```
sudo apt install ros-noetic-rosserial ros-noetic-rosserial-arduino
```

Finally, to build scale_truck_control, enter the Catkin workspace and run `catkin_make`:
```
cd ~/catkin_ws/
catkin_make
```

Make sure to run `devel/setup.bash` to make compiled packages discoverable:
```
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

## Run

After building scale_truck_control as described in the previous section, run `roslaunch`:
```
roslaunch scale_truck_control LV.launch
```

To use SuneelFreimuth/scale_truck_control_lane_detector:
```
roslaunch scale_truck_control LV_deep_learning.launch
```

