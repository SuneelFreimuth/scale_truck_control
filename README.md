# Scale Truck Control

[![Video Label](http://img.youtube.com/vi/wKmWD8BPldw/0.jpg)](https://youtu.be/wKmWD8BPldw?t=0s)

## I. Hardware
* High-level Controller: Nvidia Jetson AGX Xavier 8GB
* Low-level Controller: OpenCR 1.0 (ARM Cortex-M7)
* USB Camera: ELP-USBFHD04H-BL180
* Lidar: RPLidar A3

## II. Software
High-level Controller
* Jetpack: 4.5.1 version - Ubuntu 18.04 LTS
* OpenCV: 4.4.0 version - include options (GPU, CUDA, CUDNN)
* ZeroMQ: stable version
* ROS 1: melodic version

Low-level Controller
* ros-melodic-rosserial-arduino

## III. Demonstration
Demonstration of a platoon with 3 trucks: https://www.youtube.com/watch?v=wKmWD8BPldw

* Intro: 0:00
* Scenario 1: 0:35
* Scenario 2: 1:36
* Case Study: Camera Failure. 2:31
* Emergency Stop: 2:45

## IV. Installation
### Step 1: Install Jetpack 4.5.1 (ubuntu 18.04 LTS)
If not done already, flash the Jetson with Nvidia Jetpack 4.5.1: https://developer.nvidia.com/embedded/jetpack

### Step 2: Build OpenCV 4.4.0 from Source
If OpenCV has already been installed on the Jetson, run the following commands to remove it (not necessary if the Jetson has just been flashed with Jetpack):
```
sudo apt-get purge  libopencv* python-opencv
sudo apt-get autoremove
sudo find /usr/local/ -name "*opencv*" -exec rm -i {} \;
```

Run the following commands to install OpenCV's build dependencies:
```
sudo apt-get update
sudo apt-get upgrade

sudo apt-get -y install build-essential cmake
sudo apt-get -y install pkg-config
sudo apt-get -y install libjpeg-dev libtiff5-dev libpng-dev
sudo apt-get -y install ffmpeg libavcodec-dev libavformat-dev libswscale-dev libxvidcore-dev libx264-dev libxine2-dev
sudo apt-get -y install libv4l-dev v4l-utils
sudo apt-get -y install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev 
sudo apt-get -y install libgtk-3-dev
sudo apt-get -y install mesa-utils libgl1-mesa-dri libgtkgl2.0-dev libgtkglext1-dev
sudo apt-get -y install libatlas-base-dev gfortran libeigen3-dev
sudo apt-get -y install python3-dev python3-numpy
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
  -D CUDA_ARCH_BIN=7.2 \
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
  -D OPENCV_GENERATE_PKGCONFIG=ON \
  -D BUILD_EXAMPLES=OFF \
  ..
sudo make install -j8
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

### Step 3: Install jetson-stats
```
sudo -H pip3 install jetson-stats
jetson_release
```

### Step 4: Install ZeroMQ

Install libsodium, a dependency of libzmq:
```
wget https://download.libsodium.org/libsodium/releases/libsodium-1.0.18-stable.tar.gz
tar -xvf libsodium-*
rm *libsodium-1.0.18-stable.tar.gz
cd libsodium-stable
./configure
sudo make clean
sudo make -j8
sudo make install 
sudo ldconfig
cd ../
```

Build, check, and install the latest version of ZeroMQ
```
wget https://github.com/zeromq/libzmq/archive/master.zip
unzip master.zip
rm master.zip
cd libzmq-master
./autogen.sh 
./configure --with-libsodium
mkdir build
cd build && cmake .. -DENABLE_DRAFTS=ON
sudo make -j8 install
sudo ldconfig
cd ../../
```

Finally, install cppzmq:
```
wget https://github.com/zeromq/cppzmq/archive/master.zip
unzip master.zip
rm master.zip
cd cppzmq-master
mkdir build && cd build
cmake .. -DENABLE_DRAFTS=ON
sudo make -j8 install
sudo ldconfig
cd ../../
```

TODO: I don't think the following is necessary, but do it anyways.
>Setup the path
>~~~
>sudo cp -R /usr/local/lib/* /usr/lib
>~~~

### Step 5: Install ROS

Add the ROS repositories to `/etc/apt/sources.list.d/`:
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

Add apt keys:
```
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

Install ROS Melodic:
```
sudo apt install ros-melodic-desktop-full
```

ROS Melodic provides `setup.bash`, a script which configures the current shell's environment. Do one or both of the following:
```
# To update the environment on startup for every future shell session:
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc


# To update the environment for the current session:
source /opt/ros/melodic/setup.bash
```

Install dependencies for either Python 2 or 3:
* Python 2:
    ```
    sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
    sudo apt install python-rosdep
    sudo rosdep init
    rosdep update
    ```
* Python 3
    ```
    sudo apt-get install python3-pip python3-yaml
    sudo pip3 install rospkg catkin_pkg
    ```

### Step 6: Create a Catkin Workspace

Make a directory for the catkin workspace and run catkin_make:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```

If using Python 3:
```
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
```

### Step 7: Clone scale_truck_control and Dependencies

Enter `~/catkin_ws/src`, which was created in the last step, then clone scale_truck_control and each of its dependencies:
```
cd ~/catkin_ws/src
```
* scale_truck_control
    ```
    git clone https://github.com/SuneelFreimuth/scale_truck_control.git 
    ```
* [geometry_msgs](https://github.com/ros/common_msgs/) (provided by `common_msgs`)
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
    git clone -b kinetic-devel https://github.com/ros-perception/laser_filters.git 
    ```
* [vision_opencv](https://github.com/ros-perception/vision_opencv/) (vision_opencv, image_geometry, cv_bridge)
    ```
    git clone -b melodic https://github.com/ros-perception/vision_opencv.git
    ```

vision_opencv requires the following modifications:
* For `~/catkin_ws/src/vision_opencv/cv_bridge/CMakelist.txt`:
    ```
    --find_package(Boost REQUIRED python37)
    ++find_package(Boost REQUIRED python)
    ```
* For `~/catkin_ws/src/vision_opencv/cv_bridge/src/module.hpp`:
    ```
    --static void * do_numpy_import( )
    ++static void do_numpy_import( )
    --return nullptr;
    ```

Install rosserial-arduino for ROS Melodic for communication with the low-level controller:
```
sudo apt-get install ros-melodic-rosserial ros-melodic-rosserial-arduino
```

TODO: I believe the following is only necessary to run the GUI controller. Skip.
```
sudo apt-get install libarmadillo-dev
sudo apt-get install qtbase5-dev 
```

Finally, to build scale_truck_control, enter the Catkin workspace and run `catkin_make`:
```
cd ~/catkin_ws/
catkin_make
```

## Run

After building scale_truck_control as described in the previous section, run `roslaunch`:
```
roslaunch scale_truck_control LV.launch
```
