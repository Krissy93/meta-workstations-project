# Installation instructions
## Dependencies
Install dependencies first:
- **Basic dependencies:** useful if you have a fresh install of Ubuntu 16.04. We have an Nvidia 1060 GPU so we also install the nvidia driver:
  ```
  sudo apt-get update
  sudo apt-get upgrade
  sudo apt-get install build-essential dkms git
  software-properties-gtk -> additional drivers -> nvidia 384.130 -> apply
  reboot
  ```
- **ROS Kinetic:** it is based on python2, so our whole code is python2 based. Older versions of ROS may still work, as well as newer ones, test them yourself to find out!
  ```
  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
  sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
  sudo apt-get update
  sudo apt-get install ros-kinetic-desktop-full python-rosinstall
  rosdep init
  sudo rosdep fix-permissions
  rosdep update
  echo "source /opt/ros/kinetic/setup.bash" >> /home/$USER/.bashrc
  sudo apt-get install ros-kinetic-cv-bridge ros-kinetic-vision-opencv python-rosinstall python-rosinstall-generator python-wstool build-essential
  ```
  Close the terminal and open a new one, since we modified the bashrc and we need the modifications to take place:
  ```
  mkdir -p workspace/ros/src && cd workspace/ros
  catkin_make
  echo "source /home/$USER/workspace/ros/devel/setup.bash" >> /home/$USER/.bashrc
  ```
- **ROS Industrial:** Official page can ben found [here](http://wiki.ros.org/Industrial/Install). The previous ROS installation should have installed everything already, but to be sure type:
 ```
 apt-get install ros-kinetic-industrial-core ros-kinetic-abb ros-kinetic-universal-robot apt-get install ros-kinetic-ros-canopen
 ```
- **CUDA and CuDNN:** we installed CUDA 9 and CuDNN 7.3, other combinations may also work.<br>
  Download CUDA 9 from [here](https://developer.nvidia.com/cuda-toolkit-archive) and CuDNN 7.3 from [here](https://developer.nvidia.com/rdp/cudnn-archive).
  From the Download folder, open a terminal and install CUDA 9:
  ```
  sudo dpkg -i cuda-repo-ubuntu1604-9-0-local_9.0.176-1_amd64.deb
  sudo apt-key add /var/cuda-repo-9-0-local/7fa2af80.pub
  sudo apt-get update
  sudo apt-get install cuda
  ```
  Afterwards, install CuDNN 7.3:
  ```
  tar -zxvf cudnn*
  sudo cp cuda/include/* /usr/local/cuda/include
  sudo cp cuda/lib64/* /usr/local/cuda/lib64/
  ```
  At this point we need to apply a symbolic link to CuDNN libraries, somehow the installation steps do not do that automatically. By default CUDA should be installed in `/usr/local/cuda-9.0/targets/x86_64-linux/lib/`, but if you're not sure just type in a terminal the following command: `sudo find / -name "*libcudnn*"` to check out the right directory.
  When you found out the right folder, simply type the following:
  ```
  cd /usr/local/cuda-9.0/targets/x86_64-linux/lib/
  sudo rm libcudnn.so
  sudo rm libcudnn.so.7
  sudo ln -s libcudnn.so.7.3.0 libcudnn.so.7
  sudo ln -s libcudnn.so.7 libcudnn.so
  ```
  Check out if everything is correct by typing `ls -lha libcudnn*` in that directory. Symbolic links present an arrow meaning that they point to the corresponding file.
  Finally, ad CUDA path to the bashrc:
  ```
  echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda/lib64' >> ~/.bashrc
  echo 'export PATH=$PATH:/usr/local/cuda/bin' >> ~/.bashrc
  ```
- **libfreenect2** needed to get frames from the Kinect v2 camera. Libfreenect2 main repository can be found [here](https://github.com/OpenKinect/libfreenect2). <br>
If using a RealSense or other camera, install their dependencies (For the RealSense, check out the Librealsense repository [here](https://github.com/IntelRealSense/librealsense). We haven't tested it but it probably works).<br>
  Open a terminal and type:
    ```
    cd workspace
    git clone https://github.com/OpenKinect/libfreenect2.git
    cd libfreenect2
    sudo apt-get install cmake pkg-config libusb-1.0-0-dev libturbojpeg libjpeg-turbo8-dev libglfw3-dev
    mkdir build && cd build
    cmake .. -DENABLE_CXX11=ON -DCMAKE_C_COMPILER=/usr/bin/gcc-6 -DCUDA_PROPAGATE_HOST_FLAGS=off
    make
    make install
    echo '# ATTR{product}=="Kinect2"
    SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02c4", MODE="0666"
    SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02d8", MODE="0666"
    SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02d9", MODE="0666"' > ~/90-kinect2.rules
    sudo mv ~/90-kinect2.rules /etc/udev/rules.d/90-kinect2.rules
    ```
    Connect the Kinect v2 to a USB 3.0 port. Be sure that the USB controller can manage the Kinect, since the camera usually takes up a lot of bandwidth. After this, check the installation of libfreenect by launching Protonect:
    ```
    cd build/bin
    ./Protonect
    ```
- **OpenCV3:** this may take a while (like 40mins to 1h). Note that we only use python2, so everything must be installed for python2 not python3! Open a new terminal and type:
  ```
  sudo apt-get update
  sudo apt-get upgrade
  sudo apt-get remove x264 libx264-dev
  sudo apt-get install build-essential checkinstall cmake pkg-config yasm
  sudo apt-get install git gfortran
  sudo apt-get install libjpeg8-dev libjasper-dev libpng12-dev
  sudo apt-get install libtiff5-dev
  sudo apt-get install libavcodec-dev libavformat-dev libswscale-dev libdc1394-22-dev
  sudo apt-get install libxine2-dev libv4l-dev
  sudo apt-get install libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev
  sudo apt-get install qt5-default libgtk2.0-dev libtbb-dev
  sudo apt-get install libatlas-base-dev
  sudo apt-get install libfaac-dev libmp3lame-dev libtheora-dev
  sudo apt-get install libvorbis-dev libxvidcore-dev
  sudo apt-get install libopencore-amrnb-dev libopencore-amrwb-dev
  sudo apt-get install x264 v4l-utils
  sudo apt-get install libprotobuf-dev protobuf-compiler
  sudo apt-get install libgoogle-glog-dev libgflags-dev
  sudo apt-get install libgphoto2-dev libeigen3-dev libhdf5-dev doxygen 
  sudo apt install libpcl-dev
  sudo apt-get install python-dev python-pip
  sudo -H pip2 install -U pip numpy
  pip install --user --upgrade  numpy scipy matplotlib scikit-image scikit-learn ipython
  git clone https://github.com/opencv/opencv.git
  cd opencv
  git checkout 3.3.1
  cd ..
  git clone https://github.com/opencv/opencv_contrib.git
  cd opencv_contrib
  git checkout 3.3.1
  cd ..
  cd opencv
  mkdir build
  cd build
  cmake -D CMAKE_BUILD_TYPE=RELEASE \
        -D CMAKE_INSTALL_PREFIX=/usr/local \
        -D INSTALL_C_EXAMPLES=ON \
        -D INSTALL_PYTHON_EXAMPLES=ON \
        -D WITH_TBB=ON \
        -D WITH_V4L=ON \
        -D WITH_QT=ON \
        -D WITH_OPENGL=ON \
        -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
        -D BUILD_EXAMPLES=ON ..
  ```
  Ready? Launch this and go eat something in the meantime... Note that `-j8` represents the number of CPU cores, we have 8 so we exploit them all, but you have to use as much as your system supports. Nowadays, `-j4` is usually a safe option!
  ```
  make -j8
  sudo make install
  sudo sh -c 'echo "/usr/local/lib" >> /etc/ld.so.conf.d/opencv.conf'
  sudo ldconfig
  ```
- **pylibfreenect2:** This is the python wrapper needed to correctly set up the Kinect acquisition. Official repository and installation instructions are [here](https://github.com/r9y9/pylibfreenect2/blob/master/docs/installation.rst). Basically after `libfreenect2` installation (the default installation directory is `usr/local`, so check if it is installed there) simply type:
  ```
  pip install pylibfreenect2
  ```
- **iai_kinect2:** the main iai_kinect2 repository is [here](https://github.com/code-iai/iai_kinect2), check it out to understand how to use it! The installation steps are:
  ```
  cd workspace/ros/src/
  git clone https://github.com/code-iai/iai_kinect2.git 
  cd iai_kinect2
  rosdep install -r --from-paths .
  cd ..
  cd ..
  catkin_make -DCMAKE_BUILD_TYPE="Release"
  ```
  To test if everything works, edit the launch file located at `workspace/ros/src/iai_kinect2/kinect2_bridge/launch` and set `publish_tf` to `True`, `depth_method` to `cuda` and `reg_method` to `cl` or `cpu`.<br>
  Open a new terminal and type `roslaunch kinect2_bridge kinect2_bridge.launch`.   If everything works you should see some logs in the terminal, hopefully none in red. Open `rviz` and select `kinect2_link`, add a `PointCloud2` topic and select a random topic to see if everything works correctly. Check out the main page of the node to learn how to use it to the fullest!
- **Tensorflow Object Detection API:** the main repository can be found [here](https://github.com/tensorflow/models/tree/master/research/object_detection). Follow the instructions for installing dependencies and the API itself, we installed tensorflow-gpu since we have an Nvidia GPU. We also installed COCOAPI, it was required for the training phase, but if you plan to use only our provided frozen graph it shouldn't be needed.
## Install the project
In a new terminal, type:
```
cd workspace/ros/src
git clone xxxx
```
- **SMACH module:** this is needed for the State Machine core structure. It has been modified to correct some bugs from the [original project](http://wiki.ros.org/smach). Install the modified version by copying the `executive_smach_visualization` folder (inside the `meta-workstations-project` folder just cloned) into your `$HOME/workspace/ros/src/` directory.
- **State Machine Package:** this is the ROS package we developed, containing the State Machine node, the Gestures node and everything needed by these two. Copy the `state_machine_package` folder into your `$HOME/workspace/ros/src/` directory, and finally install everything: go the `$HOME/workspace/ros` directory and type `catkin_make`.
- **Download the networks needed for the project:** these have been uploaded in a Google Drive folder because of the files dimensions. [Download them from here](https://drive.google.com/open?id=19CisjwZMx2Rh4tP27jMk3f6nTe3vSASh) and move them into the `network` folder of the package.
