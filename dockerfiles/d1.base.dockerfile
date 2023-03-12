#################################################
## [[ Filename ]]
##   d1.base.dockerfile
##
## [[ Build Image ]]
##   sudo docker build --force-rm -f d1.base.dockerfile -t test:1.0 .
##
## [[ Run Image ]]
##   (GUI off)
##   sudo docker run --name ttest -dit test:1.0
##   (GUI on)
##   xhost +
##   sudo docker run -it -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=unix$DISPLAY --name ttest test:1.0
##
## [[ How to Run Proslam Test App ]]
##   cd /home/catkin_ws && . /home/catkin_ws/devel/setup.sh && rosrun srrg_proslam app 04.txt -use-gui

FROM osrf/ros:kinetic-desktop-full

LABEL org.opencontainers.image.authors="slkumquat@gmail.com"

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update

RUN echo "== Install Basic Tools ==" && \
    # Related to build
    apt-get install build-essential -y && \
    apt-get install cmake -y && \
    apt-get install git -y && \
    apt-get install sudo -y && \
    apt-get install wget -y && \
    apt-get install ninja-build && \
    apt-get install vim -y && \
    apt-get install python3 -y && \
    apt-get install python3-pip -y && \
    # Related to JetBrains CLion Docker develpoment...
    apt-get install -y ssh && \
    apt-get install -y gcc && \
    apt-get install -y g++ && \
    apt-get install -y gdb && \
    apt-get install -y clang && \
    apt-get install -y rsync && \
    apt-get install -y tar && \
    apt-get install -y mesa-utils && \
    # Related to X11 remote disply
    apt-get install -y libgl1-mesa-glx && \
    apt-get install -y libglu1-mesa-dev && \
    apt-get install -y mesa-common-dev && \
    apt-get install -y x11-utils && \
    apt-get install -y x11-apps && \
    apt-get clean

RUN pip3 install pyyaml
RUN pip3 install gitpython

RUN echo "== Install ProSLAM Prerequisites == " && \
    apt-get install libeigen3-dev -y && \
    apt-get install libsuitesparse-dev  -y && \
    apt-get install freeglut3-dev  -y && \
    apt-get install libqglviewer-dev  -y && \
    apt-get install libyaml-cpp-dev -y

RUN apt-get install libssl-dev -y
RUN apt-get install python-catkin-tools -y

RUN echo "== Install CMake Latest version == " && \
    cd home && \
    wget https://github.com/Kitware/CMake/releases/download/v3.20.0/cmake-3.20.0.tar.gz && \
    tar -xvzf cmake-3.20.0.tar.gz && \
    rm cmake-3.20.0.tar.gz && \
    cd cmake-3.20.0 && \
    ./bootstrap && \
    make -j4 && \
    make install && \
    cmake --version

RUN echo "== Install ceres-solver == " && \
    cd /home && \
    apt-get install libgoogle-glog-dev -y && \
    apt-get install libgflags-dev -y && \
    apt-get install libatlas-base-dev -y && \
    apt-get install libeigen3-dev -y

ENV CERES_VERSION="2.0.0"
RUN git clone https://ceres-solver.googlesource.com/ceres-solver && \
    cd ceres-solver && \
    git checkout tags/${CERES_VERSION} && \
    git reset --hard e51e9b46f6 && \
    mkdir build && cd build && \
    cmake .. && \
    make -j$(nproc) install && \
    rm -rf ../../ceres-solver

RUN echo "== Install Evo == " && \
    apt-get install wget -y &&\
    wget https://bootstrap.pypa.io/pip/2.7/get-pip.py  &&\
    sudo python2.7 get-pip.py &&\
    pip2.7 install evo --upgrade --no-binary evo &&\
    pip2.7 install matplotlib --upgrade &&\
    pip2.7 install numpy --upgrade

RUN echo "== Make Catkin Workspace == " && \
    mkdir -p /home/catkin_ws/src

RUN echo "== Install G2O == " && \
    cd /home/catkin_ws/src && \
    git clone https://github.com/yorsh87/g2o_catkin.git

RUN echo "== Clone and Build Source Codes == " && \
    cd /home/catkin_ws/src && \
    git clone https://github.com/Ssellu/vslam-pose-estimation-framework.git && \
    cd /home/catkin_ws/src/vslam-pose-estimation-framework && ./pull_srrg_packages.bash

RUN cd /home/catkin_ws &&\
    echo ". /opt/ros/kinetic/setup.sh" >> /home/.bashrc && \
    echo "export ROS_HOSTNAME=localhost" >> /home/.bashrc && \
    echo "export ROS_MASTER_URI=http://localhost:11311" >> /home/.bashrc && \
    . /home/.bashrc

RUN . /opt/ros/kinetic/setup.sh &&\
    cd /home/catkin_ws &&\
    catkin_make -DGIT_TAG=20200410_git

RUN echo ". /home/catkin_ws/devel/setup.sh" >> /home/.bashrc && \
    . /home/.bashrc
