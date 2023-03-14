#################################################
## [[ Filename ]]
##   d3.build.dockerfile
##
## [[ Build Image ]]
##   sudo docker build --force-rm -f d3.build.dockerfile -t test:1.2 .
##
## [[ Run Image ]]
##   (GUI off)
##   sudo docker run --name test -dit test:1.2
##   (GUI on)
##   xhost +
##   sudo docker run -it -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=unix$DISPLAY --name test test:1.2
##
## [[ How to Run Proslam Test App ]]
##   NOTE : Put the name of target GT into the variable `label` in command below.
##   label=00 && cd /home/catkin_ws/dataset/"$label" && . /home/catkin_ws/devel/setup.sh && rosrun srrg_proslam app "$label".txt -use-gui
##   label=00 && cd /home/catkin_ws/dataset/"$label" && . /home/catkin_ws/devel/setup.sh && rosrun srrg_proslam app "$label".txt -c "/home/catkin_ws/src/vslam-pose-estimation-framework/configurations/configuration_kitti.yaml" -use-gui

FROM test:1.1

RUN echo "== Clone and Build Source Codes == " && \
    cd /home/catkin_ws/src/vslam-pose-estimation-framework && \
    git pull origin main
RUN cd /home/catkin_ws/src && git clone https://github.com/yse/easy_profiler.git && cd easy_profiler && mkdir build install && cd build &&\
    cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=../install .. &&\
    make -j4 &&\
    make install

RUN echo "== Make ProSlam == " && \
    cd /home/catkin_ws/src/vslam-pose-estimation-framework && ./pull_srrg_packages.bash

RUN . /opt/ros/kinetic/setup.sh &&\
    cd /home/catkin_ws &&\
    catkin_make -DGIT_TAG=20200410_git

RUN echo ". /home/catkin_ws/devel/setup.sh" >> /home/.bashrc && \
    . /home/.bashrc

RUN . /opt/ros/kinetic/setup.sh &&\
    cd /home/catkin_ws/src/vslam-pose-estimation-framework && catkin build
