#################################################
## [[ Filename ]]
##   d2.load-data.dockerfile
##
## [[ Build Image ]]
##   sudo docker build --force-rm -f d2.load-data.dockerfile -t test:1.1 .
##
## [[ Run Image ]]
##   (GUI off)
##   sudo docker run --name test -dit test:1.1
##   (GUI on)
##   xhost +
##   sudo docker run -it -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=unix$DISPLAY --name test test:1.1
##
## [[ How to Run Proslam Test App ]]
##   NOTE : Put the name of target GT into the variable `label` in command below.
##   label=00 && cd /home/catkin_ws/dataset/"$label" && . /home/catkin_ws/devel/setup.sh && rosrun srrg_proslam app "$label".txt -use-gui
##   label=00 && cd /home/catkin_ws/dataset/"$label" && . /home/catkin_ws/devel/setup.sh && rosrun srrg_proslam app "$label".txt -c "/home/catkin_ws/src/vslam-pose-estimation-framework/configurations/configuration_kitti.yaml" -use-gui
FROM test:1.0

RUN echo "== Download KITTI Datasets == "

RUN rm -rf /home/catkin_ws/dataset &&\
    mkdir /home/catkin_ws/dataset

COPY dataset/*.tar.gz /home/catkin_ws/dataset/

RUN cd /home/catkin_ws/dataset &&\
    for f in *.tar.gz; \
        do \
            mkdir "${f%.tar.gz}" &&\
            tar -xzvf "${f}" --directory "${f%.tar.gz}"; \
    done &&\
    rm -rf *.tar.gz

