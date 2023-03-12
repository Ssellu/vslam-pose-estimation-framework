FROM test:1.1

CMD [ "label=00 && cd /home/catkin_ws/dataset/\"$label\" && . /home/catkin_ws/devel/setup.sh && rosrun srrg_proslam app \"$label\".txt -c \"/home/catkin_ws/src/vslam-pose-estimation-framework/configurations/configuration_kitti.yaml\" -use-gui" ]
