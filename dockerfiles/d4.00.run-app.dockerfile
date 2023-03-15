FROM test:1.2

CMD [ "label=00 && cd /home/catkin_ws/dataset/"$label" && . /home/catkin_ws/devel/setup.sh && rosrun srrg_proslam app "$label".txt" ]