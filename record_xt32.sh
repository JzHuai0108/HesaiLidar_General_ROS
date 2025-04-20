
source devel/setup.bash
roslaunch hesai_lidar hesai_lidar.launch lidar_type:="PandarXT-32" frame_id:="PandarXT-32" \
    timestamp_type:="" server_ip:="192.168.34.43"

source devel/setup.bash
rosrun hesai_lidar recording_controller $HOME/Desktop/temp/hesai/1.bag

# alternatively,
rosservice call /hesai/hesai_lidar/start_stop_recording 1 $HOME/Desktop/temp/hesai/3.bag

rosservice call /hesai/hesai_lidar/start_stop_recording 0 $HOME/Desktop/temp/hesai/3.bag
