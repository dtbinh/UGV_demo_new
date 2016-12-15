ps -ef | grep python | grep -v grep | awk '{print $2}' | xargs -r kill

roslaunch sml_world starter.launch&
pid=$$
trap "kill $pid" SIGINT
sleep 4


rosservice call /spawn_vehicle "{vehicle_id: 21, class_name: 'ConnectivityVehicle', x: 0.0, y: 0.0, yaw: 0.8, v: 10.0, node_id: -598, toggle_sim: true}"
sleep 1
rosservice call /spawn_vehicle "{vehicle_id: 22, class_name: 'ConnectivityVehicle', x: 0.0, y: 0.0, yaw: 0.8, v: 11.0, node_id: -598, toggle_sim: true}"
sleep 3
rosservice call /spawn_vehicle "{vehicle_id: 23, class_name: 'ConnectivityVehicle', x: 0.0, y: 0.0, yaw: 0.8, v: 15.0, node_id: -598, toggle_sim: true}"
sleep 1
rosservice call /spawn_vehicle "{vehicle_id: 24, class_name: 'ConnectivityVehicle', x: 0.0, y: 0.0, yaw: 0.8, v: 12.0, node_id: -598, toggle_sim: true}"
sleep 2
rosservice call /spawn_vehicle "{vehicle_id: 25, class_name: 'ConnectivityVehicle', x: 0.0, y: 0.0, yaw: 0.8, v: 15.0, node_id: -598, toggle_sim: true}"

