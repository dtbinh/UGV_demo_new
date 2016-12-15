# Kill previous running ros nodes
#ps -ef | grep python | grep -v grep | awk '{print $2}' | xargs -r kill

#roslaunch sml_world starter.launch&
pid=$$
#trap "kill $pid" SIGINT
#sleep 5
rosservice call /spawn_vehicle "{vehicle_id: 0, class_name: 'DummyVehicle', x: 0.0, y: 0.0, yaw: 0.8, v: 18.0, node_id: -420, toggle_sim: true}"
#rosservice call /spawn_vehicle "{vehicle_id: 1, class_name: 'DummyVehicle', x: -5.0, y: 2.0, yaw: 3.9, v: 12.0, node_id: -30, toggle_sim: true}"
#rosservice call /spawn_vehicle "{vehicle_id: 2, class_name: 'DummyVehicle', x: 0.0, y: 0.0, yaw: 0.8, v: 8.0, node_id: -420, toggle_sim: true}"
rosservice call /spawn_vehicle "{vehicle_id: 3, class_name: 'BusVehicle', x: -5.0, y: 2.0, yaw: 3.9, v: 5.0, node_id: -340, toggle_sim: true}"
rosservice call /spawn_vehicle "{vehicle_id: 10, class_name: 'WifiVehicle', x: 5.0, y: 2.0, yaw: 3.9, v: 20.0, node_id: -620, toggle_sim: true}"


rosservice call /spawn_vehicle "{vehicle_id: 21, class_name: 'ConnectivityVehicle', x: 0.0, y: 0.0, yaw: 0.8, v: 20.0, node_id: -598, toggle_sim: true}"
sleep 2
#rosservice call /spawn_vehicle "{vehicle_id: 22, class_name: 'ConnectivityVehicle', x: 0.0, y: 0.0, yaw: 0.8, v: 20.0, node_id: -598, toggle_sim: true}"
#sleep 2
#rosservice call /spawn_vehicle "{vehicle_id: 23, class_name: 'ConnectivityVehicle', x: 0.0, y: 0.0, yaw: 0.8, v: 20.0, node_id: -598, toggle_sim: true}"
#sleep 2
#rosservice call /spawn_vehicle "{vehicle_id: 24, class_name: 'ConnectivityVehicle', x: 0.0, y: 0.0, yaw: 0.8, v: 20.0, node_id: -598, toggle_sim: true}"
#sleep 2
