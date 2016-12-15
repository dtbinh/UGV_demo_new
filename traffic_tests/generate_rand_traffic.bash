for i in `seq 1 10`;
do
	rosservice call /spawn_vehicle "{vehicle_id: $i, class_name: 'RandomDestinationVehicle', x: 0.0, y: 0.0, yaw: 0.0, v: 15.0, node_id: 0, toggle_sim: true}"
	sleep 1
	echo 'Done'
done
