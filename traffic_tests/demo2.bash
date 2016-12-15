
# Kill previous running ros nodes
ps -ef | grep python | grep -v grep | awk '{print $2}' | xargs -r kill -9

roslaunch sml_world starter.launch&

sleep 5

rosservice call /spawn_vehicle "{vehicle_id: 1, class_name: 'SemiControlledVehicle', x: 0.0, y: 0.0, yaw: 0.8, v: 15.0, node_id: 0, toggle_sim: true}"

# Developer will control semi controlled vehicle, sending it various places

sleep 9

for i in `seq 1 3`;
do
    rosservice call /add_traffic $i 5
done