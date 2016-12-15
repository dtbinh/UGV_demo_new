demoNodeList=(-420 -340 -620 -598 -420 -340 -620 -420 -340 -620 -598 -420 -340 -620 -598 -420 -340 -620 -598 -420 -340 -620 -598)
velocityList=(10 15 20 10 15 20 10 20 10 15 20 10 15 20 10 15 20 10 15 20 10 15 20 10 15 20)
xList=(10 -20 30 40 50 10 10 30 40 50 10 10 -20 30 40 50 10 10 -20 30 40 50 10)
yList=(-10 -100 200 -40 -150 200 -10 200 -40 -150 200 -10 -100 200 -40 -150 200 -10 -100 200 -40 -150 200)


#rosservice call /spawn_vehicle "{vehicle_id: 10, class_name: 'DummyVehicle', x: ${xList[$i]}, y: ${yList[$i]}, yaw: 0.0, v: 0, node_id: -400, toggle_sim: true}"
rosservice call /spawn_vehicle "{vehicle_id: 10, class_name: 'DummyVehicle', x: 0, y: 0, yaw: 0.0, v: 0, node_id: -400, toggle_sim: true}"
rosservice call /spawn_vehicle "{vehicle_id: 20, class_name: 'DummyVehicle', x: 0, y: 50, yaw: 0.0, v: 0, node_id: -400, toggle_sim: true}"
rosservice call /spawn_vehicle "{vehicle_id: 30, class_name: 'DummyVehicle', x: 50, y: 0, yaw: 0.0, v: 0, node_id: -400, toggle_sim: true}"

sleep 1
echo 'Done'
sleep 1
