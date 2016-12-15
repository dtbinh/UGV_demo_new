demoNodeList=(-420 -340 -620 -598 -420 -340 -620 -420 -340 -620 -598 -420 -340 -620 -598 -420 -340 -620 -598 -420 -340 -620 -598)
velocityList=(10 15 20 10 15 20 10 20 10 15 20 10 15 20 10 15 20 10 15 20 10 15 20 10 15 20)
xList=(10 -20 30 40 50 10 10 30 40 50 10 10 -20 30 40 50 10 10 -20 30 40 50 10)
yList=(-10 -100 200 -40 -150 200 -10 200 -40 -150 200 -10 -100 200 -40 -150 200 -10 -100 200 -40 -150 200)

for i in `seq 6 9`;
do
    nid=${demoNodeList[$i]}
    rosservice call /spawn_vehicle "{vehicle_id: $i, class_name: 'DummyVehicle', x: ${xList[$i]}, y: ${yList[$i]}, yaw: 0.0, v: ${velocityList[$i]}, node_id: $nid, toggle_sim: true}"
    sleep 1
    echo 'Done'
done
sleep 1
#rosservice call /start_bus_route "{stops: [-258], busid: 0, startnode: -258}"
#rosservice call /start_bus_route "{stops: [-14,-258], busid: 1000, startnode: -322}"
rosservice call /start_bus_route "{stops: [-14, -382], busid: 1001, startnode: -382}"
crossIds=(10 9 8 2 3 5 6 7)

i=0

while [[ 1 ]]; do
    cid=${crossIds[$((i%8))]}
    rosservice call /crosswalk_passage "{crosswalk_id: $cid, ped_number: 10}"
    i=$((i+1))
    sleep 8
done
