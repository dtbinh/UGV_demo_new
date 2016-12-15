# rosservice call /start_bus_route "{stops: [-214,-158], busid: 1000, startnode: -158}"
# sleep 1
# rosservice call /start_bus_route "{stops: [-222, -400], busid: 1001, startnode: -400}"
#
# sleep 5

for i in `seq 1 10`;
do
    r=$[(RANDOM % 400)]
    x=$[(RANDOM / 500)-30]
    y=$[(RANDOM / 500)-30]
    # nodeList=(-2362 -2364 -2365 -2553 -2624 -2618 -2638 -2630)
    rosservice call /spawn_vehicle "{vehicle_id: $i, class_name: 'DummyVehicle', x: $x, y: $y, yaw: 0.0, v: 20.0, node_id: -$r, toggle_sim: true}"
    sleep 1
done

crossIds=(0 1 2 3 4 5 6 7 8)

while [[ 1 ]]; do
    cid=${crossIds[$((i%8))]}
    rosservice call /crosswalk_passage "{crosswalk_id: $cid, ped_number: 10}"
    i=$((i+1))
    sleep 18
done
