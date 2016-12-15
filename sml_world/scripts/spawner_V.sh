 #!/bin/bash

rosservice call spawn_vehicle "{vehicle_id: 2, class_name: 'TruckVehicle', x: 1000.0, y: 0.0, yaw: 0.0, v: 8., node_id: -250, toggle_sim: true}"

rosservice call spawn_vehicle "{vehicle_id: 1, class_name: 'ConnectivityVehicle', x: 20.0, y: 0.0, yaw: 0.8, v: 3., node_id: -250, toggle_sim: true}"
rosservice call spawn_vehicle "{vehicle_id: 3, class_name: 'ConnectivityVehicle', x: 0.0, y: -30.0, yaw: 3.0, v: 5., node_id: -250, toggle_sim: true}"
rosservice call spawn_vehicle "{vehicle_id: 4, class_name: 'ConnectivityVehicle', x: 0.0, y: 0.0, yaw: 0.8, v: 10., node_id: -250, toggle_sim: true}"
#rosservice call spawn_vehicle "{vehicle_id: 5, class_name: 'ConnectivityVehicle', x: 0.0, y: 0.0, yaw: 2.0, v: 2., node_id: -256, toggle_sim: true}"
#rosservice call spawn_vehicle "{vehicle_id: 6, class_name: 'ConnectivityVehicle', x: 0.0, y: 0.0, yaw: 2.0, v: 3., node_id: -256, toggle_sim: true}"

#rosservice call spawn_vehicle "{vehicle_id: 2, class_name: 'TruckVehicle', x: 0.0, y: 0.0, yaw: 0.0, v: 10., m: 1500, node_id: -400, toggle_sim: true}"

#rosservice call spawn_vehicle "{vehicle_id: 2, class_name: 'TruckVehicle', x: 0.0, y: 0.0, yaw: 0.0, v: 0.0, node_id: -400, toggle_sim: true}"

#rosservice call spawn_vehicle "{vehicle_id: 3, class_name: 'TruckVehicle', x: 0.0, y: 0.0, yaw: 0.0, v: 10.0, node_id: -672, toggle_sim: true}"
