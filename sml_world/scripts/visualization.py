#!/usr/bin/env python
# license removed for brevity
"""
Visualization node of the SML-World.
Created on Feb 23, 2016
@author: U{David Spulak<spulak@kth.se>}
@organization: KTH
"""

import rospy
#import roslaunch

import mocap

import sys
import signal

from sml_world.msg import WorldState
from sml_world.srv import GetMapLocation

from sml_modules.visualization_module import Visualization

INITIAL_TIME = 700

def qualisys_info():
    """
    Qualisys data listener.
    @param qs_body:
    """
    stop = False

    def signal_handler(signal, frame):
        stop = True
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    Qs = mocap.Mocap(info=1)
    bodies = Qs.find_available_bodies(printinfo=1)

    #pick the first valid body
    id_body = Qs.get_id_from_name("Iris2")

    body = mocap.Body(Qs, id_body)

    while not stop:
        pose = body.getPose()

#    qs_dict = {}
#    qs_dict[qs_body]['x']
#    qs_dict[qs_body]['y']
#    qs_dict[qs_body]['z']
#    qs_dict[qs_body]['yaw']


def qualisys_info():
    """
    Qualisys data listener.
    @param qs_body:
    """
    stop = False

    def signal_handler(signal, frame):
        stop = True
        print "What?"
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    Qs = mocap.Mocap(info=1)
    bodies = Qs.find_available_bodies(printinfo=1)

    #pick the first valid body
    id_body = Qs.get_id_from_name("Iris2")

    body = mocap.Body(Qs, id_body)

    while not stop:
        pose = body.getPose()
        #rospy.loginfo(pose)
        #rospy.logwarn(pose)

#    qs_dict = {}
#    qs_dict[qs_body]['x']
#    qs_dict[qs_body]['y']
#    qs_dict[qs_body]['z']
#    qs_dict[qs_body]['yaw']


def update_state(ws, vis_module):
    """
    Callback function for topic 'world_state'.
    @param ws: I{(WorldState)} ROS-message of the world state.
    @param vis_module: I{(VisualisationModule)} The initialized visualization
                       module used to show the current state of the simulation.
    @todo: Integrate ROS-messages for the world state.
    """
    ws_dict = {}
    ws_dict['vehicles'] = {}
    ws_dict['sensors'] = {}
    ws_dict['pedestrians'] = {}
    for vs in ws.vehicle_states:
        ws_dict['vehicles'][vs.vehicle_id] = {}
        ws_dict['vehicles'][vs.vehicle_id]['id'] = vs.vehicle_id
        ws_dict['vehicles'][vs.vehicle_id]['class_name'] = vs.class_name
        ws_dict['vehicles'][vs.vehicle_id]['x'] = vs.x
        ws_dict['vehicles'][vs.vehicle_id]['y'] = vs.y
        ws_dict['vehicles'][vs.vehicle_id]['yaw'] = vs.yaw
    for rs in ws.sensor_states:
        ws_dict['sensors'][rs.vehicle_id] = {}
        ws_dict['sensors'][rs.vehicle_id]['id'] = rs.vehicle_id
        ws_dict['sensors'][rs.vehicle_id]['range'] = rs.range
        ws_dict['sensors'][rs.vehicle_id]['angle'] = rs.angle
        ws_dict['sensors'][rs.vehicle_id]['radar_vis'] = rs.radar_vis
    for ps in ws.pedestrian_states:
        ws_dict['pedestrians'][ps.pedestrian_id] = {}
        ws_dict['pedestrians'][ps.pedestrian_id]['id'] = ps.pedestrian_id
        ws_dict['pedestrians'][ps.pedestrian_id]['x'] = ps.x
        ws_dict['pedestrians'][ps.pedestrian_id]['y'] = ps.y
        ws_dict['pedestrians'][ps.pedestrian_id]['yaw'] = ps.yaw
    ws_dict['bus_stop_ids'] = ws.bus_stop_ids
    ws_dict['bus_stop_demands'] = ws.bus_stop_demands
    ws_dict['time'] = ws.time
    ws_dict['traffic_light_nodes'] = ws.traffic_light_nodes
    ws_dict['traffic_light_status'] = ws.traffic_light_status
    vis_module.loop_iteration(ws_dict)

    for td in ws.traffic_demand:
        ws_dict[td.bus_demand]['bus_demand'] = td.bus_demand


def visualizer(vis_module):
    """
    Initialize ROS-node 'visualizer' and start subscriber to 'world_state'.
    @param vis_module: I{(VisualisationModule)} The initialized visualization
                       module used to show the current state of the simulation.
    """
    rospy.Subscriber('world_state', WorldState, update_state, vis_module)
    rospy.spin()
    rospy.logwarn('spinning')


if __name__ == '__main__':
    '''
    Takes up to three system args, which should be bools representing
    whether traffic, traffic lights, and radars should be visualised,
    in that order. For example,
    visulaization.py True False True
    will visualize the traffic on each lanelet and the sensor on each car,
    but not the traffic lights. Note that traffic and traffic lights
    can be turned off in road_network.py; turning them off here just
    stops the visualization of either.
    '''

    # Request the map location.
    rospy.init_node('visualizer')
    rospy.wait_for_service('/get_map_location')
    try:
        get_map = rospy.ServiceProxy('/get_map_location', GetMapLocation)
        resp = get_map()
        base_path = resp.base_path
        map_location = resp.map_location
    except rospy.ServiceException, e:
        raise "Service call failed: %s" % e
    # Initialize the visualization module

    # Args 1, 2 and 3 are booleans, so we need to booleanise them           # 1920, 1080    2000, 1125
    rospy.logwarn(sys.argv)
    if len(sys.argv) > 4:
        vis_module = Visualization(base_path, map_location, 1920, 1080, -1, False, sys.argv[1] == 'True',
                sys.argv[2] == 'True', sys.argv[3] == 'True')
    else:
        vis_module = Visualization(base_path, map_location, 1280, 720, 5, True)

    #First loop with empty values768
    vis_module.loop_iteration({'vehicles' : {}, 'pedestrians' : {}, 'sensors': {}, 'bus_stop_ids' : [],
        'bus_stop_demands' : [], 'time': INITIAL_TIME,
        'traffic_light_nodes': [], 'traffic_light_status': []})
    visualizer(vis_module)
qualisys_info()
