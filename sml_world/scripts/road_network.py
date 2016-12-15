#!/usr/bin/env python
# license removed for brevity
"""
RoadModule node of the SML-World.

Created on Feb 29, 2016

@author: U{David Spulak<spulak@kth.se>}
@organization: KTH
"""

import sys
import os
import math
import numpy as np

import rospy

from sml_world.msg import Pose2D
from sml_world.srv import GetMapLocation, GetMapLocationResponse
from sml_world.srv import GetTrajectory, GetTrajectoryResponse
from sml_world.srv import GetNearestNodeId, GetNearestNodeIdResponse
from sml_world.srv import GetCoordinates, GetCoordinatesResponse
from sml_world.srv import SetDestination, Lanelets, LaneletsResponse

from sml_modules.road_module.TrafficManager import TrafficManager

from sml_modules.road_module.RoadModule import RoadModule


class RoadModuleExtend(RoadModule):
    """
    Extension of the class RoadModule.

    This extension is done to be able to add more function parameters to the
    handle_get_map_location method.
    """

    def __init__(self, file_location, traffic_bool, traffic_light_bool):
        """
        Initialize the class RoadModule_new.

        @param file_location: I{(str)} Relative location of the .xml file
                              containing the road network information.

        @param traffic_bool: I{(bool)} Turn
        """
        base_path = os.path.dirname(__file__)
        self.map_location = file_location
        super(RoadModuleExtend, self).__init__(base_path, file_location)
        # Group all nodes of the road network to conneced sets
        self.ll_nodes_dict = {self.osm_lanelet_dict.keys().index(ll_key):
                         ll_ways.right_osm_way.node_ids for ll_key, ll_ways in
                         self.osm_lanelet_dict.items()}

        #Initialise the traffic manager
        #We start the traffic mananger even if traffic is 'off' since
        #We need the ros service for the vehicles to work correctly
        TrafficManager(self.ll_nodes_dict, traffic_on)

        ad_mat = np.asmatrix(self.lanelet_adjacency_matrix)
        ad_mat = 1/ad_mat + 1/ad_mat.T

        ll_sets = [set(i) for i in np.argwhere(ad_mat > 0)]
        ll_supersets = []
        for ll_set in ll_sets:
            overlap = False
            for ll_superset in ll_supersets:
                if ll_set & ll_superset:
                    ll_superset |= ll_set
                    overlap = True
            if not overlap:
                ll_supersets.append(ll_set)
        # Create dictionary that connects nodes to their position.
        self.node_to_xy = {n_id: (n_osm.x, n_osm.y) for n_id, n_osm
                      in self.osm_node_dict.items()}
        # Go through all supersets and group the connected lanelet-nodes.
        self.connected_nodes = []
        #Dictionary containing all of the node locations
        self.all_nodes_loc = {}
        for ll_superset in ll_supersets:
            nodes_dict = {}
            for ll in ll_superset:
                nodes_dict.update({n_id: self.node_to_xy[n_id] for
                                   n_id in self.ll_nodes_dict[ll]})
            self.connected_nodes.append(nodes_dict)

            #Extend to all node locations
            for node, loc in nodes_dict.iteritems():
                if not node in self.all_nodes_loc:
                    self.all_nodes_loc[node] = loc

        if traffic_light_bool:
            super(RoadModuleExtend, self).start_traffic_light_manager()

    def handle_get_map_location(self, req):
        """
        Handle the map location request.

        @param req: I{(GetMapLocation)} Request of the service that provides
                    the map location to client.
        """
        return GetMapLocationResponse(self.base_path, self.map_location)

    def handle_get_trajectory(self, req):
        """
        Handle the get trajectory request.

        @param req: I{(GetTrajectory)} Request of the service that provides a
                    trajectory to follow.
        """
        if req.loop:
            print "get_closed_path_from_node_id"
            tx, ty = self.get_closed_path_from_node_id(req.start_id)
        elif not req.end_id:
            tx = [0]
            ty = [0]
        else:
            print "get_path_between_node_ids"
            tx, ty = self.get_path_between_node_ids(req.start_id,
                                                    req.end_id)
        if np.sum(tx) == float('inf'):
            rospy.logwarn('Unable to find trajectory between ' +
                                str(req.start_id) + ' and ' + str(req.end_id))
            return GetTrajectoryResponse([])
        trajectory = []
        if tx:
            # txp[i] = tx[i+1]
            txp = tx[1:]+[tx[0]]
            # typ[i] = ty[i+1]
            typ = ty[1:]+[ty[0]]
        else:
            txp = []
            typ = []
        for x, y, xp, yp in zip(tx, ty, txp, typ):
            dx = xp - x
            dy = yp - y
            yaw = math.atan2(dy, dx)
            # Make sure yaw is never negative.
            # yaw 0..2pi
            if yaw < 0.:
                yaw += 2*math.pi
            trajectory.append(Pose2D(x, y, yaw))
        return GetTrajectoryResponse(trajectory)

    def handle_get_nearest_nodeid(self, req):
        """
        Get the node ID that is nearest to a vehicles location.

        @param req: I{(GetNearestNodeId)} Request of the service that provides
                    the ID of the node nearest to the sent coordinates.
        @param req.search_all
        """
        if req.dest_id == -1:
            node_list = self.all_nodes_loc
        else:
            # Find the connected node group that contains the destination node.
            for node_list in self.connected_nodes:
                if req.dest_id in node_list.keys():
                    break

        # Then find the node in that group that is closest to the vehicle.
        min_dist = float('inf')
        nearest_id = None
        for n_id, (x, y) in node_list.items():
            min_dist_tmp = math.sqrt(math.pow(x-req.x, 2) +
                                     math.pow(y-req.y, 2))
            if min_dist_tmp < min_dist:
                nearest_id = n_id
                min_dist = min_dist_tmp
        return GetNearestNodeIdResponse(nearest_id)

    def handle_get_coordinates(self, req):
        """
        Get the coordinates from node id

        @param req: I{(GetNearestNodeId)} Request of the service that provides
                    the ID of the node nearest to the sent coordinates.
        """
        #Find the connected node group that containes the original node
        return GetCoordinatesResponse(self.node_to_xy[req.node_id][0], self.node_to_xy[req.node_id][1])

lanelet_info = None

def lanelets_info_response(unused):
    return lanelet_info


def send_lanelet_info(lanelet_dict, all_nodes):
    '''
    Format and send the data for the laneltes dictionary
    over a new publisher called '/lanelets'

    @param lanelet_dict: Dictionary of lanelet_id -> array of node_ids
    '''
    global lanelet_info
    #To understand the data format, see Lanelets.msg
    dict_key_lengths = []
    dict_values = []
    for k, v in lanelet_dict.iteritems():
        dict_key_lengths.append(len(v))
        dict_values.extend(v)
    publisher = rospy.Service('/lanelets', Lanelets, lanelets_info_response)
    node_keys = all_nodes.keys()
    xs = [v[0] for v in all_nodes.values()]
    lanelet_info = LaneletsResponse(dict_key_lengths, dict_values, node_keys,
        xs, [v[1] for v in all_nodes.values()])


def road_network(file_location, traffic_bool, traffic_light_bool):
    """
    Initialize ROS-node 'road_network' and start the services.

    The services provided include:
        - get_map_location:  Returns the location of the map image.
        - get_trajectory:  Returns the shortest trajectory between two nodes.
        - get_nearest_nodeid:  Returns the node-ID nearest to coordinates.

    @param file_location: I{(str)} Location of the road network description
                          that should be loaded.
    @param traffic_bool: I{(bool)} Toggles whether the traffic manager should
                        be turned on.
    """
    rospy.init_node('road_network')
    rospy.logwarn(traffic_bool)
    road_module = RoadModuleExtend(file_location, traffic_bool, traffic_light_bool)
    rospy.Service('/get_map_location', GetMapLocation,
                  road_module.handle_get_map_location)
    rospy.Service('/get_trajectory', GetTrajectory,
                  road_module.handle_get_trajectory)
    rospy.Service('/get_nearest_nodeid', GetNearestNodeId,
                  road_module.handle_get_nearest_nodeid)
    rospy.Service('/get_node_coordinates', GetCoordinates,
                  road_module.handle_get_coordinates)
    send_lanelet_info(road_module.ll_nodes_dict, road_module.all_nodes_loc)
    rospy.spin()


if __name__ == '__main__':
    traffic_on = False
    traffic_light_on = False
    if len(sys.argv) > 1:
        file_location = sys.argv[1]
        traffic_on = sys.argv[2].lower() == 'true'
        traffic_light_on = sys.argv[3].lower() == 'true'

    else:
        msg = "Usage: rosrun sml_world road_network.py <file_location> <Traffic on boolean> <Traffic Lights bool>"
        raise Exception(msg)
    road_network(file_location, traffic_on, traffic_light_on)
