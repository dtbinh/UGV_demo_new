#!/usr/bin/env python
# license removed for brevity
'''
This file implements functions to read the OSM Map file
that describes the SML World environment.

This file is used by RoadModule.py

'''

import sys
import os
import numpy as np
import copy
import rospy

from sml_modules.road_module.RoadModule import RoadModule
from sml_world.srv import GetMapLocation, GetMapLocationResponse
from sml_world.srv import StartBusRoute, StartBusRouteResponse
from sml_modules.bus_vehicle_model import BusVehicle
from sml_world.msg import BusInformation, BusStops
from sml_world.srv import SpawnVehicle, SetBool, AddBusStop, AddBusStopResponse
from sml_world.srv import BusRouting, BusRoutingResponse, CloseBusStop
from sml_modules.demand_model import DemandModel
from random import randint
import IPython


class BusNetworkModule(RoadModule):
    """
    Extention of the RoadModule class to describe bus stop locations

    Auto_bus_stops toggles whether buses will be started and stopped
    automatically by the in check_for_needed_buses and
    check_for_deleted_buses.
    """

    def __init__(self, auto_bus_stops=True):

        rospy.wait_for_service('/get_map_location')
        try:
            get_map = rospy.ServiceProxy('/get_map_location', GetMapLocation)
            resp = get_map()
            base_path = resp.base_path
            file_location = resp.map_location
        except rospy.ServiceException, e:
            raise "Service call failed: %s" % e

        super(BusNetworkModule, self).__init__(base_path, file_location)

        #Get all id tags with stop tag
        self.bus_station_nodes = self.osm_node_tag_dict['stop']
        self.enableAddBus = True

        #Let sml central know about the bus stops
        self.pub_stops = rospy.Publisher('current_bus_stops', BusStops, queue_size=10)

        #Tracks the nodes that a bus id has been assigned by user
        self.responsible_nodes_by_bus = {}

        self.num_stops = len(self.bus_station_nodes)

        rospy.Service('/bus_rerouting', BusRouting,
                                self.bus_needs_rerouting)

        #Dictionary of bus ids to service states
        self.serv_states = {}

        self.calculate_distances()

        self.demand_model = DemandModel()
        for nodeid in self.bus_station_nodes:
            self.demand_model.register_bus_stop(nodeid, 30)

        self.auto_bus_stops = auto_bus_stops

        # Wait until sml world is running
        rospy.wait_for_service('spawn_vehicle')

        self.bus_stop_status = BusStops()
        self.bus_stop_status.bus_stops = self.bus_station_nodes
        self.get_and_publish_demands(None)
        #Example how to add demand (use negatives to subtract)

        rospy.Subscriber('update_demand_stats',
                        BusStops, self.get_and_publish_demands)

        ######################################

    def calculate_distances(self):
        '''
        This method calculates all of the distances between bus stops
        and thus creates the overlay from road network to bus network.

        Should be called on initialisation or when a new bus stop is
        added.

        '''
        #Dictionary of tuples to ints that represents distances between node_ids.
        #This is an overlay on top of the road network representing
        self.distance_dict = {}
        for i in self.bus_station_nodes:
            for j in self.bus_station_nodes:
                if(i != j):
                    self.distance_dict[(i, j)] = self.get_path_distance_between_node_ids(i, j)
                else:
                    self.distance_dict[(i, j)] = 0

    def handle_get_map_location(self, req):
        """
        Handle the map location request.

        @param req: I{(GetMapLocation)} Request of the service that provides
        the map location to client.
        """
        return GetMapLocationResponse(self.base_path, self.map_location)

    def three_highest_demands(self, nodes, demands):
        '''
        Finds three highest values in dictionary nodes. Could be improved using
        heapq, but for small number of nodes, it doesn't terribly matter.
        '''
        highest = [(-1, -1), (-2, -2), (-3, -3)]
        for i in range(len(nodes)):
            n_id = nodes[i]
            demand = demands[i]
            if demand > highest[0][1]:
                highest.insert(0, (n_id, demand))
            elif demand > highest[1][1]:
                highest.insert(1, (n_id, demand))
            elif demand > highest[2][1]:
                highest.insert(2, (n_id, demand))
            else:
                continue
            highest.pop() #No longer need last element
        return highest

    def check_for_needed_buses(self, bus_stop_demands):
        '''
        We want our bus system to dynamically respond to demand, we will
        check whether a new bus needs to be added at defined by:

        If our TOTAL bus demand / number of buses exceeds 200, then we will start a bus at the
        three most in demand stops.
        '''




        if len(self.bus_station_nodes) < 3:
            return
        total_demand = sum(bus_stop_demands)
        if not self.responsible_nodes_by_bus or total_demand / len(self.responsible_nodes_by_bus) < 200:
            return

        top_3 = self.three_highest_demands(self.bus_station_nodes, bus_stop_demands)

        #Determine a new id that is unique
        bus_stop_ids = [int(n[0]) for n in top_3]
        new_id = randint(100,999)
        while new_id in self.responsible_nodes_by_bus:
            new_id = randint(100,999)

        # rospy.logwarn('bus_station_nodes: ' + str(self.bus_station_nodes) + ', bus_stop_demands: ' + str(bus_stop_demands))
        # rospy.logwarn('responsible_nodes_by_bus: ' + str(self.responsible_nodes_by_bus))
        self.add_bus(new_id, bus_stop_ids)


    def get_and_publish_demands(self, unused):
        self.bus_stop_status.bus_stop_demands = self.demand_model. \
            get_all_demands(self.bus_station_nodes)
        #self.check_for_deleted_buses()
        self.check_for_needed_buses(self.bus_stop_status.bus_stop_demands)
        self.pub_stops.publish(self.bus_stop_status)

    def check_for_deleted_buses(self):
        '''
        Delete a bus if it follows the following criteria:

        It's total demands of all it's stops is under 50. We never want total
        have no buses on the road.

        TODO: Still to be finished, as buses are only stopped, not removed from road
        '''
        if len(self.responsible_nodes_by_bus) == 1:
            return
        to_delete = []
        for bus_id, nodes in self.responsible_nodes_by_bus.iteritems():
            demands = self.bus_stop_status.bus_stop_demands
            total_dem = sum([demands[i] for i in range(len(demands)) if demands[i] in nodes])
            if total_dem < 50:
                to_delete.append(bus_id)

        #Have to delete after because the size change is too much for Python to handle
        for i in to_delete:
            del self.responsible_nodes_by_bus[i]
            try:
                toggle_vehicle = '/vehicle_' + str(i) + '/toggle_simulation/'
                bus_kill = rospy.ServiceProxy(toggle_vehicle, SetBool)
                bus_kill(False)
            except rospy.ServiceException, e:
                raise Exception(e)


    def add_demand(self, node, num):
        self.demand_model.add_demand(node, num)

    def service_add_bus(self, req):
        self.enableAddBus = True
        stops = list(req.stops)
        self.add_bus(req.busid, stops)
        return True

    def add_bus(self, bus_id, responsible_nodes):
        '''
        Service for adding a bus with a list of nodes that it is responsible for
        picking up passengers from the defined stops. Startnode is currently set
        to a drawn lot on the Kista Map
        '''
        if not self.enableAddBus:
            return
        for node in responsible_nodes:
            if not node in self.bus_station_nodes:
                raise NameError(str(node) + ' is not the location of a bus node!')

        self.responsible_nodes_by_bus[bus_id] = responsible_nodes

        spawn_vehicle = '/spawn_vehicle'
        rospy.wait_for_service(spawn_vehicle)
        try:
            bus_launch = rospy.ServiceProxy(spawn_vehicle, SpawnVehicle)
            # All the 0s will be unused in bus creation because startnode accounts for it
            response = bus_launch(bus_id, BusVehicle.__name__,
                0, 0, 3, 20., None, True)
            rospy.logwarn('Successfully added one bus with responsible stops: ' + str(responsible_nodes))
            # 4.71 = 3 pi / 2
            self.enableAddBus = False
        except rospy.ServiceException, e:
            self.enableAddBus = False
            rospy.logerr('Bus route creation failed')
            raise Exception(e)
        return StartBusRouteResponse()

    def bus_needs_rerouting(self, data):
        return self.bus_needs_rerouting_logic(data.busid, data.current_node_id)

    def bus_needs_rerouting_logic(self, bus_id, start_node):
        responsible_nodes = copy.deepcopy(self.responsible_nodes_by_bus[bus_id])
        new_cycle = self.calculate_best_cycle(responsible_nodes, start_node)
        stations_cycle = [self.osm_node_dict[i] for i in new_cycle]
        return BusRoutingResponse(new_cycle)

    def bus_information_update(self, bus_info):
        # Implement some stuff later
        print('Nothing yet')

    def calculate_best_cycle(self, responsible_nodes, start_node):
        '''
        Finds lowest cost cycle on responsible_nodes that touches all nodes.
        Note that this is similar to the traveling salesman problem, which is famously
        NP-hard.
        Thus, we use a greedy algorithm to get an approximate answer.
        There may be a point in time when we want to consider the Christofides algorithm,
        which includes the use of spanning trees.
        @param responsible_nodes: List of integer nodes for bus to go to
        @param start_node: int of start node: Does not necessarily have to be a bus stop
        '''
        cycle = []
        dist = 0
        nodes_left = list(responsible_nodes)

        #Explicitly define first move since starting node may not be a bus stop
        #However, we will magically transport it to the first bus stop in order
        #to it look nice visually
        possible_first_moves = [(i, self.get_path_distance_between_node_ids(start_node, i))
                                                    for i in nodes_left]
        min_first_move = min(possible_first_moves, key = lambda t: t[1])
        cur_node = min_first_move[0]
        ret = self.calculate_best_cycle_exhaustively(responsible_nodes, cur_node)
        return ret[1]

    def calculate_best_cycle_exhaustively(self, responsible_nodes, start_node):
        '''
        NP-hard solution that searches through all possible paths to
        find the optimal path through all the nodes.

        @param responsible_nodes: List of bus nodes which bus must cycle through
        @param start_node: Bus id node that bus will go to first. Must be a bus_id.

        '''
        responsible_nodes.remove(start_node)
        if not responsible_nodes:
            return (0, [start_node])

        possible_paths = []
        for target in responsible_nodes:
            dist_to_next = self.get_path_distance_between_node_ids(start_node, target)
            if dist_to_next != float('inf'):
                ret = self.calculate_best_cycle_exhaustively(copy.deepcopy
                    (responsible_nodes), target)
                cost_found = ret[0] + dist_to_next
                path_found = ret[1]
                path_found.insert(0, start_node)
                possible_paths.append((cost_found, path_found))

        if not possible_paths:
            return (float('inf'), possible_paths)
        return min(possible_paths, key = lambda t: t[0])

    def add_bus_stop(self, bus_stop_params):
        '''
        Adds a bus stop, and recalculates the distances between stops
        '''
        self.demand_model.register_bus_stop(bus_stop_params.node_id, 30)
        self.bus_station_nodes.append(bus_stop_params.node_id)
        self.calculate_distances()
        return AddBusStopResponse()

    def close_bus_stop(self, req):
        bus_stop = req.bus_stop_id
        for bus_id, bus_nodes in self.responsible_nodes_by_bus.iteritems():
            if bus_stop in bus_nodes:
                bus_nodes.remove(bus_stop)
                #TODO: At some point get actual start node from bus
                self.bus_needs_rerouting_logic(bus_id, bus_nodes[0])



def bus_network():
    rospy.init_node('bus_network')
    bus_module = BusNetworkModule()
    rospy.Service('/start_bus_route', StartBusRoute,
        bus_module.service_add_bus)
    rospy.Subscriber('/bus_information', BusInformation,
                bus_module.bus_information_update)
    rospy.Service('/add_bus_stop', AddBusStop,
                bus_module.add_bus_stop)
    rospy.Service('/close_bus_stop', CloseBusStop,
        bus_module.close_bus_stop)
    rospy.spin()

if __name__ == '__main__':
    bus_network()
