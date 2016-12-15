"""
Traffic Manager module for the SML World.

Created on June 16, 2016

@author: U{Daniel Carballal<carba@kth.se>}
@organization: KTH
"""

import rospy
from sml_world.msg import GetTime, VehicleTrafficUpdate, LaneletTraffic
from sml_world.srv import NodeTraffic, AddTraffic, AddTrafficResponse, NodeTrafficResponse
import sml_modules.SMLClock as SMLClock
from collections import OrderedDict


TRAFFIC_CLOCK_LIM = 100
class TrafficManager():
    '''
    Class that defines the traffic state of the SML World. Created upon
    creation of the road network. Should be able to return the 
    number of cars that have passed through a lanelet in the last hour.
    '''
    def __init__(self, lanelets_to_nodes, toggle):
        '''
        @param nodes_to_lanelets: I{(dictionary)} which maps the 
        node_id to an array of lanelets it is in. Most of these 
        arrays are one or two elements long, so it is not 
        trememndously costly

        @param toggle: I{(bool)} Essentially turn traffic on/off. 
        If off, we'll just output a constant for traffic. 
        '''
        self.toggle = toggle

        #create two new dictionaries:
        #lanelet_traffic_history will contain an ordered dictionary
        #vehicle id's to the times they entered the lanelet.
        self.lanelet_traffic_history = {}
        self.nodes_to_lanelets = {}
        self.base_traffic_level = {}

        #For some optimisations
        self.vehicle_lanelets = {}
        
        self.time = 0 # Temporary time value


        for lanelet, nodes in lanelets_to_nodes.iteritems():
            self.lanelet_traffic_history[lanelet] = OrderedDict()
            self.base_traffic_level[lanelet] = 0
            for node in nodes:
                if node in self.nodes_to_lanelets:
                    self.nodes_to_lanelets[node].append(lanelet)
                else:
                    self.nodes_to_lanelets[node] = [lanelet]

        # We deal only with messages because of the high quantity
        # of throughput we need for this data.
        rospy.Subscriber('get_time', GetTime, self.update_time)

        rospy.Subscriber('/update_traffic', VehicleTrafficUpdate, 
            self.update_traffic)

        self.traffic_pub = rospy.Publisher('/lanelet_traffic_data', 
                                            LaneletTraffic, queue_size=10)

        rospy.Service('/node_traffic_service', NodeTraffic, self.give_traffic_at_node)
        self.initialise_lanelet_traffics()

        rospy.Service('/add_traffic', AddTraffic, self.add_traffic)

    def initialise_lanelet_traffics(self):
        for lane in self.lanelet_traffic_history.keys():
            self.publish_lanelet_info(lane)

    def publish_lanelet_info(self, lane):
        num_cars_passed = len(self.lanelet_traffic_history[lane])
        traffic_amount = num_cars_passed + self.base_traffic_level[lane]
        if self.toggle:
            self.traffic_pub.publish(lane, traffic_amount)

    def test_old_traffic_entry(self):
        '''
        Check if any of the values in our traffic dictionaries are too 'old';
        we throw out any values which were inserted more than an hour after
        '''
        for lanelet, odict in self.lanelet_traffic_history.iteritems():

            #While the dicitonary is not empty and our front item is too old
            dict_changed = False
            while odict and SMLClock.compare_times(odict.items()[0][1], self.time) \
                    > TRAFFIC_CLOCK_LIM:
                i = odict.popitem(last=False) #False for FIFO
                dict_changed = True
            if dict_changed:
                self.publish_lanelet_info(lanelet)

    def update_time(self, data):
        '''
        Update clock and check if we need to throw out any existing traffic entries.
        '''
        self.time = data.time
        self.test_old_traffic_entry()

    def register_vehicle(self, vehicle_id):
        '''
        Register new vehicle to Traffic Manager
        '''
        self.vehicle_lanelets[vehicle_id] = OrderedDict()

    def is_vehicle_in_lanelet(self, vehicle_id, node_lane, node_id):
        for vehicle_lane, vehicle_node_id in self.vehicle_lanelets[vehicle_id].iteritems():
            if node_lane == vehicle_lane and node_id != vehicle_node_id:
                return True
        return False

    def update_vehicle_lanelets(self, vehicle_id, lane, node):
        if len(self.vehicle_lanelets[vehicle_id]) > 5:
            self.vehicle_lanelets[vehicle_id].popitem(last=False)
        self.vehicle_lanelets[vehicle_id][lane] = node

    def update_traffic(self, data):
        # This update should work on 
        # If the vehicle if already in our dictionary, do nothing.
        # If it is not, add it to the dictionary

        # Since finding the closest node is not the most accurate method
        # Of calculating. So we will try this work around.... 
        if not data.v_id in self.vehicle_lanelets:
            self.register_vehicle(data.v_id)

        lanes = self.nodes_to_lanelets[data.n_id]
        for lane in lanes:
            if self.is_vehicle_in_lanelet(data.v_id, lane, data.n_id):
                if data.v_id in self.lanelet_traffic_history[lane]:
                    #Data update
                    self.lanelet_traffic_history[lane][data.v_id] = self.time
                else:
                    #New entry, publish info
                    self.lanelet_traffic_history[lane][data.v_id] = self.time
                    self.publish_lanelet_info(lane)
            else:
                self.update_vehicle_lanelets(data.v_id, lane, data.n_id)
                
    def add_traffic(self, data):
        self.base_traffic_level[data.lanelet] += data.traffic_level
        self.publish_lanelet_info(data.lanelet)
        return AddTrafficResponse()

    def give_traffic_at_node(self, data):
        '''
        Given a lane, this function returns the traffic at that node,
        plus a constant 5, which will be divided in the vehicle's speed
        '''
        # TODO: Set back to original
        lane = self.nodes_to_lanelets[data.node]
        if self.toggle:
            return sum(len(self.lanelet_traffic_history[l]) +
                    self.base_traffic_level[l] \
                    for l in self.nodes_to_lanelets[data.node]) / \
                    len(self.nodes_to_lanelets[data.node]) + 5
        else:
            return NodeTrafficResponse(5)
