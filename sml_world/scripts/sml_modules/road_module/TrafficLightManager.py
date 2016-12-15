#!/usr/bin/env python
# license removed for brevity
"""
Visualization node of the SML-World.

Created on June 27 2016

@author: U{Daniel Carballal<carba@kth.se>}
@organization: KTH
"""

import rospy
from sml_world.msg import GetTime, TrafficLights
from sml_world.srv import CrosswalkPassage, CrosswalkPassageResponse
from sml_world.srv import GetCrosswalkIds, GetCrosswalkNodes
from random import getrandbits

class TrafficLightManager():
    def __init__(self, lit_nodes):
        '''
        @param lit_nodes: I{(dict)} Dictionary of the node ids to their
        series, denoting all of there 

        '''
        self.traffic_light_nodes = lit_nodes.keys()
        self.traffic_light_series = lit_nodes.values()
        self.time = 0

        self.timestamp = 0

        rospy.Subscriber('/get_time', GetTime, self.update_time)
        self.pub = rospy.Publisher('/traffic_lights', TrafficLights,
            queue_size=5)

        self.flicker_lights()
        self.crosswalks = {}

    def update_time(self, data):
        if data.time != self.time:
            self.time = data.time
            #Update every five minutes for now
            if self.time % 10 == 0:
                self.flicker_lights()

    def flicker_lights(self):
        onoff = []
        for s in self.traffic_light_series:
            char = s[self.timestamp % len(s)]
            if char == '1':
                onoff.append(True)
            else:
                onoff.append(False)
        self.timestamp += 1
        self.publish_results(onoff)


    def publish_results(self, toggle):
        self.pub.publish(self.traffic_light_nodes, toggle)
        """
        self.identify_crosswalk_id()

        # generates pedestrians if light is red
        if self.time > 0:
            for i in range(len(self.traffic_light_nodes)):
                node = self.traffic_light_nodes[i]
                if not toggle[i]:
                    crosswalk_service = '/crosswalk_passage'
                    rospy.wait_for_service(crosswalk_service)
                    try:
                        crosswalk_passage = rospy.ServiceProxy(crosswalk_service, CrosswalkPassage)
                        passage = crosswalk_passage(self.crosswalks[node], 4)
                    except rospy.ServiceException, e:
                        raise NameError("Service call failed: %s" % e)
        """

    

    def identify_crosswalk_id(self):
        """
        @author: U{Nicolo' Campolongo<nicoloc@kth.se>}

        Builds a dictionary for self.crosswalks
        Key are node ids, values crosswalk ids
        """

        # how to run it just one time?
        # stupid solution
        if self.time != 710:
            return

        # Retrieve the crosswalk_ids first
        crosswalk_ids_service = '/get_crosswalk_ids'
        rospy.wait_for_service(crosswalk_ids_service)           
        try:
            ids_for_crosswalk = rospy.ServiceProxy(crosswalk_ids_service, GetCrosswalkIds)
            ids = ids_for_crosswalk()
        except rospy.ServiceException, e:
            raise NameError("Service call failed: %s" % e)
        
        for i in ids.crosswalk_ids:
            get_crosswalk_service = '/get_crosswalk_nodes'
            rospy.wait_for_service('/get_crosswalk_nodes')
            try:
                nodes_for_crosswalk = rospy.ServiceProxy(get_crosswalk_service, GetCrosswalkNodes)
                nodes = nodes_for_crosswalk(i)
                for node in self.traffic_light_nodes:
                    if node == nodes.node_1 or node == nodes.node_2:
                        self.crosswalks[node] = i
                        break
            except rospy.ServiceException, e:
                raise NameError("Service call failed: %s" % e)