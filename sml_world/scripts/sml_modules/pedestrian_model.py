#!/usr/bin/env python
"""
Module containing class for all simulated pedestrians.
Created on July 6, 2016

@author: U{Nicolo' Campolongo<nicoloc@kth.se>}
@organization: KTH
"""

import numpy as np
import math
import threading

import rospy
from roslaunch.scriptapi import ROSLaunch
from roslaunch.core import Node
import sml_world.msg as msgs
import sml_world.srv as srvs
from sml_world.msg import WorldState # USED ONLY FOR TRAFFIC LIGHTS, FOR NOW
from sml_modules.bodyclasses import WheeledVehicle
import rosnode

class Pedestrian(WheeledVehicle):
    """
    Base class for pedestrians.
    """

    def __init__(self, namespace, pedestrian_id, simulation_rate,
                 crosswalk_id, x=0., y=0., yaw=0.):
        """
        Initialize class Pedestrian.

        @param namespace: I{(string)} Namespace in which the pedestrian node is
                          started.
        @param pedestrian_id: I{(int)} ID of the pedestrian that is created.
        @param simulation_rate: I{(int)} Rate at which the pedestrian is
                                simulated (hz).
        @param crosswalk_id: I{(int)} ID of the crosswalk the pedestrian 
                            is assigned to.
        @param x: I{(float)} x-coordinate at which the pedestrian starts.
        @param y: I{(float)} y-coordinate at which the pedestrian starts.
        @param yaw: I{(float)} Initial yaw of the pedestrian in radians.
        """
        self.launcher = ROSLaunch()
        self.launcher.start()
        self.namespace = namespace
        self.pedestrian_id = int(pedestrian_id)
        self.crosswalk_id = int(crosswalk_id)
        self.class_name = self.__class__.__name__
        self.simulation_rate = simulation_rate

        # Set parameters of base pedestrian to default values.
        self.simulate = False
        self.x = x
        self.y = y
        self.yaw = yaw
        self.np_trajectory = np.zeros((0,0))
        self.i = 0
        self.at_dest = False

        # Start the simulation loop in a separate thread.
        sim_thread = threading.Thread(target=self.simulation_loop)
        #sim_thread.daemon = True
        sim_thread.start()

        self.waiting_at_stop = False

        # available services
        rospy.Service(self.namespace + 'toggle_simulation', srvs.SetBool,
                      self.handle_toggle_simulation)
        rospy.Service(self.namespace + 'set_destination', srvs.SetDestination,
                      self.handle_set_destination)
    
        self.pub_state = rospy.Publisher('/current_pedestrian_state',
                                         msgs.PedestrianState, queue_size=10)


    def simulation_loop(self):
        """The simulation loop of the pedestrian."""
        rate = rospy.Rate(self.simulation_rate)
        while not rospy.is_shutdown():
            # Simulate only if the simulate flag is set.
            if self.simulate:
                self.simulation_step()
            # Check if simulation rate could be achieved or not.
            if rate.remaining() < rospy.Duration(0):
                rate.last_time = rospy.get_rostime()
            else:
                rate.sleep()

    def simulation_step(self):
        """
        Simulate one timestep of the pedestrian.
        After the pedestrian reaches its final destination,
        its node is killed.
        """
        ref_state = self.np_trajectory[:, self.i]

        self.x = ref_state[0]
        self.y = ref_state[1]
        self.yaw = ref_state[2]
        # Make sure self.yaw is never negative.
        # self.yaw 0..2pi
        if self.yaw > 2*np.pi:
            self.yaw = 0.
        elif self.yaw < 0.:
            self.yaw += 2*np.pi
        self.i += 1

        if self.i == len(self.np_trajectory[0]):
            # in this way the pedestrians goes back and forth:
            #self.i = 0
            #self.np_trajectory = np.fliplr(self.np_trajectory)

            # if the pedestrian reaches its destination we move 
            # it outside the map before the node is killed.
            # Todo: remove it from the world state (dict) so that it is not drawn anymore
            self.x = 10000
            self.y = 10000
            pedestrian_state = msgs.PedestrianState(self.pedestrian_id, self.x, 
                                                self.y, self.yaw)
            self.pub_state.publish(pedestrian_state)
            current_node = self.namespace + '/pedestrian'
            rosnode.kill_nodes([current_node])
            return
        # publish pedestrian state
        pedestrian_state = msgs.PedestrianState(self.pedestrian_id, self.x, 
                                                self.y, self.yaw)
        self.pub_state.publish(pedestrian_state)

    def handle_set_destination(self, req):
        """
        Handle the set destination request in the spawn pedestrian.

        @param req: I{(SetDestination)} Request of the service that sets the
                    pedestrian trajectory to a specific destination.
        """
        rospy.wait_for_service('/get_node_coordinates')
        nodeCoord = rospy.ServiceProxy('/get_node_coordinates', srvs.GetCoordinates)
        p1 = nodeCoord(req.origin_id)
        p2 = nodeCoord(req.dest_id)
        m, b = compute_line_params(p1.x, p1.y, p2.x, p2.y)

        # set the "speed"
        # increase t to increase the speed!
        a1 = np.array((p1.x ,p1.y))
        b1 = np.array((p2.x, p2.y))
        dist = np.linalg.norm(a1-b1)
        t = 0.3
        v = int(dist/t)

        b = b + np.random.rand()
        xs = np.linspace(p1.x, p2.x, num=v)
        ys = np.asarray(map(lambda x: m*x + b, xs))
        self.np_trajectory = np.vstack((xs, ys, [np.arctan(m)]*v)) 
        self.loop = False
        self.at_dest = False
        msg = ("Trajectory to destination of pedestrian #%i " % self.pedestrian_id +
               "successfully set.")
        return srvs.SetDestinationResponse(True, msg)

    def handle_toggle_simulation(self, req):
        """
        Handle the toggle simulation request.

        @param req: I{(SetBool)} Enable/Disable the pedestrian simulation.
        """
        self.simulate = req.data
        if self.simulate:
            msg = "Pedestrian #%i will now be simulated." % self.pedestrian_id
        else:
            msg = "Pedestrian #%i will stop to be simulated." % self.pedestrian_id
        return srvs.SetBoolResponse(True, msg)

def compute_line_params(x1, y1, x2, y2):
    """
    given two points, compute line equation parameters m and b
    such that y = mx + b
    """    
    m = (y2-y1)/(x2-x1)
    b = y1 - m*x1
    return (m, b)
