"""
Module containing classes for all simulated vehicles.
Seperate module for bus vehicle
Created on Mar 5, 2016

@author: U{David Spulak<spulak@kth.se>}
@organization: KTH
"""

import numpy
import threading
import socket

import rospy
from roslaunch.scriptapi import ROSLaunch
from roslaunch.core import Node
import sml_world.msg as msgs
import sml_world.srv as srvs

from sml_world.msg import WorldState # USED ONLY FOR TRAFFIC LIGHTS, FOR NOW
from sml_modules.bodyclasses import WheeledVehicle

from mocap_source_2 import Mocap,Body
import serial_ports as hw_port

from sys import maxint
import time



class BaseVehicle(WheeledVehicle):
    """
    Base class for all vehicles.

    Other vehicles inherit from this class and can use/overwrite its
    functions.  It provides the following basic services:
        - /set_state: Set the vehicle state.
        - /set_speed_kph: Set the vehicles speed in kilometers per hour.
        - /set_loop: Set a closed loop trajectory from a certain node.
        - /set_destination: Set a trajectory to a certain destination node.
        - /toggle_simulation: Toggle the simulation of the vehicle on/off.

    The launch_sensor function can be called from child classes to launch
    the sensor nodes that are listed in their class variable self.sensors.
    """

    def __init__(self, namespace, vehicle_id, simulation_rate,
                 x=0., y=0., yaw=0., v=0., weight='light'):
        """
        Initialize class BaseVehicle.

        @param namespace: I{(string)} Namespace in which the vehicle node is
                          started.
        @param vehicle_id: I{(int)} ID of the vehicle that is created.
        @param simulation_rate: I{(int)} Rate at which the vehicle is
                                simulated (hz)
        @param x: I{(float)} x-coordinate at which the vehicle starts.
        @param y: I{(float)} y-coordinate at which the vehicle starts.
        @param yaw: I{(float)} Initial yaw of the vehicle in radians.
        @param v: I{(float)} Initial velocity of the vehicle.
        """
        self.launcher = ROSLaunch()
        self.launcher.start()

        self.namespace = namespace

        self.vehicle_id = int(vehicle_id)
        self.class_name = self.__class__.__name__
        self.simulation_rate = simulation_rate

        # Set parameters of base vehicle to default values.
        self.simulate = False
        self.sensors = []
        self.coms = []
        self.x = x
        self.y = y
        self.yaw = yaw
        self.goal_speed = v
        self.v = v
        self.cruising_speed = v
        self.axles_distance = 1.9
        self.np_trajectory = numpy.zeros((0,0))
        self.commands = {}

        self.weight = weight
        self.radar_vis = True

        self.loop = False
        self.at_dest = False

        self.traffic_level = 5 #Default value, no traffic

        self.traffic_lights = []
        self.lights_status = []

        # Start the simulation loop in a separate thread.
        sim_thread = threading.Thread(target=self.simulation_loop)
        sim_thread.daemon = True
        sim_thread.start()

        self.overtake = False

        self.waiting_at_stop = False

        # Register all services, pubs and subs last to prevent attempts to use
        # the services before the initialization of the vehicle is finished.
        self.pub_state = rospy.Publisher('/current_vehicle_state',
                                         msgs.VehicleState, queue_size=10)
        self.pub_demand = rospy.Publisher('/current_demand', msgs.TrafficDemand,
                                         queue_size=10)
        self.traffic_pub = rospy.Publisher('/update_traffic',
                                        msgs.VehicleTrafficUpdate, queue_size=10)
        rospy.Service(self.namespace + 'set_state', srvs.SetVehicleState,
                      self.handle_set_state)
        rospy.Service(self.namespace + 'set_speed_kph', srvs.SetSpeed,
                      self.handle_set_speed_kph)
        rospy.Service(self.namespace + 'set_loop', srvs.SetLoop,
                      self.handle_set_loop)
        rospy.Service(self.namespace + 'set_destination', srvs.SetDestination,
                      self.handle_set_destination)
        rospy.Service(self.namespace + 'toggle_simulation', srvs.SetBool,
                      self.handle_toggle_simulation)

        rospy.Service(self.namespace + 'set_radar_vis', srvs.SetRadarVis,
                      self.handle_set_radar_vis)

        rospy.Service(self.namespace + 'set_demand', srvs.SetDemand,
                      self.handle_set_demand)
        # rospy.wait_for_service(self.namespace + '/publish_com')
        # self.publish_com = rospy.ServiceProxy(self.namespace + 'publish_com',
        #                                       PublishCom)

        rospy.Subscriber('/world_state', WorldState, self.update_traffic_light_status)

        self.overtake_begin_counter = 0
        self.overtake_begin_ignore = 0


    def get_nearest_node(self, dest_id, search_all = False):
        rospy.wait_for_service('/get_nearest_nodeid')
        try:
            get_nodeid = rospy.ServiceProxy('/get_nearest_nodeid',
                                            srvs.GetNearestNodeId)
            self.current_node = get_nodeid(self.x, self.y, dest_id, search_all).node_id
        except rospy.ServiceException, e:
            raise NameError("Service call failed: %s" % e)

    def update_current_node(self):
        self.get_nearest_node(-1, True)
        self.traffic_pub.publish(self.vehicle_id, self.current_node)


    def simulation_loop(self):
        """The simulation loop of the car."""
        rate = rospy.Rate(self.simulation_rate)
        while not rospy.is_shutdown():
            # print self.x, self.y, self.yaw, self.v
            # Simulate only if the simulate flat is set.
            if self.simulate:
                self.simulation_step()
            # Check if simulatio rate could be achieved or not.
            if rate.remaining() < rospy.Duration(0):
                rate.last_time = rospy.get_rostime()
            else:
                rate.sleep()


    def simulation_step(self):
        """Simulate one timestep of the car."""
        if not self.np_trajectory.size:
            #No trajectory to go to.....
            return
        closest_ind = self.find_closest_trajectory_pose()
        ref_ind = (closest_ind + 30) # closest_ind + numpy.round(self.v / 4)
        traj_len = len(self.np_trajectory[0])
        if self.loop is True:
            ref_ind = ref_ind % traj_len
        else:
            if ref_ind > traj_len-1:
                ref_ind = traj_len-1
                if closest_ind == traj_len-1:
                    self.at_dest = True
            else:
                ref_ind = closest_ind
        ref_state = self.np_trajectory[:, int(ref_ind)]

        # update vehicle state.
        '''if self.class_name == 'TruckVehicle':
            self.update_vehicle_state_qualisys()
            self.UDP_receive()
            if self.data == "-1.00":
                self.set_control_commands_pp(ref_state, ref_ind)
            else:
                steer = int(self.data[-6:-3])
                throttle = int(self.data[:-6]) + 5
                hw_port.set_command(throttle,steer,2)
            self.update_truck_hardware()
        else:
            self.set_control_commands(ref_state)
            self.update_vehicle_state()'''

        self.set_control_commands(ref_state, ref_ind)
        self.update_vehicle_state()

        # publish vehicle state.
        vehicle_state = msgs.VehicleState(self.vehicle_id, self.class_name,
                                     self.x, self.y, self.yaw, self.v)
        self.pub_state.publish(vehicle_state)
        self.update_current_node()

        #The way that the stop light waiting works, this is necessary
        if not self.waiting_at_stop:
            self.check_for_traffic_light()
        self.get_traffic()

    def get_traffic(self):
        rospy.wait_for_service('/node_traffic_service')
        try:
            traf = rospy.ServiceProxy('/node_traffic_service', srvs.NodeTraffic)
            self.traffic_level = traf(self.current_node).traffic_level
        except rospy.ServiceException, e:
            raise NameError("Service call failed: %s" % e)


    def find_closest_trajectory_pose(self):
        """
        Find closest point to the current vehicle position in the trajectory.

        @return: The index of the closest trajectory point to the current
                 vehicle position.
        """
        np_state = numpy.array([[self.x], [self.y]])
        temp_distance = numpy.sum(
                          (self.np_trajectory[0:2, :] - np_state) ** 2,
                          axis=0)
        best_idx = numpy.argmin(temp_distance)
        return best_idx


    def set_control_commands(self, ref_state, ref_ind):
        """
        Set the control commands, depending on the vehicles controler.

        @param ref_state: I{(numpy array)} Reference state [x, y, yaw] that
                          the vehicle tries to reach.
        """
        if not self.at_dest:
            self.commands['speed'] = self.cruising_speed * (5. / self.traffic_level)
        else:
            self.commands['speed'] = 0.0
        dx = ref_state[0] - self.x
        dy = ref_state[1] - self.y
        dx_v = numpy.cos(self.yaw) * dx + numpy.sin(self.yaw) * dy

        # To overtake, move to the left a little bit and follow your original traj.
        stay_overtake = False
        if self.overtake:
            self.overtake_begin_ignore += 1
        else:
            self.overtake_begin_ignore = 0
        if self.overtake and len(self.radar_readings[0, :]) > 0:
            stay_overtake = numpy.min(self.radar_readings[0, :]) > 30
            rospy.logerr(self.overtake_begin_ignore)
            if self.overtake_begin_ignore < 3:
                stay_overtake = True
            if not stay_overtake:
                self.overtake = False
                self.overtake_begin_counter = 0
                self.commands['speed'] *= 0
            # rospy.logerr('chcek for stay overtaking: ' + str(stay_overtake))
        else:
            stay_overtake = True

        if self.overtake and stay_overtake:
            self.commands['speed'] *= 1.5
            dy_v = -numpy.sin(self.yaw) * dx + numpy.cos(self.yaw) * dy + 7.5
        else:
            dy_v = -numpy.sin(self.yaw) * dx + numpy.cos(self.yaw) * dy
        dyaw_v = ref_state[2] - self.yaw
        # Correct yaw difference. dyaw_v 0..pi
        while dyaw_v > numpy.pi:
            dyaw_v -= 2*numpy.pi
        while dyaw_v < -numpy.pi:
            dyaw_v += 2*numpy.pi
        # Calculate steering command from dy_v, dx_v and dyaw_v
        steering_command = dy_v + dyaw_v * 1.5 / (1 + dx_v)
        # Compare with max steering angle
        if steering_command > 0.5:
            steering_command = 0.5
        elif steering_command < -0.5:
            steering_command = -0.5
        self.commands['steering_angle'] = steering_command


    def update_vehicle_state(self):
        """Update the vehicle state."""
        sim_timestep = 1. / self.simulation_rate
        # Decompose v into x and y component.
        if self.v != self.commands['speed']:
            self.v = self.commands['speed']
        vx = numpy.cos(self.yaw) * self.v
        vy = numpy.sin(self.yaw) * self.v
        # Update vehicles position
        self.x += vx * sim_timestep
        self.y += vy * sim_timestep
        self.yaw += ((self.v / self.axles_distance) *
                     numpy.tan(self.commands['steering_angle']) *
                     sim_timestep)
        # Make sure self.yaw is never negative.
        # self.yaw 0..2pi
        if self.yaw > 2*numpy.pi:
            self.yaw = 0.
        elif self.yaw < 0.:
            self.yaw += 2*numpy.pi

    def launch_sensors(self):
        """Launch and register the sensors used by the vehicle."""
        # Go through sensor list.
        for sensor in self.sensors:
            # Launch sensor node.
            sensor_name = sensor.partition(' ')[0]
            subpub_name = sensor_name.lower()+'_readings'
            args = str(self.vehicle_id)+' '+sensor
            node = Node('sml_world', 'sensor.py', namespace=self.namespace,
                        args=args, name=sensor_name.lower())
            self.launcher.launch(node)
            # Register subscriptions for each of them.
            rospy.Subscriber(self.namespace + subpub_name,
                             getattr(msgs, sensor_name+'Readings'),
                             getattr(self, 'process_'+subpub_name))

    def launch_coms(self):
        """Launch and register the communications used by the vehicle."""
        # Go through list of comunication.
        for com in self.coms:
            com_name = com.partition(' ')[0]
            subpub_name = com_name.lower()+'_com'
            args = str(self.vehicle_id)+' '+com
            node = Node('sml_world', 'communication.py',
                        namespace=self.namespace, args=args,
                        name=com_name.lower())
            self.launcher.launch(node)
            # Register subscriptions for each of them.
            rospy.Subscriber(self.namespace + subpub_name,
                             getattr(msgs, com_name+'Com'),
                             getattr(self, 'process_'+subpub_name))

    def handle_set_demand(self, req):
        """
        Handle set demand.

        @param req: I{(SetDemand)} Request of the service that sets demand.
        """
        self.bus_demand = req.bus_demand
        msg = "Demand #%i successfully set." % self.bus_demand
        return srvs.SetDemandResponse(True, msg)

    def handle_set_demand(self, req):
        """
        Handle set demand.

        @param req: I{(SetDemand)} Request of the service that sets demand.
        """
        self.bus_demand = req.bus_demand
        msg = "Demand #%i successfully set." % self.bus_demand
        return srvs.SetDemandResponse(True, msg)

    def handle_set_state(self, req):
        """
        Handle the set state request.

        @param req: I{(SetState)} Request of the service that sets the vehicle
                    state.
        """
        self.x = req.x
        self.y = req.y
        self.yaw = req.yaw
        if self.v != req.v:
            self.v = req.v
        msg = "State of vehicle #%i successfully set." % self.vehicle_id
        return srvs.SetVehicleStateResponse(True, msg)

    def handle_set_speed_kph(self, req):
        """
        Handle the set speed request.

        @param req: I{(SetSpeed)} Request of the service that sets the vehicles
                    cruising speed in kmh.
        """
        self.cruising_speed = req.speed * (5. / self.traffic_level) / 3.6
        msg = "Speed of vehicle #%i successfully set." % self.vehicle_id
        return srvs.SetSpeedResponse(True, msg)

    def handle_set_loop(self, req):
        """
        Handle the set closed loop request.

        @param req: I{(SetLoop)} Request of the service that sets the vehicles
                    closed loop trajectory.
        """
        rospy.wait_for_service('/get_trajectory')
        try:
            get_traj = rospy.ServiceProxy('/get_trajectory', srvs.GetTrajectory)
            trajectory = get_traj(True, req.node_id, 0).trajectory
            if trajectory == []:
                msg = ('No trajectory found!!')
                return srvs.SetLoopResponse(False, msg)
        except rospy.ServiceException, e:
            raise NameError("Service call failed: %s" % e)
        self.np_trajectory = to_numpy_trajectory(trajectory)
        self.loop = True
        self.at_dest = False
        msg = ("Closed loop trajectory of vehicle #%i " % self.vehicle_id +
               "successfully set.")
        return srvs.SetLoopResponse(True, msg)

    def handle_set_destination(self, data):
        """
        Handle the set destination request.

        @param req: I{(SetDestination)} Request of the service that sets the
                    vehicles trajectory to a specific destination.
        """
        #If the origin_id is 0, it has not been specified and we must find
        #the closest node to where we are now
        self.dest_node = data.dest_id
        if data.origin_id == 0:
            #Will set self.current_node
            self.get_nearest_node(data.dest_id)
        else:
            self.current_node = data.origin_id
        if self.current_node == data.dest_id:
            self.at_dest = True
            msg = ("We're already there!")
            return srvs.SetDestinationResponse(True, msg)
        rospy.wait_for_service('/get_trajectory')
        get_traj = rospy.ServiceProxy('/get_trajectory', srvs.GetTrajectory)
        trajectory = get_traj(False, self.current_node, data.dest_id).trajectory
        self.np_trajectory = to_numpy_trajectory(trajectory)
        self.loop = False
        self.at_dest = False
        msg = ("Trajectory to destination of vehicle #%i " % self.vehicle_id +
               "successfully set.")
        return srvs.SetDestinationResponse(True, msg)

    def handle_toggle_simulation(self, req):
        """
        Handle the toggle simulation request.

        @param req: I{(SetBool)} Enable/Disable the vehicle simulation.
        """
        self.simulate = req.data
        if self.simulate:
            msg = "Vehicle #%i will now be simulated." % self.vehicle_id
        else:
            msg = "Vehicle #%i will stop to be simulated." % self.vehicle_id
        return srvs.SetBoolResponse(True, msg)

    def handle_set_radar_vis(self, req):
        """
        Handle the radar visualization.
        @param req: I{(SetRadarVis)} Request of the service that sets the radar
                    visualization state
        """
        self.radar_vis = req.radar_state
        msg = "Radar state of vehicle #%i successfully set to %s" % (self.vehicle_id, self.radar_vis)
        return srvs.SetRadarVisResponse(True, msg)

    def update_traffic_light_status(self, world_state):
        self.traffic_lights = world_state.traffic_light_nodes
        self.lights_status = world_state.traffic_light_status

    def revert_to_goal_speed(self):
        req = type("SetState", (object,),{})
        self.cruising_speed = self.goal_speed
        setattr(req, 'speed', self.goal_speed * 3.6)
        self.handle_set_speed_kph(req)

    def check_for_traffic_light(self):
        for i in range(len(self.traffic_lights)):
            if self.current_node == self.traffic_lights[i]:
                if not self.lights_status[i]:
                    #To avoid weird errors, we will check if
                    #we are headed towards or away from the
                    #traffic light. If we are getting closer,
                    #we will slow down until we 'reach' the
                    #node, then we will stop. If we are moving
                    #away from the node, we have already passed
                    #the node
                    closest_ind = self.find_closest_trajectory_pose()
                    next_ind = closest_ind + numpy.round(self.v / 4)

                    #Ensure we don't have an out of bounds error
                    if next_ind > len(self.np_trajectory[0]):
                        next_pos = [maxint, maxint]
                    else:
                        next_pos = self.np_trajectory[0:2, int(next_ind)]

                    closest_pos = self.np_trajectory[0:2, int(closest_ind)]


                    rospy.wait_for_service('/get_node_coordinates')
                    get_coords = rospy.ServiceProxy('/get_node_coordinates',
                                srvs.GetCoordinates)
                    light_loc = get_coords(self.traffic_lights[i])
                    #Lambda for comparing distance between two points
                    #with different formats
                    ptdist = lambda p1,p2: (p1.x-p2[0]) ** 2 + \
                            (p1.y-p2[1]) ** 2

                    if next_ind < len(self.np_trajectory[0]) and \
                                ptdist(light_loc, closest_pos) > \
                                ptdist(light_loc, next_pos) and \
                                not self.lights_status[i]:
                        self.waiting_at_stop = True
                        rate = rospy.Rate(self.simulation_rate)
                        old_cruise = self.cruising_speed
                        while ptdist(light_loc, closest_pos) > \
                                ptdist(light_loc, next_pos) and \
                                not self.lights_status[i]:
                            # Essentually do the same as simulation
                            # loop, but reduce speeds
                            self.cruising_speed /= 1.08

                            self.simulation_step()
                            closest_ind = self.find_closest_trajectory_pose()
                            next_ind = closest_ind + numpy.round(self.v / 4)
                            closest_pos = self.np_trajectory[0:2, int(closest_ind)]
                            #Ensure no out of bounds error
                            if next_ind <= len(self.np_trajectory[0]):
                                next_pos = self.np_trajectory[0:2, int(next_ind)]
                            else:
                                next_pos = [maxint, maxint]
                            if rate.remaining() < rospy.Duration(0):
                                rate.last_time = rospy.get_rostime()
                            else:
                                rate.sleep()

                        req = type("SetState", (object,),{})
                        setattr(req, 'speed', 0)
                        self.handle_set_speed_kph(req)
                        self.cruising_speed = old_cruise
                        while not self.lights_status[i]:
                            pass
                        self.waiting_at_stop = False
                    else:
                        return

                    self.revert_to_goal_speed()

                    return


def to_numpy_trajectory(trajectory):
    """
    Transform Pose2D[] message to numpy array.

    @param trajectory: I{(Pose2D[])} ROS message of vehicle trajectory.

    @return: I{(numpy array)} Numpy array representation of the trajectory.
             [[x], [y], [yaw]]
    """
    tx = []
    ty = []
    tyaw = []
    for pose in trajectory:
        tx.append(pose.x)
        ty.append(pose.y)
        tyaw.append(pose.yaw)
    return numpy.asarray([tx, ty, tyaw])

def to_pose_2d(np_trajectory):
    """
    Transform numpy [] message to Point array.

    @param: I{(numpy array)} Numpy array representation of the trajectory.
             [[x], [y], [yaw]]. Yaw is always 0

    @return trajectory: I{(Point2D[])} ROS message of vehicle trajectory.
    """
    arr = []
    for i in range(len(np_trajectory[0])):
        pose = msgs.Pose2D(np_trajectory[0,i], np_trajectory[1,i], 0.)
        arr.append(pose)
    return arr



class DummyVehicle(BaseVehicle):
    """Class for the dummy vehicle."""
    def __init__(self, namespace, vehicle_id, simulation_rate,
                 x=0., y=0., yaw=0., v=0.):
        """Initialize class DummyVehicle."""
        self.overtake_counter = 0
        super(DummyVehicle, self).__init__(namespace, vehicle_id,
                                           simulation_rate, x, y, yaw, v)

        self.sensors = ['Radar 35 15']
        self.pub_sensors = rospy.Publisher("/sensors", msgs.VehicleSensors, queue_size=10)
        self.radar_readings = numpy.asarray([[], [], []])
        self.launch_sensors()


    def simulation_step(self):
        """
        Simulate one timestep of the car.
        Same as before but sensor state is now published.
        """
        super(DummyVehicle, self).simulation_step()
        # now I'm supposing we have only 1 sensor on the car
        vehicle_sensors = msgs.VehicleSensors(int(self.vehicle_id), float(self.sensors[0].split()[1]),
                                        float(self.sensors[0].split()[2]), self.radar_vis)
        self.pub_sensors.publish(vehicle_sensors)


    def set_control_commands(self, ref_state, ref_ind):
        """
        Set the control commands, depending on the vehicles controler.

        @param ref_state: I{(numpy array)} Reference state [x, y, yaw] that
                          the vehicle tries to reach.
        """
        super(DummyVehicle, self).set_control_commands(ref_state, ref_ind)
        safety_distance = 20.
        full_stop_distance = 15.


        self.check_if_overtake_is_finished()

        # Only continue from this point if there are some radar sensings
        if not numpy.any(self.radar_readings[0, :]):
            return


        min_dist = numpy.min(self.radar_readings[0, :])
        # Set speed.
        if min_dist < full_stop_distance:
            desired_speed = 0.
            self.overtake_begin_counter = 0

        elif min_dist < safety_distance:
            desired_speed = self.cruising_speed * min_dist / safety_distance
        else:
            desired_speed = self.cruising_speed

        # Every subclass can
        if not self.overtake:
            if self.check_if_overtake(min_dist):
                if self.check_if_safe_to_overtake():
                    rospy.logwarn(str(self.vehicle_id) + ' start overtaking')
                    self.overtake = True

        self.commands['speed'] = desired_speed

    def check_if_safe_to_overtake(self):
        '''
        Checks if it is safe to pass. Dummy Vehicles have different behaviour than
        smart vehicle. Plain dummy vehicles check that there is only one vehicle in
        front of them.
        '''
        return True

    def check_if_overtake(self, min_dist=100):
        '''
        Checks if the vehicle should overtake, by distance for Dummy Vehicle and by
        speed for SmartWifi Vehicles
        '''
        if self.overtake:
            return True
        # rospy.logerr(self.radar_readings[0, :])
        # min_dist = numpy.min(self.radar_readings[0, :])
        #safety_distance = 12
        safety_distance = 20
        if self.overtake_begin_counter < 8:
            if min_dist < safety_distance:
                self.overtake_begin_counter += 1
                # rospy.logwarn('vehicle id: ' + str(self.vehicle_id) + ", begin_counter: " + str(self.overtake_begin_counter))
                # rospy.logerr(self.overtake_begin_counter)
            return False
        else:
            if min_dist < safety_distance:
                self.overtake_begin_counter = 0
                return True
            else:
                self.overtake_begin_counter = 0
                return False
            # return min_dist < safety_distance + 10
        # return min_dist < safety_distance

    def check_if_overtake_is_finished(self):
        '''
        Base vehicle analyzes radar readings. Once they see that there is no
        vehicle in front of them, they will count to 50 time steps and merge.
        This is where the 'dumb' in 'DummyVehicle' comes from.
        '''
        if not self.overtake:
            return
        if not numpy.any(self.radar_readings[0, :]):
            self.overtake_counter += 1
            if self.overtake_counter > 50 and self.overtake:
                self.overtake = False
                desired_speed  = self.cruising_speed
                self.overtake_begin_counter = 0
        else:
            self.overtake_counter = 0
            if self.overtake:
                self.overtake_begin_counter = 0
        return


    def process_radar_readings(self, rr):
        """
        Put all sensor readings into a numpy array.

        @param rr: I{(RadarReadings)} Radar readings message that needs to
                   be put into the class variable radar_readings.
        """
        # Write sensor readings in an ndarray
        self.radar_readings = numpy.asarray([[], [], []])
        for r in rr.registered_vehicles:
            self.radar_readings = numpy.concatenate(
                                    (self.radar_readings,
                                     [[r.rho], [r.theta], [r.yaw]]),
                                    axis=1)




class WifiVehicle(DummyVehicle):
    """
    Class for the wifi vehicle.

    This vehicle does nothing more than the DummyVehicle, except
    printing its wifi communication.
    """

    def __init__(self, namespace, vehicle_id, simulation_rate,
                 x=0., y=0., yaw=0., v=0.):
        """Initialize class WifiVehicle."""
        super(WifiVehicle, self).__init__(namespace, vehicle_id,
                                          simulation_rate, x, y, yaw, v)
        self.coms = ['Wifi 50']
        self.launch_coms()

    def simulation_step(self):
        """Simulate one timestep of the car."""
        rospy.wait_for_service("send_wifi_com")
        try:
            send_wifi = rospy.ServiceProxy("send_wifi_com", srvs.SendWifiCom)
            send_wifi("I am vehicle #%i" % self.vehicle_id)
        except rospy.ServiceException, e:
            raise NameError("Service call failed: %s" % e)
        super(WifiVehicle, self).simulation_step()

    def process_wifi_com(self, wm):
        """Process messages received over wifi."""
        print wm.message

from random import randint

class RandomDestinationVehicle(DummyVehicle):
    def __init__(self, namespace, vehicle_id, simulation_rate,
                 x=0., y=0., yaw=0., v=10.):
        super(RandomDestinationVehicle, self).__init__(namespace, vehicle_id,
                                          simulation_rate, x, y, yaw, v)
        self.destination = 0
        self.get_destination()

    def get_destination(self):
        if (not self.destination) or self.at_dest:
            origin_id = self.destination
            rospy.wait_for_service('/lanelets')
            try:
                get_lanelet_info = rospy.ServiceProxy('/lanelets', srvs.Lanelets)
                response = get_lanelet_info().node_ids
            except rospy.ServiceException, e:
                raise NameError("Service call failed: %s" % e)
            found_a_dest = False
            while not found_a_dest:
                try:
                    self.destination = response[randint(0, len(response))]
                    rospy.wait_for_service(self.namespace + 'set_destination')
                    dest = srvs.SetDestination()
                    dest.origin_id = origin_id
                    dest.dest_id = self.destination
                    found_a_dest  = True
                except NameError, e:
                    rospy.logwarn('Unable to find path')
            super(RandomDestinationVehicle, self).handle_set_destination(dest)

    def simulation_step(self):
        super(RandomDestinationVehicle, self).simulation_step()
        if self.at_dest:
            self.get_destination()

class SemiControlledVehicle(DummyVehicle):
    """
    @author: U{Nicolo' Campolongo<nicoloc@kth.se>}

    todo
    """
    def __init__(self, namespace, vehicle_id, simulation_rate,
                 x=0., y=0., yaw=0., v=10.):
        super(SemiControlledVehicle, self).__init__(namespace, vehicle_id,
                simulation_rate, x, y, yaw, v)
        self.stop = False

    def simulation_step(self):
        """
        """
        super(SemiControlledVehicle, self).simulation_step()
        if self.at_dest:
            if not self.stop:
                gui_service = '/turn_on_gui'
                rospy.wait_for_service(gui_service)
                try:
                    gui = rospy.ServiceProxy(gui_service, srvs.TurnOnGUI)
                    gui(self.dest_node, self.namespace)
                except rospy.ServiceException, e:
                    raise NameError("Service call failed: %s" % e)
            self.stop = True
        return
        """
            if not self.stop:
                highlight_service = '/highlight_node'
                rospy.wait_for_service(highlight_service)
                try:
                    highlight_node = rospy.ServiceProxy(highlight_service, srvs.HighlightNode)
                    highlight_node(self.dest_node, self.namespace)
                except rospy.ServiceException, e:
                    raise NameError("Service call failed: %s" % e)
            self.stop = True
        return
        """

    def handle_set_destination(self, data):
        resp = super(SemiControlledVehicle, self).handle_set_destination(data)
        self.stop = False
        return resp

    def handle_set_speed_kph(self, req):
        """
        Handle the set speed request.

        @param req: I{(SetSpeed)} Request of the service that sets the vehicles
                    cruising speed in kmh.
        """
        self.cruising_speed += req.speed
        msg = "Speed of vehicle #%i successfully set." % self.vehicle_id
        return srvs.SetSpeedResponse(True, msg)

class ConnectivityVehicle(DummyVehicle):
    '''
    A vehicle capable of connecting to and sending
    crucial information information to its counterparts.
    The vehicle will have a direct communication to the
    vehicle directly in front of it.
    '''
    def __init__(self, namespace, vehicle_id, simulation_rate,
                 x=0., y=0., yaw=0., v=10.):
        self.cur_request = -1
        # We need to communicate with our smart wifi in order to make requests
        self.smart_pub = rospy.Publisher('/smart_wifi_request_' + str(vehicle_id),
                        msgs.SmartWifiRequest, queue_size = 10)

        rospy.Subscriber('/smart_wifi_commands_' + str(vehicle_id),
                      msgs.SmartWifiCommand, self.parse_commands)


        super(ConnectivityVehicle, self).__init__(namespace, vehicle_id,
                                        simulation_rate, x, y, yaw, v)

        self.coms = ['SmartWifi 30']
        self.launch_coms()

        self.temp_max_speed = maxint
        self.pose_trajectory = None

        self.safe_to_overtake = False
        self.overtaking = False

    def handle_set_speed_kph(self, req):
        '''
        If the vehicle in front of us has requested that we maintain a speed of
        x, we will not go a speed higher than that.

        Unfortunetly, max speed is in kpm, so we have to do some strange conversions
        '''
        if req.speed > self.temp_max_speed:
            req.speed = self.temp_max_speed
        return super(ConnectivityVehicle, self).handle_set_speed_kph(req)

    def process_smartwifi_com(self, wm):
        rospy.logwarn(wm)

    def check_if_overtake(self, unused=None):
        '''
        We will not overtake if we are already overtaking
        '''
        if self.overtake and not self.overtaking:
            #Begin overtake
            self.overtaking = True
            self.publish_to_smart_wifi(4)
            return False
        elif self.overtake:
            # Already overtaking
            return False
        # If we are going more than 80% of our desired speed, we have no reason to pass
        return self.temp_max_speed < self.goal_speed * 3.6 * .8

    def revert_to_goal_speed(self):
        super(ConnectivityVehicle, self).revert_to_goal_speed()
        self.publish_to_smart_wifi(6)

    def check_if_safe_to_overtake(self, min_dist=100):
        '''
        Checks if it safe for the vehicle to overtake, mainly by checking for
        other vehicles in the area or if there is a queue longer than
        3 cars. We will send the request
        to the wifi module, which will toggle safe_to_overtake

        TODO: This always returns false, figure out why
        '''
        if self.overtake:
            return False
        if self.vehicle_id == 2:
            rospy.logwarn('SAFE TO OVERTAKE: ' + str(self.safe_to_overtake))
        self.publish_to_smart_wifi(1)
        return self.safe_to_overtake

    def check_if_overtake_is_finished(self):
        '''
        Wifi vehicle will only be told to stop merging by the high level controller
        of the wifi module. This is more intelligent because the wifi knows
        which vehicle to pass in order to safely merge again. Therefore, we do
        nothing in this function
        '''
        return

    #Whenever we change the trajectory, notify the smart wifi
    def handle_set_loop(self, data):
        resp = super(ConnectivityVehicle, self).handle_set_loop(data)
        #Wait until smart wifi is listening.....
        #Note: This will break the vehicle if the smart wifi crashes
        while self.smart_pub.get_num_connections() == 0:
            pass
        self.pose_trajectory = to_pose_2d(self.np_trajectory)
        self.publish_to_smart_wifi()
        return resp

    def handle_set_destination(self, data):
        resp = super(ConnectivityVehicle, self).handle_set_destination(data)
        while self.smart_pub.get_num_connections() == 0:
            pass
        self.pose_trajectory = to_pose_2d(self.np_trajectory)
        self.publish_to_smart_wifi()
        return resp

    def parse_commands(self, data):
        if data.command_id == 1:
            ''' Is it safe to overtake? we overtake if we receive a vehicle id'''
            self.safe_to_overtake = True
        if data.command_id == 2:
            self.temp_max_speed = data.result
            rospy.logwarn('Vehicle ' + str(self.vehicle_id) + ' now going ' + str(data.result) + \
                    ' due to ' + data.msg)
            req = type("SetState", (object,),{})
            setattr(req, 'speed', self.temp_max_speed) #Shouldn't matter value
            self.handle_set_speed_kph(req)
        if data.command_id == 3:
            '''
            Stop overtaking
            '''
            self.overtake = False
            self.overtaking = False


    def publish_to_smart_wifi(self, command=-1):
        return self.smart_pub.publish(self.pose_trajectory, command, self.goal_speed * 3.6)


class TruckVehicle(ConnectivityVehicle):
    """
    Class for the Truck vehicle.

    The truck vehicles follow the hardware trucks, i.e, mimic their state.
    The state of the hardware trucks are obtained with the help of qualisys.

    These models are also assigned a trajectory, which gives the next point to reach.
    A control command is generated to follow the trajectory and the command is sent
    to the hardware trucks.

    """

    rospy.logwarn('Truck vehicle started')
    def __init__(self, namespace, vehicle_id, simulation_rate,
                 x=0., y=0., yaw=0., v=0.):
        """Initialize class TruckVehicle."""
        super(TruckVehicle, self).__init__(namespace, vehicle_id,
                                           simulation_rate, x, y, yaw, v)
        self.sensors = ['Radar 35 10']
        self.radar_readings = numpy.asarray([[], [], []])
        self.launch_sensors()

        self.mocap = Mocap(host='SML',info=1)

        self.prev_err = 0.
        self.integral = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]

        self.prev_x = x
        self.prev_y = y
        vehicle_name = "MiniTruck"+str(self.vehicle_id)
        self.truck_id = self.mocap.get_id_from_name(vehicle_name) #("MiniTruck"+str(self.vehicle_id))#("BigTruck-OrangeScaniaR470")
        self.flag=1
        self.commands['speed'] = v

        UDP_IP = "0.0.0.0"
        UDP_PORT = 8888 + vehicle_id

        self.sock = socket.socket(socket.AF_INET, # Internet
                             socket.SOCK_DGRAM) # UDP
        self.sock.settimeout(0.003)                         # To ensure that simulation rate of 20 Hz is achieved.
        self.sock.bind((UDP_IP, UDP_PORT))
        self.data = "-1.00"
        self.alpha = 0.

    def UDP_receive(self):
        try:
            data, addr = self.sock.recvfrom(64) # buffer size is 64 bytes
            self.data = data
        except socket.timeout:
            pass

    def set_control_commands(self, ref_state, ref_ind):

        self.update_vehicle_state_qualisys()
        self.UDP_receive()
        if self.data == "-1.00":
            self.set_control_commands_pp(ref_state, ref_ind)
        else:
            steer = int(self.data[-6:-3])
            throttle = int(self.data[:-6]) + 5
            rospy.logerr('throttle: ' + str(throttle))
            hw_port.set_command(throttle,steer, self.vehicle_id)

    def set_control_commands_pp(self, ref_state, ref_ind):

        dx = ref_state[0] - self.x
        dy = ref_state[1] - self.y
        dyaw = ref_state[2] - self.yaw

        self.check_if_overtake_is_finished()

        #conversion to relative coordinates
        dx_v = numpy.cos(self.yaw) * dx + numpy.sin(self.yaw) * dy
        self.commands['speed'] = self.cruising_speed

        if self.overtake:
            self.commands['speed'] *= 1.2
            self.alpha = (2*self.alpha+3.5)/3
            dy_v = -numpy.sin(self.yaw) * dx + numpy.cos(self.yaw) * dy + self.alpha
            D = 7
        else:
            dy_v = -numpy.sin(self.yaw) * dx + numpy.cos(self.yaw) * dy
            D = 5
            self.alpha /= 3

        # Correct yaw difference. dyaw_v 0..pi
        while dyaw > numpy.pi:
            dyaw -= 2*numpy.pi
        while dyaw < -numpy.pi:
            dyaw += 2*numpy.pi

        #look ahead distance
        #error term of the pp controller (see article : A system for Semi-Autonomous Tractor Operations)
        K = 2
        gamma = (2 * dy_v + K*dyaw) / D**2         #curvature
        L = 0.05 * 32                  #distance between wheels 5cm times 30(amplitude factor of the map)

        steering_command = L * gamma

        if steering_command > 0.5:
            steering_command = 0.5
        elif steering_command < -0.5:
            steering_command = -0.5

        self.commands['steering_angle'] = steering_command

        if numpy.abs(steering_command) < 0.08:
            self.commands['steering_angle'] = 0

        safety_distance = 30.
        full_stop_distance = 20.

        # Only continue from this point if there are some radar sensings
        if not numpy.any(self.radar_readings[0, :]) or self.overtake:
            return

        min_dist = numpy.min(self.radar_readings[0, :])
        #rospy.logwarn('Virtual sensing : ' + str(min_dist))
        # Set speed.
        if min_dist < full_stop_distance:
            desired_speed = 0.
        elif min_dist < safety_distance:
            desired_speed = self.cruising_speed * min_dist / safety_distance
        else:
            desired_speed = self.cruising_speed

        # Every subclass can
        if self.check_if_overtake(min_dist):
            if self.check_if_safe_to_overtake():
                rospy.logwarn('START OVERTAKE')
                self.overtake = True

        self.commands['speed'] = desired_speed


    def update_vehicle_state_qualisys(self):
        # TODO: Handle exception when qualisys is not activated.

        truck_state = self.mocap.get_body(self.truck_id)

        if truck_state == 'off':
            rospy.logwarn("Hardware not found by Qualisys!")

        else:
            # self.x = 43.4*(truck_state['x']-1.101)
            # self.y = 36.5*(truck_state['y']+0.397)
            # rospy.logwarn('Qulisys x: ' + str(truck_state['x']) + ', y: ' + str(truck_state['y']))
            #kistademo3
            #self.x = 36.034 * (truck_state['x'] - 0.793)
            #self.y = 36.156 * (truck_state['y'] + 0.349)
            self.x = 55.56 * (truck_state['x'] - 0.818 - 0.03)
            self.y = 55.65 * (truck_state['y'] + 0.304)

            self.yaw = truck_state['yaw']*numpy.pi/180
            dx = self.x - self.prev_x
            dy = self.y - self.prev_y

            self.v = numpy.sqrt(dx*dx + dy*dy)*self.simulation_rate            # Change in time is 0.02 since we use 50 Hz
            self.prev_x = self.x
            self.prev_y = self.y

    #Send commands to the hardware
    #def update_truck_hardware(self):
    def update_vehicle_state(self):
        """Update the vehicle state."""
        #vel = self.v + self.commands['throttle']/self.m/self.simulation_rate

        vel = self.commands['speed']
        steer = self.commands['steering_angle']

        if steer > 0.5:
            steer_cmd = 25
        elif steer < -0.5:
            steer_cmd = 185
        else:
            steer_cmd = 100 - 160*steer  ##linear
            #steer_cmd = 100 - 640*steer**3 ##cubic

        #rospy.logwarn('Velocity command is '+ str(vel))
        # 130 is the lowest vel_cmd that makes the truck move.
        if vel > 12:
            vel_cmd = 161
        elif vel < 0:
            vel_cmd = 0
        else:
            vel_cmd = 3.77*vel + 117
        # rospy.logerr('throttle: ' + str(throttle))
        hw_port.set_command(vel_cmd,steer_cmd,self.vehicle_id)

    '''def process_radar_readings(self, rr):
        """
        Put all sensor readings into a numpy array.

        @param rr: I{(RadarReadings)} Radar readings message that needs to
                   be put into the class variable radar_readings.
        """
        # Write sensor readings in an ndarray
        #rospy.logwarn('Vehicle reading ' + str(rr))
        self.radar_readings = numpy.asarray([[], [], []])
        for r in rr.registered_vehicles:
            self.radar_readings = numpy.concatenate(
                                    (self.radar_readings,
                                     [[r.rho], [r.theta], [r.yaw]]),
                                    axis=1) '''
