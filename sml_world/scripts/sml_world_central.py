#!/usr/bin/env python
# license removed for brevity
"""
Central node of the SML-World.

Created on Mar 1, 2016

@author: U{David Spulak<spulak@kth.se>}
@organization: KTH
"""
import Queue

import rospy
from roslaunch.scriptapi import ROSLaunch
from roslaunch.core import Node
from sml_world.msg import TrafficDemand, VehicleState, WorldState, VehicleSensors, PedestrianState
from sml_world.msg import BusStops, GetTime, TrafficLights
from sml_world.srv import SpawnVehicle, SpawnVehicleResponse
from sml_world.srv import CrosswalkPassage, CrosswalkPassageResponse
from sml_world.srv import SetLoop, SetDestination, GetCrosswalkNodes
from sml_world.srv import SpawnPedestrian, SpawnPedestrianResponse
from sml_world.srv import GetCrosswalkIds, GetCrosswalkNodes
from sml_world.srv import SetBool, HighlightNode
from sml_modules.bus_vehicle_model import BusVehicle

from sets import Set


class ROSLaunchExtended(ROSLaunch):
    """Extended version of the ROSLaunch class."""

    def __init__(self):
        """Initialize the ROSLaunchExtended class."""
        super(ROSLaunchExtended, self).__init__()
        rospy.Service('spawn_vehicle', SpawnVehicle, self.handle_spawn_vehicle)
        rospy.Service('spawn_pedestrian', SpawnPedestrian, self.handle_spawn_pedestrian)
        rospy.Service('crosswalk_passage', CrosswalkPassage, self.handle_crosswalk_passage)
        # Working with a launch queue is necessary, because self.launch()
        # only works in the main thread.  Service calls spawn sidethreads.
        self.launch_queue = Queue.Queue()
        self.launched_processes = []
        self.start()

    def handle_crosswalk_passage(self, req):
        """
        @author: U{Nicolo' Campolongo<nicoloc@kth.se>}

        Spawn multiple pedestrians at given crosswalk.

        @param req: I{(CrosswalkPassage)} Request of the service that spawns the
                    given number of pedestrians at the given crosswalk.
        """
        for i in range(req.ped_number):
            ped = type("SpawnPedestrian", (object,),{})
            setattr(ped, 'pedestrian_id', i)
            setattr(ped, 'crosswalk_id', req.crosswalk_id)
            setattr(ped, 'toggle_sim', True)
            try:
                self.handle_spawn_pedestrian(ped)
            except:
                raise NameError("Service call failed!")
        msg = "%i pedestrians generated at crosswalk %i!" %(req.ped_number, req.crosswalk_id)
        return CrosswalkPassageResponse(True, msg)

    def handle_spawn_pedestrian(self, req):
        """
        @author: U{Nicolo' Campolongo<nicoloc@kth.se>}

        Spawn new pedestrian.

        This includes starting a new pedestrian node and calling its
        /set_destination service, as well as its /toggle_simulation
        service.

        @param req: I{(SpawnPedestrian)} Request message of the service that
                    spawns a new pedestrian.
        """
        namespace = "pedestrian_" + str(req.crosswalk_id) + str(req.pedestrian_id)
        args = "%i %i" % (req.pedestrian_id, req.crosswalk_id)
        node = Node('sml_world', 'pedestrian.py',
                    namespace=namespace, args=args,
                    name='pedestrian')
        self.launch_queue.put(node)

        crosswalk_service = '/get_crosswalk_nodes'
        rospy.wait_for_service(crosswalk_service)
        try:
            nodes_for_crosswalk = rospy.ServiceProxy(crosswalk_service, GetCrosswalkNodes)
            nodes = nodes_for_crosswalk(req.crosswalk_id)
        except rospy.ServiceException, e:
                raise NameError("Service call failed: %s" % e)
        node_1 = nodes.node_1
        node_2 = nodes.node_2

        destination_service = '/' + namespace + '/set_destination'
        rospy.wait_for_service(destination_service)
        try:
            set_dest = rospy.ServiceProxy(destination_service, SetDestination)
            set_dest(node_1, node_2)
        except rospy.ServiceException, e:
                raise NameError("Service call failed: %s" % e)
        toggle_sim_serv = '/' + namespace + '/toggle_simulation'
        rospy.wait_for_service(toggle_sim_serv)
        try:
            toggle_sim = rospy.ServiceProxy(toggle_sim_serv, SetBool)
            toggle_sim(req.toggle_sim)
        except rospy.ServiceException, e:
            raise NameError("Service call failed: %s" % e)
        msg = ("Pedestrian #%i was successfully spawned." % req.pedestrian_id)
        return SpawnPedestrianResponse(True, msg)

    def handle_spawn_vehicle(self, req):
        """
        Spawn new vehicle.

        This includes starting a new vehicle node and (if a looping node is
        set) calling its /set_loop service, as well as its /toggle_simulation
        service.

        @param req: I{(SpawnVehicle)} Request message of the service that
                    spawns a new vehicle.
        """
        namespace = "vehicle_" + str(req.vehicle_id)
        args = "%i %s %f %f %f %f" % (req.vehicle_id, req.class_name,
                                      req.x, req.y, req.yaw, req.v)
        node = Node('sml_world', 'vehicle.py',
                    namespace=namespace, args=args,
                    name=req.class_name.lower())
        self.launch_queue.put(node)

        loop_service = '/' + namespace + '/set_loop'
        rospy.wait_for_service(loop_service)

        if req.node_id != 0:
            try:
                set_loop = rospy.ServiceProxy(loop_service, SetLoop)
                set_loop(req.node_id)
            except rospy.ServiceException, e:
                raise NameError("Service call failed: %s" % e)

        # choose the first destination if the vehicle is SemiControlled
        if req.class_name == 'SemiControlledVehicle':
            req.toggle_sim = False
            highlight_service = '/highlight_node'
            rospy.wait_for_service(highlight_service)
            try:
                highlight_node = rospy.ServiceProxy(highlight_service, HighlightNode)
                # if we choose a node not included in the possible
                # destinations, all the destinations will then be highlighted
                # (check highlight_node srv for its use). -2 is currently not
                # the destinations (stored in visualization_module)
                highlight_node(-2, namespace)
            except rospy.ServiceException, e:
                raise NameError("Service call failed: %s" % e)

        toggle_sim_serv = '/' + namespace + '/toggle_simulation'
        rospy.wait_for_service(toggle_sim_serv)
        try:
            toggle_sim = rospy.ServiceProxy(toggle_sim_serv, SetBool)
            toggle_sim(req.toggle_sim)
        except rospy.ServiceException, e:
            raise NameError("Service call failed: %s" % e)
        msg = ("Vehicle #%i was successfully spawned." % req.vehicle_id)
        return SpawnVehicleResponse(True, msg)

    def spawn_vehicle(self):
        """Spawn vehicle node."""
        node = self.launch_queue.get()
        self.launched_processes.append(self.launch(node))


def update_vehicle_state(vs, vs_dict):
    """
    Write received vehicle state into world state.

    @param vs: I{(VehicleState)} Vehicle state that needs to be updated in the
               world state.
    @param vs_dict: I{(dict)} Dict of all vehicle states that is updated.
    """
    vs_dict[vs.vehicle_id] = vs

def update_pedestrian_state(ps, ps_dict):
    """
    Write received pedestrian state into world state.

    @param ps: I{(PedestrianState)} Pedestrian state that needs to be updated in the
               world state.
    @param ps_dict: I{(dict)} Dict of all pedestrian states that is updated.
    """
    ps_dict[ps.pedestrian_id] = ps

def update_traffic_demand(td, td_dict):
    """
    Write received demand into world state.

    @param td: I{(TrafficDemand)} Demand state that needs to be updated
    in the world state.
    @param td_dict: I{(dict)} Dict of demand of various transports.
    """
    td_dict[td.bus_demand] = td

def update_bus_stops(bus_status, bus_args):
    #Todo; Make this less terrifyingly disgusting
    '''
    Takes tuple to update the bus stops. Essentially copies bus_status
    to bus args.

    @param bus_status: {BusStops} Status of current bus stops
    @param bus_args: List of bus stops and demands published
    by
    '''
    bus_stops = bus_args[0]
    bus_demands = bus_args[1]
    if len(bus_demands) != len(bus_stops):
        raise NameError('Bus Stops module acting incorrectly')

    #Have to do it this way because you want to mutate objects
    #not reassign them. Since list is mutable, we can do it this
    #way as opposed to
    del bus_stops[:]
    del bus_demands[:]
    bus_stops.extend(list(bus_status.bus_stops))
    bus_demands.extend(list(bus_status.bus_stop_demands))

def update_sensor_state(sensor_state, sensor_dict):
    """
    Write received sensor state into world state
    @param sensor_state: I{(VehicleSensors)} Sensor state that needs to be updated in the
               world state.
    @param sensor_dict: I{(dict)} Dict of all sensor states that is updated.
    """
    sensor_dict[sensor_state.vehicle_id] = sensor_state

time = 0
def update_time(new_time):
    global time
    time = new_time.time


def update_lights(new_lights, lights):
    '''
    Will simply pass on the information from the Traffic Light
    Manager to everyone else. Makes it easier for condensation
    of data.
    '''
    for i in range(len(new_lights.node_ids)):
        lights[new_lights.node_ids[i]] = new_lights.on[i]


def sml_world_central():
    """Inizialize ROS-node 'sml_world' and start subs, pubs and srvs."""
    world_state = WorldState()

    vs_dict = {}  # Saves all vehicle states in a dict with vehicle_id as key
    ps_dict = {}  # same for pedestrians
    td_dict = {}
    sensor_dict = {}

    bus_stops = []
    bus_demands = []

    # node_id -> on/off bool
    traffic_lights = {}

    global time
    rospy.init_node('sml_world', log_level=rospy.WARN)
    rospy.Subscriber('current_vehicle_state', VehicleState,
                     update_vehicle_state, vs_dict)
    rospy.Subscriber('current_demand', TrafficDemand,
                     update_traffic_demand, td_dict)
    #These args must be lists as tuples are immutable
    rospy.Subscriber('current_bus_stops', BusStops,
                    update_bus_stops, [bus_stops, bus_demands])
    rospy.Subscriber('get_time', GetTime,
                    update_time)
    rospy.Subscriber('traffic_lights', TrafficLights,
                    update_lights, traffic_lights)
    rospy.Subscriber('/sensors', VehicleSensors,
        update_sensor_state, sensor_dict)
    rospy.Subscriber('current_pedestrian_state', PedestrianState,
                    update_pedestrian_state, ps_dict)

    launcher = ROSLaunchExtended()

    pub_ws = rospy.Publisher('world_state', WorldState, queue_size=10)

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        while not launcher.launch_queue.empty():
            launcher.spawn_vehicle()
        world_state.vehicle_states = vs_dict.values()
        world_state.pedestrian_states = ps_dict.values()
        world_state.traffic_demand = td_dict.values()
        world_state.bus_stop_ids = bus_stops
        world_state.bus_stop_demands = bus_demands
        world_state.time = time
        world_state.traffic_light_nodes = traffic_lights.keys()
        world_state.traffic_light_status = traffic_lights.values()
        world_state.sensor_states = sensor_dict.values()

        pub_ws.publish(world_state)
        if rate.remaining() < rospy.Duration(0):
            rospy.logwarn("SML-World central could not keep up with the " +
                          "update rate aimed for.")
            rate.last_time = rospy.get_rostime()
        else:
            rate.sleep()

if __name__ == '__main__':
    sml_world_central()
