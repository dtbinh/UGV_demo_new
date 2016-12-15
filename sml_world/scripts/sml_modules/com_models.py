"""
Module containing all available communication classes.

Created on Mar 8, 2016

@author: U{David Spulak<spulak@kth.se>}
@organization: KTH
"""
import math

import rospy
import sml_world.msg as msgs
import sml_world.srv as srvs
from roslaunch.core import Node
from roslaunch.scriptapi import ROSLaunch
from sys import maxint

from numpy import tan

class BaseCom(object):
    """Base class for communication."""

    def __init__(self, vehicle_id):
        """Initialize class BaseCom."""
        self.vehicle_id = int(vehicle_id)
        self.class_name = self.__class__.__name__
        self.pos = (None, None)
        # Register subscribers, publisher and service
        pubsub_msg = getattr(msgs, self.class_name+'Com')
        glob_pubsub_name = '/all_'+self.class_name.lower()+'_com'
        cb_func_name = 'filter_all_'+self.class_name.lower()+'_com'

        # Will register to topic named 'all_<communication_type>_com'
        # Which will callback the function named filter_all_<com_type>_com
        rospy.Subscriber(glob_pubsub_name, pubsub_msg,
                         getattr(self, cb_func_name))
        self.pub_glob = rospy.Publisher(glob_pubsub_name, pubsub_msg,
                                        queue_size=10)
        rospy.Subscriber('/world_state', msgs.WorldState,
                         self.update_vehicle_state)
        send_service_name = 'send_'+self.class_name.lower()+'_com'
        rospy.Service(send_service_name,
                      getattr(srvs, 'Send'+self.class_name+'Com'),
                      getattr(self, 'handle_'+send_service_name))

    def update_vehicle_state(self, ws):
        """Callback function for topic 'world-state'."""
        self.pos = (None, None)
        for vs in ws.vehicle_states:
            if vs.vehicle_id == self.vehicle_id:
                self.pos = (vs.x, vs.y)
                self.yaw = vs.yaw


class Wifi(BaseCom):
    """Wifi communication class."""

    def __init__(self, vehicle_id, name, com_range):
        """Initialize Wifi communication class."""
        super(Wifi, self).__init__(vehicle_id)
        self.name = name
        self.com_range = float(com_range)
        self.pub_com = rospy.Publisher(name.lower()+'_com', msgs.WifiCom,
                                       queue_size=10)

    def filter_all_wifi_com(self, wc):
        """Filter Wifi communication to what the vehicle can receive."""
        if wc.vehicle_id == self.vehicle_id:
            return
        if (not self.pos[0] or not self.pos[1]):
            return
        dx = self.pos[0] - wc.x
        dy = self.pos[1] - wc.y
        dist = math.hypot(dx, dy)
        if dist <= self.com_range:
            self.pub_com.publish(wc)

    def handle_send_wifi_com(self, req):
        """Handle sending msg over wifi request."""
        self.pub_glob.publish(self.vehicle_id, self.pos[0], self.pos[1],
                              req.message)
        return srvs.SendWifiComResponse(True, "Message sucessfully sent out.")


CONNECTION_MIN_DIST = 15
DISCONNECT_MIN_DIST = 20
MAX_ERROR = .4
OVERTAKE_MERGE_DIST = 8
ptdist = lambda p1,p2: math.sqrt((p1[0]-p2[0]) ** 2 + (p1[1]-p2[1]) ** 2)
OVERPASS_SAFETY_MAX_CARS = 3
PASS_DIST = 4

class DirectWifi(BaseCom):
    '''
    Establishes a direct connection with another vehicle_id. May only
    maintain one connection at a time
    '''
    def __init__(self, vehicle_id, sub_name, other_vehicle_id=None, other_sub_name=None, speed_cb=None, overpass_cb= None, abort_cb=None):
        self.connect_id = other_vehicle_id
        self.vehicle_id = vehicle_id
        self.sub_name = sub_name

        # Dictionary of vehicle IDs to service proxies.
        self.direct_dict = {}
        if other_vehicle_id != None:
            con = rospy.ServiceProxy('/direct_com_'+other_vehicle_id+'_'+
                        other_sub_name, srv.DirectCom)
            self.direct_dict[other_vehicle_id] = con
            self.is_connected = True
        else:
            self.con = None
            self.is_connected = False

        rospy.Service('/direct_com_' + str(vehicle_id) + '_' + sub_name, 
                    srvs.DirectCom, self.handle_request)

        self.speed_cb = speed_cb
        self.overpass_cb = overpass_cb
        self.abort_cb = abort_cb


    def establish_connection(self, v_id, sub_name):
        '''
        Establishes a connection to v_id. Ideally, that connection should
        also connect to you.
        '''
        #TODO: Connection already established?
        if not v_id in self.direct_dict:
            try:
                con = rospy.ServiceProxy('/direct_com_'+str(v_id)+'_'+sub_name,
                                            srvs.DirectCom)
                self.direct_dict[v_id] = con
            except rospy.ServiceException, e:
                raise Exception(e)

        self.connect_id = v_id
        rospy.logwarn('Connection established with ' + self.sub_name + ' of ' + str(self.vehicle_id) +
                            ' and ' + str(v_id))
        self.is_connected = True

    def communicate_to_partner(self, message):
        con = self.direct_dict[self.connect_id]
        return con(message)


    def handle_request(self, message):
        '''
        These first ones are generally messages sent front --> back
        '''
        if message.msg == 'max_speed':
            return srvs.DirectComResponse(str(self.speed_cb()))
        elif message.msg.startswith('overpass'):
            # Message should be formatted as "overpass 2", meaning we've counted
            # two vehicles so far
            cars_count = int(message.msg.split(' ')[-1])

            # Return message will be formatted as "safe 23", meaning the head
            # of the queue of traffic has id 23
            req_resp = self.handle_overpass_request(cars_count)
            if req_resp != -1:
                return srvs.DirectComResponse('safe ' + str(req_resp))
            else:
                return srvs.DirectComResponse('unsafe')
        # These messages are sent back -->front
        # The front com will tell the smart wifi to set the speed to the desired speed
        # Formatted as "set_max_speed 40", as in, set max speed to 40 and keep pushing
        # info backwards until no more connections are left in the link.
        elif message.msg.startswith('set_max_speed'):
            self.speed_cb(int(message.msg.split(' ')[-1]))
        elif message.msg == 'reset_op_flag':
            self.overpass_cb()
        elif message.msg == 'abort_overtake':
            self.abort_cb()
        else:
            rospy.logwarn('Warning: unknown message: ' + message.msg)

        return srvs.DirectComResponse('Thanks ' + str(self.connect_id) + '!')

    def speed_up_request(self, v):
        '''
        Tell the vehicle behind us to speed up to v, or their preferred speed
        '''
        if not self.is_connected:
            return
        else:
            self.communicate_to_partner('set_max_speed ' + str(v))

    def reset_overpass_flag(self):
        if not self.is_connected:
            return
        self.communicate_to_partner('reset_op_flag')


    def disconnect(self):
        self.is_connected = False
        self.connect_id = None

    def check_for_disconnect(self, my_pos, baby_pos):
        '''
        Oh baby, have we gotten too far apart?

        Returns 0 if we disconnect, 1 if we remain connected
        '''
        if ptdist(my_pos, baby_pos) > DISCONNECT_MIN_DIST:
            rospy.logwarn(self.sub_name + ' of ' + str(self.vehicle_id) + ' and ' + str(self.connect_id) + \
                            ' have disconnected')
            self.disconnect()

    def get_max_speed(self):
        if not self.is_connected:
            return maxint
        else:
            resp = self.communicate_to_partner('max_speed')
            return int(float(resp.ret_msg))

    def handle_overpass_request(self, num_cars):
        return self.overpass_cb(num_cars)

    def request_overpass(self, car_count):
        '''
        Request of the car in front of us whether or not we can pass. We have
        to attach the number of cars to the request string
        '''
        if not self.is_connected:
            return
        resp = self.communicate_to_partner('overpass ' + str(car_count))
        if resp.ret_msg.startswith('safe'):
            # return the id of the head vehicle
            return int(resp.ret_msg.split(' ')[-1])
        else:
            return -1

    def stop_overtake(self):
        '''
        Something has occurred and the vehicle behind us must stop overtaking.
        '''
        if not self.is_connected:
            return
        self.communicate_to_partner('abort_overtake')


def to_array_traj(pos_traj):
    arr = []
    for pos in pos_traj:
        arr.append([pos.x, pos.y])
    return arr

class SmartWifi(BaseCom):
    '''
    Communication class that acts through wifi, but has the limitation
    that it only connects to vehicles directly in front of or behind 
    it. Thus, we will have two radars, one in each direction, which 
    will sense when a vehicle is close by, then send a request to connect.
    We will stay connected to this vehicle so long as we retain this
    connection.
    '''
    def __init__(self, vehicle_id, name, com_range):
        super(SmartWifi, self).__init__(vehicle_id)

        self.name = name
        self.yaw = 0.
        #unused
        self.com_range = float(com_range)

        self.launcher = ROSLaunch()
        self.launcher.start()
        self.front_con = DirectWifi(vehicle_id, 'front', speed_cb=self.light_turned_green, overpass_cb=self.reset_op_flag,abort_cb=self.stop_overtaking)
        self.back_con = DirectWifi(vehicle_id, 'back', speed_cb=self.max_speed_request, overpass_cb=self.handle_overpass)
        self.desired_speed = -1 # Will be updated on the first update_traj

        # Tracks all vehicles within range of the vehicle, useful for telling
        # safety of overtaking
        self.vehicles_within_range = []
        self.trajectory = []

        self.sub = rospy.Subscriber('/smart_wifi_request_' + str(vehicle_id), 
                    msgs.SmartWifiRequest, self.update_traj)      

        self.commander = rospy.Publisher('/smart_wifi_commands_' + str(vehicle_id),
                      msgs.SmartWifiCommand, queue_size = 10)

        self.overtaking = False
        self.overtake_in_progress = False
        
        self.is_head_in_overtake = False
        

    def update_vehicle_state(self, ws):
        '''
        Update vehicle state and publish results to smart_wifi
        '''
        super(SmartWifi, self).update_vehicle_state(ws)
        self.handle_send_smartwifi_com()

    def set_speed(self, speed=None, reason='None'):
        if speed == None:
            speed = self.desired_speed
        self.send_command(2, speed, reason)

    def max_speed_request(self):
        '''
        Returns max speed that this vehicle and all vehicles behind
        this vehicle can go
        '''
        if self.overtaking:
            speed = self.desired_speed * 1.2
            self.set_speed(speed, 'overtaking')
            return speed
        speed = min(self.desired_speed, self.front_con.get_max_speed())
        self.set_speed(speed, 'traffic')
        return speed

    def light_turned_green(self, new_speed):
        speed_we_go = min(new_speed, self.desired_speed)
        self.back_con.speed_up_request(speed_we_go)
        self.set_speed(speed_we_go, 'green light')


    def update_traj(self, vehicle_request):
        '''
        Obtain the trajectory from the vehicle, as well as process
        any requests.
        '''

        # Convert the trajectory back to its original form from Pose2d
        self.trajectory = to_array_traj(vehicle_request.trajectory)

        self.desired_speed = vehicle_request.desired_speed
        if vehicle_request.request == 1:
            #Returns int
            traffic_head = self.handle_overpass(0)                          # Traffic head is the leading vehicle that you need to overtake before
            if traffic_head != -1:                                          # getting back to safety.
                self.head_of_traffic_id = traffic_head
                self.send_command(1, traffic_head, 'Safe to overtake')

        # Max speed will be the maximum of all of the vehicles in
        # front of this one
        if vehicle_request.request == 2:
            self.max_speed_request()

        # Start an overtake
        if vehicle_request.request == 3:
            self.start_overpass()

        # Vehicle has begun overtaking.
        # We will set overtake pass once we have actually passed the vehicle
        if vehicle_request.request == 4:
            self.overtaking = True
            self.overtake_pass = False
            self.max_speed_request()

        #Vehicle has finished overtaking. Unused for now.
        if vehicle_request.request == 5:
            self.stop_overtaking()

        # Light changed to green, let cars behind us know
        if vehicle_request.request == 6:
            self.light_turned_green(self.desired_speed)

    def launch_direct_wifis(self):
        '''
        Launch the wifi modules which will speak directly to the
        to other vehicles. They will start unconnected to other
        vehicles.
        '''
        self.front_con = DirectWifi(self.vehicle_id, 'front')
        self.back_con = DirectWifi(self.vehicle_id, 'back')

    def predict_trajectory(self, target, yaw, their_yaw):
        '''
        After filtering, we want to tell if the other car is on our
        trajectory, meaning that it is somewhere in front of us.

        If it is we will launch our front connection, and it in turn
        will launch its back connection to us (DirectComs are a two
        way connection)

        Returns 2 if the vehicle is on our trajectory and is in front
        of us

        Returns 1 if the vehicle is on our trajectory and is probably
        behind us (less accurate than the forward looking one)

        Returns 0 otherwise
        '''
        if abs(yaw - their_yaw) > 1:
            return 0
        #Find the closest point on our path to our target's position
        min_index_target, min_traj_value = min(enumerate(self.trajectory), key=lambda x: ptdist(self.trajectory[x[0]], (target.x, target.y)))
        min_traj_diff = ptdist(min_traj_value, (target.x, target.y))

        # Find our index on our own trajectory. The purpose of doing this
        # is that we only want to return true if there is a car in front 
        # of us. So our index must be before the targets index of our
        # trajectory for us to return true.
        min_index_self = min(range(len(self.trajectory)), key=lambda i: ptdist(self.trajectory[i], self.pos))
        if min_traj_diff < MAX_ERROR:
            if min_index_self < min_index_target and min_index_target - min_index_self < len(self.trajectory) / 2:
                return 2
        my_x = self.trajectory[min_index_self][0]
        my_y = self.trajectory[min_index_self][1]
        back_traj = math.fabs(tan(yaw) - ((target.y - my_y) / (target.x - my_x)))
        if back_traj < MAX_ERROR:
            return 1

        return 0

    def stop_overtaking(self):
        self.overtaking = False
        self.overtake_pass = False
        self.head_of_traffic_id = -1
        self.set_speed()
        self.send_command(3, -1, 'Stop overtaking')

    def reset_op_flag(self):
        rospy.logwarn('Resetting op flag')
        self.overtake_in_progress = False
        self.back_con.reset_overpass_flag()

    def filter_all_smartwifi_com(self, wc):
        '''
        Filter to find vehicles within a pretermined range. If we find such a vehicle,
        we will test wheather it is on our trajectory. If it is, we will connect to it.
        '''
        # Ensure self.pos is not (None,None)
        if wc.vehicle_id == self.vehicle_id or not self.pos[0]:
            #self.pos = (vs.x, vs.y)
            return

        dist = ptdist(self.pos, (wc.x, wc.y))
        # We want to count the number of vehicles within our range for safety.
        if dist < CONNECTION_MIN_DIST and not wc.vehicle_id in self.vehicles_within_range:
            self.vehicles_within_range.append(wc.vehicle_id)
            if abs(self.yaw - wc.yaw) > 2.:
                self.back_con.stop_overtake()
        elif dist > CONNECTION_MIN_DIST and wc.vehicle_id in self.vehicles_within_range:
            self.vehicles_within_range.remove(wc.vehicle_id)

        # if we are able to merge back from overtaking, we will set overtake_pass
        # to true. That is, we have passed the head of traffic, and we can safely
        # merge back in.
        if self.overtaking and wc.vehicle_id == self.head_of_traffic_id:
            if ptdist(self.pos, (wc.x, wc.y)) < PASS_DIST:
                if self.vehicle_id == 2:
                    rospy.logwarn('--------------- SETTTING TO TRUE')
                self.overtake_pass = True

            if ptdist(self.pos, (wc.x, wc.y)) > OVERTAKE_MERGE_DIST and self.overtake_pass:
                #Merge back into the original lane
                self.stop_overtaking()
                self.front_con.disconnect()



        # If we are connected to a vehicle, disconnect when we get out of range or when 
        # we have overtaken them. If we are connected, we ignore all vehicles that are 
        # not the one we are connected to.
        if self.front_con.is_connected:
            # Is this the vehicle we are connected to?
            if wc.vehicle_id == self.front_con.connect_id:
                # check_for_disconnect will disconnect the vehicle if it is too
                # far away. Will return true if vehicle did indeed disconnect
                self.front_con.check_for_disconnect(self.pos, (wc.x, wc.y))
            return


        if self.back_con.is_connected:
            if wc.vehicle_id == self.back_con.connect_id:
                self.back_con.check_for_disconnect(self.pos, (wc.x, wc.y))
        elif self.overtaking:
            self.stop_overtaking()

        
        if dist < CONNECTION_MIN_DIST:
            calc_traj = self.predict_trajectory(wc, self.yaw, wc.yaw)
            if calc_traj == 2 and not self.front_con.is_connected:
                self.front_con.establish_connection(wc.vehicle_id, 'back')
                #We will now update our global smartwifi to let the vehicle in front
                #of us to connect to us
                self.handle_send_smartwifi_com(wc.vehicle_id)
                self.max_speed_request()
            elif calc_traj == 1 and not self.back_con.is_connected:
                self.back_con.establish_connection(wc.vehicle_id, 'front')
                if self.overtake_in_progress:
                    self.reset_op_flag()

    def handle_send_smartwifi_com(self, back_connection_id=-1):
        '''
        Back connection signals to the vehicle with that id that their back connection
        should point to our forward connection to allow for two way communication.
        '''
        self.pub_glob.publish(self.vehicle_id, self.pos[0], self.pos[1], self.yaw, back_connection_id)
        return srvs.SendSmartWifiComResponse(True, "Message sucessfully sent out.")

    def send_command(self, command, result, msg):
        self.commander.publish(command, result, msg)


    def handle_overpass(self, num_cars):
        '''
        Returns whether an overpass is possible. There are two reasons that it is not:

        There are more than OVERPASS_SAFETY_MAX_CARS in front of the car requesting an
        overpass.
        There is another vehicle that the requesting car cannot see but the cars in 
        front of them can.

        This function returns an integer:

        -1 means that it is not safe to pass
        Any other value means is the vehicle_id of the frontmost car.
        '''
        if num_cars == OVERPASS_SAFETY_MAX_CARS or self.overtaking or self.overtake_in_progress:
            return -1
        if self.front_con.connect_id != None:
            # Three cars would be one behind, one in front and
            # one approaching in the other lane, thus unsafe.
            # NOTE: This method does not work on multiple lane
            # roads. We have to be more clever there.....
            if len(self.vehicles_within_range) >= 3:
                return -1
            else:
                ret = self.front_con.request_overpass(num_cars + 1)
        else:
            if len(self.vehicles_within_range) >= 2:
                return -1
            else:
                ret = self.vehicle_id
                # We are the top head, continously check for incoming vehicles.
                self.is_head_in_overtake = True
        if ret != -1:
            self.overtake_in_progress = True
        return ret

    def start_overpass(self):
        pass