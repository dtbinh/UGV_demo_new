import rospy
from sml_world.msg import GetTime, ProjectedDemand
from threading import Thread
import time
from sml_world.srv import AddDemand
import numpy as np

def compare_times(earlier, later):
    '''
    Compare two times as described by the formatting in SMLClock.
    Should never return a negative comparison
    '''
    if earlier > later:
        later += 2400 #Add one day
    return later - earlier

#Simple clock class used by DemandModel to model variance throughout the day
#It uses a simple int32 to talk into and out of.
#For example time= 1356 means the time is 13:56
class SMLClock():
    def __init__(self, starttime):
        self.cur_time = starttime
        self.publisher = rospy.Publisher('/get_time', GetTime, queue_size=5)
        self.proj_demand = rospy.Publisher('/projected_demand', ProjectedDemand,
                                            queue_size = 5)
        self.bus_stops = []

        loop_thread = Thread(target=self.loop)
        loop_thread.start()

        self.static_demand = [3, 4, 7, 2, 1, 4, 8, 7, 15, 24, 15, 10, 29,
                        13, 10, 11, 16, 20, 21, 23, 15, 17, 14, 9]
        self.static_demand = np.asarray(self.static_demand)
        self.static_demand /= 2

        self.proj_demand.publish(ProjectedDemand(self.get_next_four_hours()))


    def loop(self):
        while not rospy.is_shutdown():
            self.increment_min()
            time.sleep(.3)

    def increment_min(self):
        cur_hour = self.cur_time / 100
        cur_min = (self.cur_time % 100) + 1
        if cur_min == 60:
            self.add_demand_to_model_all_buses(self.get_static_demand())
            self.cur_time = ((cur_hour + 1) % 24) * 100
            self.proj_demand.publish(ProjectedDemand(self.get_next_four_hours()))
        else:
            self.cur_time = cur_hour * 100 + cur_min
        self.publisher.publish(GetTime(self.cur_time))

    def get_static_demand(self):
        hour = self.cur_time / 100
        return self.static_demand[hour]

    def get_next_four_hours(self):
        hour = self.cur_time / 100
        return self.static_demand[hour:((hour + 5) % 24)]

    def add_demand_to_model_all_buses(self, demand_added):
        try:
            d_add = rospy.ServiceProxy('/add_demand', AddDemand)
            for bus_id in self.bus_stops:
                d_add(bus_id, demand_added)
        except rospy.ROSException, e:
        #If this is reached, it means that the service was not found,
        #And there is no bus service to add values
            return

    def add_bus_stop(self, bus_id):
        self.bus_stops.append(bus_id)
