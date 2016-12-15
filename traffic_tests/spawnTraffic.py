#! /usr/bin/python

import rospy
from sml_world.srv import *
import numpy as np

# spawn buses
rospy.wait_for_service('start_bus_route')
bSrv = rospy.ServiceProxy('start_bus_route', StartBusRoute)
bSrv([-154, -210], 1000, -158)
rospy.sleep(1)
bSrv([-218, -396], 1001, -400)
rospy.sleep(1)
bSrv([-394, -388], 1002, -398)
#
def randomId(nIds):
    return nIds[np.random.randint(len(nIds))]
#
if __name__ == '__main__':
#
    rospy.wait_for_service('lanelets')
    lSrv = rospy.ServiceProxy('lanelets', Lanelets)
    x = lSrv()
    #nIds = x.values
    nIds = [-284, -390, -386, -390, -386, -284, -284, -390, -386, -390, -386, -284, -284, -390, -386, -390, -386, -284]
    x = [90, -10, -100, 30, -150, 100, 90, -10, -100, 30, -150, 100, 90, -10, -100, 30, -150, 100]
    y = [70, -60, 40, -60, 10, 50, 70, -60, 40, -60, 10, 50, 70, -60, 40, -60, 10, 50]
    # spawn vehicle
    rospy.wait_for_service('spawn_vehicle')
    for vId in range(10, 16):
        try:
            svSrv = rospy.ServiceProxy('spawn_vehicle', SpawnVehicle)
            #x = np.random.randint(100)
            #y = np.random.randint(100)
            v = np.random.randint(15) + 10
            #svSrv(vId, 'DummyVehicle', x, y, 0, v, randomId(nIds), True)
            svSrv(vId, 'DummyVehicle', x[vId], y[vId], 0, v, nIds[vId], True)
            rospy.logwarn('Vehicle ' + str(vId) + ' successfully spawned')
            print 'Vehicle ' + str(vId) + ' successfully spawned'
            rospy.sleep(2)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
#


    # cross walk
    rospy.wait_for_service('/get_crosswalk_ids')
    cwIdSrv = rospy.ServiceProxy('get_crosswalk_ids', GetCrosswalkIds)

    # Random pedestrian
    crossWalkIds = cwIdSrv().crosswalk_ids
    # Pedestrian only in upper loop
    #crossWalkIds = [2, 3, 4, 7]
    rospy.wait_for_service('/crosswalk_passage')
    cwSrc = rospy.ServiceProxy('crosswalk_passage', CrosswalkPassage)


    while not rospy.is_shutdown():
        cwId = randomId(crossWalkIds)
        resp = cwSrc(cwId, 20)
        print 'Pedestrians crossing crosswalk ' + str(cwId)
        rospy.sleep(6)
























#
