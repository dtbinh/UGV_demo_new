#!/usr/bin/env python
"""
Pedestrian node of the SML-World.

Created on July 6, 2016

@author: U{Nicolo' Campolongo<nicoloc@kth.se>}
@organization: KTH
"""

import sys

import rospy

from sml_modules.pedestrian_model import Pedestrian

def pedestrian(pedestrian_id, crosswalk_id):
    """
    Initialize ROS-node 'pedestrian' and register subs, pubs and services.
    """
    rospy.init_node('pedestrian', log_level=rospy.WARN)
    Pedestrian(rospy.get_namespace(), pedestrian_id, 20,
                crosswalk_id)
    rospy.spin()
    

if __name__ == '__main__':
    # Filter sys.argv to remove automatically added arguments
    sys.argv = [arg for arg in sys.argv if str(arg).find(':=') < 0]
    args = {}
    if len(sys.argv) > 1:
        args['pedestrian_id'] = int(sys.argv[1])
    else:
        msg = ("Usage: rosrun sml_world pedestrian.py " +
               "<pedestrian_id> <crosswalk_id>")
        raise Exception(msg)
    if len(sys.argv) > 2:
        args['crosswalk_id'] = float(sys.argv[2])
    pedestrian(**args)