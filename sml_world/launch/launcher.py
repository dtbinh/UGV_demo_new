#!/usr/bin/env python
"""
CLI for initiating and starting different nodes in SML.
Documentation can be found on the wiki

Created on Jun 8, 2016

@author: U{Daniel Carballal<carba@kth.se>}
@organization: KTH
"""
import rospy
import os.path
from subprocess import Popen
from roslaunch.core import Node
import readline

#Class for colouring text blatently stolen from Stack Overflow
class bcolours:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

commands = ["set", "run", "start", "exit"]

def completer(text, state):
    options = [i for i in commands if i.startswith(text)]
    if state < len(options):
        return options[state]
    else:
        return None

readline.parse_and_bind("tab: complete")
readline.set_completer(completer)

map_file = "no/map/set"
bus_sys = False
sml_started = False

def check_valid_map_name(map_location):
    return os.path.isfile("../scripts/" + map_location + ".xml")

def set_map(map_name):
    map_loc = "/resources/scenarios/" + map_name
    ret = check_valid_map_name(map_loc)
    if not ret:
        print("Warning: That map name is not in your resources file")
    return map_loc

def set_bus_system(switch):
    global bus_sys
    bus_sys = {'on': True, 'off': False}.get(switch)

def run_sml_world():
    global sml_started
    if map_file == "no/map/set":
        print "No map was set!"
        return -1
    if sml_started:
        print "SML World has already been launched"
        return -1
    rospy.init_node('Launcher')
    print(map_file)
    Popen("rosrun sml_world sml_world_central.py &> /dev/null", shell=True)
    Popen("rosrun sml_world road_network.py " + map_file + "&> /dev/null", shell=True)
    Popen("rosrun sml_world visualization.py &> /dev/null", shell=True)
    sml_started = True
    return 1

def enter_command():
    global map_file
    command = raw_input("SML World Command Line > ")
    args = command.split(" ")
    if args[0] == "set":
        if len(args) != 3:
            print_set_help()
        elif args[1] == "map":
            map_file = set_map(args[2])

        elif args[1] == "bussing":
            set_bus_system(args[2])
        else:
            print "Invalid variable!"
    elif args[0] == "run":
        if len(args) != 1:
            print bcolours.WARNING + \
                "run has no arguements; configuire your enviroment using set" \
                + bcolours.ENDC
        else:
            run_sml_world()

    elif args[0] == "start":
        if not sml_started:
            print bcolours.WARNING + "Please start SML first, using run" + bcolours.ENDC
        if len(args) < 2:
            print_start_help()
        elif args[1] == "dummy":
            if len(args) != 4:
                print_start_help()
            else:
                p = Popen('rosservice call /spawn_vehicle "{vehicle_id: ' + args[2] \
                    + ', class_name: \'DummyVehicle\', x: 0.0, y: 0.0, yaw: 0.0, v: 10.0, \
                    node_id: ' + args[3] + ', toggle_sim: true}" &> /dev/null', shell=True)
                p.join()
        elif args[1] == "wifi":
            if len(args) != 4:
                print_start_help()
            else:
                p = Popen('rosservice call /spawn_vehicle "{vehicle_id: ' + args[2] \
                    + ', class_name: \'WifiVehicle\', x: 0.0, y: 0.0, yaw: 0.0, v: 10.0, \
                    node_id: ' + args[3] + ', toggle_sim: true}" &> /dev/null', shell=True)
                p.join()
        elif args[1] == "base":
            if len(args) != 4:
                print_start_help()
            else:
                p = Popen('rosservice call /spawn_vehicle "{vehicle_id: ' + args[2] \
                    + ', class_name: \'WifiVehicle\', x: 0.0, y: 0.0, yaw: 0.0, v: 10.0, \
                    node_id: ' + args[3] + ', toggle_sim: true}" &> /dev/null', shell=True)
                p.join()
        elif args[1] == "bus":
            global bus_sys
            if len(args) != 4:
                print_start_help()
            elif not bus_sys:
                print 'Need to start bus system before starting buses! Use set....'
            else:
                p = Popen('rosservice call /start_bus "{vehicle_id: ' + args[2] \
                    + ', class_name: \'BusVehicle\', x: 0.0, y: 0.0, yaw: 0.0, v: 10.0, \
                    node_id: ' + args[3] + ', toggle_sim: true}" &> /dev/null', shell=True)

        print "starting"
    elif args[0] == "exit":
        return
    else:
        print_general_help()

    enter_command()

def help():
    print "HELP"

def print_set_help():
    print bcolours.FAIL
    print "  Usage: set"
    print "  set <VARIABLE> <VALUE>"
    print "  Available variables: "
    print "     map: set to the xml map file found in resources"
    print "     bussing: turn the bus system exists on or off-- Off by default"
    print bcolours.ENDC

def print_general_help():
    print bcolours.FAIL
    print "  Available commands"
    print '    ' + ', '.join(commands)
    print "  Type <COMMAND> help for more information"
    print bcolours.ENDC

def print_start_help():
    print bcolours.FAIL
    print "  Usage: start"
    print '  start <RUNNABLE> <ID> <ARGS>'
    print '  Available Runnables:'
    print '    dummy: Starts dumb vehicle. Args is single integer representing a node'
    print '    wifi: Starts wifi vehicle. Args is a single integer representing a node'
    print '    base: Starts basic vehicle. '
    print '    gui: Starts the gui for the Kista Demo'
    print bcolours.ENDC



enter_command()

