#!/usr/bin/env python
"""
Connected Mobility Arena Presentation GUI Prototype.

@author: U{Marina Rantanen<marinar@kth.se>}
"""
import os
import gtk
import pygtk
pygtk.require('2.0')
import pygtk_chart
from pygtk_chart import bar_chart

from sml_world.srv import TurnOnGUI, TurnOnGUIResponse, DrawNode
from sml_world.srv import GetCoordinates, SetDestination
from sml_world.msg import ProjectedDemand
import rospy

import mocap

import sys
import signal
import threading

green = gtk.gdk.color_parse("green")
red = gtk.gdk.color_parse("red")
standardgray = gtk.gdk.Color(red=25000, green=25000, blue=25000, pixel=0)


class CMAWindow(gtk.Window):
    """Create GUI window."""


    def __init__(self):
        gtk.Window.__init__(self)
        self.set_size_request(1000, 400)
        self.set_position(gtk.WIN_POS_CENTER)

        self.base_path = '/home/mma/catkin_ws/src/sml_world/scripts'
        #self.base_path = '/home/dcarballal/catkin/src/sml_world/scripts'

        self.bkg = gtk.Image()
        self.bkgimagepath = self.base_path + '/resources/bkg_light.png'
        self.bkg.set_from_file(self.bkgimagepath)

        self.head = gtk.Image()
        self.headimagepath = self.base_path + '/resources/heading.png'
        self.head.set_from_file(self.headimagepath)

        self.statsimage = gtk.Image()
        self.statsimagepath = self.base_path + '/resources/start.jpg'

        self.closebtn = gtk.Image()
        self.closebtnimagepath = self.base_path + '/resources/closebtn.jpg'
        self.closebtn.set_from_file(self.closebtnimagepath)

        self.statsimage.set_from_file(self.statsimagepath)

        self.hbox_graph = gtk.HBox(False, 0)
        self.hbox_info = gtk.HBox()
        self.vbox = gtk.VBox(False, 0)
        fixed = gtk.Fixed()
        self.add(self.vbox)
        self.vbox.add(fixed)

        self.vbox.pack_start(self.hbox_info, False, False, 1)
        self.hbox_info.pack_end(self.statsimage, True, True, 0)
        self.vbox.pack_start(self.hbox_graph, True, True, 0)

        prevDestEvent = gtk.Button("Previous\ndestination")
        blacklabel = prevDestEvent.get_children()[0]
        prevDestEvent.modify_bg(gtk.STATE_NORMAL, standardgray)
        blacklabel.modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('white'))
        prevDestEvent.set_size_request(110, 110)
        prevDestEvent.connect("clicked", self.on_prev_dest_clicked)

        setDestEvent = gtk.Button("Set\n Destination")
        blacklabel = setDestEvent.get_children()[0]
        setDestEvent.modify_bg(gtk.STATE_NORMAL, standardgray)
        blacklabel.modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('white'))
        setDestEvent.set_size_request(110, 110)
        setDestEvent.connect("clicked", self.on_set_dest_clicked)

        nextDestEvent = gtk.Button("Next\ndestination")
        blacklabel = nextDestEvent.get_children()[0]
        nextDestEvent.modify_bg(gtk.STATE_NORMAL, standardgray)
        blacklabel.modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('white'))
        nextDestEvent.set_size_request(110, 110)
        nextDestEvent.connect("clicked", self.on_next_dest_clicked)

        subwayevent = gtk.Button("Subway\nmeltdown")
        blacklabel = subwayevent.get_children()[0]
        subwayevent.modify_bg(gtk.STATE_NORMAL, standardgray)
        blacklabel.modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('white'))
        subwayevent.set_size_request(110, 110)
        subwayevent.connect("clicked", self.on_sub_clicked)
        closeButton = gtk.Button()
        closeButton.set_image(self.closebtn)
        closeButton.connect("clicked", self.on_close_clicked)

        Dynstats = gtk.ScrolledWindow()
        Dynstats.set_size_request(430, 240)
        textbuffer = gtk.TextBuffer()
        textbuffer.set_text('Bus information \n Active buses: 10 '
                            '\n Requested stops: \n Waiting time: 5 minutes \n Travel time: 45 minutes')
        textview = gtk.TextView(buffer=textbuffer)
        Dynstats.add(textview)

        Dynvalues = gtk.ScrolledWindow()
        Dynvalues.set_size_request(80, 240)
        textbuffer = gtk.TextBuffer()
        textbuffer.set_text(' \n 4 \n 57 \n \n +50 ')
        textview2 = gtk.TextView(buffer=textbuffer)
        Dynvalues.add(textview2)

        self.Passtats = gtk.ScrolledWindow()
        self.Passtats.set_size_request(430, 240)
        passtext = gtk.TextBuffer()
        passtext.set_text('Static bus routing equivalent: \n Active buses: 3 \n Waiting time: 30 minutes \n Travel time: 45 minutes ')
        self.passview = gtk.TextView(buffer=passtext)
        self.Passtats.add(self.passview)

        fixed.put(self.head, 60, 10)

        fixed.put(prevDestEvent, 60, 135)
        fixed.put(setDestEvent, 60, 250)

        fixed.put(nextDestEvent, 180, 135)
        fixed.put(subwayevent, 180, 250)

        fixed.put(closeButton, 1360, 135)

        fixed.put(Dynstats, 340, 135)
        fixed.put(self.Passtats, 775, 135)
        self.demand = [('now', 7, 'Now'),
            ('plusone', 15, '+1h'),
            ('plustwo', 24, '+2h'),
            ('plusthree', 15, '+3h'),
            ('plusfour', 10, '+4h')]

        rospy.Subscriber('/projected_demand', ProjectedDemand, self.update_proj_demand)
        print('Subscriber set up')

        # SemiControlledVehicle params
        self.destinations = [-78, -246, -290, -514, -54]
        """
        self.destinations = {}
        b = [(-78, 'home'), (-246, 'office'), (-290, 'mall'), (-514, 'school'), (-54, 'gym')]
        for i in b:
            node_coord_service = '/get_node_coordinates'
            rospy.wait_for_service('/get_node_coordinates')
            try:
                nodeCoord = rospy.ServiceProxy(node_coord_service, GetCoordinates)
                p = nodeCoord(i[0])
                [pixel_x, pixel_y] = self.convert_position_to_image_pixel(p.x, p.y)
                self.destinations[i[0]] = ((pixel_x, pixel_y, i[1]))
            except rospy.ServiceException, e:
                    raise NameError("Service call failed: %s" % e)    
        rospy.Service('/highlight_node', HighlightNode,
                    self.handle_highlight_node)
        """
        self.highlight_node = False 
        rospy.Service('/turn_on_gui', TurnOnGUI, self.handle_turn_on_gui)


    """
    elif self.highlight_node:
        if event.type == pygame.KEYDOWN:                
            if event.key == pygame.K_LEFT:
                self.node_index += 1
                if self.node_index == len(self.destinations):
                    self.node_index = 0
            if event.key == pygame.K_RIGHT:
                self.node_index -= 1
                if self.node_index < 0:
                    self.node_index = len(self.destinations) - 1
            if event.key == pygame.K_RETURN:
                dest_service = self.semi_contr_vehicle + '/set_destination' 
                rospy.wait_for_service(dest_service)
                try:
                    set_dest = rospy.ServiceProxy(dest_service, SetDestination)
                    set_dest(self.h_node, 0)
                    self.highlight_node = False
                    try:
                        self.destinations[self.removed[0]] = (self.removed[1])
                    except:
                        rospy.logwarn("The first destination was not in the list!")
                except rospy.ServiceException, e:
                    raise NameError("Service call failed: %s" % e)

                # this is needed to turn on the simulation when
                # the vehicle is first spawned
                toggle_sim_serv = '/' + self.semi_contr_vehicle + '/toggle_simulation'
                rospy.wait_for_service(toggle_sim_serv)
                try:
                    toggle_sim = rospy.ServiceProxy(toggle_sim_serv, SetBool)
                    toggle_sim(True)
                except rospy.ServiceException, e:
                    raise NameError("Service call failed: %s" % e)
    """

    def handle_turn_on_gui(self, req):

        self.highlight_node = True
        self.node_index = 0
        self.semi_contr_vehicle = req.namespace
        self.removed = req.node_id
        rospy.wait_for_service('/draw_node')
        try:
            draw = rospy.ServiceProxy('/draw_node', DrawNode)
            draw(self.highlight_node, self.node_index)
        except rospy.ServiceException, e:
            raise NameError("Service call failed: %s" % e)
        msg = "GUI turned on"
        return TurnOnGUIResponse(True, msg)


    def on_next_dest_clicked(self, button):
        print("\"Next destination event\" button was clicked")
        
        if self.highlight_node:
            self.node_index += 1
            #if self.node_index == len(self.destinations):
            if self.node_index == 5:
                self.node_index = 0
            rospy.wait_for_service('/draw_node')
            try:
                draw = rospy.ServiceProxy('/draw_node', DrawNode)
                draw(self.highlight_node, self.node_index)
            except rospy.ServiceException, e:
                raise NameError("Service call failed: %s" % e)

    def on_prev_dest_clicked(self, button):
        print("\"Next destination event\" button was clicked")
        
        if self.highlight_node:
            self.node_index -=1
            if self.node_index < 0:
                # 4 is the number of destinations
                self.node_index = 4
            rospy.wait_for_service('/draw_node')
            try:
                draw = rospy.ServiceProxy('/draw_node', DrawNode)
                draw(self.highlight_node, self.node_index)
            except rospy.ServiceException, e:
                raise NameError("Service call failed: %s" % e)        

    def on_set_dest_clicked(self, button):
        """
        """
        dest_service = self.semi_contr_vehicle + '/set_destination' 
        rospy.wait_for_service(dest_service)
        try:
            set_dest = rospy.ServiceProxy(dest_service, SetDestination)
            set_dest(self.destinations[self.node_index], 0)
            self.highlight_node = False
            try:
                self.destinations.append(self.removed)
            except:
                rospy.logwarn("The first destination was not in the list!")
        except rospy.ServiceException, e:
            raise NameError("Service call failed: %s" % e)

    def on_queue_clicked(self, button):
        print("\"Queue event\" button was clicked")

        return
        

    def on_road_clicked(self, button):
        print("\"Road event\" button was clicked")
        
        return

    def on_hero_clicked(self, button):
        statsimagepath = self.base_path + '/resources/hero.jpg'
        
        return

    def update_proj_demand(self, data):
        print('UPDATING DEMAND')
        
        return

    def add_demand_to_model(self, bus_id, demand_added):
        rospy.wait_for_service('/add_demand')
        d_add = rospy.ServiceProxy('/add_demand', AddDemand)
        d_add(bus_id, demand_added)

    def on_sub_clicked(self, button):
        print("\"Subway event\" button was clicked")
#        self.statsimage.set_from_file("concertstats.jpg")
        statsimagepath = self.base_path + '/resources/subway.jpg'
        self.statsimage.set_from_file(statsimagepath)
        event_id = 4
        self.stat_graph(event_id)
        self.passenger_graph(event_id)


    def stats(self, event_id):
        #Routing statistics
        if event_id == 0:
            stats = [('waitingtime', 0, 'Waiting time'),
                ('traveltime', 0, 'Travel time'),
                ('fuelconsumption', 0, 'Fuel consumption'),
                ('cost', 0, 'Cost'),
               ]
        elif event_id == 1:
            stats = [('waitingtime', 80, 'Waiting time'),
                ('traveltime', 52, 'Travel time'),
                ('fuelconsumption', 101, 'Fuel consumption'),
                ('cost', 65, 'Cost'),
               ]
        elif event_id == 2:
            stats = [('waitingtime', 90, 'Waiting time'),
                ('traveltime', 78, 'Travel time'),
                ('fuelconsumption', 150, 'Fuel consumption'),
                ('cost', 120, 'Cost'),
               ]
        elif event_id == 3:
            stats = [('waitingtime', 43, 'Waiting time'),
                ('traveltime', 67, 'Travel time'),
                ('fuelconsumption', 89, 'Fuel consumption'),
                ('cost', 100, 'Cost'),
               ]
        elif event_id == 4:
            stats = [('waitingtime', 86, 'Waiting time'),
                ('traveltime', 98, 'Travel time'),
                ('fuelconsumption', 82, 'Fuel consumption'),
                ('cost', 100, 'Cost'),
               ]
        return stats

    def stat_graph(self, event_id):
        return

        data = self.stats(event_id)

        statchart = bar_chart.BarChart()

        statchart.title.set_text('Impact of dynamic routing')
        statchart.grid.set_visible(True)
        statchart.grid.set_line_style(pygtk_chart.LINE_STYLE_DOTTED)
        statchart.set_mode(bar_chart.MODE_VERTICAL)

        for bar_info in data:
            bar = bar_chart.Bar(*bar_info)
            if bar_info[1] <= 100:
                bar.set_color(green)
            elif bar_info[1] > 100:
                bar.set_color(red)
            statchart.add_bar(bar)
            #statchart.queue_draw()


        self.hbox_graph.pack_start(statchart, True, True, 0)

        #width, height = 400, 300
        #box.set_size_request(width, height)

        def cb_bar_clicked(statchart, bar):
            print "Bar '%s' clicked." % bar.get_label()

        statchart.connect("bar-clicked", cb_bar_clicked)
        statchart.show()


    def on_close_clicked(self, button):
        print("Closing application")
        gtk.main_quit()


def qualisys_pos(body_id, x, y, yaw):
    drawingarea = gtk.DrawingArea()
    drawingarea.set_size_request(600, 300)
    drawable = drawingarea.window


def qualisys_info():
    """
    Qualisys data listener.
    @param qs_body:
    """
    stop = False

    def signal_handler(signal, frame):
        stop = True
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    Qs = mocap.Mocap(info=1)
    bodies = Qs.find_available_bodies(printinfo=1)

    #pick the first valid body
    id_body = Qs.get_id_from_name("Iris2")
    body = mocap.Body(Qs, id_body)
    pose = body.getPose()
    qsid = pose['id']
    qsx = pose['x']
    qsy = pose['y']
    qsyaw = pose['yaw']

    qualisys_pos(qsid, qsx, qsy, qsyaw)

    #while not stop:
        #pose = body.getPose()

rospy.init_node('GUI')
win = CMAWindow()
win.connect("delete-event", gtk.main_quit)
win.show_all()
gtk.gdk.threads_init()
thread = threading.Thread(target=gtk.main)
thread.start()
rospy.spin()
