"""
Visualization module for the SML World.

Created on Feb 25, 2016

@author: U{Rui Oliviera<rfoli.se>}, U{David Spulak<spulak@kth.se>}
@organization: KTH
"""

import pygame
import os
import math
import rospy
import numpy as np
from collections import defaultdict

from sml_modules import bodyclasses
from sml_modules.pedestrian_model import compute_line_params
from sml_modules.vehicle_models import BaseVehicle, DummyVehicle, WifiVehicle, TruckVehicle
from sml_modules.vehicle_models import ConnectivityVehicle, RandomDestinationVehicle
from sml_modules.vehicle_models import SemiControlledVehicle
from sml_modules.bus_vehicle_model import BusVehicle
from sml_world.srv import GetCoordinates, Lanelets, SetDestination, SetBool

from sml_world.srv import GetTrajectory
from sml_world.srv import GetCrosswalkNodes
from sml_world.srv import GetCrosswalkIds
from sml_world.srv import DrawCrosswalkId, DrawCrosswalkIdResponse
from sml_world.srv import HighlightNode, HighlightNodeResponse
from sml_world.srv import DrawNode, DrawNodeResponse
from sml_world.msg import LaneletTraffic, VehicleTrafficUpdate
from sml_world.srv import GetRadar
from sml_modules.SMLClock import SMLClock

import numpy as np

##jay
#from subprocess import call
##


#Demand bar constants
BACKGROUND_BAR_WIDTH = 34 #30
BAR_HEIGHT = 16

def red_green_mix(ratio):
    if ratio > 1.0:
        ratio = 1.0
    return (255 * ratio, 255 * ratio, 255 * (1. - ratio), .5)


class Visualization:
    """
    Class to diplay the current state of the SML World.

    Visualization is a class to visually display the current state of the
    SML World.  It is necessary that the thread running this class is the main
    program thread, otherwise Pygame will not work correctly.
    Due to that reason this class can be lauched from two ways:

    1) Using Process from multiprocessing library;
    2) By an individual secondary script that is simply giving this class the
       main thread of processing.
    """


    def __init__(self, base_path, file_path, window_width, window_height,

                 pixel_per_meter=-1, ground_projection=False,

                 traffic = True, traffic_lights=True, radars = True):
        """
        Inizialize the class Visualization.
        @param base_path:
        @param file_path:
        """
        rospy.logwarn(traffic)

        #Will provide the get_time service
        self.clock = SMLClock(700)

        self.id = 0
        # The desired dimensions and pixel resolution
        # for the window to be shown
        self.desired_window_width = float(window_width)
        self.desired_window_height = float(window_height)
        self.desired_pixel_per_meter = float(pixel_per_meter)

        # The filename which defines the map/image to
        # serve as background
        self.base_path = base_path
        map_filename = base_path + file_path
        # Determines if the visualization is meant to
        # be used for the ground projector at the
        # SML
        self.ground_projection = ground_projection

        # As measured by Rui [LEFT, RIGHT, DOWN, UP]
        self.projector_area = [3.360, 4.490, 2.920, 2.970]
#        self.projector_area = [300, 400, 20, 20]
        # The refresh rate of the visualization screen
        self.refresh_rate = float(20)

        self.bg_surface = None

        self.vehicles_dict = dict()
        self.sensors_dict = dict()
        self.pedestrians_dict = dict()

        self.areas_to_blit = []

        self.load_image_meta_data(map_filename)
        self.load_image(map_filename)

        self.load_red_car_image()
        self.load_yellow_car_image()
        self.load_green_car_image()
        self.load_blue_car_image()
        self.load_white_car_image()
        self.bus_stop_img = self.load_bus_stop_image()
        self.bus_stop_img.convert()
        self.load_crosswalk_image()
        self.load_ped_image()

        self.load_smart_car_image()
        self.load_truck_image()
        self.load_red_bus_image()
        self.load_green_bus_image()
        self.load_yellow_bus_image()
        self.load_bus_image()
        self.load_big_box_image()
        self.load_small_box_image()
        self.load_goal_image()
        self.setup_id_font()
        self.load_block_image()
        self.load_bus_image()
        self.load_block_image()
        self.load_big_red_x_image()

        # Toggle visualisation on/off
        self.traffic = traffic
        self.traffic_lights = traffic_lights
        self.radars = radars

        self.show_ids = True

        self.bus_stop_demands = []
        self.closed_bus_stops = []

        # To ensure that our bars are not overwritten
        # We define a reset variable to draw every
        # 30 iterations
        self.bus_stop_reset = 0
        self.lanelet_dict = {}
        self.lanelet_ids = {}
        self.node_coords = {}
        self.traffic_update = False
        self.traffic_news = {}
        self.demand_change = []
        rospy.wait_for_service('/lanelets')
        get_lanelet_info = rospy.ServiceProxy('/lanelets', Lanelets)
        response = get_lanelet_info()
        self.update_lanelet_info(response)
        rospy.Subscriber('/lanelet_traffic_data', LaneletTraffic, self.update_lanelet_traffic)

        # Crosswalks params
        self.crosswalks = defaultdict(list)
        self.gen_crosswalks_dict()
        self.crosswalk_id = False
        rospy.Service('/draw_crosswalk_ids', DrawCrosswalkId,
                      self.handle_draw_crosswalk_ids)

        # SemiControlledVehicle params
        self.destinations = {}
        b = [(-78, 'home'), (-246, 'office'), (-290, 'mall'), (-514, 'school'), (-54, 'gym')]
        b = []
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

        self.highlight_node = False
        self.radius = 2
        rospy.Service('/highlight_node', HighlightNode,
                    self.handle_highlight_node)
        rospy.Service('/draw_node', DrawNode, self.handle_draw_node)


    def loop_iteration(self, world_state):
        """
        Draw the received world state.

        The main loop iteration of the class.  It will try to run a fixed rate,
        as defined by self.refresh_rate.  If it does not manage to keep this
        time it will issue terminal warnings to the user.
        The loop consists in:
            1) Receive the latest vehicle states information
               through UDP
            2) Draw said states

        Returns:
        A boolean indicating if the user closed the
        visualization window (True) or not (False)
        """
        # Receive the latest vehicle states information

        self.vehicles_dict = world_state['vehicles']
        self.pedestrians_dict = world_state['pedestrians']

        self.bus_stops = list(world_state['bus_stop_ids'])

        old_demands = self.bus_stop_demands

        self.bus_stop_demands = list(world_state['bus_stop_demands'])

        self.sensors_dict = world_state['sensors']

        if len(old_demands) < len(self.bus_stop_demands):
            self.demand_change = [False for i in range(len(self.bus_stop_demands))]

        for i in range(len(old_demands)):
            if old_demands[i] == self.bus_stop_demands[i]:
                self.demand_change[i] = True
        for j in range(len(old_demands), len(self.bus_stop_demands)):
            self.demand_change[i] = True

            #Add bus stop to clocker
            self.clock.add_bus_stop(self.bus_stops[j])
        self.time = world_state['time']
        # Draw the the latest vehicle states
        self.traffic_nodes =  world_state['traffic_light_nodes']
        self.traffic_status =  world_state['traffic_light_status']

        self.display_image()


        for event in pygame.event.get():
            key=pygame.key.get_pressed()
            # Checks if the user tried to close the Window
            if event.type == pygame.QUIT or key[pygame.K_ESCAPE]:

                bashCommand = "ps -ef | grep python | grep -v grep | awk '{print $2}' | xargs -r kill -9"
                import subprocess
                process = subprocess.Popen(bashCommand, shell=True)

                pygame.quit()
                return True

            # check if a destination for the semi controlled vehicle
            # should be chosen
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
            # change speed
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    self.change_speed = True
                    return
            """

            """
            # increase/decrease speed
            try:
                speed_service = self.semi_contr_vehicle + '/set_speed'
                rospy.wait_for_service(speed_service)
                if event.key == pygame.K_UP:
                    try:
                        set_speed = rospy.ServiceProxy(speed_service, SetSpeed)
                        set_speed(1.8)
                    except:
                        raise NameError("Service call failed: %s" % e)
                if event.key == pygame.K_DOWN:
                    try:
                        set_speed = rospy.ServiceProxy(speed_service, SetSpeed)
                        set_speed(-1.8)
                    except:
                        raise NameError("Service call failed: %s" % e)
            except AttributeError:
                rospy.logwarn("porco dio")
            """



    def load_image_meta_data(self, map_filename):
        """
        Load the meta data corresponding to map_filename.

        Given the map_filename, it will find the corresponding image metadata
        of said map file.  The file is parsed, and the image metadata
        (image_width, image_height, pixel_per_meter) is gathered and stored
        in the class.
        """
        # Opening the file, reading it into a string
        # and closing it.
        meta_data_filename = map_filename + '.meta'

        f = open(meta_data_filename, 'r')

        meta_data_string = f.read()

        f.close()

        # Process the string containing the metadata
        tokens = meta_data_string.split(';')

        for token in tokens:

            property_tokens = token.split('=')

            if len(property_tokens) != 2:
                print "Error parsing meta data file!"
                continue

            attribute_token = property_tokens[0]
            value_token = property_tokens[1]

            if attribute_token == 'image_width':
                self.loaded_image_width = float(value_token)

            elif attribute_token == 'image_height':
                self.loaded_image_height = float(value_token)

            elif attribute_token == 'pixel_per_meter':
                self.loaded_image_pixel_per_meter = float(value_token)

            else:
                print "Error parsing meta data file!"
                continue

        self.original_image_width = self.loaded_image_width
        self.original_image_height = self.loaded_image_height
        self.original_image_pixel_per_meter = self.loaded_image_pixel_per_meter

        self.image_width = self.loaded_image_width
        self.image_height = self.loaded_image_height

        # By default, the center x of an image,
        # this is, the point where x and y are 0 in
        # real world meters corresponds to the center
        # of the image
        self.image_center_x = self.loaded_image_width / 2.
        self.image_center_y = self.loaded_image_height / 2.

        self.image_pixel_per_meter = self.loaded_image_pixel_per_meter

        return

    def load_image(self, map_filename):
        """
        Load the image corresponding to map_filename.

        Given the map_filename, it will find the corresponding image of said
        map file.  Depending on the desired window width/height and
        pixel_per_meter it will resize and crop the image. If in
        ground_projection mode, it will also apply the necessary
        transformations so that the image can be correctly projected on the
        ground.
        """
        pygame.init()

        image_filename = map_filename + '.bmp'
        self.bg_surface = pygame.image.load(image_filename)

        # Just a sanity check
        (loaded_image_width, loaded_image_height) = self.bg_surface.get_size()
        if (loaded_image_width != self.loaded_image_width or
            loaded_image_height != self.loaded_image_height):

            raise NameError("Meta data does not comply with the loaded image!")

        if self.ground_projection:
            # Need to crop the original image to the Projector dimensions

            if self.desired_pixel_per_meter != -1:
                print ("WARNING: Ground Projection will ignore the provided" +
                       "pixel_per_meter parameter")

            # NOTE: This bugged my mind a lot, the 2.92 and 2.97
            # should be switched
            top_left_x = ((self.loaded_image_width / 2.) -
                          self.projector_area[0] * 32. *
                          self.loaded_image_pixel_per_meter)
            top_left_y = ((self.loaded_image_height / 2.) -
                          self.projector_area[2] * 32. *
                          self.loaded_image_pixel_per_meter)
            bot_right_x = ((self.loaded_image_width / 2.) +
                           self.projector_area[1] * 32. *
                           self.loaded_image_pixel_per_meter)
            bot_right_y = ((self.loaded_image_height / 2.) +
                           self.projector_area[3] * 32. *
                           self.loaded_image_pixel_per_meter)

            top_left_x = int(round(top_left_x))
            top_left_y = int(round(top_left_y))
            bot_right_x = int(round(bot_right_x))
            bot_right_y = int(round(bot_right_y))
            background_array = pygame.PixelArray(self.bg_surface)
            cropped_image_array = background_array[top_left_x:bot_right_x,
                                                   top_left_y:bot_right_y]

            self.bg_surface = cropped_image_array.make_surface()
            self.bg_surface = pygame.transform.smoothscale(
                                self.bg_surface,
                                (int(self.desired_window_width),
                                 int(self.desired_window_height)))

            self.image_pixel_per_meter = (self.desired_window_width /
                                          ((self.projector_area[0] +
                                            self.projector_area[1]) * 32.))
            self.image_pixel_per_meter = (self.desired_window_height /
                                          ((self.projector_area[3] +
                                            self.projector_area[2]) * 32.))

            self.image_center_x = (self.desired_window_width *
                                   self.projector_area[0] /
                                   (self.projector_area[0] +
                                    self.projector_area[1]))
            self.image_center_y = (self.desired_window_height *
                                   self.projector_area[2] /
                                   (self.projector_area[2] +
                                    self.projector_area[3]))

        elif self.desired_pixel_per_meter == -1:
            # CURENTLY BUGGED
            # Only need to rescale the image in order to comply with the
            # desired dimensions
            self.bg_surface = pygame.transform.smoothscale(
                                self.bg_surface,
                                (int(self.desired_window_width),
                                 int(self.desired_window_height)))
            self.bg_surface_array = pygame.surfarray.array3d(self.bg_surface)

            x_scale_ratio = (float(self.desired_window_width) /
                             float(self.loaded_image_width))
            y_scale_ratio = (float(self.desired_window_height) /
                             float(self.loaded_image_height))

            self.image_pixel_per_meter = (self.loaded_image_pixel_per_meter *
                                          x_scale_ratio)
            # self.image_pixel_per_meter = (self.loaded_image_pixel_per_meter *
            #                               y_scale_ratio)

            self.image_center_x = self.loaded_image_width / 2. * x_scale_ratio
            self.image_center_y = self.loaded_image_height / 2. * y_scale_ratio

        else:
            # Need to rescale the image in order to comply with the desired
            # dimensionsand pixel_per_meter

            # CURENTLY BUGGED

            half_width_meters = (self.desired_window_width / 2. /
                                 self.desired_pixel_per_meter)
            half_height_meters = (self.desired_window_height / 2. /
                                  self.desired_pixel_per_meter)

            top_left_x = ((self.loaded_image_width / 2.) -
                          half_width_meters *
                          self.loaded_image_pixel_per_meter)
            top_left_y = ((self.loaded_image_height / 2.) -
                          half_height_meters *
                          self.loaded_image_pixel_per_meter)
            bot_right_x = ((self.loaded_image_width / 2.) +
                           half_width_meters *
                           self.loaded_image_pixel_per_meter)
            bot_right_y = ((self.loaded_image_height / 2.) +
                           half_height_meters *
                           self.loaded_image_pixel_per_meter)

            top_left_x = int(round(top_left_x))
            top_left_y = int(round(top_left_y))
            bot_right_x = int(round(bot_right_x))
            bot_right_y = int(round(bot_right_y))

            background_array = pygame.PixelArray(self.bg_surface)
            cropped_image_array = background_array[top_left_x:bot_right_x,
                                                   top_left_y:bot_right_y]

            self.bg_surface = cropped_image_array.make_surface()

            self.bg_surface = pygame.transform.smoothscale(
                                self.bg_surface,
                                (int(self.desired_window_width),
                                 int(self.desired_window_height)))
            self.bg_surface_array = pygame.surfarray.array3d(self.bg_surface)

            self.image_pixel_per_meter = self.desired_pixel_per_meter
            self.image_pixel_per_meter = self.desired_pixel_per_meter

            self.image_center_x = (self.desired_window_width / 2.)
            self.image_center_y = (self.desired_window_height / 2.)

        # self.bg_surface_pixel_array = pygame.PixelArray(self.bg_surface)

        self.areas_to_blit.append([0, 0, int(self.desired_window_width),
                                   int(self.desired_window_height)])

        image_size = (int(self.desired_window_width),
                      int(self.desired_window_height))

        if self.ground_projection:

            # This sets up the window position on the top left corner of the
            # screen
            x = 0
            y = 0
            os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % (x, y)

            # pygame.NOFRAME makes the visualization window not have a Frame
            self.window = pygame.display.set_mode(image_size, pygame.NOFRAME)

        else:
            self.window = pygame.display.set_mode(image_size)

        return

    def restart_image(self):
        self.window.quit()
        self.window.init()
        self.areas_to_blit = [[0, 0, int(self.desired_window_width),
                                   int(self.desired_window_height)]]

    def get_bar_images(self, demand):
        background_image = pygame.image.load(self.base_path
                                            + '/resources/white_background.png')
        bg_new_size = (BACKGROUND_BAR_WIDTH, BAR_HEIGHT)

        if demand < 20:
            foreground_image = pygame.image.load(self.base_path + '/resources/passenger1.png')
            fg_new_size = (int(BACKGROUND_BAR_WIDTH / 5.), BAR_HEIGHT)
        elif demand >= 20 and demand < 40:
            foreground_image = pygame.image.load(self.base_path + '/resources/passenger2.png')
            fg_new_size = (int(BACKGROUND_BAR_WIDTH / 5. * 2), BAR_HEIGHT)
        elif demand >= 40 and demand < 60:
            foreground_image = pygame.image.load(self.base_path + '/resources/passenger3.png')
            fg_new_size = (int(BACKGROUND_BAR_WIDTH / 5. * 3), BAR_HEIGHT)
        elif demand >= 60 and demand < 80:
            foreground_image = pygame.image.load(self.base_path + '/resources/passenger4.png')
            fg_new_size = (int(BACKGROUND_BAR_WIDTH / 5. * 4), BAR_HEIGHT)
        else:
            foreground_image = pygame.image.load(self.base_path + '/resources/passenger5.png')
            fg_new_size = (int(BACKGROUND_BAR_WIDTH), BAR_HEIGHT)


        # fg_new_size = (int(BACKGROUND_BAR_WIDTH * demand / 100.), BAR_HEIGHT)

        return pygame.transform.scale(background_image, bg_new_size), pygame.transform.scale(foreground_image,fg_new_size)

    def kista_move_bus_stops_off_road(self, bus_id, tup):
        x,y = tup
        return {
            -426: (x - 13, y - 5),
            -362: (x, y - 8),
            -302: (x - 10, y - 11),
            -444: (x + 7, y - 3),
            -274: (x, y - 7),
            -672: (x + 5, y - 6),
            -476: (x - 6, y)
        }.get(bus_id, (x,y))

    def load_bus_stops(self):
        """
        Loads bus stops onto the current map
        """
        for i in range(len(self.bus_stop_demands)):
            if not self.demand_change[i]:
                continue
            self.demand_change[i] = False
            stop_id = self.bus_stops[i]
            demand = self.bus_stop_demands[i]
            coords = self.get_node_coordinates(stop_id)
            # rospy.logerr('id: ' + str(stop_id) + 'coords: ' + str(coords))
            stopIdCoors = {}
            """stopIdCoors[-214] = (-15, 52)
            stopIdCoors[-222] = (10, 28)
            stopIdCoors[-398] = (105, -20)
            stopIdCoors[-400] = (87, -55)
            stopIdCoors[-158] = (-100, 96)
            stopIdCoors[-392] = (-132, 116)"""

            stopIdCoors[-210] = (4, 66)
            stopIdCoors[-218] = (38, 38)
            stopIdCoors[-394] = (130, -10)
            stopIdCoors[-396] = (105, -45)
            stopIdCoors[-154] = (-75, 102)
            stopIdCoors[-388] = (-115, 125)

            if stop_id in stopIdCoors.keys():
                coords = stopIdCoors[stop_id]

            coords = self.kista_move_bus_stops_off_road(stop_id, coords)
            self.draw_bus_stop_image(coords, self.bus_stop_img)

            fg_demand_bar, bg_demand_bar = self.get_bar_images(demand)
            bar_x = coords[0]
            bar_y = coords[1] + 3
            [pixel_x, pixel_y] = self.convert_position_to_image_pixel(bar_x, bar_y)

            pos = (int(round(pixel_x)), int(round(pixel_y)))


            self.window.blit(fg_demand_bar, pos)
            self.window.blit(bg_demand_bar, pos)

            text_x = coords[0] + 2
            text_y = coords[1] - 10.5
            [pixel_x, pixel_y] = self.convert_position_to_image_pixel(text_x, text_y)
            pos = (int(round(pixel_x)), int(round(pixel_y)))


            font = pygame.font.Font(None, 24)
            text = font.render(str(-stop_id), 1, (255,255,255))
            self.window.blit(text, pos)

    def get_node_coordinates(self, node_id):
        '''
        Talks with road network to get xy coordinates of nodes
        Returns tuple with x and y coordinates in form (x,y)
        Used for loading bus stops
        '''
        rospy.wait_for_service('/get_node_coordinates')
        try:
            get_coordinates = rospy.ServiceProxy('get_node_coordinates', GetCoordinates)
            coords = get_coordinates(node_id)
        except rospy.ServiceException, e:
            raise "Service call failed: %s" % e
        return (coords.x, coords.y)

    def load_ped_image(self):
        """
        @author: U{Nicolo' Campolongo<nicoloc@kth.se>}
        Load the pedestrian image.
        """
        ped_image = pygame.image.load(self.base_path + '/resources/pedestrian.jpg').convert_alpha()
        new_size = (15, 15)
        ped_image = pygame.transform.smoothscale(ped_image, new_size)
        self.ped_image = ped_image

    def load_crosswalk_image(self):
        """
        @author: U{Nicolo' Campolongo<nicoloc@kth.se>}
        Load the crosswalk image.
        """
        crosswalk_image = pygame.image.load(self.base_path + '/resources/rectangle.jpg').convert_alpha()
        new_size = (16, 6)
        crosswalk_image = pygame.transform.smoothscale(crosswalk_image, new_size)
        self.crosswalk_image = crosswalk_image

    def load_big_red_x_image(self):
        width = 10
        height = 10

        self.big_red_x = self.get_car_image(
                        self.base_path + '/resources/big_red_x.png',
                        width, height)

    def load_bus_image(self):
        bus_width_meters = 2.55
        bus_length_meters = 12

        self.bus_image = self.get_car_image(
                        self.base_path + '/resources/busOffset.png',
                        bus_width_meters, bus_length_meters)

    def load_block_image(self):
        """Load the block image."""
        block_width_meters = 2.096
        block_length_meters = (4.779 - 0.910) * 2.

        self.block_image = self.get_car_image(
                        self.base_path + '/resources/block.png',
                        block_width_meters, block_length_meters)
        return

    def load_smart_car_image(self):
        """Load the car image, used for displaying the current vehicles."""
        car_width_meters = 2.096
        car_length_meters = (4.779 - 0.910) * 2.

        self.smart_car_image = self.get_car_image(
                        self.base_path + '/resources/carSmartOffset.png',
                        car_width_meters, car_length_meters)

        return

    def load_red_car_image(self):
        """Load the car image, used for displaying the current vehicles."""
        car_width_meters = 2.096
        car_length_meters = (4.779 - 0.910) * 2.

        self.red_car_image = self.get_car_image(
                    self.base_path + '/resources/carRedOffset.png',
                    car_width_meters, car_length_meters)

        return

    def load_green_car_image(self):
        """Load the car image, used for displaying the current vehicles."""
        car_width_meters = 2.096
        car_length_meters = (4.779 - 0.910) * 2.

        self.green_car_image = self.get_car_image(
                            self.base_path + '/resources/carGreenOffset.png',
                            car_width_meters, car_length_meters)

        return

    def load_blue_car_image(self):
        """Load the car image, used for displaying the current vehicles."""
        car_width_meters = 2.096
        car_length_meters = (4.779 - 0.910) * 2.

        self.blue_car_image = self.get_car_image(
                        self.base_path + '/resources/carBlueOffset.png',
                        car_width_meters, car_length_meters)

        return

    def load_white_car_image(self):
        """Load the car image, used for displaying the current vehicles."""
        car_width_meters = 2.096
        car_length_meters = (4.779 - 0.910) * 2.

        self.white_car_image = self.get_car_image(
                        self.base_path + '/resources/carWhiteOffset.png',
                        car_width_meters, car_length_meters)

        return

    def load_yellow_car_image(self):
        """Load the car image, used for displaying the current vehicles."""
        car_width_meters = 2.096
        car_length_meters = (4.779 - 0.910) * 2.

        self.yellow_car_image = self.get_car_image(
                        self.base_path + '/resources/carYellowOffset.png',
                        car_width_meters, car_length_meters)

        return

    def get_car_image(self, car_image_filename, car_width_meters,
                      car_length_meters):
        """Load the car image stored in the file car_image_filename."""
        car_image = pygame.image.load(car_image_filename)

        (car_image_width, car_image_height) = car_image.get_size()

        # pixel_per_meter_image = car_image_height/car_width_meters
        [x_pixel_1, _] = self.convert_position_to_image_pixel(0, 0)
        [x_pixel_2, _] = self.convert_position_to_image_pixel(car_width_meters,
                                                              0)

        desired_car_width_pixels = float(x_pixel_2 - x_pixel_1)

        scale_down_ratio = desired_car_width_pixels / car_image_height

        new_size = (int(round(scale_down_ratio * car_image_width)),
                    int(round(scale_down_ratio * car_image_height)))

        car_image = pygame.transform.smoothscale(car_image, new_size)

        return car_image

    def load_bus_stop_image(self):
        """Predefined image location and size"""

        # busStop has size 195 x 297
        stop_image_location = self.base_path + '/resources/bus_stop_5.jpg'
        stop_image = pygame.image.load(stop_image_location)
        stop_width_meters = 16
        stop_height_meters = 16 * (195 / 297)
        (stop_image_width, stop_image_height) = stop_image.get_size()

        stop_image = pygame.image.load(stop_image_location)

        [x_pixel_1, _] = self.convert_position_to_image_pixel(0, 0)
        [x_pixel_2, _] = self.convert_position_to_image_pixel(stop_width_meters,
                                                              0)

        desired_stop_width_pixels = float(x_pixel_2 - x_pixel_1)
        scale_down_ratio = desired_stop_width_pixels / stop_image_height

        new_size = (int(round(scale_down_ratio * stop_image_width )),
                    int(round(scale_down_ratio * stop_image_height )))
        return pygame.transform.smoothscale(stop_image, new_size)



    def load_truck_image(self):
        """Load the truck image, used for displaying the current vehicles."""
        minitruck_width_meters = 0.08
        # minitruck_length_meters = 0.19
        minitruck_length_meters = 0.145 * 2
        # This is the SML world meters
        # of the minitruck image width

        truck_width_meters = 32. * minitruck_width_meters
        truck_length_meters = 32. * minitruck_length_meters

        self.truck_image = self.get_car_image(
                    self.base_path + '/resources/truckTopOffset.png',
                    truck_width_meters, truck_length_meters)

        return

    def load_bus_image(self):
        """Load the bus image, used for displaying the current vehicles."""
        minibus_width_meters = 0.08
        # minibus_length_meters = 0.19
        minibus_length_meters = 0.145 * 2
        # This is the SML world meters
        # of the minibus image width
        bus_width_meters = 32. * minibus_width_meters
        bus_length_meters = 32. * minibus_length_meters
        self.bus_image = self.get_bus_image(
                        self.base_path + '/resources/busTopOffset.png',
                        bus_width_meters, bus_length_meters)
        return

    def load_red_bus_image(self):
        """Load the bus image, used for displaying the current vehicles."""
        minibus_width_meters = 0.08
        # minibus_length_meters = 0.19
        minibus_length_meters = 0.145 * 2
        # This is the SML world meters
        # of the minibus image width
        bus_width_meters = 32. * minibus_width_meters
        bus_length_meters = 32. * minibus_length_meters
        self.red_bus_image = self.get_bus_image(
                        self.base_path + '/resources/redBusOffset.png',
                        bus_width_meters, bus_length_meters)
        return

    def load_green_bus_image(self):
        """Load the bus image, used for displaying the current vehicles."""
        minibus_width_meters = 0.08
        # minibus_length_meters = 0.19
        minibus_length_meters = 0.145 * 2
        # This is the SML world meters
        # of the minibus image width
        bus_width_meters = 32. * minibus_width_meters
        bus_length_meters = 32. * minibus_length_meters
        self.green_bus_image = self.get_bus_image(
                        self.base_path + '/resources/greenBusOffset.png',
                        bus_width_meters, bus_length_meters)
        return

    def load_yellow_bus_image(self):
        """Load the bus image, used for displaying the current vehicles."""
        minibus_width_meters = 0.08
        # minibus_length_meters = 0.19
        minibus_length_meters = 0.145 * 2
        # This is the SML world meters
        # of the minibus image width
        bus_width_meters = 32. * minibus_width_meters
        bus_length_meters = 32. * minibus_length_meters
        self.yellow_bus_image = self.get_bus_image(
                        self.base_path + '/resources/yellowBusOffset.png',
                        bus_width_meters, bus_length_meters)
        return

    def get_bus_image(self, bus_image_filename, bus_width_meters,
                      bus_length_meters):
        """Load the bus image, used for displaying the current vehicles."""
        bus_image = pygame.image.load(bus_image_filename)
        (bus_image_width, bus_image_height) = bus_image.get_size()

        # pixel_per_meter_image = car_image_height/car_width_meters
        [x_pixel_1, _] = self.convert_position_to_image_pixel(0, 0)
        [x_pixel_2, _] = self.convert_position_to_image_pixel(bus_width_meters,
                                                              0)

        desired_bus_width_pixels = float(x_pixel_2 - x_pixel_1)

        scale_down_ratio = desired_bus_width_pixels / bus_image_height

        new_size = (int(round(scale_down_ratio * bus_image_width)),
                    int(round(scale_down_ratio * bus_image_height)))

        bus_image = pygame.transform.smoothscale(bus_image, new_size)

        return bus_image

    def load_big_box_image(self):
        """Load the big_box image, used for displaying the current vehicles."""
        box_length_meters = 32. * 0.6

        box_image = pygame.image.load(
                        self.base_path + '/resources/waterOffset.png')

        (box_image_width, box_image_height) = box_image.get_size()

        [x_pixel_1, _] = self.convert_position_to_image_pixel(0, 0)
        [x_pixel_2, _] = self.convert_position_to_image_pixel(
                                box_length_meters,
                                0)

        desired_box_width_pixels = float(x_pixel_2 - x_pixel_1)

        scale_down_ratio = desired_box_width_pixels / (box_image_width / 2.)

        new_size = (int(round(scale_down_ratio * box_image_width)),
                    int(round(scale_down_ratio * box_image_height)))

        box = pygame.transform.smoothscale(box_image, new_size)

        self.box_image = box

        return

    def load_small_box_image(self):
        """Load small_box image, used for displaying the current vehicles."""
        box_length_meters = 32. * 0.4

        box_image = pygame.image.load(
                        self.base_path + '/resources/waterOffset.png')

        (box_image_width, box_image_height) = box_image.get_size()

        [x_pixel_1, _] = self.convert_position_to_image_pixel(0, 0)
        [x_pixel_2, _] = self.convert_position_to_image_pixel(
                                box_length_meters,
                                0)

        desired_box_width_pixels = float(x_pixel_2 - x_pixel_1)

        scale_down_ratio = desired_box_width_pixels / (box_image_width / 2.)

        new_size = (int(round(scale_down_ratio * box_image_width)),
                    int(round(scale_down_ratio * box_image_height)))

        box = pygame.transform.smoothscale(box_image, new_size)

        self.small_box_image = box

        return

    def load_goal_image(self):
        """Load the goal image, used for displaying the current vehicles."""
        box_length_meters = 32. * 0.2

        flag_image = pygame.image.load(
                        self.base_path + '/resources/finishFlag.png')

        (flag_image_width, flag_image_height) = flag_image.get_size()

        [x_pixel_1, _] = self.convert_position_to_image_pixel(0, 0)
        [x_pixel_2, _] = self.convert_position_to_image_pixel(
                                box_length_meters,
                                0)

        desired_box_width_pixels = float(x_pixel_2 - x_pixel_1)

        scale_down_ratio = desired_box_width_pixels / (flag_image_width / 2.)

        new_size = (int(round(scale_down_ratio * flag_image_width)),
                    int(round(scale_down_ratio * flag_image_height)))

        flag = pygame.transform.smoothscale(flag_image, new_size)

        self.goal_image = flag

        return

    def setup_id_font(self):
        """Define the font properties used for writing the vehicle ids."""
        font_size = 10

        self.ids_font = pygame.font.SysFont('monospace', font_size)

        self.ids_font.set_bold(True)

        self.font_color = (255, 255, 0)

        return

    def dumb_background_blit(self):
        """
        Blit the whole background image into the visualization window.

        Blits the background picture into the visualization window.  It is
        named dumb since it blits ALL of the pixels of the background into the
        window, even if this results in pixels not changing their value.
        """
        # self.window.blit(self.bg_surface, (0,0))
        # Average time: 0.033915

        pygame.surfarray.blit_array(self.window, self.bg_surface_array)
        # Average time: 0.022714

    def smart_background_blit(self):
        """
        Blit the background image only into areas of interest.

        Blits the background picture into the visualization window.  It is
        named smart since it blits only interest regions as defined by
        self.areas_to_blit.  These areas are areas that we consider that need
        to be blitted, because they were previously drawn with things that are
        not the background (e.g.: a vehicle)
        """
        for area_to_blit in self.areas_to_blit:
            self.window.blit(self.bg_surface, area_to_blit[:2], area_to_blit)
        # Empty the areas to blit
        self.areas_to_blit = []

        return

    def draw_events(self):
#        event = self.events_dict[event_id]
        event = 1
        if event == 1:
            self.draw_block(0, 0)
        elif event == 2:
            self.draw_block(10, 10)
        return

    def trajectory(self, prevpos):
        poslist = prevpos
        if len(self.vehicles_dict) > 0:
            value = dict((key, value) for key, value
                in self.vehicles_dict.iteritems())
            vehicle_info = value.values()
            vehicle_stats = vehicle_info[0].values()
            x = vehicle_stats[1]
            y = vehicle_stats[4]
            [pixel_x, pixel_y] = self.convert_position_to_image_pixel(x, y)
            pos = [pixel_x, pixel_y]
            poslist.append(pos)
        return poslist

    def handle_highlight_node(self, req):
        """
        @author: U{Nicolo' Campolongo<nicoloc@kth.se>}
        Handle the highlight node request from the SemiControlledVehicle.

        @param req: I{(HighlightNode)} Contains the node id of the current
                    destination.
        """
        try:
            # turn on node highlighting
            self.highlight_node = True
            self.node_index = 0
            # save vehicle id
            self.semi_contr_vehicle = req.namespace
            # delete the current destination so that
            # it's not shown in the possible next destinations
            self.removed = ((req.node_id, self.destinations[req.node_id]))
            del self.destinations[req.node_id]
            msg = ("Node %i succesfully highlighted!" % self.destinations[req.node_id])
            return HighlightNodeResponse(True, msg)
        except KeyError:
            msg = ("Node %i not included in possible destinations!" % req.node_id)
            return HighlightNodeResponse(False, msg)

    def handle_draw_node(self, req):
        """
        """
        self.highlight_node = req.highlight_node
        self.h_node = self.destinations.keys()[req.node_index]
        msg = "porco dio"
        return DrawNodeResponse(True, msg)

    def draw_node(self):
        """
        @author: U{Nicolo' Campolongo<nicoloc@kth.se>}

        todo
        """
        #self.h_node = self.destinations.keys()[self.node_index]
        surf = pygame.Surface((self.desired_window_width,self.desired_window_height))
        surf.set_colorkey((0,0,0))
        YELLOW = (255, 255, 0)
        pygame.draw.circle(surf, YELLOW, (self.destinations[self.h_node][0],
                            self.destinations[self.h_node][1]), self.radius, 2)
        self.radius += 4
        if self.radius >= 30:
            self.radius = 2
        surf = surf.convert()
        self.window.blit(surf, (0,0))
        self.add_surface_to_areas_to_blit(surf, (0,0))
        # draw destination name
        self.draw_destination(self.destinations[self.h_node][2])


    def gen_crosswalks_dict(self):
        """
        @author: U{Nicolo' Campolongo<nicoloc@kth.se>}

        Function used to generate the crosswalks dictionary.
        The key is given by crosswalk id, the value by the
        coordinates of the 2 nodes in the crosswalk.
        """

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
                node_coord_service = '/get_node_coordinates'
                rospy.wait_for_service('/get_node_coordinates')
                try:
                    nodeCoord = rospy.ServiceProxy(node_coord_service, GetCoordinates)
                    p1 = nodeCoord(nodes.node_1)
                    p2 = nodeCoord(nodes.node_2)
                    self.crosswalks[i].append(p1)
                    self.crosswalks[i].append(p2)
                except rospy.ServiceException, e:
                    raise NameError("Service call failed: %s" % e)
            except rospy.ServiceException, e:
                raise NameError("Service call failed: %s" % e)

    def draw_pedestrians(self):
        """
        @author: U{Nicolo' Campolongo<nicoloc@kth.se>}

        Iterate over the pedestrian in self.pedestrians_dict and draw them
        """
        for pedestrian_id in self.pedestrians_dict:
            pedestrian = self.pedestrians_dict[pedestrian_id]
            x = pedestrian['x']
            y = pedestrian['y']
            yaw = pedestrian['yaw']
            [pixel_x, pixel_y] = self.convert_position_to_image_pixel(x, y)
            self.draw_pedestrian_image(pedestrian['x'], pedestrian['y'], yaw)

        return

    def draw_crosswalks(self):
        """
        @author: U{Nicolo' Campolongo<nicoloc@kth.se>}

        Iterate over the crosswalks dict and draw them.
        First, the midpoint between the two crosswalk nodes is computed; then,
        two crosswalk images are added within a fixed distance from the midpoint.
        """
        for Id, p in self.crosswalks.iteritems():
            m, b = compute_line_params(p[0].x, p[0].y, p[1].x, p[1].y)
            alpha = np.arctan(m) # angle of the line passing through the 2 nodes
            d = 1.5
            x = (p[1].x + p[0].x) / 2
            y = (p[1].y + p[0].y) / 2
            x1 = x+d
            y1 = m*x1+b
            x2 = x-d
            y2 = m*x2+b
            yaw = math.pi /2 + alpha
            self.draw_crosswalk_image(x, y, yaw)
            self.draw_crosswalk_image(x1, y1, yaw)
            self.draw_crosswalk_image(x2, y2, yaw)
            # finally, draw the crosswalk_id
            if self.crosswalk_id:
                self.draw_id(Id, x, y)
        return

    def handle_draw_crosswalk_ids(self, req):
        """
        @author: U{Nicolo' Campolongo<nicoloc@kth.se>}

        Handle the draw crosswalk ids request.

        @param req: empty
        """
        self.crosswalk_id = not self.crosswalk_id
        msg = "Crosswalk ids visualization successfully set to %s" % self.crosswalk_id
        return DrawCrosswalkIdResponse(True, msg)


    def draw_vehicles(self):
        """Iterate over the vehicles in self.vehicles_dict and draw them."""

        red = (255, 0, 0)

        for vehicle_id in self.vehicles_dict:

            vehicle = self.vehicles_dict[vehicle_id]

            vehicle_class_name = vehicle['class_name']

            """print(vehicle_class_name)
            print(vehicle_id)
            print(vehicle)
            print('hello')"""

            prevx = vehicle['x']
            prevy = vehicle['y']
            [pix_prevx, pix_prevy] = self.convert_position_to_image_pixel(prevx-5, prevy-10)
            prev = [(pix_prevx, pix_prevy)]
            poslist = self.trajectory(prev)

            if vehicle_class_name == bodyclasses.QualisysGoal.__name__:
                self.draw_goal(vehicle['x'], vehicle['y'])

            elif vehicle_class_name == bodyclasses.QualisysBigBox.__name__:
                self.draw_box(vehicle['x'], vehicle['y'], vehicle['yaw'])

            elif vehicle_class_name == bodyclasses.QualisysSmallBox.__name__:
                self.draw_small_box(vehicle['x'], vehicle['y'], vehicle['yaw'])

            elif vehicle_class_name == BusVehicle.__name__:
                self.draw_bus(vehicle['x'], vehicle['y'], vehicle['yaw'])


            elif (vehicle_class_name == TruckVehicle.__name__):
                self.draw_truck(vehicle['x'], vehicle['y'], vehicle['yaw'])

            elif (vehicle_class_name == DummyVehicle.__name__ or
                  vehicle_class_name == BaseVehicle.__name__ or
                  vehicle_class_name == WifiVehicle.__name__ or
                  vehicle_class_name == RandomDestinationVehicle.__name__ or
                  vehicle_class_name == ConnectivityVehicle.__name__ or
                  vehicle_class_name == SemiControlledVehicle.__name__):
                if vehicle_id > -100:
                    color = vehicle_id % 5

                    if color == 0:
                        self.draw_white_car(vehicle['x'], vehicle['y'],
                                            vehicle['yaw'])

                    elif color == 1:
                        self.draw_green_car(vehicle['x'], vehicle['y'],
                                            vehicle['yaw'])

                    elif color == 2:
                        self.draw_blue_car(vehicle['x'], vehicle['y'],
                                           vehicle['yaw'])

                    elif color == 3:
                        self.draw_yellow_car(vehicle['x'], vehicle['y'],
                                             vehicle['yaw'])

                    elif color == 4:
                        self.draw_red_car(vehicle['x'], vehicle['y'],
                                          vehicle['yaw'])
            elif (vehicle_class_name == BusVehicle.__name__):
                if vehicle_id > -100:
                    color = vehicle_id % 4

                    if color == 0:
                        self.draw_bus(vehicle['x'], vehicle['y'],
                                      vehicle['yaw'])

                    elif color == 1:
                        self.draw_red_bus(vehicle['x'], vehicle['y'],
                                          vehicle['yaw'])
                        pygame.draw.lines(self.window, red, True, poslist, 3)

                    elif color == 2:
                        self.draw_green_bus(vehicle['x'], vehicle['y'],
                                            vehicle['yaw'])

                    elif color == 3:
                        self.draw_yellow_bus(vehicle['x'], vehicle['y'],
                                             vehicle['yaw'])

            # elif vehicle_class_name == smartvehicle.SmartVehicle.__name__:

            # self.draw_smart_car(vehicle['x'], vehicle['y'], vehicle['yaw'])

            else:

                print "vehicle_class_name = " + str(vehicle_class_name)
                raise NameError("Unexpected")
                # self.draw_truck(vehicle['x'], vehicle['y'], vehicle['yaw'])

            if self.show_ids:
                self.draw_id(vehicle_id, vehicle['x'], vehicle['y'])

        for vehicle_id in self.vehicles_dict:

            vehicle = self.vehicles_dict[vehicle_id]
        return

    def draw_car(self, car_x, car_y, car_yaw):
        """
        Draw a car, given its state: x, y and yaw.

        It does so by calling draw_vehicle_image with the car image as an
        argument.
        """
        self.draw_vehicle_image(car_x, car_y, car_yaw, self.car_image)
        return

    def draw_bus(self, car_x, car_y, car_yaw):
        """
        Draw a bus, given its state: x, y and yaw.

        It does so by calling draw_vehicle_image with the bus image as an
        argument.
        """
        self.draw_vehicle_image(car_x, car_y, car_yaw, self.bus_image)
        return

    def draw_red_car(self, car_x, car_y, car_yaw):
        """
        Draw a car, given its state: x, y and yaw.

        It does so by calling draw_vehicle_image with the car image as an
        argument.
        """
        self.draw_vehicle_image(car_x, car_y, car_yaw, self.red_car_image)
        return

    def draw_yellow_car(self, car_x, car_y, car_yaw):
        """
        Draw a car, given its state: x, y and yaw.

        It does so by calling draw_vehicle_image with the car image as an
        argument.
        """
        self.draw_vehicle_image(car_x, car_y, car_yaw, self.yellow_car_image)
        return

    def draw_blue_car(self, car_x, car_y, car_yaw):
        """
        Draw a car, given its state: x, y and yaw.

        It does so by calling draw_vehicle_image with the car image as an
        argument.
        """
        self.draw_vehicle_image(car_x, car_y, car_yaw, self.blue_car_image)
        return

    def draw_green_car(self, car_x, car_y, car_yaw):
        """
        Draw a car, given its state: x, y and yaw.

        It does so by calling draw_vehicle_image with the car image as an
        argument.
        """
        self.draw_vehicle_image(car_x, car_y, car_yaw, self.green_car_image)
        return

    def draw_white_car(self, car_x, car_y, car_yaw):
        """
        Draw a car, given its state: x, y and yaw.

        It does so by calling draw_vehicle_image with the car image as an
        argument.
        """
        self.draw_vehicle_image(car_x, car_y, car_yaw, self.white_car_image)
        return

    def draw_smart_car(self, car_x, car_y, car_yaw):
        """
        Draw a smart car, given its state: x, y and yaw.

        It does so by calling draw_vehicle_image with the car image as an
        argument.
        """
        self.draw_vehicle_image(car_x, car_y, car_yaw, self.smart_car_image)
        return

    def draw_truck(self, truck_x, truck_y, truck_yaw):
        """
        Draw a truck, given its state: x, y and yaw.

        It does so by calling draw_vehicle_image with the truck image as an
        argument.
        """
        self.draw_vehicle_image(truck_x, truck_y, truck_yaw, self.truck_image)
        return

    def draw_green_bus(self, truck_x, truck_y, truck_yaw):
        """
        Draw a truck, given its state: x, y and yaw.

        It does so by calling draw_vehicle_image with the truck image as an
        argument.
        """
        self.draw_vehicle_image(truck_x, truck_y, truck_yaw, self.green_bus_image)
        return

    def draw_red_bus(self, truck_x, truck_y, truck_yaw):
        """
        Draw a truck, given its state: x, y and yaw.

        It does so by calling draw_vehicle_image with the truck image as an
        argument.
        """
        self.draw_vehicle_image(truck_x, truck_y, truck_yaw, self.red_bus_image)
        return

    def draw_yellow_bus(self, truck_x, truck_y, truck_yaw):
        """
        Draw a truck, given its state: x, y and yaw.

        It does so by calling draw_vehicle_image with the truck image as an
        argument.
        """
        self.draw_vehicle_image(truck_x, truck_y, truck_yaw, self.yellow_bus_image)
        return

    def draw_bus(self, truck_x, truck_y, truck_yaw):
        """
        Draw a bus, given its state: x, y and yaw.

        It does so by calling draw_vehicle_image with the bus image as an
        argument.
        """
        self.draw_vehicle_image(truck_x, truck_y, truck_yaw, self.bus_image)
        return

    def draw_box(self, truck_x, truck_y, truck_yaw):
        """
        Draw a box, given its state: x, y and yaw.

        It does so by calling draw_vehicle_image with the box image as an
        argument.
        """
        self.draw_vehicle_image(truck_x, truck_y, truck_yaw, self.box_image)
        return

    def draw_small_box(self, truck_x, truck_y, truck_yaw):
        """
        Draw a small box, given its state: x, y and yaw.

        It does so by calling draw_vehicle_image with the small box image as an
        argument.
        """
        self.draw_vehicle_image(truck_x, truck_y, truck_yaw,
                                self.small_box_image)
        return

    def draw_vehicle_image(self, vehicle_x, vehicle_y, vehicle_yaw,
                           vehicle_image):
        """Draw a given vehicle image, given its position and yaw."""
        [pixel_x, pixel_y] = self.convert_position_to_image_pixel(vehicle_x,
                                                                  vehicle_y)
        vehicle_yaw = math.degrees(vehicle_yaw)
        vehicle_rotated = pygame.transform.rotate(vehicle_image, vehicle_yaw)
        vehicle_size_rotated = vehicle_rotated.get_size()

        new_x = pixel_x - vehicle_size_rotated[0] / 2
        new_y = pixel_y - vehicle_size_rotated[1] / 2
        pos = (int(round(new_x)), int(round(new_y)))

        if self.should_be_blit(vehicle_rotated, pos):

            self.window.blit(vehicle_rotated, pos)

            self.add_surface_to_areas_to_blit(vehicle_rotated, pos)

        return

    def draw_pedestrian_image(self, ped_x, ped_y, ped_yaw):
        """
        @author: U{Nicolo' Campolongo<nicoloc@kth.se>}

        Draw the pedestrian image, given its position and yaw
        """
        self.draw_vehicle_image(ped_x, ped_y, ped_yaw, self.ped_image)
        return

    def draw_crosswalk_image(self, crosswalk_x, crosswalk_y, crosswalk_yaw):
        """
        @author: U{Nicolo' Campolongo<nicoloc@kth.se>}

        Draw the crosswalk image, given its position and yaw
        """
        self.draw_vehicle_image(crosswalk_x, crosswalk_y, crosswalk_yaw, self.crosswalk_image)
        return

    def draw_bus_stop_image(self, stop_coords, bus_stop_image):
        [pixel_x, pixel_y] = self.convert_position_to_image_pixel(stop_coords[0], stop_coords[1])
        pos = (int(round(pixel_x)), int(round(pixel_y)))

        self.window.blit(bus_stop_image, pos)

        return

    def draw_block(self, block_x, block_y):
        """
        Draw a road block, given its position, x, y.

        It does so by calling draw_vehicle_image with the block image as an
        argument
        """
        block_yaw = 0
        self.draw_vehicle_image(block_x, block_y, block_yaw, self.block_image)
        return

    def draw_goal(self, goal_x, goal_y):
        """
        Draw a goal, given its position, x, y.

        It does so by calling draw_vehicle_image with the goal image as an
        argument
        """
        goal_yaw = 0
        self.draw_vehicle_image(goal_x, goal_y, goal_yaw, self.goal_image)
        return

    def draw_goal_circle(self, vehicle_x, vehicle_y):
        """Draw a goal circle, given its state, x, y."""
        goal_radius_meters = 0.25 * 32.
        # pixel_per_meter_image = car_image_height/car_width_meters
        [x_pixel_1, _] = self.convert_position_to_image_pixel(0, 0)
        [x_pixel_2, _] = self.convert_position_to_image_pixel(
                                        goal_radius_meters,
                                        0)

        [pixel_x, pixel_y] = self.convert_position_to_image_pixel(vehicle_x,
                                                                  vehicle_y)

        color = (255, 0, 255)
        pos = (pixel_x, pixel_y)
        radius = int(round(x_pixel_2 - x_pixel_1))
        width = 0  # Makes the circle filled

        pygame.draw.circle(self.window, color, pos, radius, width)

        blit_radius = int(round(1.2 * radius))

        self.areas_to_blit.append([pos[0] - blit_radius, pos[1] - blit_radius,
                                   pos[0] + blit_radius, pos[1] + blit_radius])
        return

    def should_be_blit(self, surface, surface_pos):
        """
        Decide if surface is worth blitting or not.

        TO BE IMPLEMENTED
        This function should receive a surface and the position
        of said surface, and decide if this image is worth blitting
        or not.
        An image is not worth blitting if it falls completely outside
        of the image area. Otherwise it should be blit
        """
        should_be_blit = True

        # (image_width, image_height) = self.bg_surface_pixel_array.get_size()

        # if surface_pos[0] < 0 and surface_pos[1] < 0

        return should_be_blit

    def draw_id(self, vehicle_id, vehicle_x, vehicle_y):
        """Draw a vehicle id, given its id, x, y and yaw."""
        [pixel_x, pixel_y] = self.convert_position_to_image_pixel(vehicle_x,
                                                                  vehicle_y)

        text_surface = self.ids_font.render(str(vehicle_id), True,
                                            self.font_color)

        text_pos = (pixel_x, pixel_y)

        self.window.blit(text_surface, text_pos)

        self.add_surface_to_areas_to_blit(text_surface, text_pos)

        return

    def add_surface_to_areas_to_blit(self, surface, surface_pos):
        """
        Add surface region to the self.areas_to_blit list.

        Add the area of the region defined by surface and surface_pos to
        self.areas_to_blit.  This will tell the smart_background_blit method
        that this area needs to be blitted with the background in the next
        screen refresh
        """
        (surface_width, surface_height) = surface.get_size()

        self.areas_to_blit.append([surface_pos[0], surface_pos[1],
                                   surface_width, surface_height])
        return

    def draw_destination(self, dest):
        """
        @author: U{Nicolo' Campolongo<nicoloc@kth.se>}

        Draw the SemiControlledVehicle destination in the top left corner
        of the screen.
        """
        dest_pos_x = 0
        dest_pos_y = 0
        dest_pos = (dest_pos_x, dest_pos_y)

        dest_font = pygame.font.Font(None, 40)
        dest_text = dest_font.render(dest, 1, (0,0,0))
        self.window.fill((255, 255, 255), [dest_pos_x, dest_pos_y,
                dest_pos_x+100, dest_pos_y+40])
        self.window.blit(dest_text, dest_pos)

    def draw_clock(self):
        '''
        Draw a clock in the top right corner of the screen. Functionality of the
        clock is currently provided in BusDemand, so the clock will not work
        without running the bus node
        '''
        clock_pos_x = 950
        clock_pos_y = 0
        clock_pos = (clock_pos_x,clock_pos_y)

        clock_font = pygame.font.Font(None, 40)
        clock_text = clock_font.render("%02d.%02d"% divmod(self.time, 100), 1, (0,0,0))
        self.window.fill((255,255,255), [clock_pos_x, clock_pos_y,
                clock_pos_x + 70, clock_pos_y  + 30])
        self.window.blit(clock_text, clock_pos)

    def sort_lanelet_by_distance(self, nodes):
        '''
        The goal of this function is to sort the nodes into a line
        so that we can better represent them in the traffic models.
        We use the node coordinates calculated in update_lanelet_info.

        @param nodes: a set of nodes of a lanelets.
        '''
        #Since most lanelets are some form of a line, we will find
        #the node furthest from the average coordinate and then continously
        #find the nearest node which we haven't used yet

        coords = {x : tuple(self.node_coords[x]) for x in nodes}
        arr_len = float(len(nodes))
        avg_node_coords = tuple(map(lambda y: sum(y) / arr_len, zip(*coords.values())))

        #furthest_node = max(coords, key=lambda i: abs(ptdiff(coords[i], avg_node_coords)))

        ptdist = lambda p1,p2: (p1[0]-p2[0]) ** 2 + (p1[1]-p2[1]) ** 2
        min_index = -1
        min_value = -1
        min_coords = (-1,-1)
        #Find max without lambdas because lambdas are hard
        for node in nodes:
            coord = coords[node]
            dist_to_avg = ptdist(coord, avg_node_coords)
            if dist_to_avg > min_value:
                min_index = node
                min_value = dist_to_avg
                min_coords = coord

        # Now that we have the furthest point, we will simply continously
        # Find the closest point to the last node, remove that node, and repeat
        ret = []
        while len(coords) > 1:
            min_coords = coords[min_index]
            coords = {index : coords[index] for index in coords if index != min_index}
            ret.append(min_index)
            min_index = min(coords, key=lambda i: ptdist(coords[i], min_coords))
        ret.append(min_index) #one more
        return ret

    def update_lanelet_info(self, data):
        '''
        Transform the arrays that describe a dictionary from the ROS
        Subscriber into a dictionary whose keys are lanelets and the
        value is the an array of node ids in that lanelet. Should only
        be called once as lanelet information does not change frequently.

        See Lanelets.srv for an explanation on how we transform from a dict
        to an array back to a different dict to make ROS happy
        '''
        self.lanelet_dict = {}
        data_index = 0
        new_dict_index = 0
        # We will keep track of the coordinates of the individual nodes
        # so as to create an overlay for traffic flow.
        self.node_coords = {}
        for arr_length in data.key_lengths:
            self.lanelet_dict[new_dict_index] = list(data.values[data_index : data_index + arr_length])
            #We have to calculate the node coordinates before we can sort the lanelet
            for node in data.values[data_index : data_index + arr_length]:
                coors = self.get_node_coordinates(node)
                self.node_coords[node] = self.convert_position_to_image_pixel(coors[0], coors[1])
            self.lanelet_dict[new_dict_index] = self.sort_lanelet_by_distance(self.lanelet_dict[new_dict_index])
            self.traffic_news[new_dict_index] = 0
            data_index += arr_length
            new_dict_index += 1

        self.nodes = {}
        for i in range(len(data.node_ids)):
            self.nodes[data.node_ids[i]] = (data.x_pos[i], data.y_pos[i])

        self.traffic_update = True

    def update_lanelet_traffic(self,data):
        self.update_traffic(data.lanelet, data.new_traffic_level)

    def get_colour_from_traffic(self, lanelet_id):
        '''
        Defines the colouring based on how many cars have been passed through
        in the past hour. Simply reads from self.traffic_news to see how many,
        and matches.
        '''
        ratio = self.traffic_news[lanelet_id] / 10.
        return red_green_mix(ratio)

    def update_traffic(self, lanelet_id, new_value):
        self.traffic_news[lanelet_id] = new_value


    def draw_nodes(self):
        '''
        Will draw all the nodes. Useful for debugging
        '''
        for node_id, coors in self.nodes.iteritems():
            coords = self.convert_position_to_image_pixel(coors[0], coors[1])
            pygame.draw.circle(self.window, (0, 0, 0), coords, 3)
            font = pygame.font.Font(None, 16)
            text = font.render(str(-node_id), 1, (255,255,255))
            self.window.blit(text, coords)

    def draw_traffic(self):
        '''
        Draws the current traffic
        '''
        if not self.traffic_update:
            return
        for lanelet, nodes in self.lanelet_dict.iteritems():
            points = [self.node_coords[node] for node in nodes]
            colour = self.get_colour_from_traffic(lanelet)

            #Ignore last point for now
            for i in range(len(points) - 1):
                pointA = points[i]
                pointB = points[i + 1]
                pygame.draw.line(self.window, colour, pointA, pointB, 3)

    def draw_sensors(self):
        """
        Draw car's sensors
        It does so by first defining a surface with the same dimension
        as the self.window, then drawing all the radars on it and finally adding it
        to the self.window
        """
        # define sensor surface with transparent background
        sensors_surf = pygame.Surface((self.desired_window_width,self.desired_window_height))
        sensors_surf.set_colorkey((0,0,0))
        # red radars
        red = (255, 0, 0)
        # draw radars
        for vehicle_id in self.vehicles_dict:
            vehicle = self.vehicles_dict[vehicle_id]
            if not vehicle_id in self.sensors_dict:
                continue
            sensor = self.sensors_dict[vehicle_id]
            if sensor['radar_vis']:
                r = sensor['range']
                a = sensor['angle']
                a = 3.14/180 * a / 2
                x1 = vehicle['x'] + np.cos(vehicle['yaw']-a) * r
                y1 = vehicle['y'] + np.sin(vehicle['yaw']-a) * r
                x2 = vehicle['x'] + np.cos(vehicle['yaw']+a) * r
                y2 = vehicle['y'] + np.sin(vehicle['yaw']+a) * r
                [pixel_x1, pixel_y1] = self.convert_position_to_image_pixel(x1, y1)
                [pixel_x2, pixel_y2] = self.convert_position_to_image_pixel(x2, y2)
                [pixel_x3, pixel_y3] = self.convert_position_to_image_pixel(vehicle['x'], vehicle['y'])
                pygame.draw.lines(sensors_surf, red, True, [[pixel_x1, pixel_y1],[pixel_x2, pixel_y2],[pixel_x3, pixel_y3]], 2)
        sensors_surf = sensors_surf.convert()
        self.window.blit(sensors_surf, (0,0))
        self.add_surface_to_areas_to_blit(sensors_surf, (0,0))

    def kista_move_traffic_lights_off_road(self, light_id, tup):
        '''
        Move the traffic lights off the road for the kista map. Unfortunetly, this
        has to be done manually or automised by someone smarter than me.
        '''
        x,y = tup
        return {
            -8: (x - 5, y - 50),
            -18: (x, y + 14),
            -54: (x - 20, y + 18),
            -324: (x - 44, y - 18),
            -386: (x + 8, y - 28),
            -502: (x - 60, y - 24)
        }.get(light_id, (x,y))


    def draw_traffic_lights(self, n_id, x, y, toggled):
        '''
        Draws one traffic light at x, y
        '''
        '''
        '''
        gray = (192,192,192)
        coords = self.kista_move_traffic_lights_off_road(n_id, (x,y))
        rect = pygame.Rect(coords, (20, 30))
        pygame.draw.rect(self.window, gray, rect)

        if toggled: #light is green
            coords = (coords[0] + 10, coords[1] + 10)
            pygame.draw.circle(self.window, (0,255,0), coords, 4)
        else: # light is red
            coords = (coords[0] + 10, coords[1] + 20)
            pygame.draw.circle(self.window, (255,0,0), coords, 4)

    def update_traffic_lights(self):
        '''
        Takes the nodes where traffic light are located and whether
        they are on or off and draws them in spectacular fashion.
        '''
        for i in range(len(self.traffic_nodes)):
            node = self.traffic_nodes[i]
            toggled = self.traffic_status[i]
            n_coors = self.nodes[node]
            coords = self.convert_position_to_image_pixel(n_coors[0], n_coors[1])
            self.draw_traffic_lights(node, coords[0], coords[1], toggled)

    def display_image(self):
        """Call the methods needed to refresh and display the current image."""
        # First, redraw the image to be the original
        # brackground, with no vehicles in it.

        # self.dumb_background_blit()
        self.smart_background_blit()
        self.draw_clock()
        self.draw_crosswalks()

        # Once the background is drawn,
        # draw the vehicles and pedestrians
        self.draw_vehicles()
        self.draw_pedestrians()

        if self.radars:
            self.draw_sensors()
        if self.traffic_lights:
            self.update_traffic_lights()
        if self.traffic:
            self.draw_traffic()
        if self.highlight_node:
            self.draw_node()

        self.draw_events()

        if self.bus_stop_reset % 30 == 0:
            self.load_bus_stops()

        self.bus_stop_reset += 1

        # Pygame functions to update the visualization
        # window
        pygame.display.flip()
        pygame.event.pump()
        return

    def convert_position_to_image_pixel(self, x_pos, y_pos):
        """
        Convert world coordinates to pixel coordinates.

        Given a position in real world meters, it will return the equivalent
        pixel in the visualization window image.
        """
        x_pixel = self.image_center_x + x_pos * self.image_pixel_per_meter
        y_pixel = self.image_center_y - y_pos * self.image_pixel_per_meter

        x_pixel = int(round(x_pixel))
        y_pixel = int(round(y_pixel))

        return [x_pixel, y_pixel]
