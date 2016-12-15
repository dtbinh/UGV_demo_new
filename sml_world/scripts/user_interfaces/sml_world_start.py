#!/usr/bin/env python
"""
Start-up GUI for intelligent V2V demo 2016.

Based on GTK+ 3 library

@author: U{Marina Rantanen<marinar@kth.se>}
"""
import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, GdkPixbuf
from gi.repository.GdkPixbuf import InterpType
import roslaunch
import os
import datetime

# Path to sml_world ROS package
basepath = os.path.join(os.path.dirname(__file__), '..', '..')


class SMLWindow(Gtk.Window):
    """
    GUI window creator.

    """

    def __init__(self):
        """
        Initialize GUI objects.

        @param self.mapgrid: Placeholder for each GUI element
        @param guibox: Placeholder for self.mapgrid
        """
        Gtk.Window.__init__(self, title="SML World Command Central")
        self.set_default_size(1024, 768)

        # Box in which to place grid
        guibox = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)

        # Grip in which to place gui elements
        self.mapgrid = Gtk.Grid(column_spacing=10, row_spacing=10)
        guibox.pack_end(self.mapgrid, expand=True, fill=True, padding=10)

        # Label printing the name of XML map file to be selected
        self.selectedmap = Gtk.Label()
        self.selectedmap.set_text("Default map")
        self.selectedmap.set_justify(Gtk.Justification.LEFT)

        # Spinbutton for selection of number of cars to generate
        caradjustment = Gtk.Adjustment(0, 0, 25, 1, 10, 0)
        self.numberofcars = Gtk.SpinButton()
        self.numberofcars.set_size_request(210, 70)
        self.numberofcars.set_adjustment(caradjustment)
        # Label printing the name of numberofcars button
        numberofcarslabel = Gtk.Label()
        numberofcarslabel.set_size_request(210, 70)
        numberofcarslabel.set_text("Number of cars: ")
        numberofcarslabel.set_justify(Gtk.Justification.LEFT)

        # Traffic light radio buttons and label
        trafficlabel = Gtk.Label()
        trafficlabel.set_size_request(210, 70)
        trafficlabel.set_text("Traffic flow indicators: ")
        trafficlabel.set_justify(Gtk.Justification.LEFT)

        self.on = Gtk.RadioButton.new_with_label_from_widget(None, "On")
        self.off = Gtk.RadioButton.new_from_widget(self.on)
        self.off.set_label("Off")

        # Radar radio buttons and label
        radarlabel = Gtk.Label()
        radarlabel.set_size_request(210, 70)
        radarlabel.set_text("Radar visibility: ")
        radarlabel.set_justify(Gtk.Justification.LEFT)

        self.radaron = Gtk.RadioButton.new_with_label_from_widget(None, "On")
        self.radaroff = Gtk.RadioButton.new_from_widget(self.radaron)
        self.radaroff.set_label("Off")

        # Spinbutton for selection of number of pedestrians to generate
        pedadjustment = Gtk.Adjustment(0, 0, 50, 10, 10, 0)
        self.numberofpeds = Gtk.SpinButton()
        self.numberofpeds.set_size_request(210, 70)
        self.numberofpeds.set_adjustment(pedadjustment)
        # Label printing the name of numberofcars button
        numberofpedslabel = Gtk.Label()
        numberofpedslabel.set_size_request(210, 70)
        numberofpedslabel.set_text("Number of pedestrians: ")
        numberofpedslabel.set_justify(Gtk.Justification.LEFT)

        # Create button for map source file selection
        selectmapbutton = Gtk.Button("Select map source")
        selectmapbutton.set_size_request(110, 110)
        selectmapbutton.connect("clicked", self.on_selectmapbutton_clicked,
                                                            self.selectedmap)

        # Create button for opening map/starting SML World
        self.mapbutton = Gtk.Button("Open map")
        self.mapbutton.set_size_request(310, 110)
        self.mapbutton.connect("clicked", self.on_mapbutton_clicked,
                                                            self.selectedmap)

        self.selectedmapimg = Gtk.Image()

        # Create load and save buttons for loading/saving launch files
        self.load = Gtk.Button("Load configuration")
        self.save = Gtk.Button("Save current configuration")
        self.load.connect("clicked", self.on_load_clicked)
        self.save.connect("clicked", self.on_save_clicked, self.selectedmap)

        # Add elements to grid
        self.mapgrid.add(numberofcarslabel)
        self.mapgrid.attach_next_to(self.numberofcars, numberofcarslabel,
                                                 Gtk.PositionType.RIGHT, 2, 1)

        self.mapgrid.attach(numberofpedslabel, 0, 2, 1, 1)
        self.mapgrid.attach_next_to(self.numberofpeds, numberofpedslabel,
                                                 Gtk.PositionType.RIGHT, 2, 1)

        self.mapgrid.attach(trafficlabel, 0, 3, 1, 1)
        self.mapgrid.attach_next_to(self.on, trafficlabel,
                                                 Gtk.PositionType.RIGHT, 1, 1)
        self.mapgrid.attach_next_to(self.off, self.on,
                                                 Gtk.PositionType.RIGHT, 1, 1)

        self.mapgrid.attach(radarlabel, 0, 4, 1, 1)
        self.mapgrid.attach_next_to(self.radaron, radarlabel,
                                                 Gtk.PositionType.RIGHT, 1, 1)
        self.mapgrid.attach_next_to(self.radaroff, self.radaron,
                                                 Gtk.PositionType.RIGHT, 1, 1)

        self.mapgrid.attach(selectmapbutton, 0, 5, 1, 1)
        self.mapgrid.attach_next_to(self.selectedmap, selectmapbutton,
                                                 Gtk.PositionType.RIGHT, 1, 1)
        self.mapgrid.attach_next_to(self.selectedmapimg, self.selectedmap,
                                             Gtk.PositionType.RIGHT, 1, 1)

        self.mapgrid.attach(self.load, 1, 6, 1, 1)
        self.mapgrid.attach_next_to(self.save, self.load,
                                             Gtk.PositionType.RIGHT, 1, 1)


        # Attach map opening button to grid just under map selection button
        # and define button to be 2 columns wide and 1 row tall
#        self.mapgrid.attach_next_to(self.mapbutton, selectmapbutton,
#                                             Gtk.PositionType.BOTTOM, 3, 1)
        self.mapgrid.attach(self.mapbutton, 0, 7, 3, 1)



        # Add guibox to SMLWindow
        self.add(guibox)

    def on_selectmapbutton_clicked(self, button, selectedmap):
        """
        Open dialog to let user select map file.

        @param dialog: Default dialog is set to folder in ROS structure
        where maps (ie XML files) are located

        """

        # Define the dialog where map XML files are located
        dialog = Gtk.FileChooserDialog("Please choose a map file", self,
                                       Gtk.FileChooserAction.OPEN,
                                       (Gtk.STOCK_CANCEL,
                                       Gtk.ResponseType.CANCEL,
                                       "Select", Gtk.ResponseType.OK))

        dialog.set_current_folder(basepath + '/scripts/resources/scenarios')
        dialog.set_default_size(800, 400)

        # Initiate filter and set it to XML files
        filter = Gtk.FileFilter()
        filter.add_pattern("*.xml")

        # Add filter to dialog
        dialog.add_filter(filter)

        # Open defined dialog
        response = dialog.run()

        # Define dialog options
        if response == Gtk.ResponseType.OK:
            print("Select clicked")
            print("File selected: " + dialog.get_filename())
            # Cut path from filename
            file = dialog.get_filename()[62:]
            filename = file[:-4]

            # Print the selected XML file name in GUI (label)
            self.selectedmap.set_text(filename)

            imgfilepath = dialog.get_filename()[:-4] + '.bmp'

            pixbuf = GdkPixbuf.Pixbuf.new_from_file(imgfilepath)
            pixbuf = pixbuf.scale_simple(180, 100, InterpType.BILINEAR)
            self.selectedmapimg.set_from_pixbuf(pixbuf)

        elif response == Gtk.ResponseType.CANCEL:
            print("Cancel clicked")

        dialog.destroy()

    def on_load_clicked(self, button):
        """
        Open dialog to let user select select launch file.

        @param dialog: Default dialog is set to folder in ROS structure
        where launch files are located

        """
        # Define the dialog to open launch file

        dialog = Gtk.FileChooserDialog("Please select launch file", self,
                                        Gtk.FileChooserAction.OPEN,
                                       (Gtk.STOCK_CANCEL,
                                        Gtk.ResponseType.CANCEL,
                                        "Select", Gtk.ResponseType.OK))

        dialog.set_current_folder(basepath + '/launch')
        dialog.set_default_size(800, 400)


        # Initiate filter and set it to XML files
        filter = Gtk.FileFilter()
        filter.add_pattern("*.launch")

        # Add filter to dialog
        dialog.add_filter(filter)

        # Open defined dialog
        response = dialog.run()

        # Define dialog options
        if response == Gtk.ResponseType.OK:
            print("Select clicked")
            print("File selected: " + dialog.get_filename())

        elif response == Gtk.ResponseType.CANCEL:
            print("Cancel clicked")

        dialog.destroy()

    def on_save_clicked(self, button, selectedmap):
        print "Saving current configuration to launch file"
        mapfile = self.selectedmap.get_text()

        numberofcars = self.numberofcars.get_value_as_int()

        date = datetime.datetime.now()
        datestr = str(date)

        # Check if customstarter exists.
        # If True then rename with current date and time
        launch_files = []
        for content in os.listdir(basepath + '/launch'):
            launch_files.append(content)
        if 'customstarter.launch' in launch_files:
            customlaunchfile = open(basepath + '/launch/customstarter%s.launch' % datestr, "w")
        else:
            customlaunchfile = open(basepath + '/launch/customstarter.launch', "w")

        vehicle_id = 1
        customlaunchfile.write('<launch>\n')
        customlaunchfile.write('    <node pkg="sml_world" name="road_network" type="road_network.py" args="/resources/scenarios/%s' % mapfile)
        customlaunchfile.write(' True" />\n')
        customlaunchfile.write('    <node pkg="sml_world" name="visualization" type="visualization.py" />\n')
        customlaunchfile.write('    <node pkg="sml_world" name="sml_world_central" type="sml_world_central.py" />\n')
        for vehicle_id in range(1, numberofcars):
            customlaunchfile.write('    <node pkg="rosservice" name="spawn_vehicle%d' % vehicle_id)
            customlaunchfile.write('" type="rosservice" args="call --wait /spawn_vehicle ')
            customlaunchfile.write('\'{vehicle_id: %d' % vehicle_id)
            customlaunchfile.write(', class_name: \'DummyVehicle\', x: 0.0, y: 0.0, yaw: 0.8, v: 10.0, node_id: -400, toggle_sim: true}\'" />\n')
        customlaunchfile.write('</launch>\n')

        customlaunchfile.close()

    def on_mapbutton_clicked(self, button, selectedmap):
        """
        Write ROS launch file including map selected with
        on_selectmapbutton_clicked and call method to launch written file

        @param launchfile: Launch file written by current method including
        selected xml file
        """
        mapfile = self.selectedmap.get_text()

        numberofcars = self.numberofcars.get_value_as_int()

        practicefile = open(basepath + '/launch/teststarter.launch', "w")
        vehicle_id = 1
        practicefile.write('<launch>\n')
        practicefile.write('    <node pkg="sml_world" name="road_network" type="road_network.py" args="/resources/scenarios/%s' % mapfile)
        practicefile.write(' True" />\n')
        practicefile.write('    <node pkg="sml_world" name="visualization" type="visualization.py" />\n')
        practicefile.write('    <node pkg="sml_world" name="sml_world_central" type="sml_world_central.py" />\n')
        for vehicle_id in range(1, numberofcars):
            practicefile.write('    <node pkg="rosservice" name="spawn_vehicle%d' % vehicle_id)
            practicefile.write('" type="rosservice" args="call --wait /spawn_vehicle ')
            practicefile.write('\'{vehicle_id: %d' % vehicle_id)
            practicefile.write(', class_name: \'DummyVehicle\', x: 0.0, y: 0.0, yaw: 0.8, v: 10.0, node_id: -400, toggle_sim: true}\'" />\n')
        practicefile.write('</launch>\n')

        practicefile.close()
        start_sml()


"""

        # Check if XML file has been selected
        # Then start SML World using selected XML file or default map file
        launchfile = basepath + '/launch/teststarter.launch'
        if mapfile == "Default map":
            print "You have selected: KistaDemo3"
            launchfile = open(launchfile, "w")
            launchfile.write('<launch>\n')
            launchfile.write('    <node pkg="sml_world" name="road_network" type="road_network.py" args="/resources/scenarios/KistaDemo3')
            launchfile.write(' True" />\n')
            launchfile.write('    <node pkg="sml_world" name="visualization" type="visualization.py" />\n')
            launchfile.write('    <node pkg="sml_world" name="sml_world_central" type="sml_world_central.py" />\n')

            if numberofcars == 1:
                launchfile.write('    <node pkg="rosservice" name="spawn_vehicle" type="rosservice" args="call --wait /spawn_vehicle ')
                launchfile.write('    \'{vehicle_id: 1, class_name: \'DummyVehicle\', x: 0.0, y: 0.0, yaw: 0.8, v: 10.0, node_id: -400, toggle_sim: true}\'" />\n')

            elif numberofcars == 2:
                launchfile.write('    <node pkg="rosservice" name="spawn_vehicle1" type="rosservice" args="call --wait /spawn_vehicle ')
                launchfile.write('    \'{vehicle_id: 1, class_name: \'DummyVehicle\', x: 0.0, y: 0.0, yaw: 0.8, v: 10.0, node_id: -400, toggle_sim: true}\'" />\n')
                launchfile.write('    <node pkg="rosservice" name="spawn_vehicle2" type="rosservice" args="call --wait /spawn_vehicle ')
                launchfile.write('    \'{vehicle_id: 2, class_name: \'DummyVehicle\', x: 0.0, y: 0.0, yaw: 0.8, v: 10.0, node_id: -582, toggle_sim: true}\'" />\n')

            elif numberofcars == 3:
                launchfile.write('    <node pkg="rosservice" name="spawn_vehicle1" type="rosservice" args="call --wait /spawn_vehicle ')
                launchfile.write('    \'{vehicle_id: 1, class_name: \'DummyVehicle\', x: 0.0, y: 0.0, yaw: 0.8, v: 10.0, node_id: -400, toggle_sim: true}\'" />\n')
                launchfile.write('    <node pkg="rosservice" name="spawn_vehicle2" type="rosservice" args="call --wait /spawn_vehicle ')
                launchfile.write('    \'{vehicle_id: 2, class_name: \'DummyVehicle\', x: 0.0, y: 0.0, yaw: 0.8, v: 15.0, node_id: -582, toggle_sim: true}\'" />\n')
                launchfile.write('    <node pkg="rosservice" name="spawn_vehicle3" type="rosservice" args="call --wait /spawn_vehicle ')
                launchfile.write('    \'{vehicle_id: 3, class_name: \'DummyVehicle\', x: 0.0, y: 0.0, yaw: 0.8, v: 10.0, node_id: -186, toggle_sim: true}\'" />\n')

            elif numberofcars == 4:
                launchfile.write('    <node pkg="rosservice" name="spawn_vehicle1" type="rosservice" args="call --wait /spawn_vehicle ')
                launchfile.write('    \'{vehicle_id: 1, class_name: \'DummyVehicle\', x: 0.0, y: 0.0, yaw: 0.8, v: 10.0, node_id: -400, toggle_sim: true}\'" />\n')
                launchfile.write('    <node pkg="rosservice" name="spawn_vehicle2" type="rosservice" args="call --wait /spawn_vehicle ')
                launchfile.write('    \'{vehicle_id: 2, class_name: \'DummyVehicle\', x: 0.0, y: 0.0, yaw: 0.8, v: 15.0, node_id: -582, toggle_sim: true}\'" />\n')
                launchfile.write('    <node pkg="rosservice" name="spawn_vehicle3" type="rosservice" args="call --wait /spawn_vehicle ')
                launchfile.write('    \'{vehicle_id: 3, class_name: \'DummyVehicle\', x: 0.0, y: 0.0, yaw: 0.8, v: 10.0, node_id: -186, toggle_sim: true}\'" />\n')
                launchfile.write('    <node pkg="rosservice" name="spawn_vehicle4" type="rosservice" args="call --wait /spawn_vehicle ')
                launchfile.write('    \'{vehicle_id: 4, class_name: \'DummyVehicle\', x: 0.0, y: 0.0, yaw: 0.8, v: 10.0, node_id: -360, toggle_sim: true}\'" />\n')

            elif numberofcars == 5:
                launchfile.write('    <node pkg="rosservice" name="spawn_vehicle1" type="rosservice" args="call --wait /spawn_vehicle ')
                launchfile.write('    \'{vehicle_id: 1, class_name: \'DummyVehicle\', x: 0.0, y: 0.0, yaw: 0.8, v: 10.0, node_id: -400, toggle_sim: true}\'" />\n')
                launchfile.write('    <node pkg="rosservice" name="spawn_vehicle2" type="rosservice" args="call --wait /spawn_vehicle ')
                launchfile.write('    \'{vehicle_id: 2, class_name: \'DummyVehicle\', x: 0.0, y: 0.0, yaw: 0.8, v: 15.0, node_id: -582, toggle_sim: true}\'" />\n')
                launchfile.write('    <node pkg="rosservice" name="spawn_vehicle3" type="rosservice" args="call --wait /spawn_vehicle ')
                launchfile.write('    \'{vehicle_id: 3, class_name: \'DummyVehicle\', x: 0.0, y: 0.0, yaw: 0.8, v: 10.0, node_id: -186, toggle_sim: true}\'" />\n')
                launchfile.write('    <node pkg="rosservice" name="spawn_vehicle4" type="rosservice" args="call --wait /spawn_vehicle ')
                launchfile.write('    \'{vehicle_id: 4, class_name: \'DummyVehicle\', x: 0.0, y: 0.0, yaw: 0.8, v: 10.0, node_id: -360, toggle_sim: true}\'" />\n')
                launchfile.write('    <node pkg="rosservice" name="spawn_vehicle5" type="rosservice" args="call --wait /spawn_vehicle ')
                launchfile.write('    \'{vehicle_id: 5, class_name: \'DummyVehicle\', x: 0.0, y: 0.0, yaw: 0.8, v: 20.0, node_id: -40, toggle_sim: true}\'" />\n')

            else:
                print "No cars will be simulated "

        else:
            print "You have selected:", mapfile

            launchfile = open(launchfile, "w")
            launchfile.write('<launch>\n')
            launchfile.write('    <node pkg="sml_world" name="road_network" type="road_network.py" args="/resources/scenarios/%s' % mapfile)
            launchfile.write(' True" />\n')
            launchfile.write('    <node pkg="sml_world" name="visualization" type="visualization.py" />\n')
            launchfile.write('    <node pkg="sml_world" name="sml_world_central" type="sml_world_central.py" />\n')

            if numberofcars == 1:
                launchfile.write('    <node pkg="rosservice" name="spawn_vehicle" type="rosservice" args="call --wait /spawn_vehicle ')
                launchfile.write('    \'{vehicle_id: 1, class_name: \'DummyVehicle\', x: 0.0, y: 0.0, yaw: 0.8, v: 10.0, node_id: -400, toggle_sim: true}\'" />\n')

            elif numberofcars == 2:
                launchfile.write('    <node pkg="rosservice" name="spawn_vehicle1" type="rosservice" args="call --wait /spawn_vehicle ')
                launchfile.write('    \'{vehicle_id: 1, class_name: \'DummyVehicle\', x: 0.0, y: 0.0, yaw: 0.8, v: 10.0, node_id: -400, toggle_sim: true}\'" />\n')
                launchfile.write('    <node pkg="rosservice" name="spawn_vehicle2" type="rosservice" args="call --wait /spawn_vehicle ')
                launchfile.write('    \'{vehicle_id: 2, class_name: \'DummyVehicle\', x: 0.0, y: 0.0, yaw: 0.8, v: 10.0, node_id: -582, toggle_sim: true}\'" />\n')

            elif numberofcars == 3:
                launchfile.write('    <node pkg="rosservice" name="spawn_vehicle1" type="rosservice" args="call --wait /spawn_vehicle ')
                launchfile.write('    \'{vehicle_id: 1, class_name: \'DummyVehicle\', x: 0.0, y: 0.0, yaw: 0.8, v: 10.0, node_id: -400, toggle_sim: true}\'" />\n')
                launchfile.write('    <node pkg="rosservice" name="spawn_vehicle2" type="rosservice" args="call --wait /spawn_vehicle ')
                launchfile.write('    \'{vehicle_id: 2, class_name: \'DummyVehicle\', x: 0.0, y: 0.0, yaw: 0.8, v: 15.0, node_id: -582, toggle_sim: true}\'" />\n')
                launchfile.write('    <node pkg="rosservice" name="spawn_vehicle3" type="rosservice" args="call --wait /spawn_vehicle ')
                launchfile.write('    \'{vehicle_id: 3, class_name: \'DummyVehicle\', x: 0.0, y: 0.0, yaw: 0.8, v: 10.0, node_id: -186, toggle_sim: true}\'" />\n')

            elif numberofcars == 4:
                launchfile.write('    <node pkg="rosservice" name="spawn_vehicle1" type="rosservice" args="call --wait /spawn_vehicle ')
                launchfile.write('    \'{vehicle_id: 1, class_name: \'DummyVehicle\', x: 0.0, y: 0.0, yaw: 0.8, v: 10.0, node_id: -400, toggle_sim: true}\'" />\n')
                launchfile.write('    <node pkg="rosservice" name="spawn_vehicle2" type="rosservice" args="call --wait /spawn_vehicle ')
                launchfile.write('    \'{vehicle_id: 2, class_name: \'DummyVehicle\', x: 0.0, y: 0.0, yaw: 0.8, v: 15.0, node_id: -582, toggle_sim: true}\'" />\n')
                launchfile.write('    <node pkg="rosservice" name="spawn_vehicle3" type="rosservice" args="call --wait /spawn_vehicle ')
                launchfile.write('    \'{vehicle_id: 3, class_name: \'DummyVehicle\', x: 0.0, y: 0.0, yaw: 0.8, v: 10.0, node_id: -186, toggle_sim: true}\'" />\n')
                launchfile.write('    <node pkg="rosservice" name="spawn_vehicle4" type="rosservice" args="call --wait /spawn_vehicle ')
                launchfile.write('    \'{vehicle_id: 4, class_name: \'DummyVehicle\', x: 0.0, y: 0.0, yaw: 0.8, v: 10.0, node_id: -360, toggle_sim: true}\'" />\n')

            elif numberofcars == 5:
                launchfile.write('    <node pkg="rosservice" name="spawn_vehicle1" type="rosservice" args="call --wait /spawn_vehicle ')
                launchfile.write('    \'{vehicle_id: 1, class_name: \'DummyVehicle\', x: 0.0, y: 0.0, yaw: 0.8, v: 10.0, node_id: -400, toggle_sim: true}\'" />\n')
                launchfile.write('    <node pkg="rosservice" name="spawn_vehicle2" type="rosservice" args="call --wait /spawn_vehicle ')
                launchfile.write('    \'{vehicle_id: 2, class_name: \'DummyVehicle\', x: 0.0, y: 0.0, yaw: 0.8, v: 15.0, node_id: -582, toggle_sim: true}\'" />\n')
                launchfile.write('    <node pkg="rosservice" name="spawn_vehicle3" type="rosservice" args="call --wait /spawn_vehicle ')
                launchfile.write('    \'{vehicle_id: 3, class_name: \'DummyVehicle\', x: 0.0, y: 0.0, yaw: 0.8, v: 10.0, node_id: -186, toggle_sim: true}\'" />\n')
                launchfile.write('    <node pkg="rosservice" name="spawn_vehicle4" type="rosservice" args="call --wait /spawn_vehicle ')
                launchfile.write('    \'{vehicle_id: 4, class_name: \'DummyVehicle\', x: 0.0, y: 0.0, yaw: 0.8, v: 10.0, node_id: -360, toggle_sim: true}\'" />\n')
                launchfile.write('    <node pkg="rosservice" name="spawn_vehicle5" type="rosservice" args="call --wait /spawn_vehicle ')
                launchfile.write('    \'{vehicle_id: 5, class_name: \'DummyVehicle\', x: 0.0, y: 0.0, yaw: 0.8, v: 20.0, node_id: -40, toggle_sim: true}\'" />\n')

            else:
                print "No cars will be simulated "

            launchfile.write('</launch>\n')

            launchfile.close()

        start_sml()
"""

def start_sml():
    """
    Method to run ROS launch file written by SML Window

    @param launchfile: File written by on_mapbutton_clicked containing selected
    or default XML file
    """
    launchfile = basepath + '/launch/teststarter.launch'

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    #print roslaunch.rlutil.check_roslaunch(launchfile)
    #roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, [launchfile])
    launch.start()


window = SMLWindow()
window.connect("delete-event", Gtk.main_quit)
window.show_all()
Gtk.main()