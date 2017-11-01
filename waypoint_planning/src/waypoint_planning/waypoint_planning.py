import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QPushButton, QListWidget, QLineEdit
from python_qt_binding.QtCore import QRegExp
from python_qt_binding.QtGui import QRegExpValidator, QFileDialog
from parse import parse



class WaypointPlanner(Plugin):

    def __init__(self, context):
        super(WaypointPlanner, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('WaypointPlanner')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                            dest="quiet",
                            help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('waypoint_planning'), 'resource', 'MyPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('Waypoint Planner')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)
        # Buttons
        self.clearWaypointsButton = self._widget.findChild(QPushButton,"clearWaypointsButton")
        self.clearWaypointsButton.clicked.connect(self.handleClearWaypoints)
        self.loadFileButton = self._widget.findChild(QPushButton,"loadFileButton")
        self.loadFileButton.clicked.connect(self.handleLoadFile)
        self.saveFileButton = self._widget.findChild(QPushButton,"saveFileButton")
        self.saveFileButton.clicked.connect(self.handleSaveFile)
        self.addWaypointButton = self._widget.findChild(QPushButton,"addWaypointButton")
        self.addWaypointButton.clicked.connect(self.handleAddWaypoint)
        self.deleteWaypointButton = self._widget.findChild(QPushButton,"deleteWaypointButton")
        self.deleteWaypointButton.clicked.connect(self.handleDeleteWaypoint)
        self.sendWaypointsButton = self._widget.findChild(QPushButton,"sendWaypointsButton")
        self.sendWaypointsButton.clicked.connect(self.handleSendWaypoints)
        #Text Edits
        self.locationLineEdit = self._widget.findChild(QLineEdit,"locationLineEdit")
        regexp = QRegExp("\d+,\d+,\d+")
        validator = QRegExpValidator(regexp)
        self.locationLineEdit.setValidator(validator)
        regexp1 = QRegExp("\d+")
        validator1 = QRegExpValidator(regexp1)
        self.orientationLineEdit = self._widget.findChild(QLineEdit,"orientationLineEdit")
        self.orientationLineEdit.setValidator(validator1)
        self.velocityLineEdit = self._widget.findChild(QLineEdit,"velocityLineEdit")
        self.velocityLineEdit.setValidator(validator1)
        #Lists
        self.newWaypointList = self._widget.findChild(QListWidget,"newWaypointList")
        self.currentWaypointList = self._widget.findChild(QListWidget,"currentWaypointList")
        self.waypoints = []
        self.currentWaypoints = []
        self.currentWaypoint = None
        self.newWaypointList.itemClicked.connect(self.newWaypointSelected)
        self.fileDialog = QFileDialog()
        self.fileDialog.setFilter("Waypoint Files (*.wp)")

    def waypointFromString(self,text):
        (x,y,z,o,v) = parse("Waypoint X:{} Y:{} Z:{} {} @degrees, {}kph",text)
        location = (x,y,z)
        return (location,o,v)

    def waypointToString(self,waypoint):
        (location, orientation, velocity) = waypoint
        (locx, locy, locz) = location
        return "Waypoint X:{} Y:{} Z:{} @{}degrees, {}kph".format(locx,locy,locz,orientation,velocity)

    def createWaypoint(self,waypoint):
        self.waypoints.append(waypoint)
        self.newWaypointList.addItem(self.waypointToString(waypoint))

    def newWaypointSelected(self,item):
        waypoint = self.waypointFromString(item.text())
        self.currentWaypoint = waypoint

    def handleClearWaypoints(self):
        self.clearWaypointsButton.setText("You Cleared It!")


    def handleLoadFile(self):
        self.fileDialog.setFileMode(QFileDialog.ExistingFile)
        if self.fileDialog.exec_():
            file_name = self.fileDialog.selectedFiles()
            f = open(file_name[0],'r')
            for line in f:
                self.waypoints.append(self.waypointFromString(line))
                self.newWaypointList.addItem(line)

    def handleSaveFile(self):
        self.fileDialog.setFileMode(QFileDialog.AnyFile)
        if self.fileDialog.exec_():
            file_name = self.fileDialog.selectedFiles()
            f = open(file_name[0],'w')
            for waypoint in self.waypoints:
                f.write(self.waypointToString(waypoint))


    def handleAddWaypoint(self):
        text = self.locationLineEdit.text()
        (x,y,z) = parse('{},{},{}',text)
        orientation = self.orientationLineEdit.text()
        velocity = self.velocityLineEdit.text()
        location = (x,y,z)
        waypoint = (location, orientation, velocity)
        self.createWaypoint(waypoint)
        self.currentWaypoint = waypoint


    def handleDeleteWaypoint(self):
        if self.waypoints is None:
            print("Can't remove, no more waypoints")
        self.waypoints.remove(self.currentWaypoint)
        self.newWaypointList.clear()
        for waypoint in self.waypoints:
            self.newWaypointList.addItem(self.waypointToString(waypoint))


    def handleSendWaypoints(self):

        pass


    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

        #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog