import os
import rospy
import rospkg
import pyqtgraph

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from .plot_widget import PlotWidget
from rqt_plot.data_plot import DataPlot

class TuningMode(Plugin):

    def __init__(self, context):
        super(TuningMode, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('TuningMode')

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
        ui_file = os.path.join(rospkg.RosPack().get_path('tuning_mode'), 'resource', 'TuningMode.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('TuningModeUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # add plot widgets to the ui
        ## Course plot ##
        course_c = self._widget.course_container
        self._tvc = PlotWidget(initial_topics=[('cc','chi_c'),('s','chi')])
        self._dpc = DataPlot(self._tvc)
        self._dpc.set_autoscale(x=True)
        self._dpc.set_autoscale(y=DataPlot.SCALE_EXTEND|DataPlot.SCALE_VISIBLE)
        self._dpc.set_xlim([0, 30.0])
        self._dpc.set_ylim([-6.28, 6.28])
        self._tvc.switch_data_plot_widget(self._dpc)
        course_c.addWidget(self._tvc, 1) # ratio of these numbers determines window proportions
        ## Roll plot
        roll_c = self._widget.roll_container
        self._tvr = PlotWidget(initial_topics=[('ci','phi_c'),('s','phi')])
        self._dpr = DataPlot(self._tvr)
        self._dpr.set_autoscale(x=True)
        self._dpr.set_autoscale(y=DataPlot.SCALE_EXTEND|DataPlot.SCALE_VISIBLE)
        self._dpr.set_xlim([0, 30.0])
        self._dpr.set_ylim([-6.28, 6.28])
        self._tvr.switch_data_plot_widget(self._dpr)
        roll_c.addWidget(self._tvr, 1) # ratio of these numbers determines window proportions


        ## Pitch plot
        pitch_c = self._widget.pitch_container
        self._tvp = PlotWidget(initial_topics=[('ci','theta_c'),('s','theta')])
        self._dpp = DataPlot(self._tvr)
        self._dpp.set_autoscale(x=True)
        self._dpp.set_autoscale(y=DataPlot.SCALE_EXTEND|DataPlot.SCALE_VISIBLE)
        self._dpp.set_xlim([0, 30.0])
        self._dpp.set_ylim([-6.28, 6.28])
        self._tvp.switch_data_plot_widget(self._dpp)
        pitch_c.addWidget(self._tvp, 1) # ratio of these numbers determines window proportions
        ## Altitude plot
        alt_c = self._widget.altitude_container
        self._tva = PlotWidget(initial_topics=[('cc','h_c'),('s','alt')])
        self._dpa = DataPlot(self._tva)
        self._dpa.set_autoscale(x=True)
        self._dpa.set_autoscale(y=DataPlot.SCALE_EXTEND|DataPlot.SCALE_VISIBLE)
        self._dpa.set_xlim([0, 30.0])
        self._dpa.set_ylim([-6.28, 6.28])
        self._tva.switch_data_plot_widget(self._dpa)
        alt_c.addWidget(self._tva, 1) # ratio of these numbers determines window proportions
        ## Airspeed plot
        va_c = self._widget.airspeed_container
        self._tvv = PlotWidget(initial_topics=[('cc','Va_c'),('s','Va')])
        self._dpv = DataPlot(self._tvv)
        self._dpv.set_autoscale(x=True)
        self._dpv.set_autoscale(y=DataPlot.SCALE_EXTEND|DataPlot.SCALE_VISIBLE)
        self._dpv.set_xlim([0, 30.0])
        self._dpv.set_ylim([-6.28, 6.28])
        self._tvv.switch_data_plot_widget(self._dpv)
        va_c.addWidget(self._tvv, 1) # ratio of these numbers determines window proportions


        # Add widget to the user interface
        context.add_widget(self._widget)
        

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
