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
        self._plot_course = self._widget.findChild(PlotWidget,"plot_course")
        self._plot_course = PlotWidget(initial_topics="/sonar/range")
        self._dp_course = DataPlot(self._plot_course)
        self._dp_course.set_autoscale(x=False)
        self._dp_course.set_autoscale(y=DataPlot.SCALE_EXTEND|DataPlot.SCALE_VISIBLE)
        self._dp_course.set_xlim([0, 10.0])
        self._plot_course.switch_data_plot_widget(self._dp_course)
        self._plot_course.add_topic('/sonar','range')
        #self._plot_course._update_plot_timer.start(self._plot_course._redraw_interval)
        self._plot_course.enable_timer(True)
        
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
