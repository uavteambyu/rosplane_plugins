import os
import rospy
import rospkg
import rosnode
import pyqtgraph
from subprocess import Popen

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi, QtCore
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtWidgets import QSlider, QLabel, QLineEdit
from .plot_widget import PlotWidget
from rqt_plot.data_plot import DataPlot
from .tune_roll import *
import dynamic_reconfigure.client

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

        # add binding to button to start tuning mode:
        #self._widget.start_tune_btn.clicked.connect(self.tune_roll)
        
        # Add widget to the user interface
        context.add_widget(self._widget)
        
        # set up dynamic reconfigure, so controller will see changes to gains
        self.client = dynamic_reconfigure.client.Client('/rosplane_controller')
        
        
        # connect sliders to ros parameters
        sliders = self._widget.findChildren(QSlider)
        self.tuners = []
        for s in sliders:
            #print s.property('objectName')
            self.tuners.append(tuneSlider(s, self.client))
            
        # set button callback
        self._widget.set_params_btn.clicked.connect(self.set_params_sliders)
        
    def set_params_sliders(self):
		print 'set all params'
		for t in self.tuners:
			t.sliderChanged(t.slider.value())        
	
    def tune_roll(self):
        # kill path follower
        (a,b) = rosnode.kill_nodes(['/fixedwing/pathfollower'])
        print a,b
        # start giving step commands in roll:
        Popen(["rosrun", "tuning_mode", "tune_roll.py", "controller_commands:=/fixedwing/controller_commands"])
        
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
 
class tuneSlider():
	def __init__(self, s, dynclient):
		self.slider = s
		s.setTracking(False)
		self.client = dynclient
		
		# extract min/max values from labels near slider:
		val = []
		for l in s.parent().findChildren(QLabel):
			try:
				# convert text from label to number, store it
			    val.append(float(l.property('text')))
			except Exception as e:
				pass
		if val[0] > val[1]:
			self.pmax = val[0]
			self.pmin = val[1]
		else:
			self.pmax = val[1]
			self.pmin = val[0]
		
		self.txt = s.parent().findChildren(QLineEdit)[0]
		self.paramkey = '/rosplane_controller/' + s.property('objectName')
		self.paramitem = s.property('objectName')
		self.paramval = 0
		self.fetch_param()	# get initial param value and put in text box
		# set slider position
		self.setSlider()

		# set callbacks to update parameters
		self.txt.textChanged.connect(self.txtChanged)
		s.valueChanged.connect(self.sliderChanged)
		#QtCore.QObject.connect(self.slider, QtCore.SIGNAL('valueChanged(int)'), self.sliderChanged)
	
	# gets param from ros and then puts it in the text box
	def fetch_param(self):
		self.paramval = float(rospy.get_param(self.paramkey))
		self.txt.setText(str(self.paramval))

	def setSlider(self):
		self.slider.setValue(int((self.paramval - self.pmin) * 100/(self.pmax-self.pmin)))

	def sliderChanged(self, svalue):
		# get normal value from slider:
		val = self.pmin + svalue/100.0 * (self.pmax-self.pmin)
		# update the text and let the text callback change the param:
		self.txt.setText(str(val))
		
	def txtChanged(self):
		# make sure text is valid first:
		try:
			val = float(self.txt.property('text'))
		except:
			print 'not a numeric value'
			return
		# update ros param
		#rospy.set_param(self.paramkey, val)
		self.client.update_configuration({self.paramitem : val})
		# update text according to ros param to confirm it worked
		self.fetch_param()
		# update slider
		self.setSlider()
		print self.paramitem, '=', self.paramval
