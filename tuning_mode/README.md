This plugin uses dynamic reconfigure to change the parameters in rosplane_controller to tune PID gains. To modify which parameters are being changed by the various sliders, modify the TuningMode.ui file. The object names of the sliders are used as the ros parameter names. The text label values near the sliders are interpreted as the min/max values for the actual parameter. The namespace within which the parameters are expected to be found is in tuning_mode.py (currently /fixedwing/rosplane_controller).

Plotting was based on ros_groundstation, which used a modified version of rqt_plot.
