import rospy
from std_msgs.msg import String
import json, re
#from .Geo import Geobase
from math import fmod, pi

# custom messages
from rosflight_msgs.msg import GPS, RCRaw
from rosplane_msgs.msg import Current_Path, Waypoint, State, Controller_Internals, Controller_Commands

class InitSub(): # could end up being taken from rosplane_msgs.msg: State ++++
    init_latlonalt = [0.0, 0.0, 0.0]
    with_init = False
    enabled = False
    GB = None
    gps_init_topic = None
    gi_sub = None
    @staticmethod
    def updateInitLatLonAlt(new_init_latlonalt):
        print 'taking latlonalt from marble'
        InitSub.reset()
        InitSub.with_init = False
        InitSub.init_latlonalt = new_init_latlonalt
        InitSub.enabled = True

    @staticmethod
    def state_callback(state):
        InitSub.init_latlonalt[0] = state.initial_lat
        InitSub.init_latlonalt[1] = state.initial_lon
        InitSub.init_latlonalt[2] = state.initial_alt
        InitSub.enabled = True # only perform the calculations if GPS init received
        InitSub.gi_sub.unregister()

    @staticmethod
    def updateGPSInitTopic(new_topic):
        print 'subscribing to', new_topic
        InitSub.reset()
        InitSub.with_init = True
        InitSub.gps_init_topic = new_topic
        InitSub.gi_sub = rospy.Subscriber(InitSub.gps_init_topic, State, InitSub.state_callback)

    @staticmethod
    def getGPSInitTopic():
        return InitSub.gps_init_topic

    @staticmethod
    def closeSubscriber():
        InitSub.reset()

    @staticmethod
    def reset():
        InitSub.init_latlonalt = [0.0, 0.0, 0.0]
        InitSub.enabled = False
        InitSub.GB = None
        if not InitSub.gi_sub is None:
            InitSub.gi_sub.unregister()
            InitSub.gi_sub = None

class StateSub():
    state_sub = None
    state_topic = None
    lat = 0.0
    lon = 0.0
    alt = 0.0
    Va = 0.0
    phi = 0.0
    theta = 0.0
    psi = 0.0
    chi = 0.0
    enabled = False

    @staticmethod
    def updateStateTopic(new_state_topic):
        print 'subscribing to', new_state_topic
        StateSub.reset()
        StateSub.state_topic = new_state_topic
        if not StateSub.state_topic is None:
            StateSub.state_sub = rospy.Subscriber(StateSub.state_topic, State, StateSub.state_callback)

    @staticmethod
    def getStateTopic():
        return StateSub.state_topic

    @staticmethod
    def state_callback(state):
    	#StateSub.alt -= InitSub.init_latlonalt[2]
        StateSub.chi = fmod(state.chi, 2*pi)
        StateSub.Va = state.Va
        StateSub.phi = state.phi
        StateSub.theta = state.theta
        StateSub.psi = state.psi
        StateSub.enabled = True

    @staticmethod
    def closeSubscriber():
        print 'closing subscriber'
        StateSub.reset()

    @staticmethod
    def reset():
        StateSub.enabled = False
        StateSub.lat = 0.0
        StateSub.lon = 0.0
        StateSub.alt = 0.0
        StateSub.Va = 0.0
        StateSub.phi = 0.0
        StateSub.theta = 0.0
        StateSub.psi = 0.0
        StateSub.chi = 0.0
        if not StateSub.state_sub is None:
            StateSub.state_sub.unregister()
            StateSub.state_sub = None



class renderable_wp():
    def __init__(self, lat, lon, alt, chi_d, chi_valid, Va_d, converted=True):
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.chi_d = chi_d # radians
        self.chi_valid = chi_valid
        self.Va_d = Va_d
        self.converted = converted

class ConInSub():
    con_in_sub = None
    controller_inners_topic = None
    theta_c = 0.0
    phi_c = 0.0
    enabled = False

    @staticmethod
    def updateConInTopic(new_controller_inners_topic):
        print 'subscribing to', new_controller_inners_topic
        ConInSub.reset()
        ConInSub.controller_inners_topic = new_controller_inners_topic
        if not ConInSub.controller_inners_topic is None:
            ConInSub.con_in_sub = rospy.Subscriber(ConInSub.controller_inners_topic, Controller_Internals, ConInSub.callback_ConIn)

    @staticmethod
    def getConInTopic():
        return ConInSub.controller_inners_topic

    @staticmethod
    def callback_ConIn(controller_internals):
        ConInSub.theta_c = controller_internals.theta_c
        ConInSub.phi_c = controller_internals.phi_c
        ConInSub.enabled = True

    @staticmethod
    def closeSubscriber():
        print 'closing subscriber'
        ConInSub.reset()

    @staticmethod
    def reset():
        ConInSub.enabled = False
        ConInSub.theta_c = 0.0
        ConInSub.phi_c = 0.0
        if not ConInSub.con_in_sub is None:
            ConInSub.con_in_sub.unregister()
            ConInSub.con_in_sub = None

class ConComSub():
    con_com_sub = None
    controller_commands_topic = None
    Va_c = 0.0
    h_c = 0.0
    chi_c = 0.0
    enabled = False

    @staticmethod
    def updateConComTopic(new_controller_commands_topic):
        print 'subscribing to', new_controller_commands_topic
        ConComSub.reset()
        ConComSub.controller_commands_topic = new_controller_commands_topic
        if not ConComSub.controller_commands_topic is None:
            ConComSub.con_com_sub = rospy.Subscriber(ConComSub.controller_commands_topic, Controller_Commands, ConComSub.callback_ConCom)

    @staticmethod
    def getConComTopic():
        return ConComSub.controller_commands_topic

    @staticmethod
    def callback_ConCom(controller_commands):
        ConComSub.Va_c = controller_commands.Va_c
        ConComSub.h_c = controller_commands.h_c
        ConComSub.chi_c = fmod(controller_commands.chi_c, 2*pi)	# wrap commanded course to match state
        ConComSub.enabled = True

    @staticmethod
    def closeSubscriber():
        print 'closing subscriber'
        ConComSub.reset()

    @staticmethod
    def reset():
        ConComSub.enabled = False
        ConComSub.Va_c = 0.0
        ConComSub.h_c = 0.0
        ConComSub.chi_c = 0.0
        if not ConComSub.con_com_sub is None:
            ConComSub.con_com_sub.unregister()
            ConComSub.con_com_sub = None
