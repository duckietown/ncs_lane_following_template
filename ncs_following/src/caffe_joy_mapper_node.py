#!/usr/bin/env python

import numpy as np
import rospy
import math
import caffe
import cv2
from cv_bridge import CvBridge, CvBridgeError

from duckietown_msgs.msg import Twist2DStamped, BoolStamped
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Image, CompressedImage, CameraInfo

from os.path import expanduser
home = expanduser("~")

from __builtin__ import True

class JoyMapper(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))

        self.joy = None
        self.last_pub_msg = None
        self.last_pub_time = rospy.Time.now()

        #initial CAFFE
        self.initial()
        self.bridge = CvBridge()
        self.joy_control = 1
        self.ncs_control = 0
        self.omega = 0
        self.count = 0

        # Setup Parameters
        self.v_gain = self.setupParam("~speed_gain", 0.41)
        self.omega_gain = self.setupParam("~steer_gain", 8.3)
        self.bicycle_kinematics = self.setupParam("~bicycle_kinematics", 0)
        self.steer_angle_gain = self.setupParam("~steer_angle_gain", 1)
        self.simulated_vehicle_length = self.setupParam("~simulated_vehicle_length", 0.18)

        # Publications
        self.pub_car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)
        self.pub_joy_override = rospy.Publisher("~joystick_override", BoolStamped, queue_size=1)
        self.pub_parallel_autonomy = rospy.Publisher("~parallel_autonomy",BoolStamped, queue_size=1)
        self.pub_anti_instagram = rospy.Publisher("anti_instagram_node/click",BoolStamped, queue_size=1)
        self.pub_e_stop = rospy.Publisher("wheels_driver_node/emergency_stop",BoolStamped,queue_size=1)
        self.pub_avoidance = rospy.Publisher("~start_avoidance",BoolStamped,queue_size=1)

        # Subscriptions
        self.image_sub = rospy.Subscriber("~image/compressed", CompressedImage, self.img_cb, queue_size=1)
        self.sub_joy_ = rospy.Subscriber("joy", Joy, self.cbJoy, queue_size=1)

        # timer
        # self.pub_timer = rospy.Timer(rospy.Duration.from_sec(self.pub_timestep),self.publishControl)
        self.param_timer = rospy.Timer(rospy.Duration.from_sec(1.0),self.cbParamTimer)
        self.has_complained = False

        self.state_parallel_autonomy = False
        self.state_verbose = False

        pub_msg = BoolStamped()
        pub_msg.data = self.state_parallel_autonomy
        pub_msg.header.stamp = self.last_pub_time
        self.pub_parallel_autonomy.publish(pub_msg)

    def initial(self):
        self.model = rospy.get_param("~model_name")
	caffe.set_device(0)
	caffe.set_mode_gpu()
	self.caffe_root = '/home/arg_ws6/caffe'
	sys.path.insert(0, self.caffe_root + 'python')
	self.net_full_conv = caffe.Net('/home/arg_ws6/models/'+self.model+'.prototxt', '/home/arg_ws6/models/'+self.model+'.caffemodel', caffe.TEST)        
	self.dim = (101, 101) #(width, height)


    def img_cb(self, data):
        #print "Image callback"
        self.count += 1
        if self.device_work == True and self.count == 4:
            self.count = 0
            try:
                #convert image_msg to cv format
                np_arr = np.fromstring(data.data, np.uint8)
                img = cv2.imdecode(np_arr,cv2.COLOR_BGR2RGB)
                img = cv2.resize(img, self.dim)
                img = img.astype(np.float32)	
                img_m = np.zeros((self.dim[1], self.dim[0], 3), np.float32)
                img_m[:] = (128.0)
                img = cv2.subtract(img, img_m)	
                img = img * 0.0078125

		# Input to caffe
		transformer = caffe.io.Transformer({'data': self.net_full_conv.blobs['data'].data.shape})
		transformer.set_transpose('data', (2,0,1))
		transformed_image = transformer.preprocess('data', img)
		# transformed_image -= np.mean(transformed_image)
		self.net_full_conv.blobs['data'].data[...] = transformed_image
		output = self.net_full_conv.forward()
                top1 = output.argmax()
		print top1
                self.omega = (top1 *(2/14.) - 1) * 6
                print self.omega
                
                if self.ncs_control == 1:
                    car_cmd_msg = Twist2DStamped()
                    car_cmd_msg.v = 1 * 0.36
                    car_cmd_msg.omega = self.omega
                    self.pub_car_cmd.publish(car_cmd_msg)
                
            except CvBridgeError as e:
                print(e)

    def cbParamTimer(self,event):
        self.v_gain = rospy.get_param("~speed_gain", 1.0)
        self.omega_gain = rospy.get_param("~steer_gain", 10)

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def cbJoy(self, joy_msg):
        self.joy = joy_msg
        self.publishControl()
        self.processButtons(joy_msg)

    def publishControl(self):
        if self.joy_control == 1:
            car_cmd_msg = Twist2DStamped()
            car_cmd_msg.header.stamp = self.joy.header.stamp
            car_cmd_msg.v = self.joy.axes[1] * self.v_gain #Left stick V-axis. Up is positive
            # Holonomic Kinematics for Normal Driving
            car_cmd_msg.omega = self.joy.axes[3] * self.omega_gain
            self.pub_car_cmd.publish(car_cmd_msg)

# Button List index of joy.buttons array:
# a = 0, b=1, x=2. y=3, lb=4, rb=5, back = 6, start =7,
# logitek = 8, left joy = 9, right joy = 10
# XXX: here we should use constants
    def processButtons(self, joy_msg):
        if (joy_msg.buttons[6] == 1): #The back button
            override_msg = BoolStamped()
            override_msg.header.stamp = self.joy.header.stamp
            override_msg.data = True
            rospy.loginfo('joystick_control = True')
            self.joy_control = 1
            self.ncs_control = 0
            self.pub_joy_override.publish(override_msg)
            
        elif (joy_msg.buttons[7] == 1): #the start button
            override_msg = BoolStamped()
            override_msg.header.stamp = self.joy.header.stamp
            override_msg.data = False
            rospy.loginfo('NCS_control = True')
            self.joy_control = 0
            self.ncs_control = 1
            self.pub_joy_override.publish(override_msg)
            
        elif (joy_msg.buttons[5] == 1): # Right back button
            self.state_verbose ^= True
            rospy.loginfo('state_verbose = %s' % self.state_verbose)
            rospy.set_param('line_detector_node/verbose', self.state_verbose) # bad - should be published for all to hear - not set a specific param

        elif (joy_msg.buttons[4] == 1): #Left back button
            self.state_parallel_autonomy ^= True
            rospy.loginfo('state_parallel_autonomy = %s' % self.state_parallel_autonomy)
            parallel_autonomy_msg = BoolStamped()
            parallel_autonomy_msg.header.stamp = self.joy.header.stamp
            parallel_autonomy_msg.data = self.state_parallel_autonomy
            self.pub_parallel_autonomy.publish(parallel_autonomy_msg)
        elif (joy_msg.buttons[3] == 1):
            anti_instagram_msg = BoolStamped()
            anti_instagram_msg.header.stamp = self.joy.header.stamp
            anti_instagram_msg.data = True
            rospy.loginfo('anti_instagram message')
            self.pub_anti_instagram.publish(anti_instagram_msg)
        elif (joy_msg.buttons[8] == 1): #power button (middle)
            e_stop_msg = BoolStamped()
            e_stop_msg.header.stamp = self.joy.header.stamp
            e_stop_msg.data = True # note that this is toggle (actual value doesn't matter)
            rospy.loginfo('E-stop message')
            self.pub_e_stop.publish(e_stop_msg)
        elif (joy_msg.buttons[9] == 1): #push left joystick button
            avoidance_msg = BoolStamped()
            rospy.loginfo('start lane following with avoidance mode')
            avoidance_msg.header.stamp = self.joy.header.stamp
            avoidance_msg.data = True
            self.pub_avoidance.publish(avoidance_msg)

        else:
            some_active = sum(joy_msg.buttons) > 0
            if some_active:
                rospy.loginfo('No binding for joy_msg.buttons = %s' % str(joy_msg.buttons))


if __name__ == "__main__":
    rospy.init_node("joy_mapper",anonymous=False)
    joy_mapper = JoyMapper()
    rospy.spin()
