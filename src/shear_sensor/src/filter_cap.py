#!/usr/bin/env python
import rospy
import builtins
import collections
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import String
from std_msgs.msg import Float64
from shear_sensor.msg import cap
from scipy.signal import savgol_filter

# create the list of capacitance
shear_cap_list=collections.deque(maxlen=20)
shear_cap_list_mean=collections.deque(maxlen=100)
normal_cap_list=collections.deque(maxlen=20)
normal_cap_list_mean=collections.deque(maxlen=100)

# float capacitance
shear_capacitance = 100.00
shear_mean_cap = 100.00

normal_capacitance = 100.00
normal_mean_cap = 100.00

# Mean list size
mean_size = 100

# set up publisher
shear_pub_cap = rospy.Publisher('shear_capacitance_val', cap)
shear_msg = cap()

normal_pub_cap = rospy.Publisher('normal_capacitance_val', cap)
normal_msg = cap()

def shear_callback(data):
    # Create the list to filter and calculate mean
    shear_cap_list.append(data.data/100000)
    if len(shear_cap_list) < 20 :
	shear_capacitance = shear_cap_list[-1] 
    else :
       # Savitzky Golay filter for continuous stream of data
       y = savgol_filter(shear_cap_list, 15, 2)
       shear_cap_list_mean.append(y[-1])
       shear_capacitance = y[-1]
       if len(shear_cap_list_mean) == mean_size :
       		shear_mean_cap = np.mean(shear_cap_list_mean)
       	        shear_msg.capacitance = shear_capacitance
       	        shear_msg.mean = shear_mean_cap
		shear_pub_cap.publish(shear_msg)
                rospy.loginfo("capacitance %s and mean %s" % (shear_msg.capacitance, shear_msg.mean))
        
def normal_callback(data):
    # Create the list to filter and calculate mean
    normal_cap_list.append(data.data/100000)
    if len(normal_cap_list) < 20 :
	normal_capacitance = normal_cap_list[-1] 
    else :
       # Savitzky Golay filter for continuous stream of data
       y = savgol_filter(normal_cap_list, 15, 2)
       normal_cap_list_mean.append(y[-1])
       normal_capacitance = y[-1]
       if len(normal_cap_list_mean) == mean_size :
       		normal_mean_cap = np.mean(normal_cap_list_mean)
       	        normal_msg.capacitance = normal_capacitance
       	        normal_msg.mean = normal_mean_cap
		normal_pub_cap.publish(normal_msg)
                rospy.loginfo("capacitance %s and mean %s" % (normal_msg.capacitance, normal_msg.mean))
        
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('filter_cap', anonymous=True)
    rospy.Subscriber("raw_cap_shear", Float64, shear_callback)
    rospy.Subscriber("raw_cap_normal", Float64, normal_callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
