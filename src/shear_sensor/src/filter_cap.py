#!/usr/bin/env python
import rospy
import matplotlib.pyplot as plt
from std_msgs.msg import String
from std_msgs.msg import Float64
from shear_sensor.msg import cap
import builtins
import collections
from scipy.signal import savgol_filter
import numpy as np

# create the list of capacitance
cap_list=collections.deque(maxlen=20)
cap_list_mean=collections.deque(maxlen=200)

#plt.axis([0, 10, 0, 1])
#plt.ion()

# float capacitance
capacitance = 100.00
mean_cap = 100.00

# set up publisher
pub_cap = rospy.Publisher('capacitance_val', cap)
#pub_mean = rospy.Publisher('mean_cap', Float64)
msg = cap()
#count = 1
def callback(data):
    # global count  
    # Create the list to filter and calculate mean
    cap_list.append(data.data/100000)
    if len(cap_list) < 20 :
	capacitance = cap_list[-1] 
    else :
       # Savitzky Golay filter for continuous stream of data
       y = savgol_filter(cap_list, 15, 2)
       # p= savgol_filter(cap_list, 19, 2)
       cap_list_mean.append(y[-1])
       capacitance = y[-1]
       #pub_cap.publish(Float64(capacitance))
       if len(cap_list_mean) == 200 :
       		mean_cap = np.mean(cap_list_mean)
       		#pub_mean.publish(Float64(mean_cap))
       	        # msg.capacitance = Float64(capacitance)
       	        msg.capacitance = capacitance
       	        # msg.mean = Float64(mean_cap)
       	        msg.mean = mean_cap
		pub_cap.publish(msg)
                #count = count + 1
                #plt.plot(count,capacitance,'r--',count, mean_cap,'bs')
                #plt.pause(0.05)
                rospy.loginfo("capacitance %s and mean %s" % (msg.capacitance, msg.mean))
        
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('filter_cap', anonymous=True)
    rospy.Subscriber("chatter", Float64, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
