#!/usr/bin/env python
import rospy
import matplotlib.pyplot as plt
from std_msgs.msg import String
from std_msgs.msg import Float64
import builtins
import collections
from scipy.signal import savgol_filter
import numpy as np

#create the list of capacitance
cap_list=collections.deque(maxlen=20)
cap_list_mean=collections.deque(maxlen=100)
#float capacitance
capacitance = 100.00
mean_cap = 100.00
pub1 = rospy.Publisher('capacitance_val', Float64)
pub2 = rospy.Publisher('mean_cap', Float64)


def callback(data):
   # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    global cap_list
    global capacitance
    cap_list.append(data.data/100000)
    cap_list_mean.append(data.data/100000)
    if len(cap_list) < 20 :
	capcitance = cap_list[-1] 
    else :
       y= savgol_filter(cap_list, 15, 2)
      # p= savgol_filter(cap_list, 19, 2)
       capacitance = y[-1]
       pub1.publish(Float64(capacitance))
       if len(cap_list_mean) == 100 :
       		mean_cap = np.mean(y)
       		pub2.publish(Float64(mean_cap))
       rospy.loginfo(rospy.get_caller_id() + "capacitance  %s", capacitance)
        
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

