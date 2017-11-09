#!/usr/bin/env python
import rospy
import matplotlib.pyplot as plt
from std_msgs.msg import String
from std_msgs.msg import Float64
import builtins
import collections
from scipy.signal import savgol_filter

#create the list of capacitance
cap_list=collections.deque(maxlen=20)
#float capacitance
capacitance = 100.00
capacitance2 = 100.00
pub1 = rospy.Publisher('capacitance_val_2', Float64)
pub2 = rospy.Publisher('capacitance_val_3', Float64)


def callback(data):
   # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    global cap_list
    global capacitance
    cap_list.append(data.data)
    if len(cap_list) < 20 :
	capcitance = cap_list[-1] 
    else :
       y= savgol_filter(cap_list, 19, 3)
       p= savgol_filter(cap_list, 19, 2)
       capacitance = y[-1]
       capacitance2 = p[-1]
       pub1.publish(Float64(capacitance))
       pub2.publish(Float64(capacitance2))
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

