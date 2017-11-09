#!/usr/bin/env python
import rospy
import matplotlib.pyplot as plt
from std_msgs.msg import String
from std_msgs.msg import Float64
from scipy import signal
import builtins

#create the list of capacitance
cap_list=collections.deque(maxlen=20)
capacitance = 1024

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    global cap_list
    global capacitance
    cap_list.append(data.data)
    if len(cap_list) < 20
	capcitance = cap_list[-1] 
    else
       y= savgol_filter(cap_list, 15, 2)
       capacitancce = y[-1]
        
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('filter', anonymous=True)
    rospy.Subscriber("chatter", Float64, callback)
    pub = rospy.Publisher('capacitance', Float64, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    pub.publish(capacitance)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

