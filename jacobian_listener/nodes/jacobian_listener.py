#!/usr/bin/env python
#spins off a thread to listen for joint_states messages
#and provides the same information (or subsets of) as a service

import roslib
roslib.load_manifest('jacobian_listener')
import rospy
from jacobian_listener.srv import *
from std_msgs.msg import Float64MultiArray
import threading
import numpy as np


#holds the latest states obtained from joint_states messages
class LatestJacobian:

    def __init__(self):
        rospy.init_node('jacobian_listener')
        self.lock = threading.Lock()
        self.jacobian = []
        self.name = ""
        self.thread = threading.Thread(target=self.jacobian_listener)
        self.thread.start()

        s = rospy.Service('return_jacobian', ReturnJacobian, self.return_jacobian)
        

    #thread function: listen for end_effector_force messages
    def jacobian_listener(self):
        #rospy.Subscriber('/jacobian_controller_name/jacobian', Float64MultiArray, self.jacobian_callback)
        rospy.Subscriber('/my_controller_name/jacobian', Float64MultiArray, self.jacobian_callback)
        rospy.spin()



    #callback function: when a end_effector_force message arrives, save the values
    def jacobian_callback(self, msg):
        self.lock.acquire()
        self.jacobian = []
        for item in msg.data:
            self.jacobian.append(item)
        self.lock.release()


    def return_jacobian(self, req):
        self.lock.acquire()
        jacobian = self.jacobian
        self.lock.release()
        return ReturnJacobianResponse(jacobian)


#run the server
if __name__ == "__main__":

    latestjacobian = LatestJacobian()

    print "jacobian_listener server started, waiting for queries"
    rospy.spin()
