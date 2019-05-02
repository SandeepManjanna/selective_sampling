#!/usr/bin/env python

#This sample code provides an easy to user interface to trigger sample taking
# Example of reading from serial
# 070616 214521 24.902 0.02 -0.00 -0.57 2.59 -0.58 84.90 7.03 0.86 0.00 14.06 9.916
# Check the order from the msg.
import time

import rospy
from adaptive_sampling.msg import SampleCandidate
from heron_water.srv import *

class SelectiveSampler(object):
    def __init__(self, threshold=1.0, replan_rate=10.0, num_pumps=8):
        self.selected_candidate = []
        self.threshold = 0
        self.data_threshold = []
        self.data_gps = []
        self.replan_rate = replan_rate
        self.state = 0
        self.used_pumps = [0] * num_pumps
        self.count = 0
        self.Tau = 30
        #Tau is a constant from Secretary Hiring Problem indicating the time until when we want to wait
        
    def sample_candidates_callback(self, msg):
        if self.data_threshold:
            max_value = max(self.data_threshold)
        self.data_threshold.append(msg.data)
        self.data_gps.append((msg.latitude, msg.longitude))
        self.count += 1
        print self.count
        if self.count > self.Tau:
            print msg.data
            print self.threshold
            #max_value = max(self.data_threshold)
            print "The max_value ="
            print max_value
            if msg.data > max_value:
                max_value = msg.data
                self.selected_candidate = (msg.latitude, msg.longitude)
                self.data_threshold.pop()
                self.data_gps.pop()
                #self.selected_candidate[1] = msg.longitude
                #self.threshold = msg.data
                self.threshold = 0
                self.count = 0
                print "Candidate Selected"
                rospy.logdebug("Candidate Selected")
                self.state = 1
            else:
                if self.count > self.Tau * 2:
                    max_index = self.data_threshold.index(max(self.data_threshold))
                    self.selected_candidate = self.data_gps.pop(max_index)
                    self.data_threshold.pop(max_index)
                    self.threshold = 0
                    self.count = 0
                    self.state = 1
                    print "Candidate Selected too long"
                else:
                    rospy.logdebug("Dropping the candidate as the data is less than the threshold")
                    print "Dropping the candidate as the data is less than the threshold"

    def spin(self):
        rospy.init_node('selective_sampling', anonymous=False)  #initializing the node
        r = rospy.Rate(self.replan_rate)
        rospy.Subscriber("/sample_candidates", SampleCandidate, self.sample_candidates_callback)
        while not rospy.is_shutdown():
            if self.state == 0:            
                # Do Nothing as long as there is no candidate selected.
                rospy.logdebug("No Candidate selected yet")
        
            elif self.state == 1:            
                # Wait for sample_waypoint service.
                print "Waiting for sample_waypoint service"
                rospy.wait_for_service('sample_waypoint')
                sample_waypoint = rospy.ServiceProxy('sample_waypoint', SampleWaypoint)
                if not [i for i, e in enumerate(self.used_pumps) if e != 1]:
                    rospy.logdebug("I am done with my samplers. Please get me to shore")
                    print "I am done with my samplers. Please get me to shore"
                    self.state=-1
                else:
                    self.pumpIndex = [i for i, e in enumerate(self.used_pumps) if e != 1][0]
                    #rospy.logdebug("Call sample_waypoint service: lat {} lon {} pumpIndex {}".format(self.current_gps[0],self.current_gps[1],self.pumpIndex))
                    print "Call sample_waypoint service: lat {} lon {} pumpIndex {}".format(self.selected_candidate[0],self.selected_candidate[1],self.pumpIndex)
                    resp = sample_waypoint(self.selected_candidate[0],self.selected_candidate[1],self.pumpIndex) # lat, lon, and pumpIndex
                    if (resp):
                        self.used_pumps[self.pumpIndex]=1
                        self.state=0
                    else:
                        rospy.logdebug("Service call to sample_waypoint Failed. Resetting the state to 0")
                        print "Service call to sample_waypoint Failed. Resetting the state to 0"
                        self.state=0
            else:
                print "Done with all the avialable samplers. Iam ready to go home"
                rospy.logdebug("Done with all the avialable samplers. Iam ready to go home")
                
            
            r.sleep()

def main():
    selective_sampler = SelectiveSampler()
    selective_sampler.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: 
        pass
