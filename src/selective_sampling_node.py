#!/usr/bin/env python

#This sample code provides an easy to user interface to trigger sample taking
# Example of reading from serial
# 070616 214521 24.902 0.02 -0.00 -0.57 2.59 -0.58 84.90 7.03 0.86 0.00 14.06 9.916
# Check the order from the msg.
import Queue
import threading
import time

from gps_tools import *
import numpy
import rospy
from adaptive_sampling.msg import ExplorerPath, SampleCandidate
from selective_sampling.msg import SamplerStatus
from std_msgs.msg import Int8

from gps_nav.srv import *
from heron_water.srv import *

class SelectiveSampler(object):
    """Class for managing the sampler robot.
    """
    def __init__(self, threshold=1.0, replan_rate=2.0, num_pumps=8, 
        expiration_time=100.0):
        """Constructor where all attributes are initialized.
        
        Args:
            threshold(float): TODO is it used?
            replan_rate(float): node spinning frequency in Hz.
            num_pumps(int): number of pumps to be used to sample.
            expiration_time(float): seconds after which sample is not valid.
        """
        # Problem related parameters.
        self.expiration_time = expiration_time # Sample valid until
                                                #expiration_time seconds pass.

        # Robot setup.
        self.used_pumps = [0] * num_pumps # State of the available pumps.

        # Node setup.
        self.replan_rate = replan_rate # Node spinning frequency in Hz.

        # Algorithm related parameters.
        self.distance_based = False # Flag for activating distance based removal of candidates around selected location.
        #Tau is a constant from Secretary Hiring Problem indicating the time until when we want to wait
        #self.Tau = 330
        #Tau for field expt : 63
        self.Tau = 50 # Secretary problem threshold count value. TODO automatically calculated.

        # Current state of the robot.
        self.state_mutex = threading.Lock()
        self.state = 0 # Current state of the robot {0: waiting to decide; 1: go to selected location}
        self.selected_locations_queue = Queue.PriorityQueue()

        self.selected_candidate = [] # Selected location to sample (lat/lon).
        self.candidate_list = [] # All location where sampled (list of lat/lon).
        self.deleted_samples = 0 # Number of deleted samples.

        # Structures to store past max data.
        self.data_timestamp = []
        self.data_threshold = []
        self.data_gps = []
        self.measurement_time = []
        # Structures for storing current explorer path.
        self.explorer_path_status = 0 # Current status of the explorer path. {0: Waiting until Tau; 1: After Tau going to current maxima; -1: done with current maxima visit, waiting until 3*Tau}

        # Temporary vars for secretary problem.
        self.threshold = 0
        self.count = 0 # Counter for how many messages have been processed.
        self.max_value = 0 # Current maximum value for the hiring of the secretary problem.

        self.isfar = False # Flag for saying whether the current measurement is far enough from already sampled areas.
        self.curr_maxima_value = 0 # The variable to hold current maximum value


        
    def distance_based_delete(self, dist):
        #print "In distance_based_delete"
        #print self.selected_candidate
        #print self.data_gps
        #print self.data_threshold
        candidate_utm = convert_gps_to_utm(self.selected_candidate[0],self.selected_candidate[1])
        gps_array = []
        data_array = []
        timestamp_array = []
        measurment_time_array = []
        j=0
        for i in range(len(self.data_gps)):
            utm_p = convert_gps_to_utm(self.data_gps[i][0],self.data_gps[i][1])
            d = numpy.linalg.norm(numpy.array([utm_p.x,utm_p.y])-numpy.array([candidate_utm.x, candidate_utm.y]))
            #print d
            if d > dist:
                gps_array.append((self.data_gps[i][0],self.data_gps[i][1]))
                data_array.append(self.data_threshold[i])
                timestamp_array.append(self.data_timestamp[i])
                measurement_time_array.append(self.measurement_time[i])
        self.data_gps = gps_array
        self.data_threshold = data_array
        self.data_timestamp = timestamp_array
        self.measurement_time = measurement_time_array
        #print "After delete"
        #print self.data_gps
        #print self.data_threshold

    def timestamp_based_delete(self):
        """Delete old samples.
        """
        for i, t in enumerate(self.data_timestamp):
            if rospy.get_rostime().to_sec() - t > self.expiration_time:
                self.data_gps.pop(i)
                self.data_threshold.pop(i)
                self.data_timestamp.pop(i)
                self.measurement_time.pop(i)
                self.deleted_samples += 1

    def check_farfrom_selected(self, lat, lon, dist):
        #Check if the data point is far from all the selected sample pointss
        datapoint_utm = convert_gps_to_utm(lat, lon)
        for i in range(len(self.candidate_list)):
            candidate_utm = convert_gps_to_utm(self.candidate_list[i][0],self.candidate_list[i][1])
            d = numpy.linalg.norm(numpy.array([datapoint_utm.x,datapoint_utm.y])-numpy.array([candidate_utm.x, candidate_utm.y]))
            #print 'distance between '+str(lat)+','+str(lon)+' and '+str(self.candidate_list[i][0])+','+str(self.candidate_list[i][1])+' is '+str(d)+'\n'
            if d < dist:
                #print 'returning false'
                return False
        return True

    def sample_candidates_callback(self, msg):
        distance = 5.0
        if self.data_threshold:
            max_value = max(self.data_threshold)
        self.isfar = self.check_farfrom_selected(msg.latitude, msg.longitude, distance)
        if(self.isfar): # TODO is it correct just incrementing if it is far enough?
            self.data_threshold.append(msg.data)
            self.data_gps.append((msg.latitude, msg.longitude))
            self.data_timestamp.append(msg.header.stamp.to_sec())
            self.measurement_time.append(rospy.get_time())
            self.count += 1
        rospy.logdebug("Current measurement counter={}".format(self.count))
        if (self.count > self.Tau and self.isfar):
            
            #print self.threshold
            #max_value = max(self.data_threshold)
            rospy.logdebug("The max_value={}".format(max_value))
            if msg.data > max_value:

                if (self.explorer_path_status == 1):
                    self.explorer_path_status = -1
                
                max_value = msg.data
                self.selected_candidate = (msg.latitude, msg.longitude)
                self.selected_locations_queue.put((self.data_timestamp[-1], self.selected_candidate, max_value))
                self.candidate_list.append(self.selected_candidate)
                self.chosen_measurement_time = self.measurement_time.pop()
                self.max_value = max_value
                self.data_threshold.pop()
                self.data_gps.pop()
                self.data_timestamp.pop()
                #self.selected_candidate[1] = msg.longitude
                #self.threshold = msg.data
                self.threshold = 0
                self.count = 0
                self.explorer_path_status = 0
                self.curr_maxima_value = 0
                rospy.logdebug("Candidate Selected")

                #SANDEEP
                #removing the old data
                if self.distance_based:
                    # 5m radius around the chosen point
                    self.distance_based_delete(distance)
                else:
                    self.data_threshold = []
                    self.data_gps = []
                    self.data_timestamp = []
                    self.measurement_time = []
            else:
                if self.count > self.Tau * 3:
                    # TODO clean the code.
                    max_index = self.data_threshold.index(max(self.data_threshold))
                    self.max_value = max(self.data_threshold)
                    self.selected_candidate = self.data_gps.pop(max_index)
                    self.candidate_list.append(self.selected_candidate)
                    self.chosen_measurement_time = self.measurement_time.pop(max_index)
                    self.selected_locations_queue.put((self.data_timestamp[max_index], self.selected_candidate, self.max_value))
                    self.data_threshold.pop(max_index)
                    self.data_timestamp.pop(max_index)
                    self.threshold = 0
                    self.count = 0
                    self.curr_maxima_value = 0
                    self.explorer_path_status = 0
                    #SANDEEP
                    #removing the old data
                    if self.distance_based:
                        # 5m radius around the chosen point
                        self.distance_based_delete(distance)
                    else:
                        self.data_threshold = []
                        self.data_gps = []
                        self.data_timestamp = []
                        self.measurement_time = []
                    rospy.logdebug("Candidate Selected too long")
                else:
                    rospy.logdebug("Current Maxima Visit status = ")
                    rospy.logdebug(self.explorer_path_status)
                    #After Tau time, go near the current maximum and wait.
                    if (self.curr_maxima_value < max(self.data_threshold)):
                        self.curr_maxima_value = max(self.data_threshold)
                        current_max_index = self.data_threshold.index(max(self.data_threshold))
                        self.wait_at_candidate = self.data_gps[current_max_index]
                        self.explorer_path_status = 1
                        rospy.logdebug("Goto current Maxima and wait there")
                        rospy.logdebug("wait point chosen is :")
                        rospy.logdebug(self.wait_at_candidate)
                    else:
                        rospy.logdebug("Dropping the candidate as the data is less than the threshold")


    def explorer_path_callback(self, explorer_path_msg):
        """Callback to save path followed by the explorer.

        Args:
            explorer_path_msg: ROS message. TODO ensure it is correct.
        """
        self.explorer_path = explorer_path_msg.path
        self.explorer_path_timestamp = explorer_path_msg.header.stamp.to_sec()
        self.explorer_path_status = 1

    def goto_waypoint_result_callback(self, msg):
        if msg.data == 1:
            rospy.loginfo("Reached destination.")
            self.state = 0
            rospy.logdebug("The self.explorer_path_status = ")
            rospy.logdebug(self.explorer_path_status)
            if self.explorer_path_status != -1:
                rospy.loginfo("Writing into the sample file")
                # Robot took a sample.
                self.selected_locations_queue.task_done()
                sample_time = rospy.get_time()
                with open('samples.txt','a') as f_handle:
                    f_handle.write(str(self.selected_candidate[0])+','+str(self.selected_candidate[1])+','+str(self.pumpIndex)+','+str(self.max_value)+','+str(sample_time)+','+str(self.chosen_measurement_time)+'\n')
            else:# self.explorer_path_status == 1:
                rospy.loginfo("Goto for maxima point Done")
                self.explorer_path_status = 0 # TODO check if it some checks are necessary
        else:
            # TODO(alberto) recovery mechanism.
            rospy.logerr("goto failed.")
            self.state = 0

    def spin(self):
        rospy.init_node('selective_sampling', anonymous=False, log_level=rospy.DEBUG)  #initializing the node
        r = rospy.Rate(self.replan_rate)
        self.state_pub = rospy.Publisher('selective_sampling_status', SamplerStatus, queue_size=1)
        self.sampler_status_msg = SamplerStatus()

        rospy.Subscriber("/sample_candidates", SampleCandidate, self.sample_candidates_callback)
        rospy.Subscriber("explorer_path", ExplorerPath, self.explorer_path_callback)
        rospy.Subscriber('waypoint_goto_result', Int8, self.goto_waypoint_result_callback)
        while not rospy.is_shutdown():
            self.state_mutex.acquire()
            current_state = self.state
            self.state_mutex.release()
            self.sampler_status_msg.stamp = rospy.get_rostime()
            self.sampler_status_msg.status = current_state
            self.state_pub.publish(self.sampler_status_msg)

            if current_state == 0:            
                # Do Nothing as long as there is no candidate selected.
                rospy.logdebug("No Candidate selected yet")
                if not self.selected_locations_queue.empty():
                    # Sample job.
                    item = self.selected_locations_queue.get()
                    self.selected_candidate = item[1]
                    self.state_mutex.acquire()
                    self.state = 1
                    self.state_mutex.release()
                    # Reset task of the robot to actually sample.
                    #if self.explorer_path_status == 1:
                    #    rospy.loginfo("spin::setting self.explorer_path_status = -1")
                    #    self.explorer_path_status = -1
                elif self.explorer_path_status == 1:
                    # If received a new path from explorer.
                    #explorer_path = numpy.array(self.explorer_path).reshape((len(self.explorer_path)/2, 2))
                    #self.centroid = numpy.mean(explorer_path,axis=0)
                    rospy.loginfo("Getting closer to current maxima:" + str(self.wait_at_candidate))
                    self.state_mutex.acquire()
                    self.state = 2
                    self.state_mutex.release()
            elif current_state == 1:
                # Goto for sampling.
                '''
                SANDEEP:
                print "Waiting for sample_waypoint service"
                rospy.wait_for_service('sample_waypoint')
                sample_waypoint = rospy.ServiceProxy('sample_waypoint', SampleWaypoint)
                '''
                if not [i for i, e in enumerate(self.used_pumps) if e != 1]:
                    rospy.loginfo("I am done with my samplers. Please get me to shore")
                    self.state_mutex.acquire()
                    self.state = -1
                    self.state_mutex.release()
                else:
                    self.pumpIndex = [i for i, e in enumerate(self.used_pumps) if e != 1][0]
                    rospy.loginfo("Call sample_waypoint service: lat {} lon {} pumpIndex {} value {}".format(self.selected_candidate[0],self.selected_candidate[1],self.pumpIndex,self.max_value))
                    '''
                    SANDEEP
                    resp = sample_waypoint(self.selected_candidate[0],self.selected_candidate[1],self.pumpIndex) # lat, lon, and pumpIndex
                    if (resp):
                        self.used_pumps[self.pumpIndex]=1
                        self.state=0
                    else:
                        rospy.logdebug("Service call to sample_waypoint Failed. Resetting the state to 0")
                        print "Service call to sample_waypoint Failed. Resetting the state to 0"
                        self.state=0
                    '''
                    rospy.wait_for_service('goto')
                    gotoWaypoint = rospy.ServiceProxy('goto', Goto)
                    resp = gotoWaypoint(self.selected_candidate[0], self.selected_candidate[1]) # lat, lon.
                    if resp:
                        result = True
                    else:
                        rospy.logerr("goto service failed.")
                        result = False
                    if result:
                        self.state_mutex.acquire()
                        self.state = 3
                        self.state_mutex.release()
                    self.used_pumps[self.pumpIndex] = 1
            elif current_state == 2:
                # Goto location just to get closer.
                rospy.wait_for_service('goto')
                gotoWaypoint = rospy.ServiceProxy('goto', Goto)
                print "Current maxima point chosen after Tau:"
                print self.wait_at_candidate
                resp = gotoWaypoint(self.wait_at_candidate[0], self.wait_at_candidate[1]) # lat, lon.
                if resp:
                    result = True
                else:
                    rospy.logerr("goto service failed.")
                    result = False
                if result:
                    self.state = 0
                    self.explorer_path_status = -1
                rospy.loginfo("Going to the current Maxima")
            elif current_state == 3:
                # going to sample.
                # TODO Recovery?
                rospy.logdebug("Going to sample.")
            else:
                rospy.loginfo("Done with all the available samplers. I am ready to go home")
                rospy.signal_shutdown("Done with all the SAMPLERS. SHUTTING DOWN")

            r.sleep()

def main():
    # Creating an instance of the class and spin until all samplers used.
    selective_sampler = SelectiveSampler()
    selective_sampler.spin()

if __name__ == '__main__':
    # Main for running selective sampling node algorithm.
    try:
        main()
    except rospy.ROSInterruptException: 
        pass
