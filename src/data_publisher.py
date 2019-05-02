import rospy
from std_msgs.msg import String
from adaptive_sampling.msg import SampleCandidate
import pandas as pd

df = pd.read_csv('../data/expt_chloro_u_2017-04-29-14-27-32.csv', header=None)
lats = df[1]
lons = df[2]
data = df[3]



pub = rospy.Publisher('/sample_candidates', SampleCandidate, queue_size=1)
rospy.init_node('data_publisher')
r = rospy.Rate(50) # 10hz
i = 0
while not rospy.is_shutdown():
    sample_candidate_msg = SampleCandidate()
    sample_candidate_msg.header.stamp = rospy.get_rostime()
    sample_candidate_msg.latitude = lats[i]
    sample_candidate_msg.longitude = lons[i]
    sample_candidate_msg.data = data[i]
    pub.publish(sample_candidate_msg)
    i += 1
    r.sleep()
