from novatel_oem7_msgs.msg import BESTPOS
from novatel_oem7_msgs.msg import BESTVEL
from novatel_oem7_msgs.msg import INSPVA
from novatel_oem7_msgs.msg import BESTUTM
# from novatel_gps_msgs.msg import 
from nav_msgs.msg import Odometry

import rospy
import time

x_pos = 0.0
y_pos = 0.0
# def callback_latlong(data):
#     global lat,lng
#     lat = data.lat
#     lng = data.lon

#GNSS heading
# def callback_heading(data):
#     global heading
#     heading=data.azimuth

def callback_xandy(data):
    global x_pos, y_pos
    x_pos = data.pose.pose.position.x
    y_pos = data.pose.pose.position.y
      

rospy.init_node('Navigation', anonymous=True)
#ROS subscription
# rospy.Subscriber("/novatel/oem7/bestpos",BESTPOS, callback_latlong)
# rospy.Subscriber("/novatel/oem7/inspva",INSPVA, callback_heading)
# rospy.Subscriber("/novatel/oem7/bestutm",BESTUTM, callback_xandy)
rospy.Subscriber("/novatel/oem7/odom",Odometry, callback_xandy)



global new_file
new_file=str(int(time.time()))+".txt"
file=open(new_file,"w")

while not rospy.is_shutdown():
    # global lat,lng
    time.sleep(0.45)
    file = open(str(new_file), "a")
    print("done")
    file.writelines("["+str(x_pos)+","+str(y_pos)+"],\n")
    file.close()
    # print(x_pos,y_pos)
