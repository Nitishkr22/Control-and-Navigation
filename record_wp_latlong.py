from novatel_oem7_msgs.msg import BESTPOS
from novatel_oem7_msgs.msg import BESTVEL
from novatel_oem7_msgs.msg import INSPVA
from novatel_oem7_msgs.msg import BESTUTM
import rospy
import time

lat = 0.0
lng = 0.0
heading = 0.0
# def callback_latlong(data):
#     global lat,lng
#     lat = data.lat
#     lng = data.lon

#GNSS heading
def callback_heading(data):
    global heading, lat, lng
    heading=data.azimuth
    lat = data.latitude
    lng = data.longitude

# def callback_xandy(data):
#     global x_pos, y_pos
#     x_pos = data.dNorth
#     y_pos = data.dEast
      

rospy.init_node('Navigation', anonymous=True)
#ROS subscription
rospy.Subscriber("/novatel/oem7/inspva",INSPVA, callback_heading)


global new_file
new_file=str(int(time.time()))+".txt"
file=open(new_file,"w")

while not rospy.is_shutdown():
    # global lat,lng
    time.sleep(0.45)
    file = open(str(new_file), "a")
    print("done")
    file.writelines("["+str(lat)+","+str(lng)+"],\n")
    file.close()
    # print(lat,lng)
