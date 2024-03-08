import numpy as np
from novatel_oem7_msgs.msg import BESTPOS
from novatel_oem7_msgs.msg import BESTVEL
from novatel_oem7_msgs.msg import INSPVA
import rospy
from std_msgs.msg import String
import actuator
import math
import pid
import time
from geopy.distance import geodesic


lat = 0
lng = 0
heading = 0
wp_threshold = 1.111395e5
#GNSS position
def callback_latlong(data):
    global lat,lng
    lat = data.lat
    lng = data.lon

#GNSS heading
def callback_heading(data):

    global heading
    heading=data.azimuth  
rospy.init_node('Navigation', anonymous=True)
#ROS subscription
rospy.Subscriber("/novatel/oem7/bestpos",BESTPOS, callback_latlong)
rospy.Subscriber("/novatel/oem7/inspva",INSPVA, callback_heading)

#TCP connection
obj = actuator.controller("169.254.178.227",5001)
obj.connect()

#PID controller
Kp = 1.65971
Ki = 0.00007
Kd = 0.710
rate_min = -100
rate_max = 100
pid_controller = pid.PIDController(Kp, Ki, Kd, rate_min, rate_max)

def calculate_steer_angle(currentLocation, wp, heading):
    """
    This function takes three inputs:
        - currentLocation: a list of two float values representing the current location
        - wp: a waypoint value
        - heading: a heading value
    It then calculates the steer output based on the current location, waypoint, and heading, and returns the steer output value.
    """
    off_y = - currentLocation[0] + float(waypoints[wp][0])
    off_x = - currentLocation[1] + float(waypoints[wp][1])

    # calculate bearing based on position error
    bearing_ppc =90.00 + math.atan2(-off_y, off_x) * 57.2957795  # Adding 90.00 is a common adjustment to align the bearing with cardinal directions (e.g., north as 0 degrees).

    # convert negative bearings to positive by adding 360 degrees
    if bearing_ppc < 0:
        bearing_ppc += 360.00

    # calculate the difference between heading and bearing
    bearing_diff = heading - bearing_ppc

    # normalize bearing difference to range between -180 and 180 degrees
    if bearing_diff < -180:
        bearing_diff = bearing_diff + 360

    if bearing_diff > 180:
        bearing_diff = bearing_diff - 360

    steer_output =  np.arctan(-1 * 2 * 3.5 * np.sin(np.pi * bearing_diff / 180) / 8)*(180/np.pi)

    steer_output = np.clip(steer_output, a_min = -30, a_max = 30)
    steer_output = (50/3)*steer_output
    steer_output = np.clip(steer_output, a_min = -500, a_max = 500)

    return steer_output

#Load waypoints
def load_waypoints(file_path):
        data_list = []
        try:
            with open(file_path, "r") as file:
                lines = file.readlines()
                for line in lines:
                    # Split each line using a comma as delimiter and convert values to floats
                    values = [float(x.strip()) for x in line.strip().split()]
                    data_list.append(values)
        except FileNotFoundError:
            print("File not found.")
        except Exception as e:
            print(f"Error occurred: {e}")
        return data_list



#file_path = 'output.txt'
#waypoints = load_waypoints(file_path)
waypoints = [[17.60132545555418,78.12661326306879],
[17.60132688277009,78.126613282807],
[17.601329242727886,78.12661328785218],
[17.601331801064575,78.12661328931584],
[17.601336244100718,78.12661326790277],
[17.60134091325874,78.12661321887452],
[17.60134820403933,78.12661322376142],
[17.601355270086565,78.1266131688615],
[17.601365698640052,78.12661311364934],
[17.601375244861497,78.12661308225933],
[17.601388508702996,78.12661302035454],
[17.601400147028258,78.12661289079324],
[17.601415879957504,78.12661258730145],
[17.60142914645408,78.12661232989568],
[17.60144608269603,78.12661201046608],
[17.601459492877506,78.12661207560612],
[17.601475867535463,78.12661166142988],
[17.601488931345227,78.12661154860785],
[17.601504833576804,78.1266113377909],
[17.601517377698084,78.12661112494952],
[17.601532752881898,78.12661092755137],
[17.601544908061232,78.12661080132918],
[17.601560014724463,78.12661073330497],
[17.60157492548657,78.12661047853827],
[17.601586728066092,78.12661027123774],
[17.60160133579248,78.12661001894627],
[17.60161289956062,78.12660981212524],
[17.60162721632764,78.12660955930093],
[17.601638516622916,78.12660933020416],
[17.60165260349245,78.12660910282706],
[17.601663946001917,78.1266088898709],
[17.601678409212994,78.12660861640302],
[17.601690309252184,78.12660842457075],
[17.601705468058622,78.12660814552183],
[17.601717884521467,78.12660787432931],
[17.60173362186569,78.12660755566046],
[17.60174648708069,78.12660726975673],
[17.601762818496848,78.12660695799357],
[17.60177610980565,78.12660668169916],
[17.601793029394106,78.12660655042977],
[17.601806734424894,78.12660638122942],
[17.601824101461617,78.12660608833372],
[17.601838234022306,78.12660598303917],
[17.601856065841677,78.12660574715821],
[17.60187060472207,78.1266057884052],
[17.601889100867517,78.12660557918097],
[17.601904094231326,78.12660549854492],
[17.60192320772315,78.12660539577342],
[17.601938661007264,78.12660530996929],
[17.60195825244207,78.12660527711046],
[17.601974074851412,78.12660518597886],
[17.60199414907905,78.1266049879687],
[17.602010325942704,78.12660480884901],
[17.602030669892994,78.12660446561243],
[17.60204716026073,78.12660419350905],
[17.60206802855363,78.12660385654638],
[17.602084861292788,78.12660352122487],
[17.60210600992233,78.12660324588407],
[17.60212290855975,78.12660296299697],
[17.602143944910072,78.12660250401143],
[17.602160741776302,78.1266021560212],
[17.602181584252488,78.12660165991403],
[17.602198161180848,78.12660110696764],
[17.60221873289723,78.12660050439285],
[17.602235153087204,78.1266000126401],
[17.602255600203012,78.12659935014885],
[17.602271786217614,78.12659876530466],
[17.602291984011334,78.12659808800132],
[17.602308055759124,78.12659755750549],
[17.602328029845264,78.12659681852483],
[17.602347948778853,78.12659607211701],
[17.602363890677385,78.12659552539891],
[17.6023837427167,78.12659496151257],
[17.60239960234733,78.12659461210737],
[17.602419346513397,78.12659432045062],
[17.60243514826837,78.12659418330179],
[17.602454715925223,78.12659400219205],
[17.602469751147904,78.12659418766904],
[17.60248710002101,78.12659505732758],
[17.602500076238943,78.12659611324607],
[17.60251565139546,78.12659794730756],
[17.602527707517762,78.12659991751586],
[17.602542236904842,78.12660309651186],
[17.60255344733268,78.12660640119648],
[17.602566996121716,78.12661187826421],
[17.602577430887056,78.1266173015312],
[17.602589772813506,78.12662553580182],
[17.602599119857423,78.12663300710345],
[17.60261000820949,78.12664357946821],
[17.60261832800104,78.12665261769817],
[17.602627672534226,78.12666502970151],
[17.602634712414375,78.1266753958173],
[17.60264314395975,78.12668851613468],
[17.60264935448084,78.12669947315247],
[17.602657042837073,78.12671309609217],
[17.602662826076493,78.12672422695348],
[17.602669527525343,78.12673836543838],
[17.60267462308897,78.12674980309279],
[17.60268052383167,78.1267642752653],
[17.602685092313084,78.12677597042695],
[17.602690234191282,78.12679073813395],
[17.602693986051207,78.12680265980818],
[17.602698344974403,78.12681769979608],
[17.602701280749024,78.12682982900422],
[17.602704441131106,78.12684519268475],
[17.602706547500087,78.12685754862345],
[17.602708577861712,78.12687314493607],
[17.602709906044105,78.12688572928334],
[17.60271088383959,78.12690160102859],
[17.60271119047953,78.12691432556659],
[17.60271073280504,78.1269302637008],
[17.602709805888363,78.1269430079448],
[17.602707903860523,78.12695890848029],
[17.602705755763985,78.12697150131451],
[17.602702364046653,78.12698712081256],
[17.602698062690077,78.12700249749733],
[17.602694009316586,78.12701471486442],
[17.602687968687004,78.1270295550725],
[17.602682441628296,78.12704103343161],
[17.602674648073837,78.12705499410664],
[17.602667734394938,78.12706570470445],
[17.602658090366717,78.12707827445296],
[17.60264979151605,78.12708784885744],
[17.602638470233178,78.12709872873302],
[17.602628874723738,78.12710665100951],
[17.602615783904557,78.12711538640633],
[17.60260486841505,78.12712108366526],
[17.602590820288366,78.12712637340088],
[17.60257929753106,78.12712902217486],
[17.602565121407665,78.12713093911006],
[17.60255398266437,78.12713107893208],
[17.602540377983228,78.12713061231905],
[17.602529799980726,78.12712984023636],
[17.602516978896087,78.12712877118354],
[17.60250699325915,78.12712796034319],
[17.602494940174015,78.12712690411485],
[17.60248557865422,78.12712609967726],
[17.602474365229188,78.12712502717477],
[17.602466044666084,78.12712429626973],
[17.602456720056132,78.12712344921978],
[17.60245008684073,78.12712282640977],
[17.602442895784293,78.12712217555065],
[17.602441577302223,78.12712202122832]]
wp = 0
message = "A,N,0,0,0,0,0,0,0,0,0\r\n" 
obj.send_data(message)
feed = obj.receive_data()
print(feed)
message = "A,N,0,1,100,0,0,0,0,0,0\r\n"
obj.send_data(message)
time.sleep(1)
feed = obj.receive_data()
print(feed)
message = "A,D,0,0,0,0,0,0,0,0,0 \r\n"
obj.send_data(message)
feed = obj.receive_data()
print(feed)
while not rospy.is_shutdown():
    try:
        if (wp == len(waypoints)-1):
            steer_angle = 0
            obj.send_data('A,D,0,1,100,0,0,0,0,0,0\r\n')
            obj.send_data('A,D,0,1,100,0,0,0,0,0,0\r\n')
            obj.send_data('A,N,0,1,100,0,0,0,0,0,0\r\n')
            time.sleep(1)
            break
        print("waypoint index =============== ",wp)
        position = [float(lat), float(lng)]
        # print("Current position = ", position)
        if ((np.linalg.norm(np.array(position) - waypoints[len(waypoints) - 1]) * wp_threshold) > 1):
            steer_angle = calculate_steer_angle(position, wp, heading)
            print("Steer Angle: ",steer_angle)
            steering_feedback = obj.receive_data().split(',')[2]
            # print("steer Feedback: ",steering_feedback)
            velocity_feedback= obj.receive_data().split(',')[3]
            print("velocity_feedback: ",velocity_feedback)
            steer_rate = pid_controller.update(steer_angle, float(steering_feedback))
            # print("Steer Rate: ",steer_rate)
            # if int(velocity_feedback)> 11:
            #     obj.send_data("A,D,0,1,10,1,"+str(steer_rate)+",0,0,0,0\r\n")
                
            if (steer_angle)>110:
                if int(velocity_feedback)> 7:
                    obj.send_data("A,D,0,1,15,1,"+str(steer_rate)+",0,0,0,0\r\n")
                else:
                    obj.send_data("A,D,7,0,0,1,"+str(steer_rate)+",0,0,0,0\r\n")
            elif int(velocity_feedback)> 10:
                obj.send_data("A,D,0,1,8,1,"+str(steer_rate)+",0,0,0,0\r\n")
            else:
                obj.send_data("A,D,8,0,0,1,"+str(steer_rate)+",0,0,0,0\r\n")
                   
            #time.sleep(1)
            if (wp < len(waypoints) and ((np.linalg.norm(np.array(position) - waypoints[wp]) * wp_threshold) < 6)):
                wp = wp + 1
                # print("\n Waypoint Number : ",wp)
        # else:
        #     steer_angle = 0
        #     obj.send_data('A,N,0,1,90,0,0,0,0,0,0\r\n')
        #     time.sleep(1)
        #     obj.send_data('A,N,O,1,90,0,0,0,0,0,0\r\n')
            
    except rospy.ROSInterruptException:
        pass
