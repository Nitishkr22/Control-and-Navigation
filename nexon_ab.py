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
from std_msgs.msg import Float32
import threading

lat = 0
lng = 0
heading = 0
global aeb_flag
aeb_flag2=2.0
aeb_flag1 = 2.0
wp_threshold = 1.111395e5
rospy.init_node('Navigation', anonymous=True)

#GNSS position
def callback_latlong(data):
    global lat,lng
    lat = data.lat
    lng = data.lon

#GNSS heading
def callback_heading(data):
    global heading
    heading=data.azimuth 

def timeout():
    global aeb_flag2
    aeb_flag2 = 5.0

def callback_aeb1(msg):
	global aeb_flag1
	aeb_flag1 = msg.data

def callback_aeb2(msg):
    global aeb_flag2, timer
    aeb_flag2 = msg.data
    print("dddddddddddd", aeb_flag2)
    if timer is not None:
        timer.cancel()
    timer = threading.Timer(3, timeout)
    timer.start()


#ROS subscription
rospy.Subscriber("/novatel/oem7/bestpos",BESTPOS, callback_latlong)
rospy.Subscriber("/novatel/oem7/inspva",INSPVA, callback_heading)

rospy.Subscriber("/radar_aeb",Float32, callback_aeb1)
rospy.Subscriber("/radar_aeb2",Float32, callback_aeb2)
timer = threading.Timer(3, timeout)
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
################################

# print("Variable stopped incrementing.")
################################3

def calculate_steer_angle(currentLocation, wp, heading):
    """
    This function takes three inputs:
        - currentLocation: a list of two float values representing the current location
        - wp: a waypoint value
        - heading: a heading value
    It then calculates the steer output based on the current location, waypoint, and heading, and returns the steer output value.
    """
    off_y = - currentLocation[0] + waypoints[wp][0]
    off_x = - currentLocation[1] + waypoints[wp][1]

    # calculate bearing based on position error
    bearing_ppc = 90.00 + math.atan2(-off_y, off_x) * 57.2957795 

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

    steer_output = 750 * np.arctan(-1 * 2 * 3.5 * np.sin(np.pi * bearing_diff / 180) / 8)
    
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
waypoints = [[17.60137572229957,78.12661389002157],
[17.60138269129881,78.12661402450182],
[17.601386406212928,78.12661407121823],
[17.601394246805327,78.12661415810655],
[17.60139836362451,78.12661417284212],
[17.601407028718018,78.12661427281324],
[17.601411573824215,78.12661431438508],
[17.601421034703037,78.12661430380604],
[17.6014259862748,78.12661432007732],
[17.60143615559617,78.1266143486897],
[17.60144143575896,78.12661434442073],
[17.601452224565712,78.12661441547175],
[17.601457517320686,78.12661442668868],
[17.60146796227343,78.12661445041421],
[17.60147314708949,78.1266144572627],
[17.601483603845427,78.12661446355841],
[17.601488751688255,78.12661450376831],
[17.60149886811202,78.1266144496236],
[17.60150380830594,78.12661441498561],
[17.601513548183835,78.12661444085697],
[17.60151830307341,78.12661444065995],
[17.60152764293548,78.12661443468393],
[17.601532198125724,78.12661443249154],
[17.601541114129617,78.12661441091468],
[17.60154547075762,78.12661437489764],
[17.601553994786276,78.12661426506801],
[17.601558155852093,78.12661421116185],
[17.601566418393595,78.12661408957733],
[17.60157049528659,78.12661404201641],
[17.601578615756168,78.12661388418098],
[17.601582703028075,78.12661379785102],
[17.601590933647874,78.12661365663939],
[17.601595094277535,78.12661355969935],
[17.601603488674506,78.12661337304395],
[17.601607725761518,78.12661327220505],
[17.601616368496632,78.12661310023778],
[17.601620731309932,78.12661300853686],
[17.601629636616398,78.12661282457273],
[17.601634143249715,78.12661273551109],
[17.601643283435134,78.12661257427554],
[17.601647923191255,78.12661251539389],
[17.60165709845129,78.12661246915958],
[17.601661714536924,78.12661239909197],
[17.601670844669336,78.12661222876606],
[17.601675332333944,78.12661215574461],
[17.60168416246023,78.12661203953287],
[17.601688493942767,78.12661194973182],
[17.601696970998493,78.12661187682285],
[17.601701119184252,78.12661182442197],
[17.60170915186786,78.12661176001055],
[17.60171303998112,78.1266116852914],
[17.601720589834507,78.12661155931286],
[17.601727735938425,78.12661149828953],
[17.60173116831786,78.12661147538131],
[17.601734514545612,78.12661145750758],
[17.601741073494814,78.12661133312056],
[17.601747364755976,78.12661121354343],
[17.60175039587085,78.12661118282077],
[17.60175630607506,78.1266111276234],
[17.60175918945886,78.1266110954626],
[17.60176491388463,78.12661103913335],
[17.601767790672167,78.12661100622287],
[17.601773612710854,78.12661096087201],
[17.601776578850743,78.12661091765267],
[17.60177955436104,78.12661091326956],
[17.601785595077324,78.12661088377139],
[17.601791736830226,78.12661084802511],
[17.601794790408523,78.12661091416801],
[17.60180107739769,78.12661086417832],
[17.601804239150553,78.12661089579667],
[17.60181048009653,78.1266107898055],
[17.60181359090283,78.12661074235761],
[17.601819785037915,78.1266106934616],
[17.601822793664084,78.12661068806386],
[17.601828680737395,78.12661061897968],
[17.60183158416807,78.12661058790343],
[17.60183722889201,78.1266105041542],
[17.601839967729347,78.12661051710043],
[17.601845411502843,78.12661051633955],
[17.601848126589466,78.12661040217041],
[17.60185360163619,78.12661023803916],
[17.60185640038922,78.12661020316295],
[17.60186181898852,78.12661024581584],
[17.601864513941862,78.1266102052322],
[17.601869768524807,78.12661005487996],
[17.60187235787112,78.12660998737734],
[17.60187748336206,78.12660989397871],
[17.60188007364867,78.1266098540428],
[17.601885336675025,78.12660974021057],
[17.60188798494246,78.12660967846818],
[17.60189331589848,78.12660960034938],
[17.60189603534268,78.12660958397014],
[17.60190154145966,78.12660946193675],
[17.601904342505843,78.12660936531151],
[17.601909992456815,78.1266092692129],
[17.601912872404206,78.12660922212066],
[17.601918637794256,78.12660909166809],
[17.601921549807553,78.12660903105757],
[17.601927394557073,78.12660887855893],
[17.601930313393613,78.12660880533154],
[17.601936179242884,78.12660871773416],
[17.601939146877186,78.12660866868913],
[17.601945030989093,78.12660853540372],
[17.601947968742593,78.12660844731862],
[17.60195388635617,78.12660831283537],
[17.601956837713352,78.12660826706555],
[17.601962738920133,78.12660812652585],
[17.60196565670449,78.12660805951714],
[17.60197136804222,78.12660794697248],
[17.601974178582417,78.12660788875323],
[17.601979793209345,78.1266077441117],
[17.601982602103398,78.1266076805287],
[17.601988292567007,78.12660756030391],
[17.60199117388639,78.12660751546267],
[17.60199695202353,78.12660744621961],
[17.601999850757792,78.12660738216657],
[17.602005714293895,78.12660724455583],
[17.602008678858212,78.12660720245157],
[17.602014578596243,78.12660706707915],
[17.602017543212238,78.12660701102745],
[17.602023521988585,78.12660692078741],
[17.602029509621957,78.12660683798018],
[17.60203248772222,78.12660674517981],
[17.60203845220057,78.12660664818792],
[17.602041422991284,78.12660657486799],
[17.602047350661216,78.12660648782739],
[17.602050342952104,78.12660649259891],
[17.602056432502042,78.12660636342297],
[17.602059493503997,78.12660629532337],
[17.602065681802966,78.12660621515217],
[17.602068803955348,78.12660625889431],
[17.602075142382205,78.12660608798531],
[17.602078326084094,78.12660603311994],
[17.60208471574072,78.12660599633314],
[17.602088030881717,78.12660601694662],
[17.60209455693989,78.12660591473364],
[17.602097843784577,78.12660588049913],
[17.60210454143436,78.12660580701349],
[17.602107892633228,78.12660576768695],
[17.60211462762996,78.12660565128594],
[17.602118002701328,78.12660561656364],
[17.602124709854237,78.12660553198953],
[17.60212806455356,78.12660546858079],
[17.602134742729934,78.12660530260159],
[17.60213807179994,78.1266052821323],
[17.602144701927003,78.12660524497603],
[17.602147964856094,78.12660523640633],
[17.602154515844486,78.12660514030635],
[17.602157745147107,78.1266050893218],
[17.60216406806727,78.12660504743458],
[17.60216718062451,78.12660499136138],
[17.602173366069263,78.12660483304091],
[17.602176450314012,78.12660481496812],
[17.602182544372905,78.12660475997778],
[17.602185520876624,78.12660469885193],
[17.602191372978787,78.12660453037455],
[17.602194232004255,78.12660447249947],
[17.602199776119704,78.12660437376125],
[17.602202468970187,78.12660434832675],
[17.60220774585459,78.1266042478974],
[17.602210280382714,78.12660420676252],
[17.602215313638702,78.12660403931716],
[17.60221774907776,78.12660397857395],
[17.602222589079073,78.1266038992555],
[17.60222496579616,78.12660386411002],
[17.60222976793384,78.12660379254721],
[17.602232213140837,78.12660375292141],
[17.602237106294066,78.12660373088083],
[17.60223960092184,78.12660368624088],
[17.60224461597748,78.12660360930852],
[17.602247164247494,78.12660356587642],
[17.60225225972282,78.12660347892367],
[17.602254793965425,78.1266033998852],
[17.602259862142848,78.12660329928066],
[17.602262385766725,78.12660326675342],
[17.60226744660505,78.12660318509188],
[17.602272473797463,78.1266031017088],
[17.602274971944478,78.12660302949551],
[17.602280010167256,78.12660297770519],
[17.60228251306198,78.12660293969476],
[17.602287543283637,78.12660284136584],
[17.60229005279432,78.1266028071936],
[17.60229508991823,78.12660274171517],
[17.60229759361744,78.12660271784812],
[17.60230265867338,78.12660266060414],
[17.60230516146806,78.12660261410228],
[17.60231018300085,78.12660245770749],
[17.602312688747965,78.1266023969773],
[17.602317736275484,78.1266023381361],
[17.602320254661702,78.12660229204552],
[17.602322756142808,78.12660222843697],
[17.602327837546426,78.12660212363608],
[17.602332942994984,78.1266019898865],
[17.6023355149281,78.12660193139929],
[17.602340667467473,78.12660182993311],
[17.602343251985854,78.12660177263726],
[17.602348461017165,78.12660165490944],
[17.602351071298852,78.12660158666766],
[17.60235633880772,78.12660147784015],
[17.602358965451607,78.12660142995671],
[17.60236429179018,78.12660130208981],
[17.602366924388715,78.1266013416251],
[17.60237229100221,78.12660124280612],
[17.602374966326845,78.12660116979386],
[17.60238037274954,78.12660102729802],
[17.602383095660347,78.12660095810303],
[17.602388524039704,78.12660083581876],
[17.602391268457083,78.12660078690745],
[17.60239672992302,78.12660071039078],
[17.602399485518,78.12660064520288],
[17.602404992384344,78.12660053711114],
[17.602407753442698,78.12660050538265],
[17.602413290366837,78.12660039685977],
[17.602416034975732,78.12660033343376],
[17.602421618791006,78.12660022910187],
[17.60242439374984,78.12660019348779],
[17.602429974119826,78.12660013292995],
[17.60243277777819,78.12660009907985],
[17.602438374689395,78.12659997798832],
[17.602441167712925,78.12659992034145],
[17.602446786025123,78.12659982587388],
[17.602449600473296,78.1265997894277],
[17.60245523940277,78.12659969516082],
[17.60245807327324,78.12659962705283],
[17.602463738694865,78.12659953596535],
[17.602466575599802,78.12659948649504],
[17.602472328220617,78.12659930930678],
[17.60247517385804,78.12659924847001],
[17.60248087762987,78.1265991703433],
[17.602483729723264,78.12659914078492],
[17.60248657808715,78.12659910295532],
[17.60248657808715,78.12659910295532],
[17.60248657808715,78.12659910295532],
[17.60248657808715,78.12659910295532],
[17.60248657808715,78.12659910295532],
[17.60248657808715,78.12659910295532],
[17.60248657808715,78.12659910295532],
[17.60248657808715,78.12659910295532],
[17.60248657808715,78.12659910295532],
[17.60248657808715,78.12659910295532],
[17.60248657808715,78.12659910295532],
[17.60248657808715,78.12659910295532]]
wp = 0
message = "A,N,0,0,0,0,0,0,0,0,0\r\n" 
obj.send_data(message)
feed = obj.receive_data()
# print(feed)
message = "A,N,0,1,100,0,0,0,0,0,0\r\n"
obj.send_data(message)
time.sleep(1)
feed = obj.receive_data()
# print(feed)
message = "A,D,0,0,0,0,0,0,0,0,0 \r\n"
obj.send_data(message)
feed = obj.receive_data()
# print(feed)
k = []
i=0
j=0
while not rospy.is_shutdown():
    try:
        if (wp == len(waypoints)-1):
            steer_angle = 0
            obj.send_data('A,D,0,1,100,0,0,0,0,0,0\r\n')
            obj.send_data('A,D,0,1,100,0,0,0,0,0,0\r\n')
            obj.send_data('A,N,0,1,100,0,0,0,0,0,0\r\n')
            time.sleep(1)
            break
        
        print("dfgwwwwwwwwwww: ",aeb_flag2)

        position = [float(lat), float(lng)]
        print("Current position = ", position)
        if ((np.linalg.norm(np.array(position) - waypoints[len(waypoints) - 1]) * wp_threshold) > 1):
            steer_angle = calculate_steer_angle(position, wp, heading)
            print("Steer Angle: ",steer_angle)
            steering_feedback = obj.receive_data().split(',')[2]
            velocity_feedback = obj.receive_data().split(',')[3]
            print("velocity_feedback",velocity_feedback)
            print("steer Feedback: ",steering_feedback)
            steer_rate = pid_controller.update(steer_angle, float(steering_feedback))
            print("Steer Rate: ",steer_rate)
            # print("feedback: ",obj.receive_data())

            ############################# AEB ###############################
            if(aeb_flag1==1.0):
                 obj.send_data("A,D,4,1,30,1,"+str(steer_rate)+",0,0,0,0\r\n")
            elif(aeb_flag1==0.5):
                 obj.send_data("A,D,4,1,50,1,"+str(steer_rate)+",0,0,0,0\r\n")
            elif(aeb_flag1==0.1 or aeb_flag1==1.5 or aeb_flag2==1.5):
                 obj.send_data("A,D,4,1,100,1,"+str(steer_rate)+",0,0,0,0\r\n")
            # elif(aeb_flag1==2.0 or aeb_flag2==2.0 or aeb_flag2==5.0 or aeb_flag1==0.0):
            #     obj.send_data("A,D,7,0,0,1,"+str(steer_rate)+",0,0,0,0\r\n")
            else:
                if int(velocity_feedback) > 9:
                    obj.send_data("A,D,4,1,10,1,"+str(steer_rate)+",0,0,0,0\r\n")
                elif int(velocity_feedback) > 12:
                    obj.send_data("A,D,4,1,20,1,"+str(steer_rate)+",0,0,0,0\r\n")
                else:
                    obj.send_data("A,D,7,0,0,1,"+str(steer_rate)+",0,0,0,0\r\n")

            
            # else:
            #      obj.send_data("A,D,7,0,0,1,"+str(steer_rate)+",0,0,0,0\r\n")
            # print("fffffffffffffffffffffffffff: ", aeb_flag2)
            # aeb_flag2=2.0
            #time.sleep(1)
            print("feedback: ",obj.receive_data())
            if (wp < len(waypoints) and ((np.linalg.norm(np.array(position) - waypoints[wp]) * wp_threshold) <6)):
                wp = wp + 1
        # else:
        #     steer_angle = 0
        #     obj.send_data('A,N,0,1,100,0,0,0,0,0,0\r\r')
        #     obj.send_data('A,N,0,1,100,0,0,0,0,0,0\r\r')
        #     obj.send_data('A,N,0,1,100,0,0,0,0,0,0\r\r')
        #     time.sleep(1)
        # rospy.sleep(0.5)
        
    except rospy.ROSInterruptException:
        # print("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaa")
        pass
