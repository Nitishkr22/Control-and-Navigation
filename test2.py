import numpy as np
from novatel_oem7_msgs.msg import BESTPOS
from novatel_oem7_msgs.msg import BESTVEL
from novatel_oem7_msgs.msg import INSPVA
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from std_msgs.msg import Bool
import actuator
import math
import pid
import time
from geopy.distance import geodesic
import controller
import controller2
import threading


#TCP connection
obj = actuator.controller("169.254.178.227",5001)
obj.connect()

#PID controller
# Kp = 1.65971
# Ki = 0.00007
# Kd = 0.710
# rate_min = -100
# rate_max = 100
# pid_controller = pid.PIDController(Kp, Ki, Kd, rate_min, rate_max)

velg = 0.0
vsub = 0.0
acc_switch = False
brake_switch = False
abs_vel = 0.0

def timeout():
    global abs_vel, acc_switch, brake_switch
    abs_vel = 0.0
    acc_switch = False
    brake_switch = False


def callback_vel(data):

    global velg
    velg=data.hor_speed  

def callback_vsub(data):

    global vsub
    vsub=data.data  

def callback_acc_switch(data):
    global acc_switch, timer
    acc_switch = data.data
    if timer is not None:
        timer.cancel()
    timer = threading.Timer(1.5, timeout)
    timer.start()



def callback_brake_switch(data):
    global brake_switch, timer
    brake_switch = data.data
    if timer is not None:
        timer.cancel()
    timer = threading.Timer(1.5, timeout)
    timer.start()

def callback_abs_velx(data):
    global abs_vel
    abs_vel = data.data
    # print("dddddddddddd", aeb_flag2)
    # if timer is not None:
    #     timer.cancel()
    # timer = threading.Timer(3, timeout)
    # timer.start()


rospy.init_node('Navigation', anonymous=True)
#ROS subscription
rospy.Subscriber("/novatel/oem7/bestvel",BESTVEL, callback_vel)
rospy.Subscriber("/user_value",Float32, callback_vsub)
rospy.Subscriber("/acc_switch_topic",Bool, callback_acc_switch)
rospy.Subscriber("/brake_action_topic",Bool, callback_brake_switch)
rospy.Subscriber("/f_vabsx_topic",Float64, callback_abs_velx)
timer = threading.Timer(3, timeout)

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


#file_path = 'output.txt'
#waypoints = load_waypoints(file_path)
waypoints = [[17.60135163641201,78.12661258003051],
[17.601355458313147,78.12661257449672],
[17.601357512241233,78.12661257372861],
[17.601361843169034,78.12661259158386],
[17.60136411148111,78.12661257811783],
[17.601368795615645,78.12661257250943],
[17.60137120097273,78.12661257953067],
[17.601376171565466,78.12661253102512],
[17.601378703655357,78.12661249883365],
[17.601383919845006,78.1266125001826],
[17.601386585319116,78.12661249120424],
[17.601392004856866,78.12661242106688],
[17.601394757989667,78.1266123772143],
[17.60140035922099,78.1266123628672],
[17.601403228201043,78.12661233258356],
[17.60140902936565,78.12661221510922],
[17.60141201922073,78.12661216303326],
[17.6014180077853,78.12661206013986],
[17.601421044586882,78.12661203066693],
[17.601427200702737,78.12661193429507],
[17.60143031777262,78.1266118937938],
[17.60143659907157,78.12661182846975],
[17.60143981736518,78.12661177296175],
[17.601446356786205,78.12661166998997],
[17.601449599330017,78.12661164341722],
[17.60145605090792,78.12661168001812],
[17.601459273805478,78.12661169904456],
[17.601465706595125,78.12661157081888],
[17.601468876968283,78.12661148741175],
[17.601475341541523,78.12661135756123],
[17.601478531646745,78.12661138908356],
[17.601484780971706,78.12661134107704],
[17.601487846777975,78.1266112395243],
[17.601493891133522,78.12661106229409],
[17.60149983814689,78.12661105707156],
[17.601502746177697,78.12661104616255],
[17.601508410721035,78.12661092351586],
[17.601511169643207,78.12661085806289],
[17.601516590496974,78.12661076518812],
[17.601519270608755,78.1266107106208],
[17.60152458990055,78.12661063451968],
[17.601527280249556,78.12661063672778],
[17.60153278783414,78.12661059992985],
[17.601535582156206,78.1266105972634],
[17.6015412799356,78.12661053367954],
[17.60154422277941,78.12661050416914],
[17.601550187188522,78.12661046180033],
[17.60155323135607,78.12661042396576],
[17.60155939594237,78.1266104311939],
[17.60156252748249,78.12661040006664],
[17.601565661379393,78.12661039410376],
[17.601571907290147,78.12661034100672],
[17.60157813909332,78.12661028783644],
[17.601581251880347,78.12661024031104],
[17.601587442463224,78.12661014147623],
[17.601590521231724,78.1266101161722],
[17.601596654225112,78.12661001367238],
[17.601599706513746,78.12660994759835],
[17.601605707137473,78.12660992936173],
[17.601608725569843,78.12660987296279],
[17.601614717213852,78.12660978796966],
[17.601617679843518,78.12660974793775],
[17.601623560256968,78.12660970032341],
[17.601626518123272,78.12660966485915],
[17.601632373703303,78.12660956541667],
[17.60163529010327,78.12660953844006],
[17.601641118743768,78.12660948960671],
[17.60164404697496,78.12660947796677],
[17.601649875009755,78.12660940546792],
[17.601652801236945,78.12660935485876],
[17.601658688385292,78.12660930053664],
[17.601661648369053,78.12660927689309],
[17.60166759948178,78.12660924713036],
[17.601670598433422,78.12660927399658],
[17.601676643724417,78.12660923764844],
[17.601679691705368,78.12660922350314],
[17.601685827255434,78.12660919796264],
[17.601688927364258,78.12660917245219],
[17.601695190528776,78.12660913768426],
[17.601698355093223,78.1266090809153],
[17.601704752641936,78.1266090692194],
[17.601707991610414,78.1266090325544],
[17.601714559799767,78.12660901404223],
[17.601717891549963,78.12660898861395],
[17.601724600322214,78.12660895354047],
[17.601727984031044,78.12660895663578],
[17.601734860737,78.12660890234704],
[17.601738326384112,78.12660889195072],
[17.601745309767722,78.12660882524318],
[17.601748819612308,78.12660880709903],
[17.601755847263686,78.12660882867624],
[17.60175934471536,78.12660873593991],
[17.601766399867426,78.12660871514636],
[17.601769933147864,78.12660872656944],
[17.601776963386836,78.12660873011316],
[17.601780467067282,78.12660873680015],
[17.601787413327564,78.12660875975267],
[17.601790885631836,78.12660876988556],
[17.60179784147917,78.12660894055513],
[17.601801263353572,78.12660893844499],
[17.601808133940718,78.126608973853],
[17.601811537476294,78.1266089232482],
[17.601818415125006,78.12660886862372],
[17.601821871216025,78.12660889506122],
[17.60182875382984,78.12660893880127],
[17.60183571959743,78.12660891405994],
[17.601839163516036,78.12660895237339],
[17.60184603325852,78.12660902777094],
[17.601849497398632,78.12660899174284],
[17.60185647740804,78.12660895477897],
[17.601859951060046,78.12660904906828],
[17.601866964364643,78.12660922672669],
[17.601870488495152,78.12660924270371],
[17.601877467486183,78.12660916035348],
[17.60188095016314,78.12660918753558],
[17.601887966956266,78.12660925476526],
[17.60189145783028,78.126609285739],
[17.601898359355516,78.12660931592333],
[17.601901801174574,78.12660928302698],
[17.60190870335209,78.12660924283989],
[17.601912137323804,78.12660923268695],
[17.60191896385417,78.12660923089666],
[17.60192236961403,78.12660921456587],
[17.601929145417078,78.1266091525354],
[17.601932502548053,78.12660914779049],
[17.60193922448317,78.12660915436513],
[17.60194256211853,78.12660912839392],
[17.601949207153112,78.12660905116715],
[17.601952535130955,78.12660900694898],
[17.601959179001568,78.12660897993678],
[17.601962503210267,78.12660895658627],
[17.601969152554496,78.12660887135256],
[17.601972475742134,78.12660885752125],
[17.601979164779927,78.12660879306686],
[17.601982516946183,78.1266087691816],
[17.601989211038273,78.12660868894925],
[17.601992574796547,78.12660867698722],
[17.601999310670212,78.12660862786113],
[17.60200268858218,78.12660859294078],
[17.60200941080499,78.126608404967],
[17.60201277002771,78.12660834830324],
[17.602019529780428,78.1266082607733],
[17.602022915771567,78.12660820986373],
[17.60202971950895,78.12660812062242],
[17.602033113753553,78.12660801224376],
[17.602039949644812,78.12660785187435],
[17.602043376473095,78.12660782881953],
[17.60205017977758,78.12660774399507],
[17.602053627526363,78.12660766793009],
[17.602060514262103,78.126607568363],
[17.602063945735846,78.12660750566181],
[17.602070842885023,78.12660735179591],
[17.602074293518278,78.1266072745427],
[17.602081114502653,78.12660719217027],
[17.60208452432977,78.12660717669026],
[17.602091367731322,78.12660712489371],
[17.60209475780438,78.12660704636495],
[17.602101588999705,78.12660693728714],
[17.602105072510575,78.12660691612344],
[17.602111915333715,78.12660678995809],
[17.602115345250656,78.12660671635726],
[17.602122194752482,78.12660664161594],
[17.60212559798622,78.12660658964774],
[17.602132447732853,78.12660639785447],
[17.602135877678258,78.12660631904927],
[17.60214274423391,78.12660624139242],
[17.602146168062852,78.12660617004741],
[17.60215301870911,78.1266060235927],
[17.602156449747206,78.12660598078958],
[17.602163289905054,78.12660588254711],
[17.602166706773087,78.12660579128865],
[17.602173510272316,78.12660559842287],
[17.602180329189874,78.12660549677497],
[17.60218371200246,78.12660541906759],
[17.602190505621312,78.12660517927252],
[17.602193906562455,78.12660507972893],
[17.602197301771373,78.12660498741089],
[17.602204032964643,78.12660478679399],
[17.602210797177474,78.12660462956987],
[17.602214158245676,78.12660453965407],
[17.60222086455735,78.1266043675504],
[17.602224218794333,78.12660429266793],
[17.602230917837588,78.12660414605531],
[17.602234265090402,78.12660405830512],
[17.60224094286579,78.12660387905837],
[17.602244285315386,78.12660380935212],
[17.602250966313388,78.12660367008256],
[17.60225429993606,78.1266035813008],
[17.60226094452233,78.12660340723865],
[17.602264263391156,78.12660332212323],
[17.602270923191995,78.12660319693015],
[17.602274251479493,78.12660310050681],
[17.602280933570064,78.12660295962637],
[17.60228427763348,78.1266028688558],
[17.60229095852103,78.12660272401354],
[17.60229428877695,78.12660265598635],
[17.602300967818604,78.12660253448352],
[17.602304298577064,78.1266024733372],
[17.602310845028562,78.12660227913742],
[17.602314087128303,78.12660218815576],
[17.602320441461934,78.12660206717905],
[17.60232352367201,78.12660202432224],
[17.602329609440634,78.12660184844387],
[17.602332616937613,78.12660174715774],
[17.602338540102714,78.12660160340096],
[17.602341493644133,78.12660155171463],
[17.60234744899696,78.12660140644151],
[17.60235045954917,78.12660131648526],
[17.60235654420526,78.12660117502323],
[17.602359613531927,78.12660110633205],
[17.602365869087627,78.1266009486074],
[17.602368998583696,78.1266009236764],
[17.60237537863935,78.12660076752464],
[17.60237857367457,78.12660069837567],
[17.602385077918644,78.12660052003142],
[17.602388324426094,78.12660045927304],
[17.602394882245125,78.12660034131453],
[17.60239814748306,78.12660028664737],
[17.60240471993248,78.12660015656475],
[17.602407995057007,78.12660009043282],
[17.602414540844375,78.12659996147856],
[17.602417801163668,78.12659988915846],
[17.602424294916876,78.12659975073221],
[17.602427494469264,78.12659971036676],
[17.60243377088367,78.1265996051465],
[17.60243684583093,78.1265995134064],
[17.60244287983133,78.12659932822115],
[17.602445874051888,78.1265992672989],
[17.602451812798524,78.12659915440271],
[17.602454767215058,78.12659906918181],
[17.602460668125307,78.12659891163536],
[17.60246361670228,78.12659883240595],
[17.602469550724848,78.12659869482586],
[17.602472529075268,78.12659862163626],
[17.602478497294612,78.12659844768355],
[17.602481493668805,78.12659838436176],
[17.602487501119885,78.12659825118],
[17.602490537212496,78.1265981548689],
[17.602496575327972,78.12659801861334],
[17.60249959441454,78.12659794948505],
[17.60250563938789,78.12659780884209],
[17.602511652490044,78.1265977334839],
[17.60251464220389,78.1265976916081],
[17.602520636021488,78.1265976889784],
[17.60252358332567,78.12659774725276],
[17.60252950935676,78.12659794542152],
[17.60253244689189,78.1265981201065],
[17.602538334065876,78.12659845258294],
[17.602541237043077,78.12659868805557],
[17.602544145761883,78.12659896260402],
[17.602549896991842,78.12659964895032],
[17.602555628506373,78.12660041795199],
[17.602558463806467,78.12660084635971],
[17.60256413624504,78.12660188835166],
[17.60256696523544,78.12660253128006],
[17.602572528737454,78.12660394173173],
[17.602575298991358,78.1266046913777],
[17.60258078842044,78.12660628067802],
[17.60258348845475,78.12660716346602],
[17.60258882365909,78.1266091404398],
[17.602591488858117,78.12661008534464],
[17.60259679816892,78.12661219797849],
[17.602599425487917,78.12661329073474],
[17.602604602382943,78.12661562468061],
[17.602607176050235,78.12661690072964],
[17.602612291826272,78.12661948905442],
[17.602614806879668,78.12662085894927],
[17.60261985181977,78.1266236467953],
[17.602622365393252,78.12662511088601],
[17.60262721773385,78.1266282668299],
[17.602629591973997,78.12662987836728],
[17.60263432799163,78.12663320043856],
[17.602636652204826,78.12663492127963],
[17.60264115243192,78.1266385953364],
[17.602643411858775,78.12664058410274],
[17.602647830264566,78.1266444545715],
[17.602650024447083,78.1266464539473],
[17.60265419963735,78.12665061365485],
[17.602656219786844,78.12665279290924],
[17.602660136405152,78.12665715263972],
[17.602662056661735,78.1266593399143],
[17.602665788729816,78.12666387965426],
[17.60266760371752,78.12666618613524],
[17.602671072043456,78.12667095015101],
[17.602672770060583,78.12667334401327],
[17.602676088596972,78.12667821444586],
[17.60267772870801,78.1266806344689],
[17.60268099886425,78.1266854652199],
[17.60268261159258,78.12668787438814],
[17.602685754901138,78.12669275328993],
[17.602687303131972,78.12669522104335],
[17.602690364256713,78.12670013453553],
[17.602691922970298,78.12670256086804],
[17.60269505854496,78.12670732721115],
[17.602696583042885,78.12670971814792],
[17.602699602721138,78.12671449403949],
[17.602701111467432,78.1267168451555],
[17.602704101393506,78.12672154896865],
[17.602705555962316,78.12672393144389],
[17.60270844536697,78.12672867364698],
[17.602709850964235,78.1267310908973],
[17.602712675417433,78.12673579281108],
[17.60271405532789,78.12673817597623],
[17.60271673464506,78.12674289333869],
[17.60271803571718,78.12674527663354],
[17.602720579860435,78.12674997698703],
[17.602723053272456,78.1267547099417],
[17.602724258273504,78.12675706678824],
[17.602726574244564,78.12676180926091],
[17.602727734129395,78.12676417039981],
[17.60272997162489,78.12676889865205],
[17.602731053078827,78.12677127087126],
[17.602733152533144,78.12677595578343],
[17.602734193944947,78.12677831470003],
[17.602736159928366,78.12678308982152],
[17.602737110403886,78.12678549624371],
[17.602738999549306,78.12679025092908],
[17.602739942084007,78.12679263443592],
[17.602741767221747,78.12679742827206],
[17.602742659148024,78.1267998591293],
[17.602744368463632,78.12680482636789],
[17.602745197165326,78.126807342993],
[17.602746878651025,78.12681249465804],
[17.602747723207237,78.12681510966618],
[17.602749355480753,78.12682044372212],
[17.602750130266468,78.12682317370545],
[17.602751655224694,78.12682872292598],
[17.60275246368737,78.12683165297261],
[17.602753857809045,78.12683738609148],
[17.602754552891298,78.12684028886089],
[17.60275583916488,78.12684619350748],
[17.602756458571694,78.12684919360925],
[17.602757574080368,78.1268552747549],
[17.60275807592466,78.12685836884489],
[17.602759024192963,78.12686459282484],
[17.602759449622585,78.12686774999361],
[17.602760194853047,78.12687406079978],
[17.602760472791726,78.1268772430452],
[17.602760961488933,78.1268836070637],
[17.602761137911916,78.12688679223481],
[17.602761421610303,78.12689318137022],
[17.602761496565915,78.1268963699651],
[17.602761384376475,78.12690260720493],
[17.602761371412086,78.12690567278402],
[17.602761052413975,78.12691170391854],
[17.602760840464004,78.12691464716504],
[17.602760350844363,78.12692037599945],
[17.602760075161715,78.1269231838907],
[17.60275940957817,78.126928651898],
[17.6027589785317,78.12693129173562],
[17.60275811462131,78.12693643572068],
[17.60275763837514,78.12693893376029],
[17.60275663376855,78.12694378498622],
[17.602756093648686,78.12694614393124],
[17.60275492633109,78.12695077620913],
[17.602754292827527,78.12695305629576],
[17.6027529672236,78.12695761557168],
[17.602752255832677,78.12695990205944],
[17.602750768134662,78.12696451031664],
[17.602749955517357,78.12696683644529],
[17.602748209066075,78.12697149348597],
[17.602747275114663,78.12697384296486],
[17.60274531393976,78.12697856056587],
[17.60274427523649,78.12698093769006],
[17.602742035123256,78.12698566993265],
[17.60274085879606,78.1269880374065],
[17.602738412774208,78.12699279189155],
[17.60273581594007,78.12699757990605],
[17.60273443508517,78.12699998089057],
[17.602731579757126,78.12700479010935],
[17.602730084782056,78.12700720422926],
[17.60272702776822,78.12701203161032],
[17.60272545761192,78.12701442741786],
[17.60272219275787,78.12701921771762],
[17.602720504062344,78.1270215965272],
[17.602718802168546,78.12702398874727],
[17.602715300050697,78.12702878621634],
[17.602711664142337,78.1270335744946],
[17.602709784643938,78.12703594530964],
[17.60270596221717,78.12704071911644],
[17.602704073814234,78.12704317596064],
[17.602700257311586,78.12704807644886],
[17.6026982780318,78.12705051656725],
[17.60269428247331,78.12705543427747],
[17.602692317499194,78.12705791844363],
[17.602688405523576,78.12706294940182],
[17.602686408680196,78.12706548928924],
[17.602682374352725,78.12707047459925],
[17.60268039678723,78.12707292003259],
[17.602676510231372,78.12707778266311],
[17.602674596713975,78.1270801772914],
[17.602670796275827,78.12708476942954],
[17.602668884862865,78.1270870346702],
[17.60266510922709,78.1270914294974],
[17.602663206756432,78.12709359114643],
[17.602659384269128,78.1270977520205],
[17.60265743028532,78.12709979151494],
[17.602653577326645,78.12710392316087],
[17.602651623135536,78.1271059430032],
[17.602647662555306,78.12710999146967],
[17.602645612465526,78.12711198046189],
[17.602641189156582,78.12711578513832],
[17.602638939549713,78.1271176709006],
[17.602634388483523,78.12712137974582],
[17.602631998468993,78.1271231138413],
[17.602627149055305,78.1271265174091],
[17.60249799993328,78.12714503502743],
[17.602491659419258,78.12714456704698],
[17.602488452164902,78.1271443108303],
[17.602482115347392,78.12714372540403],
[17.602478859765124,78.12714340758889],
[17.602472410343626,78.12714282469918],
[17.60246917754522,78.12714256749342],
[17.60246267132433,78.12714189413983],
[17.602459410130738,78.12714156535041],
[17.60245290984057,78.12714094806799],
[17.602449613062404,78.12714061441929],
[17.60244304713249,78.12714004334889],
[17.602439772561535,78.12713970274093],
[17.602433098091538,78.1271391118092],
[17.60242978833828,78.12713880671295],
[17.60242317562181,78.1271381856129],
[17.602419781118687,78.12713791492345],
[17.602413076647203,78.12713733426398],
[17.602409691491644,78.1271370275878],
[17.602402926401066,78.12713632718746],
[17.602399543530456,78.12713605321402],
[17.602392726628352,78.12713549624203],
[17.602389242033606,78.12713517009459],
[17.60238233098775,78.1271345072376],
[17.60237886620037,78.12713421208623],
[17.60237182739928,78.12713356112091],
[17.602368313089404,78.1271332657817],
[17.602361208200914,78.12713272558092],
[17.60235410434983,78.1271320258819],
[17.60235050734622,78.12713169919682],
[17.60234689786447,78.12713139766205],
[17.602339635746862,78.12713082922014],
[17.602332267450205,78.12713018786899],
[17.602328560692836,78.12712985218799],
[17.602321177902315,78.12712919688033],
[17.602317463074577,78.12712885857167],
[17.602309969388614,78.12712830213714],
[17.602306207097726,78.12712800461422],
[17.60229859065312,78.12712739358648],
[17.602294778236732,78.12712706303796],
[17.602287059696273,78.127126495889],
[17.60228320507203,78.12712617599544],
[17.602275408487493,78.12712560496959],
[17.60227149259816,78.12712528249368],
[17.602263678440064,78.12712463750523],
[17.602259721591413,78.12712434787657],
[17.602251824356088,78.12712378925919],
[17.602247861690568,78.12712344994044],
[17.602239862200804,78.1271227698923],
[17.60223581199078,78.12712246067278],
[17.602227729051798,78.12712179497625],
[17.60222368533642,78.12712145611445],
[17.602215579174036,78.12712078658554],
[17.602211511430383,78.12712040566208],
[17.602203375512257,78.1271196424426],
[17.60219929209042,78.12711930397944],
[17.6021910822816,78.12711859837589],
[17.60218695236414,78.12711822495108],
[17.60217871136246,78.12711745587366],
[17.602174570916205,78.12711710427496],
[17.60216624824303,78.1271163624365],
[17.60216209186003,78.1271159796631],
[17.60215374894079,78.12711513248071],
[17.6021495721333,78.12711468769257],
[17.602141197919508,78.12711366547575],
[17.60213701822653,78.12711314178496],
[17.602128673174725,78.12711187146125],
[17.602124487798058,78.12711119156259],
[17.60211617794707,78.127109773316],
[17.60211197208015,78.12710902431196],
[17.60210360385651,78.12710750931964],
[17.602095258720674,78.12710577607425],
[17.602091099992556,78.12710481683142],
[17.60208693220444,78.12710392467429],
[17.602078529197684,78.12710211539658],
[17.602070194406725,78.12710017492817],
[17.602066010861044,78.12709915347445],
[17.60205765899161,78.12709721251868],
[17.60205340781403,78.12709622750025],
[17.602045009511308,78.12709430634925],
[17.60204081900782,78.12709335850145],
[17.602032570157927,78.12709144884917],
[17.602028488896366,78.1270905181232],
[17.60202044417866,78.12708886687385],
[17.60201651504196,78.12708804735672],
[17.60200877553395,78.12708644758054],
[17.602005013471196,78.12708568934474],
[17.60199758337999,78.12708423441431],
[17.6019939597152,78.1270835113095],
[17.60198691198103,78.12708211952973],
[17.60198352353246,78.1270814900328],
[17.601976854752678,78.12708029174185],
[17.60197353488796,78.12707967756916],
[17.60196713013467,78.12707845773711],
[17.6019640111934,78.12707791482181],
[17.60195789003335,78.12707684956081],
[17.601954982831465,78.12707634266744],
[17.60194928847886,78.12707527277512],
[17.601946517333065,78.1270747953295],
[17.60194101479597,78.12707405086365],
[17.60193837869456,78.12707366524235],
[17.60193324891777,78.12707287934018],
[17.601930763543873,78.12707248662596],
[17.601925920616058,78.12707187985781],
[17.60192355150712,78.12707161708218],
[17.60191896463189,78.12707101124238],
[17.601916755070178,78.12707076461867],
[17.60191241710185,78.12707031296074],
[17.601910359542984,78.12707005507873],
[17.6019062897533,78.12706962382221],
[17.601904332260307,78.12706942076376],
[17.601900405349692,78.12706908910037],
[17.601898468988352,78.12706890924397],
[17.601894597294656,78.12706862336898],
[17.601892656860077,78.12706846311343],
[17.601888713312604,78.1270682065442],
[17.601886723275566,78.12706809938182],
[17.601882674214465,78.1270678806108],
[17.601880620818182,78.1270677621153],
[17.601876416298477,78.12706750036743],
[17.60187425757331,78.1270673899238],
[17.601869829663492,78.1270671679929],
[17.601867568993356,78.12706706665881],
[17.60186292864409,78.12706681938327],
[17.601860525757147,78.12706672651645],
[17.601855576940522,78.12706656069336],
[17.601852975755385,78.12706642674533],
[17.60184758613194,78.12706620391432],
[17.601841918887043,78.12706601394864],
[17.601838978129788,78.12706590309874],
[17.601832826928522,78.12706573075165],
[17.60182964720745,78.12706567512026],
[17.60182312778782,78.12706561000134],
[17.60181975390872,78.1270655334581],
[17.601812853009026,78.12706540845241],
[17.601809340209005,78.12706541833161],
[17.60180218691364,78.12706536913531],
[17.601798572265178,78.1270653443976],
[17.601791288056912,78.1270652474155],
[17.601787657974462,78.12706527531866],
[17.601780367647088,78.12706535138683],
[17.601776729123536,78.12706534701914],
[17.601769586551878,78.12706535097658],
[17.601766102102204,78.12706538142717],
[17.60175920075148,78.12706546123025],
[17.60175582786341,78.12706547602349],
[17.601749147890217,78.12706559308391],
[17.601745793918248,78.12706565885307],
[17.601739083807033,78.1270657461781],
[17.60173575186125,78.12706577965415],
[17.6017290560272,78.12706582722443],
[17.60172572337736,78.1270658614568],
[17.601719025955077,78.12706594549286],
[17.6017156730687,78.12706596628992],
[17.60170895358869,78.12706600827188],
[17.6017055739368,78.12706603795621],
[17.601698852470783,78.12706612196929],
[17.601695497487107,78.12706615245438],
[17.60168887361481,78.12706618294037],
[17.601685610440228,78.12706626074107],
[17.6016790985575,78.12706626486691],
[17.60167588418116,78.12706628373218],
[17.60166960777942,78.12706633770863],
[17.601666498774872,78.12706637727901],
[17.601660388464037,78.12706630763584],
[17.601657313803226,78.12706628449762],
[17.60165110510298,78.12706625650728],
[17.601647996163475,78.12706622351793],
[17.601641710007627,78.12706613917209],
[17.601638523558243,78.12706611484568],
[17.601632130751906,78.1270660335413],
[17.6016289124774,78.12706597983673],
[17.60162249564145,78.1270658620639],
[17.60161923795755,78.1270657989994],
[17.60161277652525,78.12706568568575],
[17.60160951689137,78.12706564207608],
[17.601603047389506,78.12706552975273],
[17.601599796449932,78.12706545676602],
[17.601593276556407,78.12706527292683],
[17.60159001416364,78.12706514428022],
[17.601583457897732,78.1270648440872],
[17.601576921440405,78.1270646409733],
[17.601573651957388,78.12706449347208]]
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
message = "A,N,0,0,0,0,0,0,0,0,0 \r\n"
obj.send_data(message)
feed = obj.receive_data()
print(feed)

def accn(kp, acc):
    return acc+kp
def dccn(kp, acc):
    return 2*(acc-kp)

def damping(error):
    kp = 1-np.exp(-0.0006*abs(error))
    return kp
desired_vel = 6.0
acc = 1
# cont = controller.Controller2D()
obj.send_data("A1,D,1,0,0,0,0,0,0,0,0\r\n")
time.sleep(1)

kp = 0.00008
ki = 0.0006
kd = 0.28

pid_controller = controller2.PIDControllervel(kp, ki, kd)

max_dist = 20.0
min_dist = 8.0
ego_speed = 9.0
# acc_switch = False
brake_rate = 0

# setpoint = 10.0
# while not rospy.is_shutdown():
while not rospy.is_shutdown():
    try:
        velocity_feedback = float(obj.receive_data().split(',')[3]) #velocity feedback from controller
        # velocity_feedback =  np.clip(1, 0, 100)
        print("Ego_velocity: ",velg*(18/5))  # velg is the velocity from gnss 
        vel_gnss = velg*(18/5)
        # print("adfd: ",vsub)   # vsub is the desired velocity
        # setpoint = vsub
        # control_signal = pid_controller.compute(setpoint, vel_gnss)  # (desired_vel, feedback)

        # throttle_input1 = np.clip(control_signal, 0, 100)
        # print("throttle: ",throttle_input1)

        if acc_switch:
            print("ACC is ON")
            setpoint = abs_vel
            control_signal = pid_controller.compute(setpoint, vel_gnss)  # (desired_vel, feedback)

            throttle_input1 = np.clip(control_signal, 0, 19)
            print("throttle: ",throttle_input1)
            
            print("velocity from radar: ",abs_vel)
            if(vel_gnss>abs_vel):
                # obj.send_data("A1,D,2,0,0,0,0,0,0,0,0\r\n")
                throttle = 1.0
                if brake_switch:
                    print("brake applied")
                    brake_rate = 20
                else:
                    print("NO brake")
                    brake_rate = 0
            else:
                
                throttle = throttle_input1
        else:
            print("ACC is OFF")
            setpoint = ego_speed
            control_signal = pid_controller.compute(setpoint, vel_gnss)  # (desired_vel, feedback)

            throttle_input1 = np.clip(control_signal, 0, 100)
            print("throttle: ",throttle_input1)
            if(vel_gnss>ego_speed):
                throttle = 1.0
            else:
                
                throttle = throttle_input1
            
        
        # if(vel_gnss>setpoint):
        #     # obj.send_data("A1,D,2,0,0,0,0,0,0,0,0\r\n")
        #     throttle_input = 2
        # else:
        #     throttle_input = throttle_input1
            
            # obj.send_data("A1,D,"+str(throttle_input)+",0,0,0,0,0,0,0,0\r\n")
                # obj.send_data("A,D,4,1,30,1,"+str(steer_rate)+",0,0,0,0\r\n")

        obj.send_data("A1,D,"+str(throttle)+",1,"+str(brake_rate)+",0,0,0,0,0,0\r\n")
        

    except Exception as e:
        print("Error:", e)