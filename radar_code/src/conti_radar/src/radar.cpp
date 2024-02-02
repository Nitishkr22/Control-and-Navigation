#include "pdk/pdk_interface.hpp"
#include "pdk_RadarDetectionImage.pb.h"
#include "pdk_RadarStatus.pb.h"
#include "pdk_RadarObjectList.pb.h"


#include <map>
#include <cstdio>
#include <vector>
#include <iostream>
#include <string>
#include <thread>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include "conti_radar/radar_msgs.h"

//call back function
ros::Publisher radar_pub;
std::map<int, std::vector<float>> objectmap;

bool sortDistance(pb::PDK::RadarObjectList_RadarObject obj1, pb::PDK::RadarObjectList_RadarObject obj2)
{
	return pow(pow(obj1.f_distx(),2) + pow(obj1.f_disty(),2), 0.5) < pow(pow(obj2.f_distx(),2) + pow(obj2.f_disty(),2), 0.5);
}

void OnReceiveRDI(const std::string &buffer)
{
    //std::vector<float> target_vrel;
    //std::vector<float> target_azi;
    std::vector<pb::PDK::RadarObjectList_RadarObject > objectList;
    pb::PDK::RadarDetectionImage im{};
    im.ParseFromString(buffer);
    pb::PDK::RadarObjectList obj;
    obj.ParseFromString(buffer);
// obj detection size 

    if (obj.a_radarobjectlist_size() == 0 )
    {
       return;
    }

	for(auto radarobject : obj.a_radarobjectlist())
	{
		objectList.push_back(radarobject);
	}
	sort(objectList.begin(), objectList.end(), sortDistance);
	
    //int random_object_nr = std::rand() % static_cast<int>(obj.a_radarobjectlist_size());
    pb::PDK::RadarObjectList_RadarObject ro = objectList[0];
// for (int i = 0; i < im.u_nofdetections(); i++){

//         pb::PDK::RadarDetectionImage_RadarDetection ri = im.a_radardetectionlist(i); 

//         // std::cout << "Object ID: = " << ob.u_objid() << std::endl;
//         // std::cout << "Distance in X:  = " << ob.f_distx() << std::endl;

//         //csvFile << "Column 1,Column 2\n";
//         csvFile << timestampStr  << "," << im.t_header().u_sensorid()<< "," << ri.f_range() << "," << ri.f_vrelrad() << "," << ri.a_azang_hyp(0) << "," <<
//         ri.a_rcs_hyp(0) << "," << ri.f_rangevar() << "," << ri.f_vrelradvar() << "," << ri.f_azangvar() << "," << ri.f_pdh0() << "," << 
//         ri.f_elangvar() << ","  << ri.e_elevationstate() << ","  << ri.u_pdh0flags() << ","<<ri.f_snr()<< std::endl;
//         //csvFile << "3,4\n";
//         //csvFile << "5,6\n";
//     }
//     csvFile.close();



    conti_radar::radar_msgs radar_msg;
    radar_msg.object_ID = ro.u_objid();
    radar_msg.x = ro.f_distx();
    radar_msg.y = ro.f_disty();
    radar_msg.velx = ro.f_vabsx();
    radar_msg.vely = ro.f_vabsy();

    radar_pub.publish(radar_msg);

    
}

// callback fucntion
void OnReceiveStatus(const std::string &buffer)
{
    pb::PDK::RadarStatus rs{};
    rs.ParseFromString(buffer);
    std::string s_serialnumber;
    for (auto &n : rs.a_serialnumber())
    {
        auto i = std::stoi(n);
        s_serialnumber += static_cast<char>(i);
    }
    std::cout << "Received radar status from sensor " << rs.t_header().u_sensorid() << std::endl;
    if (rs.a_ethernet_ip_size() == 4)
    {
        std::cout << "Sensor has IP: "
                  << std::stoi(rs.a_ethernet_ip(0), nullptr, 16) << "."
                  << std::stoi(rs.a_ethernet_ip(1), nullptr, 16) << "."
                  << std::stoi(rs.a_ethernet_ip(2), nullptr, 16) << "."
                  << std::stoi(rs.a_ethernet_ip(3), nullptr, 16)
                  << " and ";
    }
    std::cout << "serial number: " << s_serialnumber << std::endl;
}

//Main fucntion start 
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "radar");
    ros::NodeHandle rd;
    radar_pub = rd.advertise<conti_radar::radar_msgs>("radar_pose2d", 1000);
    
    std::string config{};
    config = "/opt/pdk/etc/pdk_config.json";
 /*   if (argc > 1)
    {
        config = argv[1];
        if ((config == "-c") && (argc > 2))
        {
            config = argv[2];
        }
    }
*/
//Checking the Libarary Version
    std::cout << "Library version: " << PDK::GetVersion() << std::endl;
    PDK::CInterface::Init(config);
    PDK::CInterface::SetSensorObjectsCallback([](const std::string &) { std::cout << "ars430di" << std::endl; });
    std::cout << "--------------------------------" << std::endl;
    PDK::CInterface::SetSensorObjectsCallback(OnReceiveRDI);
   
    while (true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}
