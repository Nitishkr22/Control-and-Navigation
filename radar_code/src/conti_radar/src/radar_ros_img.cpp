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
#include <fstream>
#include <chrono>
#include <cstring>
#include "conti_radar/radar_msgs.h"
#include "conti_radar/radar_img.h"


ros::Publisher radar_img;
void OnReceiveRDI(const std::string &buffer)
{
    pb::PDK::RadarDetectionImage im{};
    im.ParseFromString(buffer);
    //std::cout << im.u_nofdetections() <<std::endl;
    //std::cout << "Received radar status from sensor " << obj.t_header().u_sensorid() << std::endl;

    int random_object_nr = std::rand() % static_cast<int>(im.u_nofdetections());  //selecting one detection from nofdetections
    //std::cout << "object "<< obj.u_nofusedobjects()<<std::endl;
    pb::PDK::RadarDetectionImage_RadarDetection ro = im.a_radardetectionlist(random_object_nr); 
    
    std::cout << "Received " << im.u_nofdetections() << " detections" << std::endl;


    pb::PDK::RadarStatus rs{};
    rs.ParseFromString(buffer);
    std::string s_serialnumber;

    conti_radar::radar_img ran;
  
    for (int i = 0; i < im.u_nofdetections(); i++){

        pb::PDK::RadarDetectionImage_RadarDetection ri = im.a_radardetectionlist(i); 

        ran.range.push_back(ri.f_range());
        ran.id = im.t_header().u_sensorid();
        ran.azimuth.push_back(ri.a_azang_hyp(0));
        ran.VrelRad.push_back(ri.f_vrelrad());
        ran.RCS.push_back(ri.a_rcs_hyp(0));
        ran.rangevar.push_back(ri.f_rangevar());
        ran.vrelradvar.push_back(ri.f_vrelradvar());
        ran.azimuthvar.push_back(ri.f_azangvar());
        ran.SNR.push_back(ri.f_snr());
    }
    radar_img.publish(ran);

}

void OnReceiveStatus(const std::string &buffer)
{
    pb::PDK::RadarStatus rs{};
    rs.ParseFromString(buffer);
    std::string s_serialnumber;
    /*for (auto &n : rs.a_serialnumber())  // auto is used if data type is not known
    {
        auto i = std::stoi(n);  // stoi() converts string to integer value
        s_serialnumber += static_cast<char>(i); // static_cast is used to convert i to any other data type, here char
    }*/
    //std::cout << "Received radar status from sensor " << rs.t_header().u_sensorid() << std::endl;
    /*if (rs.a_ethernet_ip_size() == 4)
    {
        std::cout << "Sensor has IP: "
                  << std::stoi(rs.a_ethernet_ip(0), nullptr, 16) << "."
                  << std::stoi(rs.a_ethernet_ip(1), nullptr, 16) << "."
                  << std::stoi(rs.a_ethernet_ip(2), nullptr, 16) << "."
                  << std::stoi(rs.a_ethernet_ip(3), nullptr, 16)
                  << " and ";
    }
    std::cout << "serial number: " << s_serialnumber << std::endl; */
}


void OnVehicleDynamics(const PDK::CVehicleDynamics &vdy_)
{
    std::cout << "Received vehicle dynamics" << std::endl
              << "LongVel: " << vdy_.LongVel.value << std::endl
              << "YawRate: " << vdy_.YawRate.value << std::endl
              << "LatAcc: " << vdy_.LatAccel.value << std::endl
              << "LongAcc: " << vdy_.LongAccel.value << std::endl;
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "radar_ros_img");
    ros::NodeHandle nh,n2;
    radar_img = nh.advertise<conti_radar::radar_img>("radar_img", 1000);
    // pub1 = n2.advertise<conti_radar::radar_img>("azimuth", 1000);


    std::string config{};
    config = "/opt/pdk/etc/pdk_config.json";


    if (argc > 1)
    {
        config = argv[1];
        if ((config == "-c") && (argc > 2))
        {
            config = argv[2];
        }
    }
    std::cout << "Library version: " << PDK::GetVersion() << std::endl;
    PDK::CInterface::Init(config);
    // PDK::CInterface::SetMountingParametersCallback([](const int, const PDK::CMountingParameters &) { std::cout << "My Mounting Dummy Subscriber" << std::endl; });
    // PDK::CInterface::SetRDICallback([](const std::string &){std::cout << " Dynamic Object Tracking Callback" << std::endl;});

  //std::this_thread::sleep_for(std::chrono::seconds(15));
    // PDK::CInterface::StopMountingParametersSubscriber();

  //std::this_thread::sleep_for(std::chrono::seconds(15));
    // PDK::CInterface::SetMountingParametersCallback(OnMountingParameters);
    PDK::CInterface::SetRDICallback(OnReceiveRDI);
    std::this_thread::sleep_for(std::chrono::seconds(15));

    // PDK::CInterface::StopRDISubscriber();
    // PDK::CInterface::StopRadarStatusSubscriber();


    while (true)
    {
        //std::cout << (PDK::CInterface::PublishVehicleDynamics(vehicle_dynamics) ? "sent" :  "not_sent") << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }



}