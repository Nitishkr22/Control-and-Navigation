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
#include "conti_radar/radar_obj.h"

using namespace std::chrono;

ros::Publisher radar_obj;
void OnReceiveObjects(const std::string &buffer)
{
    pb::PDK::RadarObjectList obj{};
    obj.ParseFromString(buffer);
    //std::cout << im.u_nofdetections() <<std::endl;
    //std::cout << "Received radar status from sensor " << obj.t_header().u_sensorid() << std::endl;

    int random_object_nr = std::rand() % static_cast<int>(obj.u_nofusedobjects());  //selecting one detection from nofdetections
    //std::cout << "object "<< obj.u_nofusedobjects()<<std::endl;
    pb::PDK::RadarObjectList_RadarObject ro = obj.a_radarobjectlist(random_object_nr); 
    
    std::cout << "Received " << obj.u_nofusedobjects() << " detections" << std::endl;
    // std::cout << "Received " << obj.u_nofusedobjects() << " detections" << std::endl;
        // << "Showing attributes for detections " << random_object_nr << ":" << std::endl; ob.f_objectscore()
    
    
    pb::PDK::RadarStatus rs{};
    rs.ParseFromString(buffer);
    std::string s_serialnumber;

    conti_radar::radar_obj objct;
    
    for (int i = 0; i < obj.u_nofusedobjects(); i++){

        pb::PDK::RadarObjectList_RadarObject ob = obj.a_radarobjectlist(i); 

        // std::cout << "Object ID: = " << ob.u_objid() << std::endl;
        std::cout << "Object_score:  = " << ob.f_objectscore() << std::endl;
        objct.id = obj.t_header().u_sensorid();
        objct.f_DistX.push_back(ob.f_distx());
        objct.f_DistY.push_back(ob.f_disty());
        objct.f_VrelX.push_back(ob.f_vrelx());
        objct.f_VrelY.push_back(ob.f_vrely());
        objct.f_ArelX.push_back(ob.f_arelx());
        objct.f_ArelY.push_back(ob.f_arely());
        objct.f_DistXStd.push_back(ob.f_distxstd());
        objct.f_DistYStd.push_back(ob.f_distystd());
        objct.f_VrelXStd.push_back(ob.f_vrelxstd());
        objct.f_VrelYStd.push_back(ob.f_vrelystd());
        objct.f_ArelXStd.push_back(ob.f_arelxstd());
        objct.f_ArelYStd.push_back(ob.f_arelystd());
        objct.f_LDeltaX_left.push_back(ob.f_ldeltax_left());
        objct.f_LDeltaX_mid.push_back(ob.f_ldeltax_mid());
        objct.f_LDeltaX_right.push_back(ob.f_ldeltax_right());
        objct.f_LDeltaY_left.push_back(ob.f_ldeltay_left());
        objct.f_LDeltaY_mid.push_back(ob.f_ldeltay_mid());
        objct.f_LDeltaY_right.push_back(ob.f_ldeltay_right());
        objct.f_RCS.push_back(ob.f_rcs());

        // objct.e_LDeltaQual.push_back(ob.e_ldeltaqual());
        objct.f_ObjectScore.push_back(ob.f_objectscore());
        // objct.u_LifeCycles.push_back(ob.u_lifecycles());
        // objct.e_DynamicProperty.push_back(ob.e_dynamicproperty());
        // objct.e_ObjState.push_back(ob.e_objstate());
        objct.f_VabsX.push_back(ob.f_vabsx());
        objct.f_VabsY.push_back(ob.f_vabsy());
        objct.f_AabsX.push_back(ob.f_aabsx());
        objct.f_AabsY.push_back(ob.f_aabsy());
        objct.f_VabsXStd.push_back(ob.f_aabsxstd());
        objct.f_VabsYStd.push_back(ob.f_aabsystd());
        objct.f_AabsXStd.push_back(ob.f_aabsxstd());
        objct.f_AabsYStd.push_back(ob.f_aabsystd());
        objct.f_Orientation.push_back(ob.f_orientation());
        objct.f_OrientationStd.push_back(ob.f_orientationstd());
        objct.f_DistZ.push_back(ob.f_distz());
        objct.f_DistZStd.push_back(ob.f_distzstd());
    }
    radar_obj.publish(objct);

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
    ros::init(argc, argv, "radar_ros_obj");
    ros::NodeHandle nh;
    radar_obj = nh.advertise<conti_radar::radar_obj>("radar_obj", 1000);
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
    // PDK::CInterface::SetRDICallback(OnReceiveObject)
    PDK::CInterface::SetSensorObjectsCallback(OnReceiveObjects);
    std::this_thread::sleep_for(std::chrono::seconds(15));

    // PDK::CInterface::StopRDISubscriber();
    // PDK::CInterface::StopRadarStatusSubscriber();


    while (true)
    {
        //std::cout << (PDK::CInterface::PublishVehicleDynamics(vehicle_dynamics) ? "sent" :  "not_sent") << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }



}