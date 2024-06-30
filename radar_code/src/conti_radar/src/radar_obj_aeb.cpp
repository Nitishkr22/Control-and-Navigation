#include "pdk/pdk_interface.hpp"
#include "pdk_RadarDetectionImage.pb.h"
#include "pdk_RadarStatus.pb.h"
#include "pdk_RadarObjectList.pb.h"
#include "pdk_PoseWithVelocity.pb.h"
#include "pdk_IMUData.pb.h"

#include <map>
#include <cstdio>
#include <vector>
#include <iostream>
#include <string>
#include <thread>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
// #include "/conti_radar/radar_msgs.h"
#include <fstream>
#include <chrono>
#include <cstring>
using namespace std::chrono;
float ego_vel = 0.0;
ros::Publisher radar_aeb;
void OnReceiveRDI(const std::string &buffer)
{
    pb::PDK::RadarDetectionImage im{};
    im.ParseFromString(buffer);
    //std::cout << im.u_nofdetections() <<std::endl;
    int random_object_nr = std::rand() % static_cast<int>(im.u_nofdetections());  //selecting one detection from nofdetections
    //std::cout << "check: "<< im.o_objectdetection().u_nofusedobjects() <<std::endl;
    pb::PDK::RadarDetectionImage_RadarDetection ri = im.a_radardetectionlist(random_object_nr); 
    
    std::cout << "Received " << im.u_nofdetections() << " detections" << std::endl
    << "Showing attributes for detections " << random_object_nr << ":" << std::endl;
    
    std::cout << "Range of RDI = " << ri.f_snr() << std::endl;

}


// std::ofstream csvFile("example.csv", std::ios::app);
void OnReceiveObjects(const std::string &buffer)
{
    pb::PDK::RadarObjectList obj{};
    obj.ParseFromString(buffer);
    //std::cout << im.u_nofdetections() <<std::endl;
    //std::cout << "Received radar status from sensor " << obj.t_header().u_sensorid() << std::endl;

    int random_object_nr = std::rand() % static_cast<int>(obj.u_nofusedobjects());  //selecting one detection from nofdetections
    //std::cout << "object "<< obj.u_nofusedobjects()<<std::endl;
    pb::PDK::RadarObjectList_RadarObject ro = obj.a_radarobjectlist(random_object_nr); 
    
    std::cout << "Received " << obj.u_nofusedobjects() << " detections" << std::endl
        << "Showing attributes for detections " << random_object_nr << ":" << std::endl;
    
    std::cout << "ego_velocity:  = " << ego_vel << std::endl;
 
    pb::PDK::RadarStatus rs{};
    rs.ParseFromString(buffer);
    std::string s_serialnumber;
    std::ofstream csvFile("radar_data.csv", std::ios::app);
    // csvFile << "Distance in X,Column 2\n";

        
    // Get the current time as a time point with seconds precision
    auto currentTime = system_clock::now();
    auto currentTimeT = system_clock::to_time_t(currentTime);

    // Convert the time point to a local time struct with microseconds precision
    tm timestampTm;
    localtime_r(&currentTimeT, &timestampTm);
    auto timestampMs = duration_cast<microseconds>(currentTime.time_since_epoch()) % 1000000;

    // Format the timestamp as a string
    char timestampStr[30];
    strftime(timestampStr, sizeof(timestampStr), "%H:%M:%S", &timestampTm);
    sprintf(timestampStr + strlen(timestampStr), ".%06ld", timestampMs.count());
    float lm, mr,obj_size, road_width=2.0, vrelative = 0.0, ttc = 0.0, flag = 2.0;

    // conti_radar::radar_obj objct;
    std_msgs::Float32 flagp;

    for (int i = 0; i < obj.u_nofusedobjects(); i++){

        pb::PDK::RadarObjectList_RadarObject ob = obj.a_radarobjectlist(i); 
        float range = pow(pow(ob.f_distx(),2) + pow(ob.f_disty(),2), 0.5);
        // float azimuth = std::atan(ob.f_disty()/ob.f_distx()); // azimuth output is in radians
        lm = pow((pow((ob.f_ldeltax_left() - ob.f_ldeltax_mid()),2)+pow((ob.f_ldeltay_left() - ob.f_ldeltay_mid()),2)),0.5);
        mr = pow((pow((ob.f_ldeltax_right() - ob.f_ldeltax_mid()),2)+pow((ob.f_ldeltay_right() - ob.f_ldeltay_mid()),2)),0.5);
        obj_size = lm*mr;
        // if (ob.f_vabsx()<0){
        //     absvel = -pow(pow(ob.f_vabsx(),2) + pow(ob.f_vabsy(),2), 0.5);
        // }
        // else{
        //     absvel = pow(pow(ob.f_vabsx(),2) + pow(ob.f_vabsy(),2), 0.5);
        // }
        

        

        // std::cout << "road_width: = " << road_width << std::endl;
        // std::cout << "dist_in y:  = " << ob.f_disty() << std::endl;

        //csvFile << "Column 1,Column 2\n";
        // flagp.data = ob.f_distx();
        // radar_aeb.publish(flagp);#
        if (range < 40 && obj_size>0.1){
            if (ob.f_disty()<=2.0 && ob.f_disty()>=-2.0){
                // std::cout << "dist_in y:  = " << ob.f_disty() << std::endl;
                vrelative = ego_vel - ob.f_vabsx();
                if (vrelative!=0){
                    ttc =  range/vrelative;
                }
                if (ttc>6.0 && ttc<10){
                    flag = 1.0;
                }
                else if(ttc<6.0 && ttc>4.0){
                    flag = 0.5;
                }
                else if(ttc<4.0 && ttc>0.0){
                    flag = 0.0;
                }
                else{
                    flag = 2.0;
                }
                if(range<4.0){
                    flag = 1.5;
                }
                std::cout<<"flag= "<<flag<<std::endl;
            
                csvFile << timestampStr  << "," << obj.t_header().u_sensorid()<< "," << ob.f_distx() << "," << ob.f_disty() << "," << ob.f_objectscore() << "," <<
                ob.f_vrelx()<< "," << ob.f_vrely() << "," << range << "," << obj_size << "," << ob.f_distxstd() << "," << ob.f_distystd() << "," << 
                ob.f_orientation() << ","  << ob.f_orientationstd() << ","  << ob.e_dynamicproperty() << ","  << ob.f_arelystd() << "," << ob.f_ldeltax_left() << "," << 
                ob.f_ldeltax_mid() << "," << ob.f_ldeltax_right() << "," << ob.f_ldeltay_left() << "," << ob.f_ldeltay_mid() << "," <<ob.f_ldeltay_right() << "," <<
                ob.f_vabsx()<< "," << ob.f_vabsy() << "," << ob.f_aabsx() << "," << ob.f_aabsy() << "," << ego_vel<<","<<vrelative<<","<<ttc<<","<<flag<<std::endl;

            }
        
        
        //csvFile << "3,4\n";
        //csvFile << "5,6\n";
    }
    flagp.data = flag;
    radar_aeb.publish(flagp);
    }
    csvFile.close();
}

//Status function//
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
    ego_vel = vdy_.LongVel.value;
    std::cout << "Received vehicle dynamics" << std::endl
              << "LongVel: " << vdy_.LongVel.value << std::endl
              << "YawRate: " << vdy_.YawRate.value << std::endl
              << "LatAcc: " << vdy_.LatAccel.value << std::endl
              << "LongAcc: " << vdy_.LongAccel.value << std::endl;
}


int main(int argc, char *argv[])
{
    //std::ofstream csvFile("example.csv", std::ios::app);
    ros::init(argc, argv, "radar_ros_aeb");
    ros::NodeHandle nh;
    radar_aeb = nh.advertise<std_msgs::Float32>("radar_ros_aeb", 1000);



    std::ofstream csvFile("radar_data.csv");
    csvFile << "timestamp, Sensor ID, Distance in X, Distance in Y, Object_score, Relative Velocity in X, Relative Velocity in Y, " << 
                "Range, object_size, Standard deviation of x-position, Standard deviation of y-position," << 
                "Orientation, Orientation_std, dynamic_property," <<
                "Standard deviation of relative acceleration in y, X-distance between reference point and bounding box (left) [m], " <<
                "X-distance between reference point and bounding box (mid) [m], X-distance between reference point and bounding box (right) [m], " <<
                "Y-distance between reference point and bounding box (left) [m], Y-distance between reference point and bounding box (mid) [m], " <<
                "Y-distance between reference point and bounding box (right) [m], Absolute velocity in x-direction [m/s], Absolute velocity in y-direction [m/s], " << 
                "Absolute acceleration in x-direction [m/s], Absolute acceleration in y-direction [m/s], Ego_velocity[m/s], Relative velocity, TTC, flag \n";
    csvFile.close();


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

    // Set VDY values which will be sent (example 10m/s longitudinal velocity)
    // Usually this has to come from vehiclebus, GPS information etc
    float f_LongVel = 10.0f;
    float f_YawRate = 0.0f;
    float f_LongAccel = 0.0f;
    float f_LatAccel = 0.0f;

    std::cout << "Library version: " << PDK::GetVersion() << std::endl;
    PDK::CInterface::Init(config);

    // Determine timestamp
    auto CurrentTimeStamp = std::chrono::system_clock::now();
    auto Seconds = std::chrono::duration_cast<std::chrono::seconds>(CurrentTimeStamp.time_since_epoch());
    auto Nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(CurrentTimeStamp.time_since_epoch()) - Seconds;
    // set vehicle dynamics signals
    // PDK::CVehicleDynamics vehicle_dynamics = {
    //     {// TimeStamp Posix time, i.es. time since since 00:00:00 Coordinated Universal Time(UTC), Thursday, 1 January
    //      // 1970
    //      static_cast<uint32_t>(Seconds.count()), static_cast<uint32_t>(Nanoseconds.count())},
    //     {/* LongVel [m/s] */
    //      PDK::CVehicleDynamicsSignal::eSignalState::VALID, f_LongVel},
    //     {/* YawRate [rad/s] */
    //      PDK::CVehicleDynamicsSignal::eSignalState::VALID, f_YawRate},
    //     {/* LongAccel [m/s^2] */
    //      PDK::CVehicleDynamicsSignal::eSignalState::VALID, f_LongAccel},
    //     {/* LatAccel [m/s^2] */
    //      PDK::CVehicleDynamicsSignal::eSignalState::VALID, f_LatAccel}};
    //PDK::CInterface::SetRadarStatusCallback(OnReceiveStatus);
    //std::this_thread::sleep_for(std::chrono::seconds(2));

    //PDK::CInterface::SetRDICallback([](const std::string &) { std::cout << "My RDI Dummy Subscriber" << std::endl; });
    

    PDK::CInterface::SetSensorObjectsCallback([](const std::string &){std::cout << "Dummy Dynamic Object Tracking Callback" << std::endl;});
    PDK::CInterface::SetVehicleDynamicsCallback([](const PDK::CVehicleDynamics &) { std::cout << "My VDY Dummy Subscriber" << std::endl; });

    //std::this_thread::sleep_for(std::chrono::seconds(2));
    // PDK::CInterface::SetRDICallback(OnReceiveRDI);

    PDK::CInterface::StopSensorObjectsSubscriber();
    PDK::CInterface::SetSensorObjectsCallback(OnReceiveObjects);
    PDK::CInterface::SetVehicleDynamicsCallback(OnVehicleDynamics);

    //PDK::CInterface::SetRadarStatusCallback(OnReceiveStatus);

    


    while (true)
    {
        //std::cout << (PDK::CInterface::PublishVehicleDynamics(vehicle_dynamics) ? "sent" :  "not_sent") << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }



}