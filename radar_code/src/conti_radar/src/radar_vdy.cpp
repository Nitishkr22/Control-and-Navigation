#include "ros/ros.h"
#include "std_msgs/String.h"
#include "novatel_oem7_msgs/BESTVEL.h"
#include "novatel_oem7_msgs/CORRIMU.h"


#include "pdk/pdk_interface.hpp"
#include "pdk/VehicleDynamics.hpp"
#include <chrono>
#include <iostream>
#include <thread>

float f_LongVel;
float f_YawRate;    
float f_LongAccel;
float f_LatAccel;

void CORRIMU_callback(const novatel_oem7_msgs::CORRIMU msg){
    f_YawRate = msg.yaw_rate;
    f_LongAccel = msg.longitudinal_acc;
    f_LatAccel = msg.lateral_acc;
}


void BESTVEL_callback(const novatel_oem7_msgs::BESTVEL msg){
    f_LongVel = msg.hor_speed;
}

int main(int argc, char *argv[])
{

    // Set VDY values which will be sent (example 10m/s longitudinal velocity)
    // Usually this has to come from vehiclebus, GPS information etc


    ros::init(argc,argv,"GNSS");
    ros::NodeHandle node;
    ros::Subscriber bestvel_sub = node.subscribe("/novatel/oem7/bestvel",1000,BESTVEL_callback);
    ros::Subscriber corrimu_sub = node.subscribe("/novatel/oem7/corrimu",1000,CORRIMU_callback);

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
    PDK::CInterface::Init(config);

    auto CurrentTimeStamp = std::chrono::system_clock::now();
    auto Seconds = std::chrono::duration_cast<std::chrono::seconds>(CurrentTimeStamp.time_since_epoch());
    auto Nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(CurrentTimeStamp.time_since_epoch()) - Seconds;
    
    while(ros::ok()){
        ros::spinOnce();

        PDK::CVehicleDynamics vehicle_dynamics = {
        {// TimeStamp Posix time, i.es. time since since 00:00:00 Coordinated Universal Time(UTC), Thursday, 1 January
         // 1970
         static_cast<uint32_t>(Seconds.count()), static_cast<uint32_t>(Nanoseconds.count())},
        {/* LongVel [m/s] */
         PDK::CVehicleDynamicsSignal::eSignalState::VALID, f_LongVel},
        {/* YawRate [rad/s] */
         PDK::CVehicleDynamicsSignal::eSignalState::VALID, f_YawRate},
        {/* LongAccel [m/s^2] */
         PDK::CVehicleDynamicsSignal::eSignalState::VALID, f_LongAccel},
        {/* LatAccel [m/s^2] */
         PDK::CVehicleDynamicsSignal::eSignalState::VALID, f_LatAccel}
         };
        if(PDK::CInterface::PublishVehicleDynamics(vehicle_dynamics)){
            std::cout << "Published vdy" << std::endl;
        } else std::cout << "Not published vdy" << std::endl;
        std::cout << f_LongVel << " " <<
        f_YawRate << " " << 
        f_LongAccel << " " <<
        f_LatAccel << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    }
    
}