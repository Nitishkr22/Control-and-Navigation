#include "pdk/pdk_interface.hpp"
#include "pdk_RadarDetectionImage.pb.h"
#include "pdk_RadarObjectList.pb.h"
#include "novatel_oem7_msgs/BESTVEL.h"
#include "novatel_oem7_msgs/CORRIMU.h"
#include "ros/ros.h"
#include <iostream>
#include <cmath> // Include cmath for trigonometric functions
#include <thread>
#include <fstream>
#include "pdk_MsgHeader.pb.h"
#include <ctime>
#include <iomanip>
#include <sstream> 
#include <vector>
#include <algorithm>
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"

double f_LongVel;
float f_YawRate;    
float f_LongAccel;
float f_LatAccel;

ros::Publisher acc_switch_pub;
ros::Publisher brake_action_pub;
ros::Publisher f_vabsx_pub;
ros::Publisher range_pub;
ros::Publisher x_pos_pub;
ros::Publisher y_pos_pub;
ros::Publisher f_aabsx_pub;
ros::Publisher f_aabsy_pub;

std_msgs::Bool acc_switch_msg;
std_msgs::Bool brake_action_msg;
std_msgs::Float64 f_vabsx_msg;
std_msgs::Float64 range_msg;
std_msgs::Float64 x_pos_msg;
std_msgs::Float64 y_pos_msg;
std_msgs::Float64 f_aabsx_msg;
std_msgs::Float64 f_aabsy_msg;

void CORRIMU_callback(const novatel_oem7_msgs::CORRIMU msg){
    f_YawRate = msg.yaw_rate;
    f_LongAccel = msg.longitudinal_acc;
    f_LatAccel = msg.lateral_acc;
}


void BESTVEL_callback(const novatel_oem7_msgs::BESTVEL msg){
    f_LongVel = msg.hor_speed;
}


void OnReceiveObjects(const std::string &buffer)
{
    pb::PDK::RadarObjectList obj{};
    obj.ParseFromString(buffer);
    
    // double minRange = std::numeric_limits<double>::max();
    
    pb::PDK::RadarObjectList_RadarObject minRangeObject;
    std::vector<pb::PDK::RadarObjectList_RadarObject> filteredObjects;
    bool acc_switch = false, brake_action = false;

    if (obj.u_nofusedobjects() == 0)
    {
        return;
    }
    std::cout << "Received " << obj.u_nofusedobjects() << " objects" << std::endl;
    std::cout<<"ego velocity "<<f_LongVel <<std::endl;

    double max_range = 100.0, min_range = 5.0;

    for (const auto &ro : obj.a_radarobjectlist())
    {
        // Check if the calculated range is less than 10 meters
        double range = std::sqrt(ro.f_distx() * ro.f_distx() + ro.f_disty() * ro.f_disty());
        double lm,mr,obj_size;
        lm = pow((pow((ro.f_ldeltax_left() - ro.f_ldeltax_mid()),2)+pow((ro.f_ldeltay_left() - ro.f_ldeltay_mid()),2)),0.5);
        mr = pow((pow((ro.f_ldeltax_right() - ro.f_ldeltax_mid()),2)+pow((ro.f_ldeltay_right() - ro.f_ldeltay_mid()),2)),0.5);
        obj_size = lm*mr;

        
        // if (range < 10.0)
        if (range < 80.0 &&
            // ro.e_dynamicproperty() == 0 &&
            // ro.f_orientation() != 0 &&
            ro.f_rcs() > 1.0 &&
            ro.f_objectscore() > 0.5 &&
            ro.f_disty()<=1.8 && ro.f_disty()>=-1.8&&
            obj_size > 1.5)
        {
            // Extract timestamp from MsgHeader
            filteredObjects.push_back(ro);
            std::cout<<"interesting objects: "<<filteredObjects.empty()<<std::endl;

            minRangeObject = ro;
        


            std::sort(filteredObjects.begin(), filteredObjects.end(),
            [](const pb::PDK::RadarObjectList_RadarObject &a, const pb::PDK::RadarObjectList_RadarObject &b) {
                double rangeA = std::sqrt(a.f_distx() * a.f_distx() + a.f_disty() * a.f_disty());
                double rangeB = std::sqrt(b.f_distx() * b.f_distx() + b.f_disty() * b.f_disty());
                return rangeA < rangeB;
            });

            std::size_t vectorSize = filteredObjects.size();


            if (!filteredObjects.empty())
            {
                // Access the object with the minimum range
                pb::PDK::RadarObjectList_RadarObject minRangeObject = filteredObjects.front();

            // Write the object features along with timestamps to the CSV file
            
            double rangeA = std::sqrt(minRangeObject.f_distx() * minRangeObject.f_distx() + minRangeObject.f_disty() * minRangeObject.f_disty());

            if (rangeA < max_range ) //&& rangeA > min_range
            {
                acc_switch = true;
                acc_switch_msg.data = true; // Set acc_switch_msg value
                // Publish acc_switch
                acc_switch_pub.publish(acc_switch_msg);
                f_vabsx_msg.data = minRangeObject.f_vabsx() * (18/5);
                f_aabsx_msg.data = minRangeObject.f_aabsx();
                f_aabsy_msg.data = minRangeObject.f_aabsy();
                f_vabsx_pub.publish(f_vabsx_msg);
                f_aabsx_pub.publish(f_aabsx_msg);
                f_aabsy_pub.publish(f_aabsy_msg);
                range_msg.data = rangeA;
                range_pub.publish(range_msg);

                x_pos_msg.data = minRangeObject.f_distx();
                y_pos_msg.data = minRangeObject.f_disty();

                x_pos_pub.publish(x_pos_msg);
                y_pos_pub.publish(y_pos_msg);

                double brake_point = (max_range + min_range) / 2;
                if (rangeA < brake_point) {
                    brake_action = true;
                } else {
                    brake_action = false;
                }
                brake_action_msg.data = brake_action; // Set brake_action_msg value
                // Publish brake_action
                brake_action_pub.publish(brake_action_msg);
            } 
            else {
                acc_switch = false;
                acc_switch_msg.data = false; // Set acc_switch_msg value
                brake_action = false;
                // Publish acc_switch
                brake_action_msg.data = brake_action; // Set brake_action_msg value
                // Publish brake_action
                brake_action_pub.publish(brake_action_msg);
                acc_switch_pub.publish(acc_switch_msg);
                if (rangeA !=0){
                    range_msg.data = rangeA;
                    range_pub.publish(range_msg);

                    x_pos_msg.data = minRangeObject.f_distx();
                    y_pos_msg.data = minRangeObject.f_disty();

                    x_pos_pub.publish(x_pos_msg);
                    y_pos_pub.publish(y_pos_msg);
                }
                
            }
            
            }    
            std::cout << "Object within 10 meters range:" << std::endl
                      << "u_ObjId " << ro.u_objid() << std::endl
                      << "f_DistX " << ro.f_distx() << " m" << std::endl
                      << "f_DistY " << ro.f_disty() << " m" << std::endl
                      << "Calculated Range: " << range << " m" << std::endl
                      << "f_VrelX " << ro.f_vabsy() << " m/s" << std::endl
                      << "f_VrelY " << ro.f_vabsy() << " m/s" << std::endl
                      <<"ego velocity "<<f_LongVel <<std::endl
                      // Include other relevant information
                      << std::endl;
                                            
        }

    }

}




int main(int argc, char *argv[])
{

    ros::init(argc,argv,"testacc");
    ros::NodeHandle node;
    ros::Subscriber bestvel_sub = node.subscribe("/novatel/oem7/bestvel",1000,BESTVEL_callback);
    ros::Subscriber corrimu_sub = node.subscribe("/novatel/oem7/corrimu",1000,CORRIMU_callback);


    //publish the values
    acc_switch_pub = node.advertise<std_msgs::Bool>("acc_switch_topic", 1000);
    brake_action_pub = node.advertise<std_msgs::Bool>("brake_action_topic", 1000);
    f_vabsx_pub = node.advertise<std_msgs::Float64>("f_vabsx_topic", 1000);
    range_pub = node.advertise<std_msgs::Float64>("range_topic", 1000);
    x_pos_pub = node.advertise<std_msgs::Float64>("x_pos_topic", 1000);
    y_pos_pub = node.advertise<std_msgs::Float64>("y_pos_topic", 1000);
    f_aabsx_pub = node.advertise<std_msgs::Float64>("f_aabsx_topic", 1000);
    f_aabsy_pub = node.advertise<std_msgs::Float64>("f_aabsy_topic", 1000);

   // Initialization and other setup code...
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

    PDK::CInterface::SetSensorObjectsCallback(OnReceiveObjects);
 
    while(ros::ok()){
        ros::spinOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Rest of the code...

    return 0;
}
