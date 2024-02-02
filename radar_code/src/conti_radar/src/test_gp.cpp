#include "pdk/pdk_interface.hpp"
#include "pdk_RadarDetectionImage.pb.h"
#include "pdk_RadarObjectList.pb.h"
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

// void OnReceiveDetections(const std::string &buffer)
// {
//     pb::PDK::RadarDetectionImage radarImage{};
//     radarImage.ParseFromString(buffer);

//     std::cout << "Received Radar Image" << std::endl;

//     for (const auto &detection : radarImage.a_radardetectionlist())
//     {
//         // Check if the f_Range is less than 4 meters
//         if (detection.f_range() < 80.0)
//         {
//             // Calculate x and y components of distances
//             float x_dist = detection.f_range() * std::cos(detection.a_azang_hyp(0)); // Assuming the first azimuth hypothesis
//             float y_dist = detection.f_range() * std::sin(detection.a_azang_hyp(0)); // Assuming the first azimuth hypothesis

//             std::cout << "Detection within 4 meters range:" << std::endl
//                       << "f_Range: " << detection.f_range() << " m" << std::endl
//                       << "f_VrelRad: " << detection.f_vrelrad() << " m/s" << std::endl
//                       << "x_dist: " << x_dist << " m" << std::endl
//                       << "y_dist: " << y_dist << " m" << std::endl
//                       // Include other relevant information
//                       << std::endl;
//         }
//     }
// }


void OnReceiveRadarImage(const std::string &buffer)
{
    pb::PDK::RadarDetectionImage radarImage{};
    radarImage.ParseFromString(buffer);

    if (radarImage.u_nofdetections() == 0)
    {
        return;
    }

    // Open the CSV file in append mode
    std::ofstream outputFile("radar_image_data.csv", std::ios::out | std::ios::trunc);

    std::cout << "Received " << radarImage.u_nofdetections() << " radar detections" << std::endl;

    for (const auto &detection : radarImage.a_radardetectionlist())
    {
        // Extract timestamp from SensorMsgHeader
        pb::PDK::MsgHeader_Time timestamp = radarImage.t_header().t_commonheader().t_stamp();

        // Get system time in nanoseconds
        auto systemTime = std::chrono::high_resolution_clock::now().time_since_epoch().count();

        // Format system time to a human-readable string
        auto systemTimeInSeconds = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        std::tm *timeInfo = std::localtime(&systemTimeInSeconds);
        char buffer[80];
        std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", timeInfo);
        std::ostringstream formattedTime;
        formattedTime << buffer << "." << std::setw(9) << std::setfill('0') << systemTime % 1000000000;

        // Write the radar detection features along with timestamps to the CSV file
        outputFile << formattedTime.str() << ","
                   << timestamp.u_sec() << "," << timestamp.u_nsec() << ","
                   << detection.f_range() << "," << detection.f_vrelrad() << ","
                   // Include other relevant information
                   << std::endl;
    }

    // Close the CSV file
    outputFile.close();
}


void OnReceiveObjects(const std::string &buffer)
{
    pb::PDK::RadarObjectList obj{};
    obj.ParseFromString(buffer);
    
    // double minRange = std::numeric_limits<double>::max();
    
    // pb::PDK::RadarObjectList_RadarObject minRangeObject;
    std::vector<pb::PDK::RadarObjectList_RadarObject> filteredObjects;

    if (obj.u_nofusedobjects() == 0)
    {
        return;
    }

    // Open the CSV file in append mode
    // std::ofstream outputFile("radar_data.csv", std::ios::out | std::ios::trunc);
    // outputFile << "Timestamp_Sec, Timestamp_Nsec, Formatted_Time, Radar_Epoch_Time, Obj_ID, Dist_X, Dist_Y, RCS, Range, Dynamicproperty, Orientation, Objectsize, objectscore" << std::endl;
    std::ofstream outputFile("radar_data.csv", std::ios::app);
    std::ofstream outputFile1("radar_data1.csv", std::ios::app);
    std::cout << "Received " << obj.u_nofusedobjects() << " objects" << std::endl;

    double max_range = 20.0, min_range = 10.0;

    for (const auto &ro : obj.a_radarobjectlist())
    {
        // Check if the calculated range is less than 10 meters
        double range = std::sqrt(ro.f_distx() * ro.f_distx() + ro.f_disty() * ro.f_disty());
        double lm,mr,obj_size;
        lm = pow((pow((ro.f_ldeltax_left() - ro.f_ldeltax_mid()),2)+pow((ro.f_ldeltay_left() - ro.f_ldeltay_mid()),2)),0.5);
        mr = pow((pow((ro.f_ldeltax_right() - ro.f_ldeltax_mid()),2)+pow((ro.f_ldeltay_right() - ro.f_ldeltay_mid()),2)),0.5);
        obj_size = lm*mr;
        ////////////////////////////////////////////////////////////////
        




        ///////////////////////////////////////////////////////////////
        // if (range < 10.0)
        if (range < 40.0 &&
            // ro.e_dynamicproperty() == 0 &&
            // ro.f_orientation() != 0 &&
            // ro.f_rcs() > 0.0 &&
            // ro.f_objectscore() > 0.5 &&
            obj_size > 0.01)
        {
            // Extract timestamp from MsgHeader
            filteredObjects.push_back(ro);
            pb::PDK::MsgHeader_Time timestamp = obj.t_header().t_commonheader().t_stamp();
            // uint64_t epochTime = static_cast<uint64_t>(timestamp.u_sec()) * 1000000000 + timestamp.u_nsec();
            // std::cout<<"epoch time is: "<<epochTime<<std::endl;
            std::cout<<"interesting objects: "<<filteredObjects.empty()<<std::endl;
            auto systemTime1 = std::chrono::high_resolution_clock::now().time_since_epoch().count(); // system epoch time in nanosecond
            
            auto systemTime = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()); // system epoch time in seconds
            uint64_t radar_time_epoch = static_cast<uint64_t>(systemTime) * 1000000000 + timestamp.u_nsec();
            // std::cout<<"radar_epoch time is: "<<radar_time_epoch<<std::endl;
            // std::cout<<"epoch time is: "<<systemTime<<std::endl;
            std::tm* timeInfo = std::localtime(&systemTime);
            
            char buffer[80];
            std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", timeInfo);
            
            std::string radar_time_std = buffer;
            radar_time_std += "." + std::to_string(timestamp.u_nsec());
            // std::cout<<"formatted time is: "<<radar_time_std<<std::endl;
            // if (range < minRange)
            // {
            // // Update the minimum range and corresponding object
            // minRange = range;
            // minRangeObject = ro;
        
            // std::cout<<"minimumrange="<<minRange<<std::endl;


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
            outputFile << timestamp.u_sec() << "," << timestamp.u_nsec() << ","<<radar_time_std << ","<<radar_time_epoch << ","
                       << minRangeObject.u_objid() << "," << minRangeObject.f_distx() << "," << minRangeObject.f_disty() << ","
                       << minRangeObject.f_rcs() << "," << rangeA << ","<<vectorSize<<","<<minRangeObject.e_dynamicproperty()<<","<<minRangeObject.f_orientation()<<","<<obj_size<<","<<minRangeObject.f_objectscore()
                       // Include other relevant information
                       << std::endl;
            
            //////////////////////////////////////// ACC starts /////////////////////
            bool acc_switch = false, brake_action = false;
            if (rangeA < max_range && rangeA > min_range){
                acc_switch = true;
                double brake_point = (max_range+min_range)/2;
                if (rangeA<brake_point){
                    brake_action = true;
                    
                }
                else{
                    brake_action = false;
                }
            }
            else{
                acc_switch = false;
            }
            

            }

            // std::cout << "Object within 10 meters range:" << std::endl
            //           << "u_ObjId " << ro.u_objid() << std::endl
            //           << "f_DistX " << ro.f_distx() << " m" << std::endl
            //           << "f_DistY " << ro.f_disty() << " m" << std::endl
            //           << "Calculated Range: " << range << " m" << std::endl
            //           << "f_VrelX " << ro.f_vrelx() << " m/s" << std::endl
            //           << "f_VrelY " << ro.f_vrely() << " m/s" << std::endl
            //           // Include other relevant information
            //           << std::endl;
                      
            // outputFile1 << timestamp.u_sec() << "," << timestamp.u_nsec() << ","<<radar_time_std << ","<<radar_time_epoch << ","
            //            << ro.u_objid() << "," << ro.f_distx() << "," << ro.f_disty() << ","
            //            << ro.f_rcs() << "," << range << ","<<ro.e_dynamicproperty()<<","<<ro.f_orientation()<<","<<obj_size<<","<<ro.f_objectscore()
            //            // Include other relevant information
            //            << std::endl;
                      
        }
        // else{
        //     std::cout<<"No object of interest: "<<filteredObjects.empty()<<std::endl;
        //     // publish acc switch to be OFF that is no object detected
        // }

    }

    // Close the CSV file
    outputFile.close();
}




int main(int argc, char *argv[])
{
    std::ofstream outputFile("radar_data.csv", std::ios::out | std::ios::trunc);
    outputFile << "Timestamp_Sec, Timestamp_Nsec, Formatted_Time, Radar_Epoch_Time, Obj_ID, Dist_X, Dist_Y, RCS, Range, Vectorsize, Dynamicproperty, Orientation, Objectsize, objectscore" << std::endl;
    outputFile.close();

    std::ofstream outputFile1("radar_data1.csv", std::ios::out | std::ios::trunc);
    outputFile1 << "Timestamp_Sec, Timestamp_Nsec, Formatted_Time, Radar_Epoch_Time, Obj_ID, Dist_X, Dist_Y, RCS, Range, Dynamicproperty, Orientation, Objectsize, objectscore" << std::endl;
    outputFile1.close();
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

    // PDK::CInterface::SetRDICallback(OnReceiveDetections);
    // PDK::CInterface::SetRDICallback(OnReceiveRadarImage);
    PDK::CInterface::SetSensorObjectsCallback(OnReceiveObjects);
    // std::this_thread::sleep_for(std::chrono::seconds(15));

    while (true)
    {
        //std::cout << (PDK::CInterface::PublishVehicleDynamics(vehicle_dynamics) ? "sent" :  "not_sent") << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Rest of the code...

    return 0;
}
