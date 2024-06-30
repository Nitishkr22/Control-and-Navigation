#include "pdk/pdk_interface.hpp"
#include "pdk/VehicleDynamics.hpp"
#include <chrono>
#include <iostream>
#include <thread>
int main(int argc, char *argv[])
{
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
    float f_LongVel = 5.0f;
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
         PDK::CVehicleDynamicsSignal::eSignalState::VALID, f_LatAccel}};
    while (true)
    {
        std::cout << (PDK::CInterface::PublishVehicleDynamics(vehicle_dynamics) ? "sent" :  "not_sent") << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}