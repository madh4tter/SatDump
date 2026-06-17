#pragma once

#include <string>
#include <functional>
#include <vector>
#include <memory>
#include "nlohmann/json.hpp"
#include "nlohmann/json_utils.h"

namespace imu
{
    enum imu_status_t
    {
        IMU_ERROR_OK = 0,  // No error
        IMU_ERROR_CMD = 1, // Read/command error
        IMU_ERROR_CON = 2, // Connection error
    };

    // Mirrors rotator::RotatorHandler. heading/pitch are in degrees:
    //   heading: compass heading the device/antenna boresight is facing, 0-360, true north
    //            (declination correction is the handler's responsibility, not the caller's)
    //   pitch:   angle above the horizontal plane, typically 0..90 for sky pointing
    // cal_sys/cal_mag: 0-3 calibration confidence (BNO055-style). Handlers without a notion of
    //   calibration (e.g. a simple tilt sensor) should just always report 3/3 once readings are valid.
    class ImuHandler
    {
    public:
        virtual std::string get_id() = 0;
        virtual void set_settings(nlohmann::json settings) = 0;
        virtual nlohmann::json get_settings() = 0;
        virtual imu_status_t get_pos(float *heading, float *pitch, int *cal_sys, int *cal_mag) = 0;
        virtual void render() = 0;
        virtual bool is_connected() = 0;
        virtual void connect() = 0;
        virtual void disconnect() = 0;
        virtual ~ImuHandler() {}
    };

    struct ImuHandlerOption
    {
        std::string name;
        std::function<std::shared_ptr<ImuHandler>()> construct;
    };

    struct RequestImuHandlerOptionsEvent
    {
        std::vector<ImuHandlerOption> &opts;
    };

    std::vector<ImuHandlerOption> getImuHandlerOptions();
}
