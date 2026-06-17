#pragma once

#include <string>
#include <functional>
#include <vector>
#include <memory>
#include "nlohmann/json.hpp"
#include "nlohmann/json_utils.h"

namespace gps
{
    enum gps_status_t
    {
        GPS_ERROR_OK = 0,  // No error, fix is valid
        GPS_ERROR_NOFIX = 1, // Connected but no valid fix yet
        GPS_ERROR_CON = 2,   // Connection error
    };

    // Mirrors imu::ImuHandler / rotator::RotatorHandler. lat/lon in degrees, alt in meters.
    class GpsHandler
    {
    public:
        virtual std::string get_id() = 0;
        virtual void set_settings(nlohmann::json settings) = 0;
        virtual nlohmann::json get_settings() = 0;
        virtual gps_status_t get_fix(double *lat, double *lon, double *alt) = 0;
        virtual void render() = 0;
        virtual bool is_connected() = 0;
        virtual void connect() = 0;
        virtual void disconnect() = 0;
        virtual ~GpsHandler() {}
    };

    struct GpsHandlerOption
    {
        std::string name;
        std::function<std::shared_ptr<GpsHandler>()> construct;
    };

    struct RequestGpsHandlerOptionsEvent
    {
        std::vector<GpsHandlerOption> &opts;
    };

    std::vector<GpsHandlerOption> getGpsHandlerOptions();
}
