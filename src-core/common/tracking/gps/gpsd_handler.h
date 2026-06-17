#pragma once

#include "gps_handler.h"
#include <mutex>
#include <atomic>
#include <thread>

namespace gps
{
    // Connects to a running gpsd instance (https://gpsd.io/) over its JSON TCP protocol,
    // typically localhost:2947. gpsd handles all the NMEA/UBX parsing and multi-constellation
    // fusion itself -- this handler just consumes its "TPV" (Time-Position-Velocity) reports.
    //
    // Requires gpsd to be running and pointed at the actual GPS device, e.g.:
    //   gpsd /dev/ttyAMA0 -F /var/run/gpsd.sock
    class GpsdHandler : public GpsHandler
    {
    private:
        char input_host[100] = "127.0.0.1";
        int input_port = 2947;

        int sock_fd = -1;
        std::mutex sock_mtx;

        std::atomic<bool> reader_should_run{false};
        std::thread reader_thread;

        std::mutex fix_mtx;
        double last_lat = 0, last_lon = 0, last_alt = 0;
        bool has_fix = false;

        void reader_run();
        void l_connect(const char *host, int port);
        void l_disconnect();

    public:
        GpsdHandler();
        ~GpsdHandler();

        std::string get_id() override;
        void set_settings(nlohmann::json settings) override;
        nlohmann::json get_settings() override;
        gps_status_t get_fix(double *lat, double *lon, double *alt) override;
        void render() override;
        bool is_connected() override;
        void connect() override;
        void disconnect() override;
    };
}
