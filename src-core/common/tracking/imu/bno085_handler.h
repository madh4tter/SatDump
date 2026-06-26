#pragma once

#include "imu_handler.h"
#include <mutex>
#include <thread>
#include <atomic>

namespace imu
{
    class Bno085Handler : public ImuHandler
    {
    private:
        // I2C file descriptors
        int fd     = -1;
        int rst_fd = -1;

        // Settings
        char  input_device[100]   = "/dev/i2c-1";
        int   input_address       = 0x4A;
        int   input_rst_gpio      = 22;   // GPIO22 = Pi physical pin 15
        float input_declination   = 0.0f;
        float input_offset_heading= 0.0f;
        float input_offset_pitch  = 0.0f;

        // Report interval (default 50ms = 20Hz)
        uint32_t report_interval_us = 50000;

        // SHTP state
        uint8_t seq[6]        = {0};
        uint8_t shtp_data[300]= {0};
        uint16_t shtp_len     = 0;
        uint8_t  shtp_channel = 0;
        uint8_t  shtp_seq     = 0;
        bool reset_complete   = false;

        // Latest parsed values
        std::mutex val_mtx;
        float    last_heading  = 0.0f;
        float    last_pitch    = 0.0f;
        int      last_accuracy = 0;
        bool     has_data      = false;

        // Reader thread
        std::atomic<bool> reader_should_run{false};
        std::thread       reader_thread;

        // Low-level I2C
        bool i2c_read(uint8_t *buf, uint16_t len);
        bool i2c_write(const uint8_t *buf, uint16_t len);

        // SHTP protocol (based on proven SparkFun BNO080 Arduino library)
        bool shtp_send(uint8_t channel, const uint8_t *payload, uint8_t len);
        bool shtp_receive();

        // Startup and feature enable
        bool startup();
        bool enable_rotation_vector(uint32_t interval_us);

        // Rotation vector parsing
        void parse_rotation_vector(const uint8_t *data, uint16_t len);

        // Reader thread entry
        void reader_run();

        // Connect/disconnect implementation
        void l_connect(const char *device, int address, int rst_gpio);
        void l_disconnect();

    public:
        Bno085Handler();
        ~Bno085Handler();

        std::string   get_id()                          override;
        void          set_settings(nlohmann::json s)    override;
        nlohmann::json get_settings()                   override;
        imu_status_t  get_pos(float *heading, float *pitch,
                               int *cal_sys, int *cal_mag) override;
        void          render()                          override;
        bool          is_connected()                    override;
        void          connect()                         override;
        void          disconnect()                      override;
    };
}