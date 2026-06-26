#pragma once

#include "imu_handler.h"
#include <mutex>

namespace imu
{
    // Reads orientation (heading, pitch) and calibration status from a Bosch BNO055
    // over Linux I2C (e.g. /dev/i2c-1 on a Raspberry Pi). Sensor fusion is done onboard
    // the BNO055's own Cortex-M0, so this handler is just a register reader -- no AHRS
    // filtering happens on the Pi.
    class Bno055Handler : public ImuHandler
    {
    private:
        int fd = -1;
        char input_device[100] = "/dev/i2c-1";
        int input_address = 0x28;
        float input_declination = 0.0f;  // local magnetic declination, added to raw heading
        float input_offset_heading = 0.0f; // mounting correction, added after declination
        float input_offset_pitch = 0.0f;   // mounting correction

        std::mutex dev_mtx;

        bool writeReg(uint8_t reg, uint8_t val);
        bool readBytes(uint8_t reg, uint8_t *buf, int len);

        void l_connect(const char *device, int address);
        void l_disconnect();

    public:
        Bno055Handler();
        ~Bno055Handler();

        std::string get_id() override;
        void set_settings(nlohmann::json settings) override;
        nlohmann::json get_settings() override;
        imu_status_t get_pos(float *heading, float *pitch, int *cal_sys, int *cal_mag) override;
        void render() override;
        bool is_connected() override;
        void connect() override;
        void disconnect() override;
    };
}