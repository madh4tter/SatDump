#include "bno055_handler.h"
#include "imgui/imgui.h"
#include "core/style.h"
#include "logger.h"

#ifdef __linux__
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#endif

namespace imu
{
    // BNO055 register map (subset used here)
    static const uint8_t REG_OPR_MODE = 0x3D;
    static const uint8_t REG_CALIB_STAT = 0x35;
    static const uint8_t REG_EULER_H_LSB = 0x1A; // heading, roll, pitch follow as 6 bytes
    static const uint8_t MODE_NDOF = 0x0C;        // fused 9-DOF absolute orientation mode

    Bno055Handler::Bno055Handler()
    {
    }

    Bno055Handler::~Bno055Handler()
    {
        l_disconnect();
    }

    bool Bno055Handler::writeReg(uint8_t reg, uint8_t val)
    {
#ifdef __linux__
        uint8_t buf[2] = {reg, val};
        return ::write(fd, buf, 2) == 2;
#else
        (void)reg;
        (void)val;
        return false;
#endif
    }

    bool Bno055Handler::readBytes(uint8_t reg, uint8_t *buf, int len)
    {
#ifdef __linux__
        if (::write(fd, &reg, 1) != 1)
            return false;
        return ::read(fd, buf, len) == len;
#else
        (void)reg;
        (void)buf;
        (void)len;
        return false;
#endif
    }

    void Bno055Handler::l_connect(const char *device, int address)
    {
#ifdef __linux__
        std::lock_guard<std::mutex> lock(dev_mtx);
        if (fd >= 0)
        {
            ::close(fd);
            fd = -1;
        }

        fd = ::open(device, O_RDWR);
        if (fd < 0)
        {
            logger->error("Could not open I2C device %s", device);
            return;
        }

        if (ioctl(fd, I2C_SLAVE, address) < 0)
        {
            logger->error("Could not set I2C slave address 0x%02X", address);
            ::close(fd);
            fd = -1;
            return;
        }

        // Switch to NDOF fusion mode -- gives calibrated absolute heading/pitch/roll
        if (!writeReg(REG_OPR_MODE, MODE_NDOF))
        {
            logger->error("Could not set BNO055 to NDOF mode");
            ::close(fd);
            fd = -1;
            return;
        }
#else
        (void)device;
        (void)address;
        logger->error("BNO055 I2C support is only implemented for Linux");
#endif
    }

    void Bno055Handler::l_disconnect()
    {
#ifdef __linux__
        std::lock_guard<std::mutex> lock(dev_mtx);
        if (fd >= 0)
            ::close(fd);
        fd = -1;
#endif
    }

    std::string Bno055Handler::get_id()
    {
        return "bno055";
    }

    void Bno055Handler::set_settings(nlohmann::json settings)
    {
        std::string vdevice = getValueOrDefault(settings["device"], std::string(input_device));
        memcpy(input_device, vdevice.data(), std::min(vdevice.size() + 1, sizeof(input_device)));
        input_address = getValueOrDefault(settings["address"], input_address);
        input_declination = getValueOrDefault(settings["declination"], input_declination);
        input_offset_heading = getValueOrDefault(settings["offset_heading"], input_offset_heading);
        input_offset_pitch = getValueOrDefault(settings["offset_pitch"], input_offset_pitch);
    }

    nlohmann::json Bno055Handler::get_settings()
    {
        nlohmann::json v;
        v["device"] = std::string(input_device);
        v["address"] = input_address;
        v["declination"] = input_declination;
        v["offset_heading"] = input_offset_heading;
        v["offset_pitch"] = input_offset_pitch;
        return v;
    }

    imu_status_t Bno055Handler::get_pos(float *heading, float *pitch, int *cal_sys, int *cal_mag)
    {
        std::lock_guard<std::mutex> lock(dev_mtx);
        if (fd < 0)
            return IMU_ERROR_CON;

        uint8_t euler[6];
        if (!readBytes(REG_EULER_H_LSB, euler, 6))
            return IMU_ERROR_CMD;

        // Each value is a signed 16-bit little-endian, raw/16.0 = degrees
        int16_t raw_heading = (int16_t)(euler[0] | (euler[1] << 8));
        // bytes 2,3 = roll, not used here
        int16_t raw_pitch = (int16_t)(euler[4] | (euler[5] << 8));

        float h = (raw_heading / 16.0f) + input_declination + input_offset_heading;
        h = fmodf(h + 360.0f, 360.0f);

        // BNO055 reports pitch as device tilt; sign convention depends on physical mounting.
        // Adjust input_offset_pitch / flip sign here if your mount orientation differs.
        float p = (raw_pitch / 16.0f) + input_offset_pitch;

        uint8_t cal = 0;
        if (!readBytes(REG_CALIB_STAT, &cal, 1))
            return IMU_ERROR_CMD;

        *heading = h;
        *pitch = p;
        *cal_sys = (cal >> 6) & 0x03;
        *cal_mag = (cal >> 4) & 0x03;

        return IMU_ERROR_OK;
    }

    void Bno055Handler::render()
    {
        bool connected = is_connected();

        if (connected)
            style::beginDisabled();
        ImGui::InputText("I2C Device##bno055device", input_device, sizeof(input_device));
        ImGui::InputInt("I2C Address##bno055address", &input_address);
        if (connected)
            style::endDisabled();

        ImGui::InputFloat("Magnetic Declination##bno055decl", &input_declination);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Local magnetic declination in degrees, added to raw heading to get true north");

        ImGui::InputFloat("Heading Offset##bno055offh", &input_offset_heading);
        ImGui::InputFloat("Pitch Offset##bno055offp", &input_offset_pitch);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Corrects for physical misalignment between the sensor and the antenna boresight");

        if (connected)
        {
            if (ImGui::Button("Disconnect##bno055disconnect"))
                disconnect();
        }
        else
        {
            if (ImGui::Button("Connect##bno055connect"))
                connect();
        }
    }

    bool Bno055Handler::is_connected()
    {
        std::lock_guard<std::mutex> lock(dev_mtx);
        return fd >= 0;
    }

    void Bno055Handler::connect()
    {
        l_connect(input_device, input_address);
    }

    void Bno055Handler::disconnect()
    {
        l_disconnect();
    }
}
