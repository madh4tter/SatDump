#include "bno085_handler.h"
#include "imgui/imgui.h"
#include "core/style.h"
#include "logger.h"
#include <cmath>
#include <cstring>
#include <chrono>

#ifdef __linux__
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#endif

// Max I2C read chunk — Linux i2c-dev supports up to 8192 bytes,
// but we mirror SparkFun's conservative 32-byte chunk approach
// which is proven to work reliably with the BNO085's clock stretching.
static const uint16_t I2C_CHUNK = 32;
static const uint16_t MAX_PACKET = 300; // BNO085 can send up to ~276 bytes

// SHTP channel numbers (BNO085 datasheet Figure 1-26)
static const uint8_t CH_COMMAND    = 0;
static const uint8_t CH_EXECUTABLE = 1;
static const uint8_t CH_CONTROL    = 2;
static const uint8_t CH_REPORTS    = 3;

// SH-2 report IDs
static const uint8_t REPORT_PRODUCT_ID_REQUEST  = 0xF9;
static const uint8_t REPORT_PRODUCT_ID_RESPONSE = 0xF8;
static const uint8_t REPORT_SET_FEATURE         = 0xFD;
static const uint8_t REPORT_ROTATION_VECTOR     = 0x05;
static const uint8_t REPORT_BASE_TIMESTAMP      = 0xFB;
static const uint8_t EXECUTABLE_RESET_COMPLETE  = 0x1;

// Q14 scale for quaternion components
static const float Q14 = 1.0f / 16384.0f;

namespace imu
{
    // ── Low-level I2C helpers ─────────────────────────────────────────────────

    bool Bno085Handler::i2c_read(uint8_t *buf, uint16_t len)
    {
#ifdef __linux__
        ssize_t n = ::read(fd, buf, len);
        return n == (ssize_t)len;
#else
        (void)buf; (void)len; return false;
#endif
    }

    bool Bno085Handler::i2c_write(const uint8_t *buf, uint16_t len)
    {
#ifdef __linux__
        ssize_t n = ::write(fd, buf, len);
        return n == (ssize_t)len;
#else
        (void)buf; (void)len; return false;
#endif
    }

    // ── SHTP packet send ──────────────────────────────────────────────────────
    // Mirrors SparkFun's sendPacket() exactly:
    // Write [len_lsb, len_msb, channel, seq] + payload as one I2C transaction.

    bool Bno085Handler::shtp_send(uint8_t channel, const uint8_t *payload, uint8_t payload_len)
    {
        uint8_t packet_len = payload_len + 4;
        uint8_t buf[256];
        buf[0] = packet_len & 0xFF;
        buf[1] = (packet_len >> 8) & 0xFF;
        buf[2] = channel;
        buf[3] = seq[channel]++;
        memcpy(buf + 4, payload, payload_len);
        return i2c_write(buf, packet_len);
    }

    // ── SHTP packet receive ───────────────────────────────────────────────────
    // Mirrors SparkFun's receivePacket() + getData() exactly:
    //   1. Read 4-byte header to get total packet length.
    //   2. Read remaining payload in I2C_CHUNK-byte chunks,
    //      each chunk prefixed by a fresh 4-byte header (which we discard).
    // This is the key detail our previous hand-rolled code got wrong.

    bool Bno085Handler::shtp_receive()
    {
        // Step 1: read 4-byte header
        uint8_t hdr[4];
        if (!i2c_read(hdr, 4))
            return false;

        uint16_t pkt_len = ((uint16_t)hdr[1] << 8) | hdr[0];
        pkt_len &= 0x7FFF; // strip continuation bit

        shtp_channel = hdr[2];
        shtp_seq     = hdr[3];

        if (pkt_len == 0)
        {
            shtp_len = 0;
            return false;
        }
        if (pkt_len <= 4)
        {
            shtp_len = 0;
            return true;  // valid empty packet, don't abort the flush
        }
        uint16_t data_len = pkt_len - 4;
        if (data_len > MAX_PACKET)
            data_len = MAX_PACKET;

        // Step 2: read payload in chunks, discarding the 4-byte header
        // re-sent at the start of each chunk (SparkFun getData() pattern)
        uint16_t remaining = data_len;
        uint16_t spot = 0;

        while (remaining > 0)
        {
            uint16_t to_read = remaining;
            if (to_read > I2C_CHUNK - 4)
                to_read = I2C_CHUNK - 4;

            uint8_t chunk[I2C_CHUNK];
            uint16_t chunk_total = to_read + 4;
            if (!i2c_read(chunk, chunk_total))
                return false;

            // First 4 bytes of each chunk are a repeated header — discard
            memcpy(shtp_data + spot, chunk + 4, to_read);
            spot      += to_read;
            remaining -= to_read;
        }

        shtp_len = data_len;

        // Detect reset-complete packet (SparkFun checks this in receivePacket)
        if (shtp_channel == CH_EXECUTABLE && shtp_len > 0 &&
            shtp_data[0] == EXECUTABLE_RESET_COMPLETE)
            reset_complete = true;

        return true;
    }

    // ── Startup sequence ──────────────────────────────────────────────────────
    // Mirrors SparkFun's softReset() then begin() Product ID check.
    // No advertisement parsing needed — just flush then check comms.
 bool Bno085Handler::startup()
    {
        reset_complete = false;
        memset(seq, 0, sizeof(seq));

        // Hardware reset via RST pin if wired (GPIO sysfs)
#ifdef __linux__
        if (rst_fd >= 0)
        {
            uint8_t low = '0', high = '1';
            ::write(rst_fd, &low, 1);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            ::write(rst_fd, &high, 1);
        }
        else
#endif
        {
            uint8_t rst_payload = 0x01;
            shtp_send(CH_EXECUTABLE, &rst_payload, 1);
        }
        // Always wait for reset regardless of method
        std::this_thread::sleep_for(std::chrono::milliseconds(300));

        // Software reset: send 0x01 on executable channel (SparkFun softReset)
        uint8_t rst_payload = 0x01;
        shtp_send(CH_EXECUTABLE, &rst_payload, 1);

        // SparkFun softReset: delay(50), flush, delay(50), flush
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        while (shtp_receive()) {}
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        while (shtp_receive()) {}

        // SparkFun begin(): send Product ID request immediately after softReset
        // No extra flushing - softReset already cleared the queue
        uint8_t pid[2] = {REPORT_PRODUCT_ID_REQUEST, 0x00};
        shtp_send(CH_CONTROL, pid, 2);

        // SparkFun: single receivePacket() call, check for response
        // We retry a few times to account for Linux scheduling jitter
        for (int i = 0; i < 10; i++)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            if (shtp_receive() &&
                shtp_channel == CH_CONTROL &&
                shtp_len >= 16 &&
                shtp_data[0] == REPORT_PRODUCT_ID_RESPONSE)
            {
                logger->info("BNO085: Product ID confirmed, SW %d.%d",
                             shtp_data[2], shtp_data[3]);
                return true;
            }
        }

        logger->error("BNO085: No Product ID response after startup");
        return false;
    }

    // ── Enable rotation vector report ────────────────────────────────────────
    // SparkFun setFeatureCommand() translated directly to Linux I2C.

    bool Bno085Handler::enable_rotation_vector(uint32_t interval_us)
    {
        uint8_t payload[17];
        payload[0]  = REPORT_SET_FEATURE;
        payload[1]  = REPORT_ROTATION_VECTOR;
        payload[2]  = 0; // feature flags
        payload[3]  = 0; // change sensitivity LSB
        payload[4]  = 0; // change sensitivity MSB
        payload[5]  = (interval_us >>  0) & 0xFF;
        payload[6]  = (interval_us >>  8) & 0xFF;
        payload[7]  = (interval_us >> 16) & 0xFF;
        payload[8]  = (interval_us >> 24) & 0xFF;
        payload[9]  = 0; // batch interval
        payload[10] = 0;
        payload[11] = 0;
        payload[12] = 0;
        payload[13] = 0; // sensor-specific config
        payload[14] = 0;
        payload[15] = 0;
        payload[16] = 0;

        return shtp_send(CH_CONTROL, payload, 17);
    }

    // ── Reader thread ─────────────────────────────────────────────────────────

    void Bno085Handler::reader_run()
    {
#ifdef __linux__
        if (!startup() || !reader_should_run)
        {
            logger->error("BNO085: Startup failed, reader thread exiting");
            return;
        }

        // Enable rotation vector at requested interval
        if (!enable_rotation_vector(report_interval_us) || !reader_should_run)
        {
            logger->error("BNO085: Failed to enable rotation vector");
            return;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        while (reader_should_run)
        {
            if (!shtp_receive())
            {
                // No data / error — brief pause to avoid hammering the bus
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }

            // Rotation vector is on CH_REPORTS (3)
            // The sensor often prepends a REPORT_BASE_TIMESTAMP (0xFB) on the
            // same channel — SparkFun's parseInputReport skips it, we do too.
            if (shtp_channel == CH_REPORTS && shtp_len >= 14)
            {
                if (shtp_data[0] == REPORT_BASE_TIMESTAMP && shtp_len >= 19)
                {
                    // Timestamp (5 bytes) followed immediately by sensor report
                    parse_rotation_vector(shtp_data + 5, shtp_len - 5);
                }
                else if (shtp_data[0] == REPORT_ROTATION_VECTOR)
                {
                    parse_rotation_vector(shtp_data, shtp_len);
                }
            }
        }
#endif
    }

    void Bno085Handler::parse_rotation_vector(const uint8_t *data, uint16_t len)
    {
        if (len < 14 || data[0] != REPORT_ROTATION_VECTOR)
            return;

        int accuracy = data[2] & 0x03; // bits 1:0 of status byte

        // Quaternion components: signed int16, Q14 scale
    int16_t qi_raw = (int16_t)(data[5]  | (data[6]  << 8));
    int16_t qj_raw = (int16_t)(data[7]  | (data[8]  << 8));
    int16_t qk_raw = (int16_t)(data[9]  | (data[10] << 8));
    int16_t qr_raw = (int16_t)(data[11] | (data[12] << 8));

        float qi = qi_raw * Q14;
        float qj = qj_raw * Q14;
        float qk = qk_raw * Q14;
        float qr = qr_raw * Q14;

        // Quaternion → yaw (heading) and pitch
        float yaw_rad = atan2f(2.0f*(qr*qk + qi*qj), 
                                1.0f - 2.0f*(qj*qj + qk*qk));
        float pitch_rad = asinf(fmaxf(-1.0f, fminf(1.0f,
                                  2.0f * (qr*qj - qk*qi))));

        float heading = yaw_rad * (180.0f / (float)M_PI);
        float pitch   = pitch_rad * (180.0f / (float)M_PI);

        // Apply declination + mounting offsets, normalise heading to 0-360
        heading += input_declination + input_offset_heading;
        heading  = fmodf(heading + 360.0f, 360.0f);
        pitch   += input_offset_pitch;

        std::lock_guard<std::mutex> lock(val_mtx);
        last_heading  = heading;
        last_pitch    = pitch;
        last_accuracy = accuracy;
        has_data      = true;
    }

    // ── Connect / disconnect ──────────────────────────────────────────────────

    void Bno085Handler::l_connect(const char *device, int address, int rst_gpio)
    {
#ifdef __linux__
        l_disconnect();

        fd = ::open(device, O_RDWR);
        if (fd < 0)
        {
            logger->error("BNO085: Cannot open I2C device %s", device);
            return;
        }
        if (ioctl(fd, I2C_SLAVE, address) < 0)
        {
            logger->error("BNO085: Cannot set I2C slave address 0x%02X", address);
            ::close(fd); fd = -1;
            return;
        }

        // Open RST GPIO via sysfs if configured
        rst_fd = -1;
        if (rst_gpio >= 0)
        {
            // Export GPIO
            int export_fd = ::open("/sys/class/gpio/export", O_WRONLY);
            if (export_fd >= 0)
            {
                char gpio_str[8];
                snprintf(gpio_str, sizeof(gpio_str), "%d", rst_gpio);
                ::write(export_fd, gpio_str, strlen(gpio_str));
                ::close(export_fd);
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }

            // Set direction to output
            char dir_path[64];
            snprintf(dir_path, sizeof(dir_path), "/sys/class/gpio/gpio%d/direction", rst_gpio);
            int dir_fd = ::open(dir_path, O_WRONLY);
            if (dir_fd >= 0)
            {
                ::write(dir_fd, "out", 3);
                ::close(dir_fd);
            }

            // Open value file
            char val_path[64];
            snprintf(val_path, sizeof(val_path), "/sys/class/gpio/gpio%d/value", rst_gpio);
            rst_fd = ::open(val_path, O_WRONLY);
        }

        memset(seq, 0, sizeof(seq));
        reset_complete = false;

        reader_should_run = true;
        reader_thread = std::thread(&Bno085Handler::reader_run, this);
#else
        (void)device; (void)address; (void)rst_gpio;
        logger->error("BNO085: I2C only supported on Linux");
#endif
    }

    void Bno085Handler::l_disconnect()
    {
        reader_should_run = false;

#ifdef __linux__
        if (fd >= 0) { ::close(fd); fd = -1; }
        if (rst_fd >= 0) { ::close(rst_fd); rst_fd = -1; }
#endif

        if (reader_thread.joinable())
            reader_thread.join();

        std::lock_guard<std::mutex> lock(val_mtx);
        has_data = false;
    }

    // ── ImuHandler interface ──────────────────────────────────────────────────

    Bno085Handler::Bno085Handler() {}
    Bno085Handler::~Bno085Handler() { l_disconnect(); }

    std::string Bno085Handler::get_id() { return "bno085"; }

    void Bno085Handler::set_settings(nlohmann::json s)
    {
        std::string v = getValueOrDefault(s["device"], std::string(input_device));
        memcpy(input_device, v.data(), std::min(v.size()+1, sizeof(input_device)));
        input_address        = getValueOrDefault(s["address"],        input_address);
        input_rst_gpio       = getValueOrDefault(s["rst_gpio"],       input_rst_gpio);
        input_declination    = getValueOrDefault(s["declination"],    input_declination);
        input_offset_heading = getValueOrDefault(s["offset_heading"], input_offset_heading);
        input_offset_pitch   = getValueOrDefault(s["offset_pitch"],   input_offset_pitch);
    }

    nlohmann::json Bno085Handler::get_settings()
    {
        nlohmann::json v;
        v["device"]         = std::string(input_device);
        v["address"]        = input_address;
        v["rst_gpio"]       = input_rst_gpio;
        v["declination"]    = input_declination;
        v["offset_heading"] = input_offset_heading;
        v["offset_pitch"]   = input_offset_pitch;
        return v;
    }

    imu_status_t Bno085Handler::get_pos(float *heading, float *pitch, int *cal_sys, int *cal_mag)
    {
        if (fd < 0) return IMU_ERROR_CON;
        std::lock_guard<std::mutex> lock(val_mtx);
        if (!has_data) return IMU_ERROR_CMD;
        *heading = last_heading;
        *pitch   = last_pitch;
        *cal_sys = last_accuracy;
        *cal_mag = last_accuracy;
        return IMU_ERROR_OK;
    }

    void Bno085Handler::render()
    {
        bool connected = is_connected();
        if (connected) style::beginDisabled();
        ImGui::InputText("I2C Device##bno085", input_device, sizeof(input_device));
        ImGui::InputInt("I2C Address##bno085addr", &input_address);
        ImGui::InputInt("RST GPIO (-1 = none)##bno085rst", &input_rst_gpio);
        if (connected) style::endDisabled();

        ImGui::InputFloat("Mag Declination##bno085decl", &input_declination);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Local magnetic declination in degrees (east = positive)");
        ImGui::InputFloat("Heading Offset##bno085offh", &input_offset_heading);
        ImGui::InputFloat("Pitch Offset##bno085offp",   &input_offset_pitch);

        if (connected)
        {
            std::lock_guard<std::mutex> lock(val_mtx);
            if (has_data)
            {
                auto col = [](int a) -> ImVec4 {
                    if (a >= 3) return style::theme.green.Value;
                    if (a >= 1) return style::theme.yellow.Value;
                    return {0.5f,0.5f,0.5f,1.0f};
                };
                ImGui::TextColored(col(last_accuracy),
                    "Heading: %.1f°  Pitch: %.1f°  Acc: %d/3",
                    last_heading, last_pitch, last_accuracy);
            }
            else
            {
                ImGui::TextColored(style::theme.yellow.Value, "Waiting for first report...");
            }
            if (ImGui::Button("Disconnect##bno085disc")) disconnect();
        }
        else
        {
            if (ImGui::Button("Connect##bno085conn")) connect();
        }
    }

    bool Bno085Handler::is_connected() { return fd >= 0; }

    void Bno085Handler::connect()
    {
        l_connect(input_device, input_address, input_rst_gpio);
    }

    void Bno085Handler::disconnect()
    {
        l_disconnect();
    }
}