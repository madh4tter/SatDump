#include "gpsd_handler.h"
#include "imgui/imgui.h"
#include "core/style.h"
#include "logger.h"

#ifdef __linux__
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <netdb.h>
#endif

namespace gps
{
    GpsdHandler::GpsdHandler()
    {
    }

    GpsdHandler::~GpsdHandler()
    {
        l_disconnect();
    }

    void GpsdHandler::l_connect(const char *host, int port)
    {
#ifdef __linux__
        l_disconnect();

        struct addrinfo hints, *res = nullptr;
        memset(&hints, 0, sizeof(hints));
        hints.ai_family = AF_UNSPEC;
        hints.ai_socktype = SOCK_STREAM;

        char port_str[16];
        snprintf(port_str, sizeof(port_str), "%d", port);

        if (getaddrinfo(host, port_str, &hints, &res) != 0)
        {
            logger->error("Could not resolve gpsd host %s", host);
            return;
        }

        int fd = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
        if (fd < 0)
        {
            logger->error("Could not create socket for gpsd connection");
            freeaddrinfo(res);
            return;
        }

        if (::connect(fd, res->ai_addr, res->ai_addrlen) < 0)
        {
            logger->error("Could not connect to gpsd at %s:%d", host, port);
            ::close(fd);
            freeaddrinfo(res);
            return;
        }
        freeaddrinfo(res);

        // Enable streaming JSON TPV reports
        const char *watch_cmd = "?WATCH={\"enable\":true,\"json\":true};\n";
        if (::send(fd, watch_cmd, strlen(watch_cmd), 0) < 0)
        {
            logger->error("Could not send WATCH command to gpsd");
            ::close(fd);
            return;
        }

        {
            std::lock_guard<std::mutex> lock(sock_mtx);
            sock_fd = fd;
        }

        {
            std::lock_guard<std::mutex> lock(fix_mtx);
            has_fix = false;
        }

        reader_should_run = true;
        reader_thread = std::thread(&GpsdHandler::reader_run, this);
#else
        (void)host;
        (void)port;
        logger->error("gpsd support is only implemented for Linux");
#endif
    }

    void GpsdHandler::l_disconnect()
    {
        reader_should_run = false;

#ifdef __linux__
        {
            std::lock_guard<std::mutex> lock(sock_mtx);
            if (sock_fd >= 0)
            {
                // Unblock any pending recv() in the reader thread
                ::shutdown(sock_fd, SHUT_RDWR);
                ::close(sock_fd);
                sock_fd = -1;
            }
        }
#endif

        if (reader_thread.joinable())
            reader_thread.join();

        std::lock_guard<std::mutex> lock(fix_mtx);
        has_fix = false;
    }

    void GpsdHandler::reader_run()
    {
#ifdef __linux__
        std::string buffer;
        char chunk[4096];

        while (reader_should_run)
        {
            int fd;
            {
                std::lock_guard<std::mutex> lock(sock_mtx);
                fd = sock_fd;
            }
            if (fd < 0)
                break;

            ssize_t n = ::recv(fd, chunk, sizeof(chunk) - 1, 0);
            if (n <= 0)
            {
                // Connection closed or error -- stop, leave reconnect to the user via UI
                break;
            }
            chunk[n] = '\0';
            buffer += chunk;

            // gpsd sends newline-delimited JSON objects
            size_t pos;
            while ((pos = buffer.find('\n')) != std::string::npos)
            {
                std::string line = buffer.substr(0, pos);
                buffer.erase(0, pos + 1);

                if (line.empty())
                    continue;

                try
                {
                    nlohmann::json j = nlohmann::json::parse(line);
                    if (j.contains("class") && j["class"] == "TPV")
                    {
                        // mode: 0=unknown, 1=no fix, 2=2D fix, 3=3D fix
                        int mode = j.contains("mode") ? j["mode"].get<int>() : 0;
                        if (mode >= 2 && j.contains("lat") && j.contains("lon"))
                        {
                            std::lock_guard<std::mutex> lock(fix_mtx);
                            last_lat = j["lat"].get<double>();
                            last_lon = j["lon"].get<double>();
                            // altMSL (mean sea level) preferred; fall back to "alt" (older gpsd) or 0
                            if (j.contains("altMSL"))
                                last_alt = j["altMSL"].get<double>();
                            else if (j.contains("alt"))
                                last_alt = j["alt"].get<double>();
                            else
                                last_alt = 0;
                            has_fix = true;
                        }
                        else
                        {
                            std::lock_guard<std::mutex> lock(fix_mtx);
                            has_fix = false;
                        }
                    }
                }
                catch (std::exception &)
                {
                    // Malformed/partial line, ignore and continue
                }
            }
        }

        {
            std::lock_guard<std::mutex> lock(fix_mtx);
            has_fix = false;
        }
#endif
    }

    std::string GpsdHandler::get_id()
    {
        return "gpsd";
    }

    void GpsdHandler::set_settings(nlohmann::json settings)
    {
        std::string vhost = getValueOrDefault(settings["host"], std::string(input_host));
        memcpy(input_host, vhost.data(), std::min(vhost.size() + 1, sizeof(input_host)));
        input_port = getValueOrDefault(settings["port"], input_port);
    }

    nlohmann::json GpsdHandler::get_settings()
    {
        nlohmann::json v;
        v["host"] = std::string(input_host);
        v["port"] = input_port;
        return v;
    }

    gps_status_t GpsdHandler::get_fix(double *lat, double *lon, double *alt)
    {
        if (!is_connected())
            return GPS_ERROR_CON;

        std::lock_guard<std::mutex> lock(fix_mtx);
        if (!has_fix)
            return GPS_ERROR_NOFIX;

        *lat = last_lat;
        *lon = last_lon;
        *alt = last_alt;
        return GPS_ERROR_OK;
    }

    void GpsdHandler::render()
    {
        bool connected = is_connected();

        if (connected)
            style::beginDisabled();
        ImGui::InputText("gpsd Host##gpsdhost", input_host, sizeof(input_host));
        ImGui::InputInt("gpsd Port##gpsdport", &input_port);
        if (connected)
            style::endDisabled();

        if (connected)
        {
            bool fixed;
            double lat, lon, alt;
            {
                std::lock_guard<std::mutex> lock(fix_mtx);
                fixed = has_fix;
                lat = last_lat;
                lon = last_lon;
                alt = last_alt;
            }

            if (fixed)
                ImGui::TextColored(style::theme.green.Value, "Fix: %.5f, %.5f, %.0fm", lat, lon, alt);
            else
                ImGui::TextColored(style::theme.yellow.Value, "Connected, waiting for fix...");

            if (ImGui::Button("Disconnect##gpsddisconnect"))
                disconnect();
        }
        else
        {
            if (ImGui::Button("Connect##gpsdconnect"))
                connect();
        }
    }

    bool GpsdHandler::is_connected()
    {
        std::lock_guard<std::mutex> lock(sock_mtx);
        return sock_fd >= 0;
    }

    void GpsdHandler::connect()
    {
        l_connect(input_host, input_port);
    }

    void GpsdHandler::disconnect()
    {
        l_disconnect();
    }
}
