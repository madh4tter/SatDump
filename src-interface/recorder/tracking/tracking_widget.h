#pragma once

#include "common/geodetic/geodetic_coordinates.h"
#include "init.h"
#include "libs/predict/predict.h"
#include "nlohmann/json.hpp"
#include "nlohmann/json_utils.h"
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "common/tracking/obj_tracker/object_tracker.h"
#include "common/tracking/sat_finder/sat_finder_ui.h"
#include "common/tracking/scheduler/scheduler.h"
#include "common/tracking/imu/imu_handler.h"
#include "common/tracking/gps/gps_handler.h"
#include "import_export.h"

namespace satdump
{
    class TrackingWidget
    {
    private: // QTH Config
        double qth_lon = 0;
        double qth_lat = 0;
        double qth_alt = 0;

    private: // Config Exporter
        TrackingImportExport config_import_export;

    public: // Handlers
        std::function<void(AutoTrackCfg, SatellitePass, TrackedObject)> aos_callback = [](AutoTrackCfg, SatellitePass, TrackedObject) {};
        std::function<void(AutoTrackCfg, SatellitePass, TrackedObject)> los_callback = [](AutoTrackCfg, SatellitePass, TrackedObject) {};

    private:
        ObjectTracker object_tracker = ObjectTracker(true);
        AutoTrackScheduler auto_scheduler;

        SatFinder sat_finder;

        std::shared_ptr<rotator::RotatorHandler> rotator_handler;
        std::vector<rotator::RotatorHandlerOption> rotator_options;
        std::string rotator_options_str;
        int selected_rotator_handler = 0;

        std::shared_ptr<imu::ImuHandler> imu_handler;
        std::vector<imu::ImuHandlerOption> imu_options;
        std::string imu_options_str;
        int selected_imu_handler = 0;

        std::shared_ptr<gps::GpsHandler> gps_handler;
        std::mutex gps_handler_mtx;
        std::vector<gps::GpsHandlerOption> gps_options;
        std::string gps_options_str;
        int selected_gps_handler = 0;
        bool gps_live_qth_enabled = false; // when true, GPS fixes overwrite qth_lon/lat/alt live
        double gps_min_move_meters = 50.0; // only push setQTH() when the fix has moved at least this far
        double gps_min_update_period_s = 5.0; // ...and at most this often

        bool gpsth_should_run = true;
        std::thread gpsth_thread;
        void gpsth_run();

        bool config_window_was_asked = false, show_window_config = false;
        bool satellite_finder_was_asked = false, show_satellite_finder_window = false;

    private:
        void saveConfig()
        {
            nlohmann::ordered_json cfg;
            cfg["enabled_objects"] = auto_scheduler.getTracked();
            cfg["rotator_algo"] = object_tracker.getRotatorConfig();
            cfg["autotrack_cfg"] = auto_scheduler.getAutoTrackCfg();
            if (rotator_handler)
                cfg["rotator_config"][rotator_handler->get_id()] = rotator_handler->get_settings();
            if (imu_handler)
                cfg["imu_config"][imu_handler->get_id()] = imu_handler->get_settings();
            if (gps_handler)
                cfg["gps_config"][gps_handler->get_id()] = gps_handler->get_settings();
            cfg["gps_live_qth_enabled"] = gps_live_qth_enabled;

            db->set_user_json("recorder_tracking", cfg);
        }

        void loadConfig()
        {
            nlohmann::json cfg = db->get_user_json("recorder_tracking");
            if (!cfg.is_null())
            {
                auto enabled_satellites = getValueOrDefault(cfg["enabled_objects"], std::vector<TrackedObject>());
                nlohmann::json rotator_algo_cfg;
                if (cfg.contains("rotator_algo"))
                    rotator_algo_cfg = cfg["rotator_algo"];

                auto_scheduler.setTracked(enabled_satellites);
                object_tracker.setRotatorConfig(rotator_algo_cfg);
                auto_scheduler.setAutoTrackCfg(getValueOrDefault<AutoTrackCfg>(cfg["autotrack_cfg"], AutoTrackCfg()));
                gps_live_qth_enabled = getValueOrDefault(cfg["gps_live_qth_enabled"], gps_live_qth_enabled);
            }
        }

    public:
        TrackingWidget();
        ~TrackingWidget();

        void render();
        void renderConfig();
        void renderSatfinder();
    };
} // namespace satdump
