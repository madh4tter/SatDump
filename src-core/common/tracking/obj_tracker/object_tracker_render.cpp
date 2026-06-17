#include "common/geodetic/geodetic_coordinates.h"
#include "common/tracking/tle.h"
#include "core/style.h"
#include "imgui/imgui.h"
#include "imgui/imgui_stdlib.h"
#include "init.h"
#include "object_tracker.h"
#include "utils/string.h"
#include "utils/time.h"

namespace satdump
{
    void ObjectTracker::renderPolarPlot()
    {
        int d_pplot_size = ImGui::GetWindowContentRegionMax().x;
        ImDrawList *draw_list = ImGui::GetWindowDrawList();
        draw_list->AddRectFilled(ImGui::GetCursorScreenPos(), ImVec2(ImGui::GetCursorScreenPos().x + d_pplot_size, ImGui::GetCursorScreenPos().y + d_pplot_size), style::theme.widget_bg);

        // Draw the "target-like" plot with elevation rings
        float radius = 0.45;
        float radius1 = d_pplot_size * radius * (3.0 / 9.0);
        float radius2 = d_pplot_size * radius * (6.0 / 9.0);
        float radius3 = d_pplot_size * radius * (9.0 / 9.0);

        draw_list->AddCircle({ImGui::GetCursorScreenPos().x + (d_pplot_size / 2), ImGui::GetCursorScreenPos().y + (d_pplot_size / 2)}, radius1, style::theme.green, 0, 2);
        draw_list->AddCircle({ImGui::GetCursorScreenPos().x + (d_pplot_size / 2), ImGui::GetCursorScreenPos().y + (d_pplot_size / 2)}, radius2, style::theme.green, 0, 2);
        draw_list->AddCircle({ImGui::GetCursorScreenPos().x + (d_pplot_size / 2), ImGui::GetCursorScreenPos().y + (d_pplot_size / 2)}, radius3, style::theme.green, 0, 2);

        draw_list->AddLine({ImGui::GetCursorScreenPos().x + (d_pplot_size / 2), ImGui::GetCursorScreenPos().y},
                           {ImGui::GetCursorScreenPos().x + (d_pplot_size / 2), ImGui::GetCursorScreenPos().y + d_pplot_size}, style::theme.green, 2);
        draw_list->AddLine({ImGui::GetCursorScreenPos().x, ImGui::GetCursorScreenPos().y + (d_pplot_size / 2)},
                           {ImGui::GetCursorScreenPos().x + d_pplot_size, ImGui::GetCursorScreenPos().y + (d_pplot_size / 2)}, style::theme.green, 2);

        // Draw the satellite's trace
        if (upcoming_pass_points.size() > 1)
        {
            upcoming_passes_mtx.lock();
            for (int i = 0; i < (int)upcoming_pass_points.size() - 1; i++)
            {
                auto &p1 = upcoming_pass_points[i];
                auto &p2 = upcoming_pass_points[i + 1];

                float point_x1, point_x2, point_y1, point_y2;
                point_x1 = point_x2 = ImGui::GetCursorScreenPos().x + (d_pplot_size / 2);
                point_y1 = point_y2 = ImGui::GetCursorScreenPos().y + (d_pplot_size / 2);

                point_x1 += az_el_to_plot_x(d_pplot_size, radius, p1.az, p1.el);
                point_y1 -= az_el_to_plot_y(d_pplot_size, radius, p1.az, p1.el);

                point_x2 += az_el_to_plot_x(d_pplot_size, radius, p2.az, p2.el);
                point_y2 -= az_el_to_plot_y(d_pplot_size, radius, p2.az, p2.el);

                draw_list->AddLine({point_x1, point_y1}, {point_x2, point_y2}, style::theme.orange, 2.0);
            }
            upcoming_passes_mtx.unlock();
        }

        // Draw the current satellite position
        if (sat_current_pos.el > 0)
        {
            float point_x = ImGui::GetCursorScreenPos().x + (d_pplot_size / 2);
            float point_y = ImGui::GetCursorScreenPos().y + (d_pplot_size / 2);

            point_x += az_el_to_plot_x(d_pplot_size, radius, sat_current_pos.az, sat_current_pos.el);
            point_y -= az_el_to_plot_y(d_pplot_size, radius, sat_current_pos.az, sat_current_pos.el);

            draw_list->AddCircleFilled({point_x, point_y}, 5 * ui_scale, style::theme.red);
        }

        if (rotator_handler && rotator_handler->is_connected())
        {
#if 1
            {
                float point_x = ImGui::GetCursorScreenPos().x + (d_pplot_size / 2);
                float point_y = ImGui::GetCursorScreenPos().y + (d_pplot_size / 2);

                point_x += az_el_to_plot_x(d_pplot_size, radius, rot_current_pos.az, rot_current_pos.el);
                point_y -= az_el_to_plot_y(d_pplot_size, radius, rot_current_pos.az, rot_current_pos.el);

                draw_list->AddCircle({point_x, point_y}, 9 * ui_scale, style::theme.light_cyan, 0, 2.0);
            }
#else // WIP, the idea is to draw the *actual* antenna beamwidth
            {
                float beamwidth = 10;

                for (int i = 0; i < 50; i++)
                {
                    float point_x = ImGui::GetCursorScreenPos().x + (d_pplot_size / 2);
                    float point_y = ImGui::GetCursorScreenPos().y + (d_pplot_size / 2);
                    float point_x2 = ImGui::GetCursorScreenPos().x + (d_pplot_size / 2);
                    float point_y2 = ImGui::GetCursorScreenPos().y + (d_pplot_size / 2);

                    float az1 = rot_current_req_pos.az + beamwidth * sin((i / 50.0) * 2 * M_PI);
                    float el1 = rot_current_req_pos.el + beamwidth * cos((i / 50.0) * 2 * M_PI);

                    int y = i + 1;

                    float az2 = rot_current_req_pos.az + beamwidth * sin((y / 50.0) * 2 * M_PI);
                    float el2 = rot_current_req_pos.el + beamwidth * cos((y / 50.0) * 2 * M_PI);

                    point_x += az_el_to_plot_x(d_pplot_size, radius, az1, el1);
                    point_y -= az_el_to_plot_y(d_pplot_size, radius, az1, el1);

                    point_x2 += az_el_to_plot_x(d_pplot_size, radius, az2, el2);
                    point_y2 -= az_el_to_plot_y(d_pplot_size, radius, az2, el2);

                    draw_list->AddLine({point_x, point_y}, {point_x2, point_y2}, style::theme.light_cyan, 2.0);
                }
            }
#endif

            if (rotator_engaged)
            {
                float point_x = ImGui::GetCursorScreenPos().x + (d_pplot_size / 2);
                float point_y = ImGui::GetCursorScreenPos().y + (d_pplot_size / 2);

                point_x += az_el_to_plot_x(d_pplot_size, radius, rot_current_req_pos.az, rot_current_req_pos.el);
                point_y -= az_el_to_plot_y(d_pplot_size, radius, rot_current_req_pos.az, rot_current_req_pos.el);

                draw_list->AddLine({point_x - 5 * ui_scale, point_y}, {point_x - 12 * ui_scale, point_y}, style::theme.light_cyan, 2.0);
                draw_list->AddLine({point_x + 5 * ui_scale, point_y}, {point_x + 12 * ui_scale, point_y}, style::theme.light_cyan, 2.0);
                draw_list->AddLine({point_x, point_y - 5 * ui_scale}, {point_x, point_y - 12 * ui_scale}, style::theme.light_cyan, 2.0);
                draw_list->AddLine({point_x, point_y + 5 * ui_scale}, {point_x, point_y + 12 * ui_scale}, style::theme.light_cyan, 2.0);
            }
        }

        // Draw the handheld device/antenna pointing direction from the IMU, if available.
        // This sits on the same polar plot as the satellite (red dot) and rotator (cyan ring),
        // so the user can visually rotate the device until this marker lines up with the satellite.
        {
            SatAzEl imu_pos;
            int cal_sys = 0, cal_mag = 0;
            if (getImuPos(imu_pos, cal_sys, cal_mag))
            {
                float point_x = ImGui::GetCursorScreenPos().x + (d_pplot_size / 2);
                float point_y = ImGui::GetCursorScreenPos().y + (d_pplot_size / 2);

                point_x += az_el_to_plot_x(d_pplot_size, radius, imu_pos.az, imu_pos.el);
                point_y -= az_el_to_plot_y(d_pplot_size, radius, imu_pos.az, imu_pos.el);

                float ch = 10 * ui_scale;
                ImColor imu_col = style::theme.yellow;
                draw_list->AddLine({point_x - ch, point_y}, {point_x + ch, point_y}, imu_col, 2.0);
                draw_list->AddLine({point_x, point_y - ch}, {point_x, point_y + ch}, imu_col, 2.0);
                draw_list->AddCircle({point_x, point_y}, ch * 0.7f, imu_col, 16, 1.5);

                // Draw a connecting line to the satellite position, when tracking one and it's above horizon,
                // so the "how far off am I" gap is visually obvious, not just numeric.
                if (tracking_mode != TRACKING_NONE && sat_current_pos.el > 0)
                {
                    float sat_x = ImGui::GetCursorScreenPos().x + (d_pplot_size / 2);
                    float sat_y = ImGui::GetCursorScreenPos().y + (d_pplot_size / 2);
                    sat_x += az_el_to_plot_x(d_pplot_size, radius, sat_current_pos.az, sat_current_pos.el);
                    sat_y -= az_el_to_plot_y(d_pplot_size, radius, sat_current_pos.az, sat_current_pos.el);

                    draw_list->AddLine({point_x, point_y}, {sat_x, sat_y}, ImColor(style::theme.orange.Value.x, style::theme.orange.Value.y, style::theme.orange.Value.z, 0.6f), 1.5);
                }

                // Small calibration indicator, bottom-left corner of the plot.
                // BNO055-style: 0 = uncalibrated (grey), 1-2 = partial (yellow), 3 = full (green)
                auto cal_color = [](int level) -> ImColor
                {
                    if (level >= 3) return style::theme.green;
                    if (level >= 1) return style::theme.yellow;
                    return ImColor(128, 128, 128, 255);
                };

                float dot_r = 4 * ui_scale;
                float dot_y = ImGui::GetCursorScreenPos().y + d_pplot_size - 10 * ui_scale;
                float dot_x = ImGui::GetCursorScreenPos().x + 10 * ui_scale;
                draw_list->AddCircleFilled({dot_x, dot_y}, dot_r, cal_color(cal_sys));
                draw_list->AddCircleFilled({dot_x + 14 * ui_scale, dot_y}, dot_r, cal_color(cal_mag));
            }
        }
#if 1
        if (next_aos_time != 0 && next_los_time != 0)
        {
            double timeOffset = 0, ctime = getTime();
            if (next_aos_time > ctime)
                timeOffset = next_aos_time - ctime;
            else
                timeOffset = next_los_time - ctime;

            int hours = timeOffset / 3600;
            int minutes = fmod(timeOffset / 60, 60);
            int seconds = fmod(timeOffset, 60);

            std::string time_dis =
                (hours < 10 ? "0" : "") + std::to_string(hours) + ":" + (minutes < 10 ? "0" : "") + std::to_string(minutes) + ":" + (seconds < 10 ? "0" : "") + std::to_string(seconds);

            auto &style = ImGui::GetStyle();
            ImVec2 cur = ImGui::GetCursorPos();
            ImVec2 curs = ImGui::GetCursorScreenPos();
            std::string obj_name = "None";
            if (tracking_mode == TRACKING_HORIZONS)
                obj_name = horizonsoptions[current_horizons_id].second;
            else if (tracking_mode == TRACKING_SATELLITE)
                obj_name = satoptions[current_satellite_id];
            ImVec2 size = ImGui::CalcTextSize(obj_name.c_str());
            draw_list->AddRectFilled(curs, ImVec2(curs.x + size.x + 2 * style.FramePadding.x, curs.y + size.y), style::theme.overlay_bg);
            ImGui::TextColored(style::theme.green, "%s", obj_name.c_str());
            curs = ImGui::GetCursorScreenPos();
            char buff[9];
            snprintf(buff, sizeof(buff), "Az: %.1f", sat_current_pos.az);
            size = ImGui::CalcTextSize(buff);
            draw_list->AddRectFilled(curs, ImVec2(curs.x + size.x + 2 * style.FramePadding.x, curs.y + size.y), style::theme.overlay_bg);
            ImGui::TextColored(style::theme.green, "Az: %.1f", sat_current_pos.az);
            curs = ImGui::GetCursorScreenPos();
            snprintf(buff, sizeof(buff), "El: %.1f", sat_current_pos.el);
            size = ImGui::CalcTextSize(buff);
            draw_list->AddRectFilled(curs, ImVec2(curs.x + size.x + 2 * style.FramePadding.x, curs.y + size.y), style::theme.overlay_bg);
            ImGui::TextColored(style::theme.green, "El: %.1f", sat_current_pos.el);

            if (next_aos_time != -1 && next_los_time != -1)
            {
                if (next_aos_time > ctime)
                    size = ImGui::CalcTextSize(("AOS in" + time_dis).c_str());
                else
                    size = ImGui::CalcTextSize(("LOS in" + time_dis).c_str());
            }

            ImGui::SetCursorPosY(cur.y + d_pplot_size - 20 * ui_scale);
            curs = ImGui::GetCursorScreenPos();
            draw_list->AddRectFilled(curs, ImVec2(curs.x + size.x + 2 * style.FramePadding.x, curs.y + size.y), style::theme.overlay_bg);
            ImGui::TextColored(style::theme.green, "%s in %s", next_aos_time > ctime ? "AOS" : "LOS", time_dis.c_str());

            ImGui::SetCursorPos(cur);
        }
#endif
        ImGui::Dummy(ImVec2(d_pplot_size + 3 * ui_scale, d_pplot_size + 3 * ui_scale));
    }

    void ObjectTracker::renderSelectionMenu()
    {
        bool update_global = false;

        if (backend_needs_update)
            style::beginDisabled();

        if (ImGui::BeginTable("##trackingradiotable", 2, ImGuiTableFlags_None))
        {
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            if (ImGui::RadioButton("Satellites", tracking_mode == TRACKING_SATELLITE))
            {
                tracking_mode = TRACKING_SATELLITE;
                update_global = true;
            }
            ImGui::TableSetColumnIndex(1);
            if (ImGui::RadioButton("Horizons", tracking_mode == TRACKING_HORIZONS))
            {
                if (horizonsoptions.size() == 1)
                    horizonsoptions = pullHorizonsList();
                tracking_mode = TRACKING_HORIZONS;
                update_global = true;
            }
            ImGui::EndTable();
        }

        ImGui::SetNextItemWidth(ImGui::GetWindowContentRegionMax().x);
        if (tracking_mode == TRACKING_HORIZONS)
        {
            if (ImGui::BeginCombo("###horizonsselectcombo", horizonsoptions[current_horizons_id].second.c_str()))
            {
                ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
                ImGui::InputTextWithHint("##horizonssatellitetracking", u8"\uf422   Search", &horizons_searchstr);
                for (int i = 0; i < (int)horizonsoptions.size(); i++)
                {
                    bool show = true;
                    if (horizons_searchstr.size() != 0)
                        show = satdump::isStringPresent(horizonsoptions[i].second, horizons_searchstr);
                    if (show)
                    {
                        if (ImGui::Selectable(horizonsoptions[i].second.c_str(), i == current_horizons_id))
                        {
                            current_horizons_id = i;
                            update_global = true;
                        }
                    }
                }
                ImGui::EndCombo();
            }

            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Horizons ID %d", horizonsoptions[current_horizons_id].first);
        }
        else if (tracking_mode == TRACKING_SATELLITE)
        {
            if (ImGui::BeginCombo("###satelliteselectcombo", satoptions[current_satellite_id].c_str()))
            {
                ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
                ImGui::InputTextWithHint("##searchsatellitetracking", u8"\uf422   Search", &satellite_searchstr);
                for (int i = 0; i < (int)satoptions.size(); i++)
                {
                    bool show = true;
                    if (satellite_searchstr.size() != 0)
                        show = isStringPresent(satoptions[i], satellite_searchstr);
                    if (show)
                    {
                        if (ImGui::Selectable(satoptions[i].c_str(), i == current_satellite_id))
                        {
                            current_satellite_id = i;
                            update_global = true;
                        }
                    }
                }
                ImGui::EndCombo();
            }

            if (ImGui::IsItemHovered())
            {
                auto reg = db_keplers->all();
                ImGui::SetTooltip("NORAD ID %d", reg[current_satellite_id].norad);
            }
        }

        if (backend_needs_update)
            style::endDisabled();

        if (update_global)
            backend_needs_update = true;
    }

    void ObjectTracker::renderObjectStatus()
    {
        if (ImGui::BeginTable("##trackingwidgettable", 2, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg))
        {
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::Text("Azimuth");
            ImGui::TableSetColumnIndex(1);
            ImGui::Text("%.3f", sat_current_pos.az);

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::Text("Elevation");
            ImGui::TableSetColumnIndex(1);
            ImGui::Text("%.3f", sat_current_pos.el);

            if (next_aos_time != 0 && next_los_time != 0)
            {
                double timeOffset = 0, ctime = getTime();
                if (next_aos_time > ctime)
                    timeOffset = next_aos_time - ctime;
                else
                    timeOffset = next_los_time - ctime;

                int hours = timeOffset / 3600;
                int minutes = fmod(timeOffset / 60, 60);
                int seconds = fmod(timeOffset, 60);

                std::string time_dis =
                    (hours < 10 ? "0" : "") + std::to_string(hours) + ":" + (minutes < 10 ? "0" : "") + std::to_string(minutes) + ":" + (seconds < 10 ? "0" : "") + std::to_string(seconds);

                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::Text("Next Event");
                ImGui::TableSetColumnIndex(1);
                ImGui::Text("%s", time_dis.c_str());
            }

            if (tracking_mode == TRACKING_SATELLITE && satellite_object != nullptr)
            {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::Text("Azimuth Rate");
                ImGui::TableSetColumnIndex(1);
                ImGui::Text("%.2f", satellite_observation_pos.azimuth_rate * RAD_TO_DEG);

                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::Text("Elevation Rate");
                ImGui::TableSetColumnIndex(1);
                ImGui::Text("%.2f", satellite_observation_pos.elevation_rate * RAD_TO_DEG);

                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::Text("Range (km)");
                ImGui::TableSetColumnIndex(1);
                ImGui::Text("%.2f", satellite_observation_pos.range);
            }

            {
                SatAzEl imu_pos;
                int cal_sys = 0, cal_mag = 0;
                if (getImuPos(imu_pos, cal_sys, cal_mag))
                {
                    float az_err = sat_current_pos.az - imu_pos.az;
                    if (az_err > 180) az_err -= 360;
                    if (az_err < -180) az_err += 360;
                    float el_err = sat_current_pos.el - imu_pos.el;
                    float total_err = sqrt(az_err * az_err + el_err * el_err);

                    ImVec4 err_col = style::theme.green.Value;
                    if (total_err >= 15)
                        err_col = style::theme.red.Value;
                    else if (total_err >= 5)
                        err_col = style::theme.yellow.Value;

                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::Text("Device Heading");
                    ImGui::TableSetColumnIndex(1);
                    ImGui::Text("%.1f / %.1f", imu_pos.az, imu_pos.el);

                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::Text("Pointing Error");
                    ImGui::TableSetColumnIndex(1);
                    ImGui::TextColored(err_col, "Turn %+.1f / Tilt %+.1f", az_err, el_err);
                }
            }

            ImGui::EndTable();
        }
    }
} // namespace satdump
