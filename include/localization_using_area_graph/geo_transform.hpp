/**
 * @file geo_transform.hpp
 * @author Jiajie Zhang
 * @brief 地理坐标与局部坐标系转换工具
 * @version 0.1
 * @date 2024-12-02
 */

#pragma once
#ifndef _GEO_TRANSFORM_HPP_
#define _GEO_TRANSFORM_HPP_

#include <array>
#include <cmath>

/**
 * @brief 地理坐标结构体
 */
struct GeoCoordinate {
    double longitude;   // 经度
    double latitude;    // 纬度
    double altitude;    // 海拔高度
};

namespace geo_transform {
    constexpr double EARTH_RADIUS = 6378137.0;  // 地球半径（米）

    /**
     * @brief 将经纬度转换为相对于参考点的局部坐标
     * @param ref 参考点经纬度 [latitude, longitude]
     * @param cur 当前点经纬度 [latitude, longitude]
     * @return 局部坐标 [x, y]
     */
    inline std::array<double, 2> toCartesian(const std::array<double, 2>& ref, const std::array<double, 2>& cur) {
        double lat_rad = ref[0] * M_PI / 180.0;
        double d_lat = (cur[0] - ref[0]) * M_PI / 180.0;
        double d_lon = (cur[1] - ref[1]) * M_PI / 180.0;

        double y = EARTH_RADIUS * d_lat;
        double x = EARTH_RADIUS * std::cos(lat_rad) * d_lon;

        return {x, y};
    }

    /**
     * @brief 将局部坐标转换回经纬度（相对于参考点）
     * @param ref_latlon 参考点经纬度 [latitude, longitude]
     * @param local_xy 局部坐标 [x, y]
     * @return 经纬度 [latitude, longitude]
     */
    inline std::array<double, 2> toGeographic(const std::array<double, 2>& ref_latlon, const std::array<double, 2>& local_xy) {
        double ref_lat_rad = ref_latlon[0] * M_PI / 180.0;
        
        double d_lat = local_xy[1] / EARTH_RADIUS;
        double d_lon = local_xy[0] / (EARTH_RADIUS * std::cos(ref_lat_rad));

        double lat = ref_latlon[0] + d_lat * 180.0 / M_PI;
        double lon = ref_latlon[1] + d_lon * 180.0 / M_PI;

        return {lat, lon};
    }

    /**
     * @brief 将map坐标系下的位置转换为地理坐标
     * @param map_x map坐标系x
     * @param map_y map坐标系y
     * @param ref_longitude 参考经度
     * @param ref_latitude 参考纬度
     * @param map_extrinsic_trans 地图外参平移 [x, y, z]
     * @param map_yaw_angle 地图外参偏航角（弧度）
     * @return 地理坐标结构体
     */
    inline GeoCoordinate mapToGeographic(double map_x, double map_y,
                                       double ref_longitude, double ref_latitude,
                                       const std::array<double, 3>& map_extrinsic_trans,
                                       double map_yaw_angle) {
        // 1. 从map坐标系转回AGmap坐标系
        // 首先减去平移
        double x_ag = map_x - map_extrinsic_trans[0];
        double y_ag = map_y - map_extrinsic_trans[1];
        
        // 然后进行反向旋转
        double cos_yaw = std::cos(-map_yaw_angle);  // 注意这里是负角
        double sin_yaw = std::sin(-map_yaw_angle);
        double x_local = x_ag * cos_yaw - y_ag * sin_yaw;
        double y_local = x_ag * sin_yaw + y_ag * cos_yaw;

        // 2. 从局部坐标转换为经纬度
        std::array<double, 2> ref_latlon = {ref_latitude, ref_longitude};
        std::array<double, 2> local_xy = {x_local, y_local};
        auto geo = toGeographic(ref_latlon, local_xy);

        GeoCoordinate result;
        result.latitude = geo[0];
        result.longitude = geo[1];
        result.altitude = 0.0;
        return result;
    }
}

#endif // _GEO_TRANSFORM_HPP_ 