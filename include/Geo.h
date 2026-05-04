//coordinate parser header
//version 2May26
//author 26colacito

#ifndef CIFP_GEO_H
#define CIFP_GEO_H

#include <cmath>
#include <optional>
#include <string_view>

namespace cifp {

// coordinates
struct LatLon {
    double lat_deg = 0.0;  // +N, -S
    double lon_deg = 0.0;  // +E, -W

    // super rough but it works for this application
    // (screw alaskan airways, i dont wanna deal with colored or R routes, or funky NDB/DMEs)
    bool in_conus() const {
        return lat_deg >= 24.0 && lat_deg <= 50.0 &&
               lon_deg >= -125.0 && lon_deg <= -66.0;
    }
};

// great-circle distance in nautical miles between two coordinates
// params a: first coordinate, b: second coordinate
// return distance NM
inline double nm_distance(const LatLon& a, const LatLon& b) {
    constexpr double kEarthRadiusNm = 3440.065;  // mean Earth radius, NM
    constexpr double kDegToRad = 0.017453292519943295;

    const double phi1 = a.lat_deg * kDegToRad;
    const double phi2 = b.lat_deg * kDegToRad;
    const double dphi = (b.lat_deg - a.lat_deg) * kDegToRad;
    const double dlam = (b.lon_deg - a.lon_deg) * kDegToRad;

    const double s1 = std::sin(dphi * 0.5);
    const double s2 = std::sin(dlam * 0.5);
    const double h = s1 * s1 + std::cos(phi1) * std::cos(phi2) * s2 * s2;
    return 2.0 * kEarthRadiusNm * std::asin(std::sqrt(std::min(1.0, h)));
}

// parse ARINC 424 latitude
std::optional<double> parse_arinc_lat(std::string_view field);

// parse ARINC 424 longitude
std::optional<double> parse_arinc_lon(std::string_view field);

}  // namespace cifp

#endif  // CIFP_GEO_HPP
