//AIRNC 424 coordinate parsing
//version 2May26
//author 26colacito

#include "../include/Geo.h"

#include <cctype>

namespace cifp {

namespace {

// pull a non-negative integer out of [s + offset, s + offset + len)
// params s: input string_view, offset: start position, len: length
// return -1 for not int or out of bounds
int parse_int(std::string_view s, size_t offset, size_t len) {
    if (offset + len > s.size()) return -1;
    int value = 0;
    for (size_t i = 0; i < len; ++i) {
        const char c = s[offset + i];
        if (c < '0' || c > '9') return -1;
        value = value * 10 + (c - '0');
    }
    return value;
}

}  // namespace

// parse an ARINC 424 latitude field
// params field: input string_view
// return parsed lat in degrees, else std::nullopt
std::optional<double> parse_arinc_lat(std::string_view field) {
    // expect "Nddmmssss"
    if (field.size() < 9) return std::nullopt;
    const char hemi = field[0];
    if (hemi != 'N' && hemi != 'S') return std::nullopt; // hemisphere

    const int dd  = parse_int(field, 1, 2); // degrees
    const int mm  = parse_int(field, 3, 2); // minutes
    const int ss  = parse_int(field, 5, 2); // seconds
    const int hss = parse_int(field, 7, 2); // hundredths of a second
    if (dd < 0 || mm < 0 || ss < 0 || hss < 0) return std::nullopt;

    // combine silly format into good degrees
    double deg = dd + mm / 60.0 + (ss + hss / 100.0) / 3600.0;
    if (hemi == 'S') deg = -deg;
    return deg;
}

// parse an ARINC 424 longitude field
// params field: input string_view
// return parsed lon in degrees, else std::nullopt
std::optional<double> parse_arinc_lon(std::string_view field) {
    // expect "Wdddmmssss"
    if (field.size() < 10) return std::nullopt;
    const char hemi = field[0];
    if (hemi != 'E' && hemi != 'W') return std::nullopt; // hemisphere

    const int ddd = parse_int(field, 1, 3); // degrees
    const int mm  = parse_int(field, 4, 2); // minutes
    const int ss  = parse_int(field, 6, 2); // seconds
    const int hss = parse_int(field, 8, 2); // hundredths of a second
    if (ddd < 0 || mm < 0 || ss < 0 || hss < 0) return std::nullopt;

    // combine silly format into good degrees
    double deg = ddd + mm / 60.0 + (ss + hss / 100.0) / 3600.0;
    if (hemi == 'W') deg = -deg;
    return deg;
}

}  // namespace cifp
