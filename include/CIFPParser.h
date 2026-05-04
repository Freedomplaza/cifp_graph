//CIFP Parser file header
//version 2May26
//author 26colacito

#ifndef CIFP_PARSER_H
#define CIFP_PARSER_H

#include <string>

#include "NavGraph.h"

namespace cifp {

// statistics from parsing
struct ParseStats {
    std::size_t navaids_loaded   = 0;
    std::size_t waypoints_loaded = 0;
    std::size_t airway_legs_read = 0;
    std::size_t edges_added      = 0;   // edges_added == 2 * airway segments kept
    std::size_t skipped_high_alt = 0;   // airway legs filtered out (jet routes)
    std::size_t skipped_outside_conus = 0;
    std::size_t discontinuities_skipped = 0;  // CIFP "break" records (blank dist/MEA)
};

class CIFPParser {
public:
    // If `low_altitude_only` is true (default), filter out high-altitude
    // jet routes (J* and Q*) and any individual leg whose MEA is at or
    // above FL180 (18,000 ft) — the floor of US Class A airspace and the
    // boundary of the Low Altitude Enroute structure.
    explicit CIFPParser(bool low_altitude_only = true)
        : low_altitude_only_(low_altitude_only) {}

    // Reads the file at `path` and populates `graph`. Returns false on I/O
    // error (graph may be partially populated). Stats are always written.
    bool parse(const std::string& path, NavGraph& graph, ParseStats& stats);

private:
    bool low_altitude_only_;

    // helpers for each node record type
    bool handle_navaid(const std::string& line, NavGraph& graph);
    bool handle_waypoint(const std::string& line, NavGraph& graph);

    // airway records arrive as a sequence of legs sharing a route id
    // processed as we go, holding the previous fix in `prev_*`
    struct AirwayState {
        std::string route_id;        // e.g. "V16"
        NodeId      prev_node = kInvalidNode;
        double      pending_distance_nm = 0.0;
        bool        pending_is_blank = false;  // true if dist/MEA fields were blank
        bool        route_is_low_alt = false;
    } airway_;

    bool handle_airway(const std::string& line, NavGraph& graph, ParseStats& stats);
};

}  // namespace cifp

#endif  // CIFP_PARSER_HPP
