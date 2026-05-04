//parse the CIFP file and create a NavGraph
//version 2May26
//author 26colacito

#include "../include/CIFPParser.h"

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <string>
#include <string_view>

namespace cifp {

namespace {

// inclusive slice using 1-based ARINC column positions
// params line: input line, first_1based: 1-based index of first char, length: slice length
// returns a string_view into `line` (no allocation)
std::string_view slice(const std::string& line, std::size_t first_1based, std::size_t length) {
    if (first_1based == 0 || first_1based - 1 + length > line.size()) {
        return {};
    }
    return std::string_view(line.data() + (first_1based - 1), length);
}

// trim leading/trailing whitespace from a view
// params sv: input string_view
// return trimmed string
std::string trimmed(std::string_view sv) {
    while (!sv.empty() && (sv.front() == ' ' || sv.front() == '\t' || sv.front() == '\r' || sv.front() == '\n'))
        sv.remove_prefix(1);
    while (!sv.empty() && (sv.back() == ' ' || sv.back() == '\t' || sv.back() == '\r' || sv.back() == '\n'))
        sv.remove_suffix(1);
    return std::string(sv);
}

// parse fixed-width all-digit integer
// params sv: input string_view
// returns -1 on any non-digit, otherwise the integer value
int parse_fixed_int(std::string_view sv) {
    if (sv.empty()) return -1;
    int value = 0;
    for (char c : sv) {
        if (c < '0' || c > '9') return -1;
        value = value * 10 + (c - '0');
    }
    return value;
}

}  // namespace

// handle a VHF NAVAID record, put it in the NavGraph
// params line: input line, graph: the NavGraph to interface with
// return success
bool CIFPParser::handle_navaid(const std::string& line, NavGraph& graph) {
    // CIFP VHF NAVAID record (4.1.2) key columns (1-based):
    //   14-17: Identifier (4 chars, blank-padded right)
    //   33-41: Primary (VOR) latitude
    //   42-51: Primary (VOR) longitude
    //   55-63: DME latitude  (used as fallback when no VOR coord)
    //   64-73: DME longitude
    //   94-123: Name (30 chars, for VORs, RDU -> RALEIGH/DURHAM)
    //
    // navaid type classified by looking at type field (col 28-32):
    //   col 28: V (VOR present), I (Localizer present), ' ' (no VOR)
    //   col 29: D (DME present), T (TACAN present, implies DME component), ' ' (no DME)
    //   VD -> VOR/DME, VT -> VORTAC, etc
    //   DME never specified on LOC components, cannot differentiate LOC from LOC/DME

    Node n;
    n.ident = trimmed(slice(line, 14, 4)); // trim her up!
    if (n.ident.empty()) return false;

    // get coords
    auto lat = parse_arinc_lat(slice(line, 33, 9));
    auto lon = parse_arinc_lon(slice(line, 42, 10));
    if (!lat || !lon) {
        // try DME fallback (legacy VOR removed, i.e. ATL DME).
        lat = parse_arinc_lat(slice(line, 56, 9));
        lon = parse_arinc_lon(slice(line, 65, 10));
    }
    if (!lat || !lon) return false;

    n.position.lat_deg = *lat;
    n.position.lon_deg = *lon;
    if (!n.position.in_conus()) return false; // check if the navaid is in CONUS

    // type classification from cols 28-32
    const std::string_view klass = slice(line, 28, 5);
    const bool has_v = klass.size() > 0 && klass[0] == 'V';
    const bool has_d = klass.size() > 1 && klass[1] == 'D';
    const bool has_t = klass.size() > 2 && klass[1] == 'T';
    const bool has_i = klass.size() > 0 && klass[1] == 'I';
    if (has_i) {
        n.kind = FixKind::LOC;
    } else if (has_v && has_t) {
        n.kind = FixKind::VORTAC;
    } else if (has_v && has_d) {
        n.kind = FixKind::VOR_DME;
    } else if (has_v) {
        n.kind = FixKind::VOR;
    } else if (has_t) {
        n.kind = FixKind::TACAN;
    } else if (has_d) {
        n.kind = FixKind::DME;
    } else {
        n.kind = FixKind::Unknown;
    }

    n.name = trimmed(slice(line, 94, 30)); // get the name
    graph.upsert_node(n); // put her in!
    return true; // holy shit, it worked
}

// handle a waypoint record, put it in the NavGraph
// params line: input line, graph: the NavGraph to interface with
// return success
bool CIFPParser::handle_waypoint(const std::string& line, NavGraph& graph) {
    // CIFP Enroute Waypoint record (4.1.4) key columns (1-based):
    //   14-18: Identifier (5 chars)
    //   33-41: Latitude
    //   42-51: Longitude
    //   99-123: Name (expect blank)

    Node n;
    n.ident = trimmed(slice(line, 14, 5)); // trim her up!
    if (n.ident.empty()) return false;

    // get coords
    auto lat = parse_arinc_lat(slice(line, 33, 9));
    auto lon = parse_arinc_lon(slice(line, 42, 10));
    if (!lat || !lon) return false;

    n.position.lat_deg = *lat;
    n.position.lon_deg = *lon;
    if (!n.position.in_conus()) return false; // check if in conus

    n.kind = FixKind::Waypoint; // we know what kind it's gonna be
    n.name = trimmed(slice(line, 99, 25)); // get name (probably nothing)
    graph.upsert_node(n); // add it to the navgraph
    return true; // success
}

// handle an airway record, put it in the NavGraph as an edge
// params line: input line, graph: the NavGraph to interface with, stats: parsing statistics
// return success
bool CIFPParser::handle_airway(const std::string& line, NavGraph& graph, ParseStats& stats) {
    // CIFP Enroute Airway record (4.1.6) key columns:
    //   14-19: Route Identifier (Vxxx or Txxx for low, Qxxx or Jxxx for high)
    //   26-29: Sequence Number (4 digits, increments by 10)
    //   30-34: Fix Identifier (5 chars; NAVAID idents are 3 or 4 chars + blank)
    //   46   : Level - 'L' = low altitude, 'H' = high altitude, ' ' = unspecified
    //   71-74: Outbound magnetic course (tenths of a degree, unused)
    //   75-78: Outbound distance from this fix to the NEXT fix (tenths of NM)
    //   84-88: Minimum Altitude (MEA) in feet MSL OUTBOUND
    //   94-98: Maximum Altitude (MAA) in feet MSL OUTBOUND
    stats.airway_legs_read++;

    // trim her up!
    const std::string route_id = trimmed(slice(line, 14, 6));
    const std::string fix_id   = trimmed(slice(line, 30, 5));
    if (route_id.empty() || fix_id.empty()) return false;

    const char level = line.size() >= 46 ? line[45] : ' ';

    // filter low vs high altitude
    if (low_altitude_only_ && level == 'H') {
        stats.skipped_high_alt++;
        // reset state so a high-alt route doesn't bleed into the next one
        airway_.route_id.clear();
        airway_.prev_node = kInvalidNode;
        return false;
    }

    NodeId fix_node = graph.find(fix_id); // get ID
    // skip the airway if a fix isn't found
    if (fix_node == kInvalidNode) {
        airway_.route_id.clear();
        airway_.prev_node = kInvalidNode;
        return false;
    }

    const bool same_route = (route_id == airway_.route_id);
    // check if we're starting to define a new airway
    if (!same_route) {
        // reset all the held information from the last airway
        airway_.route_id = route_id;
        airway_.prev_node = fix_node;
        airway_.pending_distance_nm = 0.0;
        airway_.pending_is_blank = false;
        airway_.route_is_low_alt = (level == 'L');

        // read this record's outbound distance/MEA, they describe the leg
        // from THIS fix to the NEXT one, so we just stash them. a blank
        // distance field marks this as a "starts a new segment" record;
        // the next leg edge will be skipped
        const std::string_view dist_field = slice(line, 75, 4);
        const int dist_tenths = parse_fixed_int(dist_field);
        airway_.pending_distance_nm = dist_tenths > 0 ? dist_tenths * 0.1 : 0.0;
        airway_.pending_is_blank = (dist_tenths < 0);
        return true;
    }

    // continuing an existing airway: add an edge from prev_node -> fix_node
    // using the distance/MEA we stashed on the last record. the
    // current record's own distance/MEA describes the next leg.
    //
    // CIFP encodes airway discontinuities (e.g. V264 breaks at PKE and
    // resumes at DRK) as a record where the last record had blank
    // distance/MEA. we detect that as pending_distance_nm == 0 AND
    // pending_mea_ft == 0 (set explicitly when those fields were blank)
    const bool is_discontinuity = airway_.pending_is_blank;
    if (airway_.prev_node != kInvalidNode &&
        airway_.prev_node != fix_node &&
        !is_discontinuity) {
        const int mea_ft  = parse_fixed_int(slice(line, 84, 5)); // forward
        const int mea2_ft = parse_fixed_int(slice(line, 89, 5)); // reverse (if specified)
        const int maa_ft  = parse_fixed_int(slice(line, 94, 5)); // always bidirectional

        // check if we missed a high alt airway (FL180 boundary)
        const bool leg_too_high = low_altitude_only_ && mea_ft >= 18000;
        if (leg_too_high) {
            stats.skipped_high_alt++;
        } else {
            // add the forward edge
            Edge fwd;
            fwd.to = fix_node;
            fwd.airway = route_id;
            fwd.distance_nm = airway_.pending_distance_nm;
            fwd.mea_ft = std::max(0, mea_ft);
            fwd.maa_ft = std::max(0, maa_ft);

            // add the reverse edge
            Edge rev;
            rev.to = airway_.prev_node;
            rev.airway = route_id;
            rev.distance_nm = airway_.pending_distance_nm;
            rev.mea_ft = std::max(0, mea2_ft >= 0 ? mea2_ft : mea_ft); // use reverse MEA if specified, otherwise forward MEA
            rev.maa_ft = fwd.maa_ft;

            graph.add_edge(airway_.prev_node, fwd); // add forward direction
            graph.add_edge(fix_node, rev); // add reverse direction
            stats.edges_added += 2;
        }
    } else if (is_discontinuity) { // skip discontinuities
        stats.discontinuities_skipped++;
    }

    // slide the window: this fix becomes the new "previous", and we
    // refresh the pending distance from this record's outbound field

    // if the field is blank, mark the next record as a discontinuity
    airway_.prev_node = fix_node;
    const std::string_view dist_field = slice(line, 75, 4);
    const int dist_tenths = parse_fixed_int(dist_field);
    airway_.pending_distance_nm = dist_tenths > 0 ? dist_tenths * 0.1 : 0.0;
    airway_.pending_is_blank = (dist_tenths < 0);  // -1 == non-numeric/blank
    return true;
}

// actually parse the CIFP file
// params path: CIFP path, graph: the NavGraph to interface with, stats: parsing statistics
// return success
bool CIFPParser::parse(const std::string& path, NavGraph& graph, ParseStats& stats) {
    std::ifstream in(path);
    if (!in) return false; // check if the file exists lmao

    std::string line;
    line.reserve(140);  // CIFP records are 132 chars + CRLF

    // ARINC 424 file is naturally ordered: NAVAIDs first, waypoints second, airways last
    while (std::getline(in, line)) {
        if (line.size() < 6) continue;

        // strip a trailing \r left over from CRLF line endings (the file is DOS-formatted; std::getline only strips \n)
        if (line.back() == '\r') line.pop_back();

        const std::string_view prefix(line.data(), 6);

        if (prefix == "SUSAD ") { // VHF NAVAIDs
            if (handle_navaid(line, graph)) stats.navaids_loaded++;
            else                            stats.skipped_outside_conus++;
        } else if (prefix == "SUSAEA") { // waypoints
            if (handle_waypoint(line, graph)) stats.waypoints_loaded++;
            else                              stats.skipped_outside_conus++;
        } else if (prefix == "SUSAER") { // enroute airways
            handle_airway(line, graph, stats);
        }
        // Everything else (airports, SIDs/STARs/SIAPs, headers) is ignored
    }

    return true;
}

}  // namespace cifp
