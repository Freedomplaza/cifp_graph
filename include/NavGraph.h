//NavGraph adjacency list data structure header
//version 2May26
//author 26colacito

// Design priorities (from highest to lowest):
//   1. O(1) lookup of a fix by its identifier (e.g. "ABQ", "AAARG").
//      A pathfinding algorithm calls this on every neighbor expansion,
//      so it MUST be fast.
//   2. O(1) iteration of all outbound edges from a fix. We store edges
//      in a contiguous std::vector per node so the inner loop of Dijkstra
//      / A* is cache-friendly.
//   3. Edges are bidirectional in the graph (most low-altitude airways
//      can be flown either direction). For each ARINC airway leg we add
//      one edge in each direction, both carrying the same MEA.
//   4. NodeId is a small integer index (uint32_t). std::string identifier
//      → NodeId map lives once in the graph; everywhere else (edges,
//      open lists, came-from maps) uses the integer. This is the standard
//      "intern the strings at build time" trick.

#ifndef CIFP_NAV_GRAPH_H
#define CIFP_NAV_GRAPH_H

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "Geo.h"

namespace cifp {

using NodeId = std::uint32_t;
inline constexpr NodeId kInvalidNode = static_cast<NodeId>(-1);

enum class FixKind : std::uint8_t {
    Unknown,
    VOR,        // VHF Omnidirectional Range (without DME)
    VORTAC,     // VOR co-located with TACAN
    VOR_DME,    // VOR co-located with DME
    TACAN,      // TACtical Air Navigation (military VOR/DME)
    LOC,        // Localizer (with or without DME, CIFP does not distinguish)
    DME,        // Distance Measuring Equipment
    Waypoint,   // Enroute fix / intersection (5-letter)
};

struct Node {
    std::string ident;   // Trimmed identifier (e.g. "ABQ", "AAARG")
    std::string name;    // Human-readable name when available
    LatLon position;
    FixKind kind = FixKind::Unknown;
};

// single outbound airway leg leaving a node
struct Edge {
    NodeId      to = kInvalidNode;
    std::string airway;     // Route identifier, e.g. "V16", "T211"
    double      distance_nm = 0.0;
    int         mea_ft = 0;
    int         maa_ft = 0;
};

class NavGraph {
public:
    // insert or update node
    NodeId upsert_node(const Node& n);

    // add an airway leg edge from `from` to `edge.to`
    void add_edge(NodeId from, Edge edge);

    // node lookup by id
    NodeId find(std::string_view ident) const;

    // true if id in range
    bool valid(NodeId id) const { return id < nodes_.size(); }

    // get node by id
    const Node& node(NodeId id) const { return nodes_[id]; }

    // get all outbound edges from node id
    const std::vector<Edge>& edges_from(NodeId id) const {
        return adjacency_[id];
    }

    // get total num nodes
    std::size_t num_nodes() const { return nodes_.size(); }

    // get total num edges
    std::size_t num_edges() const;

private:
    std::vector<Node>                       nodes_;
    std::vector<std::vector<Edge>>          adjacency_;
    std::unordered_map<std::string, NodeId> ident_index_;
};

}  // namespace cifp

#endif  // CIFP_NAV_GRAPH_HPP
