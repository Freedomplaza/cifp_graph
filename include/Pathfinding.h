// Pathfinding.hpp - Reference implementation of Dijkstra over the NavGraph.
//
// This file isn't strictly part of the database — it's here to demonstrate
// that the NavGraph API is genuinely friendly to a pathfinding algorithm:
// every operation we need (lookup by ident, iterate neighbors, get edge
// weight, get coordinate for an A* heuristic) is O(1) and lives behind a
// short, obvious method name.
//
// Cost model: distance_nm + a soft penalty for legs whose MEA exceeds the
// aircraft's service ceiling. We skip them entirely if the aircraft can't
// reach their MEA at all.

#ifndef CIFP_PATHFINDING_H
#define CIFP_PATHFINDING_H

#include <string>
#include <vector>

#include "NavGraph.h"

namespace cifp {

struct PathStep {
    std::string from_ident;
    std::string to_ident;
    std::string airway;
    double      distance_nm = 0.0;
    int         mea_ft = 0;
};

struct PathResult {
    bool found = false;
    double total_distance_nm = 0.0;
    int    highest_mea_ft = 0;
    std::vector<PathStep> steps;
};

// find shortest path between nodes via airways
PathResult shortest_path(const NavGraph& graph,
                         const std::string& start_ident,
                         const std::string& goal_ident,
                         int aircraft_ceiling_ft = 18000,
                         bool rnav_available = true);

}  // namespace cifp

#endif  // CIFP_PATHFINDING_HPP
