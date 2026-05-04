//Dijkstra search of NavGraph implementation
//version 2May26
//author 26colacito

#include "../include/Pathfinding.h"

#include <limits>
#include <queue>
#include <vector>

namespace cifp {

// find the shortest path from one node to another via airways
// params graph: the NavGraph, start_ident: starting node ID, goal_ident: ending node ID, aircraft_ceiling_ft: hard MAA
// return PathResult of shortest path
PathResult shortest_path(const NavGraph& graph, const std::string& start_ident, const std::string& goal_ident, int aircraft_ceiling_ft, bool rnav_available) { 
    PathResult result;

    //find the start and end
    const NodeId start = graph.find(start_ident);
    const NodeId goal  = graph.find(goal_ident);
    if (start == kInvalidNode || goal == kInvalidNode) return result; // check if either are invalid
    // check if the user is stupid
    if (start == goal) { 
        result.found = true; // confirm user is stupid
        return result;
    }

    constexpr double kInf = std::numeric_limits<double>::infinity();
    std::vector<double> dist(graph.num_nodes(), kInf);
    std::vector<NodeId> came_from(graph.num_nodes(), kInvalidNode);
    // track which edge index we took into each node, so we can reconstruct the airway and MEA used for each step
    std::vector<std::size_t> came_via_edge(graph.num_nodes(), 0);

    using QueueEntry = std::pair<double, NodeId>;  // (distance, node)
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> open; // pqueue for Dijkstra

    dist[start] = 0.0;
    open.emplace(0.0, start);

    // main search loop
    while (!open.empty()) {
        auto [d, u] = open.top();
        open.pop();
        if (d > dist[u]) continue;        // stale entry
        if (u == goal) break;             // early exit

        const auto& edges = graph.edges_from(u);
        // check each edge from current node
        for (std::size_t i = 0; i < edges.size(); ++i) {
            const Edge& e = edges[i];
            if ((e.mea_ft > aircraft_ceiling_ft) || ((e.airway[0] == 'T') && !rnav_available)) continue;  // unflyable
            const double weight = (e.distance_nm > 0.0) ? e.distance_nm : 1.0;
            const double nd = d + weight;
            // check if this path is best
            if (nd < dist[e.to]) {
                dist[e.to] = nd;
                came_from[e.to] = u;
                came_via_edge[e.to] = i;
                open.emplace(nd, e.to);
            }
        }
    }

    if (dist[goal] == kInf) return result; // no path found

    // reconstruct the path we took
    std::vector<NodeId> chain;
    for (NodeId cur = goal; cur != start; cur = came_from[cur]) {
        chain.push_back(cur);
        if (came_from[cur] == kInvalidNode) return result;  // broken chain
    }
    chain.push_back(start);

    result.found = true; // it worked!
    result.total_distance_nm = dist[goal];
    result.steps.reserve(chain.size() - 1);
    // reconstruct the path we took
    for (std::size_t i = chain.size(); i-- > 1; ) {
        const NodeId from = chain[i];
        const NodeId to   = chain[i - 1];
        const Edge& e = graph.edges_from(from)[came_via_edge[to]];

        PathStep step;
        step.from_ident = graph.node(from).ident;
        step.to_ident   = graph.node(to).ident;
        step.airway     = e.airway;
        step.distance_nm = e.distance_nm;
        step.mea_ft     = e.mea_ft;
        result.steps.push_back(step);

        if (e.mea_ft > result.highest_mea_ft) {
            result.highest_mea_ft = e.mea_ft;
        }
    }
    return result;
}

}  // namespace cifp
