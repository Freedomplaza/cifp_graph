//main driver for cifp_graph final project
//version 2May26
//author 26colacito

#include <iomanip>
#include <iostream>
#include <string>

#include "../include/CIFPParser.h"
#include "../include/FlightPlanService.h"
#include "../include/NavGraph.h"
#include "../include/Pathfinding.h"

namespace {

// map FixKind to string names
// params k: the FixKind to map
// return string name
const char* kind_name(cifp::FixKind k) {
    using cifp::FixKind; // see NavGraph.h for fix definitions
    switch (k) {
        case FixKind::VOR:      return "VOR";
        case FixKind::VORTAC:   return "VORTAC";
        case FixKind::DME:      return "DME";
        case FixKind::LOC:      return "LOC";
        case FixKind::TACAN:    return "TACAN";
        case FixKind::VOR_DME:   return "VOR/DME";
        case FixKind::Waypoint: return "FIX or WP";
        default:                return "?";
    }
}

// print node info
// params g: the NavGraph to interface with, ident: node identifier
void print_node(const cifp::NavGraph& g, const std::string& ident) {
    const cifp::NodeId id = g.find(ident); // get node ID
    if (id == cifp::kInvalidNode) {
        std::cout << "  " << ident << ": not found\n";
        return;
    }
    const auto& n = g.node(id);
    // format printing ID, type, position, and name
    std::cout << "  " << std::left << std::setw(7) << n.ident
              << std::setw(10) << kind_name(n.kind)
              << " " << std::fixed << std::setprecision(4)
              << n.position.lat_deg << ", " << n.position.lon_deg
              << "  " << n.name << "\n";
}

}  // namespace

// main entry point
int main(int argc, char* argv[]) {
    const std::string path = (argc >= 2) ? argv[1] : "data/FAACIFP18"; // path to CIFP file
    const std::string start = (argc >= 3) ? argv[2] : "RDU"; // start ID
    const std::string goal  = (argc >= 4) ? argv[3] : "JFK"; // dest ID

    cifp::NavGraph graph;
    cifp::CIFPParser parser(/*low_altitude_only=*/true); // low alt airways only
    cifp::ParseStats stats;

    // parse the CIFP file
    std::cout << "Parsing CIFP file: " << path << "\n";
    if (!parser.parse(path, graph, stats)) {
        std::cerr << "ERROR: could not open " << path << "\n";
        return 1;
    }

    // generic stats
    std::cout << "\n=== Database Stats ===\n";
    std::cout << "  NAVAIDs loaded            : " << stats.navaids_loaded   << "\n";
    std::cout << "  Enroute waypoints loaded  : " << stats.waypoints_loaded << "\n";
    std::cout << "  Airway leg records read   : " << stats.airway_legs_read << "\n";
    std::cout << "  Edges added (bi-directional): " << stats.edges_added    << "\n";
    std::cout << "  High-altitude legs skipped: " << stats.skipped_high_alt << "\n";
    std::cout << "  Discontinuity breaks      : " << stats.discontinuities_skipped << "\n";
    std::cout << "  Outside-CONUS records     : " << stats.skipped_outside_conus << "\n";
    std::cout << "  Graph: " << graph.num_nodes() << " nodes, "
              << graph.num_edges() << " edges\n";

    std::cout << "\n=== Sample lookups ===\n";
    print_node(graph, "SSF"); // expect VOR
    print_node(graph, "ITTA"); // expect LOC
    print_node(graph, "JFK"); // expect VOR/DME
    print_node(graph, "RDU"); // expect VORTAC
    print_node(graph, "DODGR"); // expect FIX or WP

    // get neighbors via airways
    std::cout << "\n=== Sample neighbors of RDU ===\n";
    const cifp::NodeId rdu = graph.find("RDU");
    if (rdu != cifp::kInvalidNode) {
        for (const auto& e : graph.edges_from(rdu)) {
            std::cout << "  -> " << std::left << std::setw(7)
                      << graph.node(e.to).ident
                      << "  via " << std::setw(5) << e.airway
                      << "  " << std::fixed << std::setprecision(1)
                      << e.distance_nm << " NM"
                      << "  MEA " << e.mea_ft << " ft\n";
        }
    }

    // Construct the real-time query service. The service wraps the graph
    // with an LRU cache and per-query metric collection. Cache capacity of
    // 1024 is plenty for interactive use.
    cifp::FlightPlanService service(graph, /*cache_capacity=*/1024);

    // Build a query request. This is what an HTTP handler would assemble
    // from URL parameters or a JSON body.
    cifp::QueryRequest req;
    req.from_ident          = start;
    req.to_ident            = goal;
    //req.rnav_available       = false;

    std::cout << "\n=== Pathfinding " << start << " -> " << goal
              << " (low altitude, ceiling 17,000 ft) ===\n";

    // First query — cache miss, runs Dijkstra.
    cifp::QueryResponse resp = service.query(req);

    if (!resp.ok) {
        std::cout << "  No path found";
        if (!resp.error_message.empty()) std::cout << " (" << resp.error_message << ")";
        std::cout << ".\n";
    } else {
        std::cout << "Found Path: \n";
        std::cout << start << " ";
        auto lastAirway = resp.path.steps.front().airway;
        // print in FPL format
        for (const auto& s : resp.path.steps) {
            if (s.airway != lastAirway) {
                std::cout << lastAirway << " ";
                lastAirway = s.airway;
                std::cout << s.from_ident << " ";
            }
            // print in detailed format
            /*
            std::cout << "  " << std::left << std::setw(7) << s.from_ident
                      << " -> " << std::setw(7) << s.to_ident
                      << "  " << std::setw(5) << s.airway
                      << "  " << std::fixed << std::setprecision(1)
                      << std::setw(6) << s.distance_nm << " NM"
                      << "   MEA " << s.mea_ft << " ft\n";
            */
        }
        std::cout << lastAirway << " " << goal << "\n";
        std::cout << "  -----\n"
                  << "  Total distance: " << std::fixed << std::setprecision(1)
                  << resp.path.total_distance_nm << " NM\n"
                  << "  Highest MEA on route: " << resp.path.highest_mea_ft
                  << " ft\n"
                  << "  Server latency: " << std::setprecision(1)
                  << resp.latency_us << " us"
                  << " (cache " << (resp.served_from_cache ? "HIT" : "MISS") << ")\n";

        // Re-issue the same query. The service should serve it from cache
        // in microseconds — typically 50-100x faster than the cold query.
        cifp::QueryResponse resp2 = service.query(req);
        std::cout << "  Same query, second time: " << std::setprecision(1)
                  << resp2.latency_us << " us"
                  << " (cache " << (resp2.served_from_cache ? "HIT" : "MISS") << ")\n";
    }

    // Print service-wide metrics. In a long-running service these would be
    // exposed via a /metrics endpoint for monitoring tools to scrape.
    cifp::ServiceMetrics m = service.metrics();
    std::cout << "\n=== Service Metrics ===\n";
    std::cout << "  Total queries     : " << m.total_queries     << "\n";
    std::cout << "  Successful        : " << m.successful_queries << "\n";
    std::cout << "  Failed            : " << m.failed_queries    << "\n";
    std::cout << "  Cache hits        : " << m.cache_hits        << "\n";
    std::cout << "  Cache misses      : " << m.cache_misses      << "\n";
    std::cout << "  Cache hit rate    : " << std::fixed << std::setprecision(1)
              << (m.cache_hit_rate() * 100.0) << "%\n";
    std::cout << "  Avg latency       : " << std::setprecision(1)
              << m.avg_latency_us() << " us\n";

    return 0;
}