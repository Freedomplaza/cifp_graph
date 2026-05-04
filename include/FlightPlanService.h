//Flight Plan Service file header
//version 2May26
//author 26colacito

#ifndef CIFP_FLIGHT_PLAN_SERVICE_H
#define CIFP_FLIGHT_PLAN_SERVICE_H

#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "LRUCache.h"
#include "NavGraph.h"
#include "Pathfinding.h"

namespace cifp {

// query request structure
struct QueryRequest {
    std::string from_ident; // Fix or NAVAID identifier
    std::string to_ident;
    int aircraft_ceiling_ft = 18000; // generic low alt MAA
    bool rnav_available = true;
};

// query response structure
struct QueryResponse {
    bool        ok = false;
    std::string error_message;
    PathResult  path; // valid when ok == true
    double      latency_us = 0.0; // measured server-side
    bool        served_from_cache = false;
};

// service metrics structure
struct ServiceMetrics {
    std::uint64_t total_queries     = 0;
    std::uint64_t successful_queries = 0;
    std::uint64_t failed_queries    = 0;
    std::uint64_t cache_hits        = 0;
    std::uint64_t cache_misses      = 0;
    double        total_latency_us  = 0.0; // for averaging
    double        max_latency_us    = 0.0;
    double        min_latency_us    = 0.0; // 0 means "no queries yet"

    // get the average latency
    double avg_latency_us() const {
        return total_queries ? total_latency_us / total_queries : 0.0;
    }
    // get the cache hit rate
    double cache_hit_rate() const {
        const std::uint64_t total = cache_hits + cache_misses;
        return total ? static_cast<double>(cache_hits) / total : 0.0;
    }
};

class FlightPlanService {
public:
    FlightPlanService(const NavGraph& graph, std::size_t cache_capacity = 1024);

    // get a response
    QueryResponse query(const QueryRequest& req);

    // for benchmarking
    ServiceMetrics metrics() const;
    void reset_metrics();

    // direct access when necessary
    const NavGraph& graph() const { return graph_; }

private:
    static std::string make_cache_key(const QueryRequest& req);

    const NavGraph& graph_;
    std::size_t     cache_capacity_;
    LRUCache<std::string, PathResult> cache_;
    mutable std::mutex mutex_;
    ServiceMetrics  metrics_;
};

}  // namespace cifp

#endif  // CIFP_FLIGHT_PLAN_SERVICE_H
