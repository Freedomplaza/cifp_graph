//Flight Plan Service implementation
//version 2May26
//author 26colacito

#include "../include/FlightPlanService.h"

#include <algorithm>
#include <chrono>

namespace cifp {

// constructor
FlightPlanService::FlightPlanService(const NavGraph& graph, std::size_t cache_capacity)
    : graph_(graph),
      cache_capacity_(cache_capacity),
      cache_(cache_capacity == 0 ? 1 : cache_capacity) {}

// make cache key
// params req: the query request
// return the cache key string
std::string FlightPlanService::make_cache_key(const QueryRequest& req) {
    // format: "from|to|ceiling|rnav"
    std::string key;
    key.reserve(req.from_ident.size() + req.to_ident.size() + 8);
    key.append(req.from_ident);
    key.push_back('|');
    key.append(req.to_ident);
    key.push_back('|');
    key.append(std::to_string(req.aircraft_ceiling_ft));
    key.push_back('|');
    key.append(req.rnav_available ? "1" : "0");
    return key;
}

// query a flight plan
// params req: the query request
// return the query response
QueryResponse FlightPlanService::query(const QueryRequest& req) {
    QueryResponse resp;
    const auto t_start = std::chrono::steady_clock::now(); // ladies and gentlemen, start your clocks!

    // check if user is stupid
    if (req.from_ident.empty() || req.to_ident.empty()) {
        resp.ok = false;
        resp.error_message = "from_ident and to_ident must be non-empty";
        return resp;
    }

    const std::string key = make_cache_key(req); // make cache key

    { // lock cache for multithread protection
        std::lock_guard<std::mutex> lock(mutex_);

        if (cache_capacity_ > 0) {
            auto cached = cache_.get(key);
            if (cached) { // we got a hit
                resp.ok = true;
                resp.path = *cached;
                resp.served_from_cache = true;
            }
        }

        if (!resp.served_from_cache) { // we missed :(
            // now run the pathfinding
            resp.path = shortest_path(graph_, req.from_ident, req.to_ident, req.aircraft_ceiling_ft, req.rnav_available);
            resp.ok = resp.path.found;
            if (!resp.ok) {
                resp.error_message = "no route found (check identifiers and ceiling)";
            } else if (cache_capacity_ > 0) { // route found, put it in the cache
                cache_.put(key, resp.path);
            }
        }

        // for benchmarking
        const auto t_end = std::chrono::steady_clock::now(); // ...and we're done!
        const double us = std::chrono::duration<double, std::micro>(t_end - t_start).count();
        resp.latency_us = us;

        // more benchmarking bs
        ++metrics_.total_queries;
        if (resp.ok) ++metrics_.successful_queries;
        else         ++metrics_.failed_queries;
        if (resp.served_from_cache) ++metrics_.cache_hits;
        else                        ++metrics_.cache_misses;

        metrics_.total_latency_us += us;
        if (us > metrics_.max_latency_us) metrics_.max_latency_us = us;
        if (metrics_.min_latency_us == 0.0 || us < metrics_.min_latency_us) {
            metrics_.min_latency_us = us;
        }
    }
    return resp;
}

// get metrics for benchmarking
// return copy of the current metrics
ServiceMetrics FlightPlanService::metrics() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return metrics_;
}

// reset the metrics
void FlightPlanService::reset_metrics() {
    std::lock_guard<std::mutex> lock(mutex_);
    metrics_ = ServiceMetrics{};
}

}  // namespace cifp
