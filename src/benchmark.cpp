//benchmarking for CIFP_Graph performance analysis
//version 2May26
//author 26colacito

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "../include/CIFPParser.h"
#include "../include/FlightPlanService.h"
#include "../include/NavGraph.h"
#include "../include/Pathfinding.h"

using cifp::NavGraph;
using cifp::FlightPlanService;
using cifp::QueryRequest;
using cifp::QueryResponse;

namespace {

// structure for random pair of fixes
struct IdentPair { std::string from, to; };

// get random pairs of identifiers
// params g: the NavGraph, n: number of pairs to sample, seed: random seed
// return vector of ID pairs
std::vector<IdentPair> sample_pairs(const NavGraph& g, std::size_t n, std::uint32_t seed = 42) {
    std::vector<cifp::NodeId> connected;
    connected.reserve(g.num_nodes() / 2);
    for (cifp::NodeId i = 0; i < g.num_nodes(); ++i) {
        if (!g.edges_from(i).empty()) connected.push_back(i); // check if they have an airway connection
    }

    std::mt19937 rng(seed); // random number generator
    std::uniform_int_distribution<std::size_t> pick(0, connected.size() - 1); // random index picker

    std::vector<IdentPair> out;
    out.reserve(n);
    // pick IDs from the rng
    while (out.size() < n) {
        const auto a = connected[pick(rng)];
        const auto b = connected[pick(rng)];
        if (a == b) continue;
        out.push_back({g.node(a).ident, g.node(b).ident});
    }
    return out;
}

// get the percentile of a vector
// params v: the vector to compute the percentile of, p: the percentile to compute (0.0 to 1.0)
// return the p-th percentile value
double percentile(std::vector<double>& v, double p) {
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    const std::size_t idx = std::min(v.size() - 1, static_cast<std::size_t>(p * v.size()));
    return v[idx];
}

// structure for run results
struct RunResult {
    std::vector<double> latencies_us;
    std::size_t successes = 0;
    double wall_time_s = 0.0;
};

// run the service workload
// params svc: the FlightPlanService, pairs: the pairs to test, ceiling_ft: the altitude ceiling
// return the run results
RunResult run_workload(FlightPlanService& svc, const std::vector<IdentPair>& pairs, int ceiling_ft) {
    RunResult r;
    r.latencies_us.reserve(pairs.size());
    const auto t0 = std::chrono::steady_clock::now(); // start timer
    // actually do the working
    for (const auto& p : pairs) {
        QueryRequest req;
        req.from_ident = p.from;
        req.to_ident = p.to;
        req.aircraft_ceiling_ft = ceiling_ft;
        QueryResponse resp = svc.query(req);
        r.latencies_us.push_back(resp.latency_us);
        if (resp.ok) ++r.successes;
    }
    const auto t1 = std::chrono::steady_clock::now(); // end timer
    r.wall_time_s = std::chrono::duration<double>(t1 - t0).count();
    return r;
}

// print latency summary
// params label: the label for this summary, r: the run results, csv: the output CSV stream
void print_latency_summary(const std::string& label, RunResult r, std::ofstream& csv) {
    auto lats = r.latencies_us;  // copy for sort
    const double mean = lats.empty() ? 0.0 : // avg
        std::accumulate(lats.begin(), lats.end(), 0.0) / lats.size();
    // percentiles
    const double p50 = percentile(lats, 0.50);
    const double p95 = percentile(lats, 0.95);
    const double p99 = percentile(lats, 0.99);
    const double max = lats.empty() ? 0.0 : lats.back();
    const double qps = r.wall_time_s > 0 ?
        static_cast<double>(lats.size()) / r.wall_time_s : 0.0;
    // formatting and printing
    std::cout << "  " << std::left << std::setw(28) << label
              << " queries=" << std::setw(6) << lats.size()
              << " ok="      << std::setw(6) << r.successes
              << std::fixed << std::setprecision(1)
              << " mean=" << std::setw(8) << mean << "us"
              << " p50="  << std::setw(8) << p50  << "us"
              << " p95="  << std::setw(8) << p95  << "us"
              << " p99="  << std::setw(8) << p99  << "us"
              << " max="  << std::setw(8) << max  << "us"
              << " qps="  << std::setprecision(0) << qps << "\n";

    csv << label << "," << lats.size() << "," << r.successes << ","
        << mean << "," << p50 << "," << p95 << "," << p99 << "," << max << ","
        << qps << "\n";
}

}  // namespace

// run scenario 1
void bench_baseline(const NavGraph& g, std::ofstream& csv) {
    std::cout << "\n=== Scenario 1: Baseline service throughput ===\n";
    std::cout << "  Random fix pairs, low-altitude only, ceiling 17,000 ft\n";

    const auto pairs = sample_pairs(g, 1000);

    FlightPlanService cached(g, /*cache_capacity=*/1024);
    auto r1 = run_workload(cached, pairs, 17000);
    print_latency_summary("with LRU cache (cold)", r1, csv);

    // expect all hits
    auto r2 = run_workload(cached, pairs, 17000);
    print_latency_summary("with LRU cache (warm)", r2, csv);

    FlightPlanService uncached(g, /*cache_capacity=*/0);
    auto r3 = run_workload(uncached, pairs, 17000);
    print_latency_summary("no cache", r3, csv);

    const double cold_mean = std::accumulate(r1.latencies_us.begin(),
        r1.latencies_us.end(), 0.0) / r1.latencies_us.size();
    const double warm_mean = std::accumulate(r2.latencies_us.begin(),
        r2.latencies_us.end(), 0.0) / r2.latencies_us.size();
    std::cout << "  -> warm cache speedup vs cold: "
              << std::fixed << std::setprecision(1)
              << (cold_mean / warm_mean) << "x\n";
}

// run scenario 2
void bench_lookup_vs_scan(const NavGraph& g, std::ofstream& csv) {
    std::cout << "\n=== Scenario 2: Hash-map ident lookup vs. linear scan ===\n";
    std::cout << "  Demonstrates O(1) interning vs O(N) per-lookup cost.\n";

    // pick 10k random ids
    std::mt19937 rng(7);
    std::uniform_int_distribution<cifp::NodeId> pick(0, g.num_nodes() - 1);
    std::vector<std::string> targets;
    targets.reserve(10000);
    for (int i = 0; i < 10000; ++i) targets.push_back(g.node(pick(rng)).ident);

    // hash map baseline
    {
        const auto t0 = std::chrono::steady_clock::now();
        std::size_t hits = 0;
        for (const auto& s : targets) {
            if (g.find(s) != cifp::kInvalidNode) ++hits;
        }
        const auto t1 = std::chrono::steady_clock::now();
        const double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
        const double per = us / targets.size();
        std::cout << "  hash-map find()       total=" << std::fixed
                  << std::setprecision(0) << us << "us, per-lookup="
                  << std::setprecision(3) << per << "us, hits=" << hits << "\n";
        csv << "hashmap_find," << targets.size() << "," << hits << ","
            << per << "," << per << "," << per << "," << per << ","
            << per << "," << (1e6/per) << "\n";
    }

    // alternative implementation linear scan
    {
        const auto t0 = std::chrono::steady_clock::now();
        std::size_t hits = 0;
        for (const auto& s : targets) {
            for (cifp::NodeId i = 0; i < g.num_nodes(); ++i) {
                if (g.node(i).ident == s) { ++hits; break; }
            }
        }
        const auto t1 = std::chrono::steady_clock::now();
        const double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
        const double per = us / targets.size();
        std::cout << "  linear scan           total=" << std::fixed
                  << std::setprecision(0) << us << "us, per-lookup="
                  << std::setprecision(3) << per << "us, hits=" << hits << "\n";
        std::cout << "  -> hash-map speedup vs scan: "
                  << std::setprecision(0) << (us / (us / 100.0 ? per : 1)) << "x (approx)\n";
        csv << "linear_scan," << targets.size() << "," << hits << ","
            << per << "," << per << "," << per << "," << per << ","
            << per << "," << (1e6/per) << "\n";
    }
}

// run scenario 3
void bench_memory(const NavGraph& g) {
    std::cout << "\n=== Scenario 3: Memory footprint ===\n";

    // baseline adjacency list
    const std::size_t N = g.num_nodes();
    const std::size_t E = g.num_edges();
    const std::size_t list_bytes_est =
          N * sizeof(cifp::Node)
        + E * sizeof(cifp::Edge)
        + N * sizeof(std::vector<cifp::Edge>)
        + N * (sizeof(std::string) + sizeof(cifp::NodeId) + 16);

    // alternative adjacency matrix
    const std::size_t matrix_bytes = N * N * 8;

    std::cout << "  Graph: N=" << N << " nodes, E=" << E << " edges\n";
    std::cout << "  Adjacency list (this impl) : ~"
              << std::fixed << std::setprecision(2)
              << (list_bytes_est / (1024.0 * 1024.0)) << " MB estimated\n";
    std::cout << "  Adjacency matrix (8B/cell) : ~"
              << (matrix_bytes / (1024.0 * 1024.0 * 1024.0)) << " GB if allocated\n";
    std::cout << "  -> matrix would be "
              << std::setprecision(0) << (static_cast<double>(matrix_bytes) / list_bytes_est)
              << "x larger than the adjacency list.\n";
}

// run scenario 4
void bench_scaling(const NavGraph& g, std::ofstream& csv) {
    std::cout << "\n=== Scenario 4: Cache size scaling ===\n";
    std::cout << "  Same workload, varying LRU capacity.\n";

    const auto pairs = sample_pairs(g, 2000);

    for (std::size_t cap : {0, 32, 128, 512, 2048}) {
        FlightPlanService svc(g, cap);
        auto r = run_workload(svc, pairs, 17000);
        // rerun with cache
        auto r2 = run_workload(svc, pairs, 17000);
        auto m = svc.metrics();
        std::cout << "  cap=" << std::setw(5) << cap
                  << "  2nd-pass mean=" << std::fixed << std::setprecision(2)
                  << (std::accumulate(r2.latencies_us.begin(),
                       r2.latencies_us.end(), 0.0) / r2.latencies_us.size())
                  << "us  hit_rate=" << std::setprecision(3) << m.cache_hit_rate()
                  << "  qps=" << std::setprecision(0)
                  << (r2.latencies_us.size() / r2.wall_time_s) << "\n";
        csv << "cache_cap_" << cap << "," << r2.latencies_us.size() << ","
            << r2.successes << ","
            << (std::accumulate(r2.latencies_us.begin(), r2.latencies_us.end(),
                                0.0) / r2.latencies_us.size())
            << ",,,,," << (r2.latencies_us.size() / r2.wall_time_s) << "\n";
    }
}

// main entry point
int main(int argc, char* argv[]) {
    const std::string path = (argc >= 2) ? argv[1] : "data/FAACIFP18";

    NavGraph graph;
    cifp::CIFPParser parser(true);
    cifp::ParseStats stats;
    const auto t_load_start = std::chrono::steady_clock::now();
    if (!parser.parse(path, graph, stats)) {
        std::cerr << "ERROR: could not open " << path << "\n";
        return 1;
    }
    const double load_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t_load_start).count();
    std::cout << "Loaded graph in " << std::fixed << std::setprecision(1)
              << load_ms << " ms: "
              << graph.num_nodes() << " nodes, " << graph.num_edges() << " edges\n";

    std::ofstream csv("bench_results.csv");
    csv << "scenario,queries,ok,mean_us,p50_us,p95_us,p99_us,max_us,qps\n";

    bench_baseline(graph, csv);
    bench_lookup_vs_scan(graph, csv);
    bench_memory(graph);
    bench_scaling(graph, csv);

    std::cout << "\nResults written to bench_results.csv\n";
    return 0;
}
