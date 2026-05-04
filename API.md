# API Documentation

This project creates a real-time query API for low-altitude airway routing
across the continental United States, based on the FAA's CIFP (Coded
Instrument Flight Procedures) data file.

The API has two layers:
1. **Graph layer** (`NavGraph`): the database in memory.
2. **Service layer** (`FlightPlanService`): the request/response API.
   Adds caching, metrics, and thread safety on top of the graph.

## Service Layer

### Endpoint: `query`

```cpp
QueryResponse FlightPlanService::query(const QueryRequest& req);
```

Resolves a flight-plan request from a starting fix to a goal fix,
honoring an aircraft service ceiling. Cached results are returned
immediately; cache misses trigger a Dijkstra search.

#### Request

```cpp
struct QueryRequest {
    std::string from_ident;          // e.g. "LAX"
    std::string to_ident;            // e.g. "ABQ"
    int aircraft_ceiling_ft = 18000; // any leg with MEA > ceiling is unflyable
    bool rnav_available = true;      // allows filtering of GPS routes
};
```

#### Response

```cpp
struct QueryResponse {
    bool        ok;                  // true = path found
    std::string error_message;       // populated when ok == false
    PathResult  path;                // full route description
    double      latency_us;          // measured server-side
    bool        served_from_cache;   // true = found path in cache
};

struct PathResult {
    bool                  found;             
    double                total_distance_nm;
    int                   highest_mea_ft;
    std::vector<PathStep> steps;
};

struct PathStep {
    std::string from_ident;
    std::string to_ident;
    std::string airway;              // route ID, e.g. "V210"
    double      distance_nm;
    int         mea_ft;
};
```

#### Example

```cpp
cifp::NavGraph graph;
cifp::CIFPParser parser(true);
cifp::ParseStats stats;
parser.parse("data/FAACIFP18", graph, stats);

cifp::FlightPlanService service(graph, /*cache_capacity=*/1024);

cifp::QueryRequest req;
req.from_ident = "LAX";
req.to_ident   = "ABQ";
req.aircraft_ceiling_ft = 17000;
req.rnav_available = true;

cifp::QueryResponse resp = service.query(req);
if (resp.ok) {
    std::cout << "Total distance: " << resp.path.total_distance_nm << " NM\n";
    for (const auto& step : resp.path.steps) {
        std::cout << "  " << step.from_ident << " -> " << step.to_ident
                  << " via " << step.airway << "\n";
    }
}
```

#### Errors

| `error_message`                                    | Cause                                |
| -------------------------------------------------- | ------------------------------------ |
| `"from_ident and to_ident must be non-empty"`      | Empty input                          |
| `"no route found (check identifiers and ceiling)"` | Idents not in graph or no path found |

### Endpoint: `metrics`

```cpp
ServiceMetrics FlightPlanService::metrics() const;
```

Returns service-wide counters since the last `reset_metrics()` call.

```cpp
struct ServiceMetrics {
    uint64_t total_queries;
    uint64_t successful_queries;
    uint64_t failed_queries;
    uint64_t cache_hits;
    uint64_t cache_misses;
    double   total_latency_us;
    double   max_latency_us;
    double   min_latency_us;
    double   avg_latency_us() const;
    double   cache_hit_rate() const;
};
```

## Graph Layer (Read API)

For applications that want raw graph access (custom pathfinders, analytics, visualization):

| Method                            | Returns                       | Complexity     |
| --------------------------------- | ----------------------------- | -------------- |
| `find(ident)`                     | `NodeId` or `kInvalidNode`    | O(1) amortized |
| `node(id)`                        | `const Node&`                 | O(1)           |
| `edges_from(id)`                  | `const std::vector<Edge>&`    | O(1)           |
| `num_nodes()`                     | `std::size_t`                 | O(1)           |
| `num_edges()`                     | `std::size_t`                 | O(N)           |

A `Node` has `ident`, `name`, `kind` (FixKind), and `position` (LatLon).
An `Edge` has `to`, `airway`, `distance_nm`, `mea_ft`, `maa_ft`.

## Loading

```cpp
cifp::CIFPParser parser(/*low_altitude_only=*/true);
cifp::ParseStats stats;
bool ok = parser.parse("data/FAACIFP18", graph, stats);
```

Returns false on I/O error. `ParseStats` reports record counts for each
section processed, plus filtering counts (high-altitude legs skipped,
discontinuities, outside-CONUS records).

## Thread Safety

- `NavGraph` is read-only after `parse()` returns. Concurrent reads from
  multiple threads are safe without external synchronization.
- `FlightPlanService::query` is internally synchronized via mutex.
  Multiple threads may call it concurrently.
- `metrics()` is also synchronized.

## Limitations

- Continental US only. Alaska, Hawaii, and territories are filtered out
  at parse time.
- Low-altitude airway structure only by default. Pass `false` to the
  `CIFPParser` constructor to load all altitudes.
- The CIFP cycle is a static snapshot. To pick up FAA updates, replace
  `data/FAACIFP18` and re-parse.
- Identifier collisions: if a NAVAID and a waypoint share the same ident
  (rare), the second one to be loaded wins. The CIFP source orders
  NAVAIDs first.
