# CIFP Nav Graph — Real-Time Flight Plan Query API

A C++17 real-time API that ingests the FAA's Coded Instrument Flight
Procedures (CIFP) data file and serves low-latency flight-plan queries
over the resulting in-memory graph. Built for my ADV Data Structures
capstone project.

The system answers requests of the form "find the shortest low-altitude
airway route from fix A to fix B for an aircraft with ceiling X and equipment Y" in
microseconds (warm cache) to under 3 milliseconds (cold)*.

*: Mean times used.

## Contents

```
include/
  Geo.h                Coordinate type, ARINC 424 lat/lon parsing,
                       great-circle distance.
  NavGraph.h           Node, Edge, NavGraph (adjacency-list graph).
  CIFPParser.h         Streaming parser for the CIFP file.
  Pathfinding.h        Dijkstra over the graph (cost-aware, MEA-aware).
  LRUCache.h           Generic O(1) LRU cache (template).
  FlightPlanService.h  Real-time query API + metrics.

src/
  Geo.cpp NavGraph.cpp CIFPParser.cpp Pathfinding.cpp
  FlightPlanService.cpp                              # service layer
  main.cpp                                           # demo entry point
  benchmark.cpp                                      # performance harness

data/
  EMPTY                                              # user adds the current FAA CIFP here

API.md                Endpoint documentation (request/response shapes)
PERFORMANCE.md        Benchmark results and complexity analysis
```

## Example

```cpp
#include "CIFPParser.h"
#include "FlightPlanService.h"
#include "NavGraph.h"

cifp::NavGraph graph;
cifp::CIFPParser parser(true);
cifp::ParseStats stats;
parser.parse("data/FAACIFP18", graph, stats);

cifp::FlightPlanService service(graph, /*cache_capacity=*/2048);

cifp::QueryRequest req;
req.from_ident = "LAX";
req.to_ident   = "ABQ";
req.aircraft_ceiling_ft = 17000
req.rnav_available = true;

auto resp = service.query(req);
// resp.path.steps holds the airway-by-airway route
// resp.latency_us reports server-side time
// resp.served_from_cache distinguishes hot vs cold queries
```

## Database contents

Loaded from `data/FAACIFP18` (CIFP cycle 2605):

| Item                       | Count   |
| -------------------------- | ------- |
| VHF NAVAIDs                | 1,910   |
| Localizers                 | (a subset of NAVAIDs, classified separately) |
| Enroute waypoints          | 28,126  |
| Bidirectional airway edges | 23,532  |
| High-altitude legs filtered out | 3,570 |
| Discontinuity breaks skipped    |   141 |

A leg is low-altitude if the FAA's `Level` field is `L`, OR the field is
unspecified and the leg's MEA is below 18,000'. Pass `false` to the parser
to load all altitudes.

## Performance highlights

Full numbers in `PERFORMANCE.md`. The headlines:

- Cold-cache mean query: **4756 µs** (~360 qps)
- Warm-cache mean query: **34.1 µs** (~28,596 qps)
- Hash-map ident lookup vs linear scan: **1,900× faster**
- Adjacency list vs adjacency matrix: **1,353× less memory**

Every measured operation matches its theoretical complexity class.

## Documentation

- `API.md` — request/response API, error conditions, examples
- `PERFORMANCE.md` — benchmark methodology, results, complexity verification
