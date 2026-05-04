# Performance Analysis

This document records the empirical performance measurements for the CIFP
Nav Graph project, the design choices they validate, and the comparisons
required by the rubric.

All measurements were collected by `benchmark.cpp` on my Mac laptop. The full output is preserved in
`bench_results.csv` for plotting.

## 1. Throughput, Latency, and Caching

The headline measurement: query latency before and after warming the
LRU cache.

| Workload                 | Mean     | p50      | p95      | p99     | Max      | Throughput |
| ------------------------ | -------- | -------- | -------- | ------- | -------- | ---------- |
| Cold cache (1st pass)    | 2756 µs  | 2761 µs  | 4325 µs  | 4879 µs | 18772 µs | 363 qps    |
| Warm cache (2nd pass)    | 34.1 µs  | 2.8 µs   | 5.2 µs   | 979  µs | 4968 µs  | 28,596 qps |
| Cache disabled           | 2687 µs  | 2727 µs  | 4279 µs  | 4426 µs | 4701 µs  | 372 qps    |

**Speedup from caching: 81× on mean latency, 79× on throughput.** The
warm-cache numbers are dominated by `unordered_map::find` on the cache key
plus a `PathResult` copy. Cold-cache and no-cache numbers are nearly
identical (as expected — cache miss ≈ no cache, plus a small insert cost).

The p99/max values stay elevated even on a warm cache because the very
first query to each unique key still pays the full pathfinding cost.

## 2. Hash Map Lookup vs. Linear Scan

The graph compresses string identifiers into dense `uint32_t` IDs at parse
time. To verify this matters, we benchmarked 10,000 lookups both ways.

| Method                             | Per-lookup | Speedup        |
| ---------------------------------- | ---------- | -------------- |
| `NavGraph::find` (unordered_map)   | 0.245 µs   | (baseline)     |
| Linear scan over `nodes_`          | 470 µs     | ~1,900× slower |

The 1,900× ratio demonstrates the speed difference between O(1) lookup and O(N) lookup.

## 3. Memory Footprint (List vs. Matrix)

The graph is sparse — 30,036 nodes but only 23,532 directed edges. 
An adjacency-list representation should be far superior.

| Representation                       | Size      |
| ------------------------------------ | --------- |
| Adjacency list (this implementation) | 5.09 MB   |
| Adjacency matrix (8 bytes/cell)      | 6.72 GB   |

**Matrix would be ≈1,353× larger.** Beyond memory, an adjacency matrix
also breaks Dijkstra's complexity guarantee: edge iteration becomes O(N)
per node visited instead of O(degree), turning the search from
O((V+E) log V) into O(V² log V). For this graph, V=30,036, so we'd be
trading a few-millisecond search for a multi-second one.

This is the single most impactful design decision in the project, and the
one most clearly justified by the data shape.

## 4. Cache Capacity Scaling

Same 2,000-query workload (1,000 unique pairs), varying LRU capacity:

| Capacity | Hit rate | Mean latency   | Throughput |
| -------- | -------- | -------------- | ---------- |
| 0        | 0%       | 2702 µs        | 370 qps    |
| 32       | 0%       | 2883 µs        | 347 qps    |
| 128      | 0%       | 2705 µs        | 370 qps    |
| 512      | 0%       | 2701 µs        | 370 qps    |
| 2048     | 49.3%    | 32.85 µs       | 29,690 qps |

This is a textbook **working set effect**. The 1,000 unique queries form
the working set; any cache smaller than that thrashes — the entry that
gets evicted is exactly the one that comes back next. Only when capacity
exceeds the working set does the hit rate take off. The 49.6% hit rate
at cap=2048 reflects that only the second pass finds entries cached.

The implication for production: cache capacity must be tuned to the
expected unique-query distribution, not the total query volume.

## 5. Theoretical vs. Empirical Complexity

| Operation                      | Theoretical    | Measured | Match? |
| ------------------------------ | -------------- | -------- | ------ |
| `find(ident)`                  | O(1)           | 0.245 µs | Yes    |
| Linear scan find (alternative) | O(N)           | 470 µs   | Yes    |
| `edges_from(id)` access        | O(1)           | <10ns    | Yes    |
| Dijkstra search                | O((V+E) log V) | ~2800 µs | Yes    |
| LRU `get`/`put`                | O(1) amortized | 2.8 µs   | Yes    |

Every measured operation matches its theoretical complexity class. No
weird surprises.

## 6. Design Trade-offs

| Decision                                      | Win                          | Cost                         |
| --------------------------------------------- | ---------------------------- | ---------------------------- |
| Adjacency list over matrix                    | 1353× memory, faster search  | Slightly more code           |
| Intern strings to `uint32_t` IDs              | 1900× faster inner loop      | Map lookup at API edge       |
| Per-node `std::vector<Edge>`                  | Cache-friendly iteration     | Pointer chasing across nodes |
| LRU cache in service layer                    | 65× warm-query speedup       | Memory proportional to capacity |
| Bidirectional edges (one per direction)       | No direction branching       | 2× edge storage              |
| Asymmetric MEA support                        | Handling of mountain routes  | Extra branch per leg parsing |
| Graph immutable after load                    | Lock-free reads possible     | Re-parse to add data         |

## 7. Where This Solution Excels and Where It Struggles

**Excels at**: high-volume read-heavy query workloads where some queries
repeat. A flight-planning service serving the major US airport pairs
falls into this category — JFK→BOS will be queried orders of magnitude
more often than random fix pairs.

**Struggles with**: workloads where every query is unique (no cache
benefit), where the underlying data changes frequently, although since
the practical application of this is cockpit mounted navigation, the
percieved difference to the operator will be so marginal that it can
almost entirely be ignored.
