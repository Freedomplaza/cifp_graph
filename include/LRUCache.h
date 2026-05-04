//Header and implementation for LRUCache, fixed size least recently used route cache
//version 2May26
//author 26colacito

#ifndef CIFP_LRU_CACHE_H
#define CIFP_LRU_CACHE_H

#include <cstddef>
#include <list>
#include <optional>
#include <unordered_map>
#include <utility>

namespace cifp {

template <typename Key, typename Value, typename Hash = std::hash<Key>>
class LRUCache {
public:
    explicit LRUCache(std::size_t capacity) : capacity_(capacity) {} // constructor

    // key lookup, returns cached value or nullopt
    std::optional<Value> get(const Key& key) {
        auto map_it = index_.find(key);
        if (map_it == index_.end()) { // check if it missed
            ++misses_; // it missed
            return std::nullopt;
        }

        order_.splice(order_.begin(), order_, map_it->second);
        ++hits_; // it hit
        return map_it->second->second;
    }

    // insert or overwrite, kills least recently used if full
    void put(const Key& key, Value value) {
        auto map_it = index_.find(key);
        if (map_it != index_.end()) {
            map_it->second->second = std::move(value);
            order_.splice(order_.begin(), order_, map_it->second);
            return;
        }
        // kill LRU
        if (order_.size() >= capacity_) {
            const Key& evict_key = order_.back().first;
            index_.erase(evict_key);
            order_.pop_back();
            ++evictions_;
        }
        order_.emplace_front(key, std::move(value));
        index_.emplace(key, order_.begin());
    }

    // util
    std::size_t size() const     { return order_.size(); }
    std::size_t capacity() const { return capacity_; }
    std::size_t hits() const     { return hits_; }
    std::size_t misses() const   { return misses_; }
    std::size_t evictions() const{ return evictions_; }
    double hit_rate() const {
        const std::size_t total = hits_ + misses_;
        return total ? static_cast<double>(hits_) / total : 0.0;
    }

    void reset_metrics() { hits_ = misses_ = evictions_ = 0; }

private:
    using ListEntry = std::pair<Key, Value>;
    using ListIt    = typename std::list<ListEntry>::iterator;

    std::size_t capacity_;
    std::list<ListEntry> order_; // front = most recent
    std::unordered_map<Key, ListIt, Hash> index_; // key -> position in list
    std::size_t hits_ = 0;
    std::size_t misses_ = 0;
    std::size_t evictions_ = 0;
};

}  // namespace cifp

#endif  // CIFP_LRU_CACHE_H
