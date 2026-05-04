//implementation of the adjacency-list graph.
//version 2May26
//author 26colacito

#include "../include/NavGraph.h"

namespace cifp {

// insert or update a node in the graph
// params n: the node to insert or update
// return the ID of the inserted or updated node
NodeId NavGraph::upsert_node(const Node& n) {
    if (n.ident.empty()) return kInvalidNode;

    auto it = ident_index_.find(n.ident);
    if (it != ident_index_.end()) {
        // fill in missing data in case node was input mostly blank from an airway
        Node& existing = nodes_[it->second];
        if (existing.kind == FixKind::Unknown && n.kind != FixKind::Unknown) {
            existing.kind = n.kind;
        }
        if (existing.name.empty() && !n.name.empty()) {
            existing.name = n.name;
        }
        if (existing.position.lat_deg == 0.0 &&
            existing.position.lon_deg == 0.0 &&
            (n.position.lat_deg != 0.0 || n.position.lon_deg != 0.0)) {
            existing.position = n.position;
        }
        return it->second;
    }

    const NodeId id = static_cast<NodeId>(nodes_.size()); // new node ID
    nodes_.push_back(n); // add the node
    adjacency_.emplace_back();  // empty edge list
    ident_index_.emplace(n.ident, id); // map identifier to node ID
    return id;
}

// add airway edge
// params from: source node ID, edge: the edge to add
void NavGraph::add_edge(NodeId from, Edge edge) {
    if (from >= adjacency_.size()) return; // invalid source
    if (edge.to >= nodes_.size()) return; // invalid target
    adjacency_[from].push_back(std::move(edge)); // add that jawn in!
}

// find node by identifier
// params ident: the identifier to search for
NodeId NavGraph::find(std::string_view ident) const {
    auto it = ident_index_.find(std::string(ident)); // try to find it
    return (it == ident_index_.end()) ? kInvalidNode : it->second; // return node ID or invalid
}

// count edges
// return count of edges
std::size_t NavGraph::num_edges() const {
    std::size_t total = 0;
    for (const auto& list : adjacency_) total += list.size(); // go through them and sum them
    return total;
}

}  // namespace cifp
