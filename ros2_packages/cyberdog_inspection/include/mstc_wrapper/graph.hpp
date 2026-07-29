#pragma once

#include <algorithm>
#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "mstc_wrapper/types.hpp"

namespace mstc_wrapper {

struct NodeHash {
  size_t operator()(const Node & p) const noexcept
  {
    uint64_t key =
      (static_cast<uint64_t>(static_cast<uint32_t>(p.first)) << 32) |
      static_cast<uint32_t>(p.second);
    return std::hash<uint64_t>{}(key);
  }
};

struct NodeEq {
  bool operator()(const Node & a, const Node & b) const noexcept
  {
    return a.first == b.first && a.second == b.second;
  }
};

using AdjList = std::unordered_map<Node, std::vector<std::pair<Node, double>>, NodeHash, NodeEq>;

class Graph {
public:
  explicit Graph(const WeighedEdges & g)
  : graph_(g)
  {
    build_adjacency(g);
  }

  double batch_dijkstra(
    const Node & start,
    const std::vector<Node> & targets,
    std::vector<Node> & out_path,
    Node & out_target) const
  {
    using PQItem = std::pair<double, Node>;
    std::priority_queue<PQItem, std::vector<PQItem>, std::greater<>> pq;

    std::unordered_map<Node, double, NodeHash> dist;
    std::unordered_map<Node, Node, NodeHash> prev;

    pq.push({0.0, start});
    dist[start] = 0.0;

    std::unordered_set<Node, NodeHash> target_set(targets.begin(), targets.end());

    while (!pq.empty()) {
      auto [cur_dist, u] = pq.top();
      pq.pop();

      if (cur_dist > dist[u]) {
        continue;
      }

      if (target_set.count(u)) {
        out_target = u;
        out_path.clear();
        for (Node cur = u; prev.find(cur) != prev.end(); cur = prev[cur]) {
          out_path.push_back(cur);
        }
        out_path.push_back(start);
        std::reverse(out_path.begin(), out_path.end());
        return cur_dist;
      }

      auto it = adj_.find(u);
      if (it == adj_.end()) {
        continue;
      }

      for (const auto & [v, weight] : it->second) {
        double new_dist = cur_dist + weight;
        if (!dist.count(v) || new_dist < dist[v]) {
          dist[v] = new_dist;
          prev[v] = u;
          pq.push({new_dist, v});
        }
      }
    }

    return std::numeric_limits<double>::infinity();
  }

  bool has_node(const Node & n) const
  {
    return adj_.find(n) != adj_.end();
  }

private:
  void build_adjacency(const WeighedEdges & edges)
  {
    adj_.clear();
    adj_.reserve(edges.size() * 2 + 1);

    for (const auto & ew : edges) {
      const Edge & e = ew.first;
      const double w = static_cast<double>(ew.second);
      adj_[e.first].push_back({e.second, w});
      adj_[e.second].push_back({e.first, w});
    }
  }

  WeighedEdges graph_;
  AdjList adj_;
};

}  // namespace mstc_wrapper
