#pragma once

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace mstc_wrapper {

template<typename T>
using Point = std::pair<T, T>;

using PointGrid = Point<int>;
using PointWorld = Point<float>;
using PathGrid = std::vector<PointGrid>;
using PathWorld = std::vector<PointWorld>;
using PathsGrid = std::unordered_map<std::string, PathGrid>;
using PathsWorld = std::unordered_map<std::string, PathWorld>;

using RoboGridPoses = std::unordered_map<std::string, PointGrid>;
using RoboWorldPoses = std::unordered_map<std::string, PointWorld>;

using Node = std::pair<int, int>;
using Edge = std::pair<Node, Node>;
using WeighedEdges = std::vector<std::pair<Edge, float>>;

}  // namespace mstc_wrapper
