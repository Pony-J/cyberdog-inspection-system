#include "grpc/client/client.hpp"
#include "mstc_wrapper/types.hpp"

namespace mstc_grpc
{

MSTCClient::MSTCClient(std::shared_ptr<grpc::Channel> channel)
    : stub_(inspection::MSTCService::NewStub(channel)) {}

mstc_wrapper::PathsGrid MSTCClient::Plan(const mstc_wrapper::RoboGridPoses& robot_poses,const mstc_wrapper::WeighedEdges& edges)
{
    inspection::PlanRequest request;

    // Convert robot_poses
    for (const auto& [name, pos] : robot_poses)
    {
        (*request.mutable_robot_poses())[name].set_row(pos.first);
        (*request.mutable_robot_poses())[name].set_col(pos.second);
    }

    // Convert edges
    for (const auto& e : edges)
    {
        auto edge = request.add_weighed_edges();
        edge->mutable_from()->set_row(e.first.first.first);
        edge->mutable_from()->set_col(e.first.first.second);
        edge->mutable_to()->set_row(e.first.second.first);
        edge->mutable_to()->set_col(e.first.second.second);
        edge->set_weight(e.second);
    }

    inspection::PlanResponse response;
    grpc::ClientContext context;

    grpc::Status status = stub_->Plan(&context, request, &response);
    std::unordered_map<std::string, std::vector<std::pair<int,int>>> result;

    if (status.ok())
    {
        for (const auto& [name, path] : response.paths())
        {
            std::vector<std::pair<int,int>> nodes;
            for (const auto& node : path.nodes())
            {
                nodes.emplace_back(node.row(), node.col());
            }
            result[name] = nodes;
        }
    }
    else
    {
        std::cerr << "gRPC call failed: " << status.error_message() << std::endl;
    }
    return result;
}

}