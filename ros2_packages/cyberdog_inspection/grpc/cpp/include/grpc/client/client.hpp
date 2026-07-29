#pragma once

#include <grpcpp/grpcpp.h>
#include <iostream>
#include <memory>
#include <unordered_map>

#include "grpc/generated/mstc_service.grpc.pb.h"
#include "mstc_wrapper/types.hpp"

namespace mstc_grpc
{

class MSTCClient
{
public:
    MSTCClient(std::shared_ptr<grpc::Channel> channel);

    mstc_wrapper::PathsGrid Plan(const mstc_wrapper::RoboGridPoses& robot_poses,const mstc_wrapper::WeighedEdges& edges);

private:
    std::unique_ptr<inspection::MSTCService::Stub> stub_;
};

}