import grpc
from concurrent import futures
import time
import networkx as nx
import numpy as np
import sys
import os

script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, script_dir)

from generated import mstc_service_pb2
from generated import mstc_service_pb2_grpc

# import MSTC_Star algorithm
try:
    from MSTC_Star.mcpp.mstc_star_planner import MSTCStarPlanner
except ImportError:
    project_root = os.path.abspath(os.path.join(script_dir, '../..'))
    sys.path.insert(0, os.path.join(project_root, 'external'))
    from MSTC_Star.mcpp.mstc_star_planner import MSTCStarPlanner

# Implement the MSTCService service
class MSTCServiceServicer(mstc_service_pb2_grpc.MSTCServiceServicer):

    @staticmethod
    def _plan(edges, robot_poses):
        # Create a graph from edges
        G = nx.Graph()
        G.add_weighted_edges_from(edges)

        # make robot positions
        positions = [v for (_, v) in robot_poses.items()]

        planner = MSTCStarPlanner(G, len(positions), positions, np.inf, True)
        plans = planner.allocate()
        paths, _ = planner.simulate(plans)

        return paths

    def Plan(self, request, context):
        print("Request received")
        response = mstc_service_pb2.PlanResponse()
        # Convert request robot_poses to a dict for easier processing
        robot_poses = {k: (v.row, v.col) for k, v in request.robot_poses.items()}
        edges = [((e.in_node.row, e.in_node.col), (e.out_node.row, e.out_node.col), e.weight) for e in request.weighed_edges]

        paths = self._plan(edges, robot_poses)

        # Example logic: just create trivial paths from robot positions
        for k, p in zip(robot_poses.keys(), paths):
            path = mstc_service_pb2.Path()
            for (r, c) in p:
                path.nodes.add(row=int(r), col=int(c))
            response.paths[k].CopyFrom(path)
        print("Returning response")
        return response

# Wrap server in a class
class MSTCServer:
    
    def __init__(self, host='[::]', port=50051):
        self.server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
        mstc_service_pb2_grpc.add_MSTCServiceServicer_to_server(MSTCServiceServicer(), self.server)
        self.server.add_insecure_port(f'{host}:{port}')

    def start(self):
        self.server.start()
        print("Server started.")
        try:
            while True:
                time.sleep(86400)
        except KeyboardInterrupt:
            self.server.stop(0)