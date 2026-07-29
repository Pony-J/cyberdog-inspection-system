#!/usr/bin/env python3
"""
MSTC gRPC Server
"""

import sys
import os
import argparse

script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(script_dir, '../..'))
sys.path.insert(0, script_dir)
sys.path.insert(0, os.path.join(project_root, 'external'))  # MSTC_Star

from server import MSTCServer

def main():
    parser = argparse.ArgumentParser(description='MSTC gRPC Server')
    parser.add_argument('--port', type=int, default=50051, help='gRPC server port (default: 50051)')
    parser.add_argument('--host', type=str, default='[::]', help='gRPC server host (default: [::])')
    args = parser.parse_args()

    print(f"Starting MSTC gRPC Server on {args.host}:{args.port}")
    print("Press Ctrl+C to stop")
    
    server = MSTCServer(host=args.host, port=args.port)
    server.start()

if __name__ == '__main__':
    main()

