#!/usr/bin/env python3

import argparse
import os
import sys


def find_package_root():
    env_root = os.environ.get("CYBERDOG_INSPECTION_ROOT", "")
    candidates = [
        env_root,
        os.path.abspath(os.path.join(os.path.dirname(__file__), "..")),
        os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")),
        os.path.abspath(
            os.path.join(os.path.dirname(__file__), "../../share/cyberdog_inspection")
        ),
        os.path.abspath(os.path.join(os.getcwd(), "src/cyberdog_inspection")),
        os.path.abspath(os.path.join(os.getcwd(), "../src/cyberdog_inspection")),
    ]

    for candidate in candidates:
        if not candidate:
            continue
        server_py = os.path.join(candidate, "grpc", "python", "server.py")
        if os.path.exists(server_py):
            return candidate

    raise RuntimeError(
        "Cannot locate cyberdog_inspection package root. Set "
        "CYBERDOG_INSPECTION_ROOT to the package source directory."
    )


def main():
    parser = argparse.ArgumentParser(description="MSTC gRPC Server Wrapper")
    parser.add_argument("--port", type=int, default=50051)
    parser.add_argument("--host", type=str, default="[::]")
    args = parser.parse_args()

    package_root = find_package_root()
    grpc_python_dir = os.path.join(package_root, "grpc", "python")
    external_dir = os.path.join(package_root, "external")

    sys.path.insert(0, grpc_python_dir)
    sys.path.insert(0, external_dir)

    from server import MSTCServer

    print(f"Using cyberdog_inspection root at: {package_root}")
    print(f"Starting MSTC gRPC Server on {args.host}:{args.port}")
    server = MSTCServer(host=args.host, port=args.port)
    server.start()


if __name__ == "__main__":
    main()
