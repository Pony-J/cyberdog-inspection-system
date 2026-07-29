from setuptools import setup
from glob import glob
import os


package_name = "cyberdog_web_bridge"


def data_files_for(directory: str, install_subdir: str):
    files = []
    for path in glob(os.path.join(directory, "*")):
        if os.path.isfile(path):
            files.append((install_subdir, [path]))
    return files


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md", "API_HANDOFF.md"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*")),
        *data_files_for(package_name + "/static", "share/" + package_name + "/static"),
    ],
    install_requires=[
        "setuptools",
        "fastapi",
        "uvicorn",
        "pydantic",
    ],
    zip_safe=True,
    maintainer="dev",
    maintainer_email="dev@cyberdog.local",
    description="Jetson-side web bridge for CyberDog visualization and inspection control.",
    entry_points={
        "console_scripts": [
            "bridge_node = cyberdog_web_bridge.bridge_node:main",
        ],
    },
)
