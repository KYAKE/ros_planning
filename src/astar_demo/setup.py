import os
from glob import glob

from setuptools import setup


package_name = "astar_demo"


def package_tree(source_root: str, install_root: str):
    entries = {}
    for path in glob(os.path.join(source_root, "**", "*"), recursive=True):
        if os.path.isfile(path):
            rel_dir = os.path.dirname(os.path.relpath(path, source_root))
            if rel_dir == ".":
                install_dir = os.path.join("share", package_name, install_root)
            else:
                install_dir = os.path.join("share", package_name, install_root, rel_dir)
            entries.setdefault(install_dir, []).append(path)
    return sorted(entries.items())


data_files = [
    ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
    (f"share/{package_name}", ["package.xml"]),
]
data_files += package_tree("launch", "launch")
data_files += package_tree("maps", "maps")
data_files += package_tree("urdf", "urdf")
data_files += package_tree("meshes", "meshes")
data_files += package_tree("rviz", "rviz")


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jason",
    maintainer_email="jason@example.com",
    description="Standalone ROS2 A* demo with app-friendly topics.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "demo_server = astar_demo.demo_server:main",
        ],
    },
)
