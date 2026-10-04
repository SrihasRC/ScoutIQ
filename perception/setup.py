from setuptools import find_packages, setup

setup(
    name="roomwatch-perception",
    version="0.1.0",
    description="RoomWatch Perception Pipeline (GroundingDINO + MobileSAM + Semantic Mapping)",
    author="RoomWatch Team",
    packages=find_packages(),
    package_data={
        "roomwatch_perception": ["cfg/gdino/*.py"],
    },
    include_package_data=True,
    python_requires=">=3.8",
    entry_points={
        "console_scripts": [
            "rw-semantic-construct = roomwatch_perception.semantic_map_construction:main",
            "rw-semantic-update = roomwatch_perception.semantic_map_update:main",
        ],
    },
)
