from setuptools import find_packages, setup

setup(
    name="scoutiq-perception",
    version="0.1.0",
    description="ScoutIQ Perception Pipeline (GroundingDINO + MobileSAM + Semantic Mapping)",
    author="ScoutIQ Team",
    packages=find_packages(),
    package_data={
        "scoutiq_perception": ["cfg/gdino/*.py"],
        "roomwatch_perception": ["cfg/gdino/*.py"],
    },
    include_package_data=True,
    python_requires=">=3.8",
    entry_points={
        "console_scripts": [
            "scoutiq-semantic-construct = scoutiq_perception.semantic_map_construction:main",
            "scoutiq-semantic-update = scoutiq_perception.semantic_map_update:main",
            "rw-semantic-construct = scoutiq_perception.semantic_map_construction:main",
            "rw-semantic-update = scoutiq_perception.semantic_map_update:main",
        ],
    },
)
