from setuptools import find_packages, setup
from glob import glob
import os

package_name = "localization_practice"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'LICENSE']),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
        (
            os.path.join("share", package_name, "worlds"),
            glob(os.path.join("worlds", "*.wbt")),
        ),
        (
            os.path.join("share", package_name, "protos"),
            glob(os.path.join("protos", "*.proto")),
        ),
        (
            os.path.join("share", package_name, "maps"),
            glob(os.path.join("maps", "*.pgm"))
            + glob(os.path.join("maps", "*.png"))
            + glob(os.path.join("maps", "*.yaml")),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob(os.path.join("config", "*.yaml")),
        ),
        (
            os.path.join("share", package_name, "urdf"),
            glob(os.path.join("urdf", "*.urdf")),
        ),
        (
            os.path.join("share", package_name, "rviz"),
            glob(os.path.join("rviz", "*.rviz")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ярослав Кузнецов",
    maintainer_email="xumuk228123@gmail.com",
    description="AMCL localization and ground truth in Webots",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "circle_motion = localization_practice.circle_motion:main",
            "collect_localization = localization_practice.collect_localization:main",
        ],
    },
)
