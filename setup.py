from setuptools import setup
import os

package_name = "mngr_fr_pkg"

setup(
    name=package_name,
    version="0.0.3",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    entry_points={
        "console_scripts": [
            "img_sub = " + package_name + ".img_sub:main",
            "cam_pub = " + package_name + ".cam_pub:main",
            "upd = " + package_name + ".update_users:main",
            "greet = " + package_name + ".greet_user:main",
        ],
    },
    zip_safe=True,
    maintainer="Kabalin Andrei",
    maintainer_email="kabalin.andrei@gmail.com",
    description="A face recognition script for the office manager robot",
    license="Apache License 2.0",
    tests_require=["pytest"],
    setup_requires=["flake8"],
)