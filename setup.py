from setuptools import setup
import os

package_name = "mngr_fr_pkg"

setup(
    name=package_name,
    version="0.0.2",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=[
        "setuptools",
        "face-recognition==1.3.0",
        "face-recognition-models==0.3.0",
        "opencv-python==4.7.0.72",
    ],
    entry_points={
        'console_scripts': [
            'img_sub = mngr_fr_pkg.img_sub:main',
            'cam_pub = mngr_fr_pkg.cam_pub:main',
            'upd = mngr_fr_pkg.update_users:main',
            'greet = mngr_fr_pkg.greet_user:main',
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