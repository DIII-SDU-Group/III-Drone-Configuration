from setuptools import find_packages, setup
import os

package_name = "iii_drone_configuration"
description = "The III-Drone configuration sub system. This package contains the configuration of the drone system, including configuration server node, parameter handler, parameter files, and C++ configurator classes."
maintainer = "Frederik Falk Nyboe"
maintainer_email = "ffn@sdu.dk"
license = "proprietary"
version = "2.2.0"

setup(
    name=package_name,
    version=version,
    packages=[package_name],
    data_files=[
        (os.path.join("share", package_name), ["package.xml"]),
    ],
    install_requires=["PyYAML>=6.0,<7", "setuptools"],
    zip_safe=True,
    maintainer=maintainer,
    maintainer_email=maintainer_email,
    description=description,
    license=license,
    entry_points={
        "console_scripts": [
            "configuration_server = iii_drone_configuration.configuration_server_node:main",
            "configuration_client = iii_drone_configuration.configuration_client_node:main",
        ],
    },
)
