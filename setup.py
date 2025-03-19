from setuptools import setup
import os
from glob import glob

package_name = "on_track_sys_id"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*")),
        (os.path.join("share", package_name, "params"), glob("params/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Onur Dikici",
    maintainer_email="odikici@ethz.ch",
    description="The on_track_sys_id package",
    license="MIT",
    entry_points={
        "console_scripts": ["on_track_sys_id = on_track_sys_id.on_track_sys_id:main"],
    },
)
