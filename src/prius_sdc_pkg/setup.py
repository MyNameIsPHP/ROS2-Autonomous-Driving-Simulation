from setuptools import setup
import os 
from glob import glob
package_name = 'prius_sdc_pkg'
lane_module= "prius_sdc_pkg/lane"
models_module= "prius_sdc_pkg/models"
object_module= "prius_sdc_pkg/object"
traffsign_module= "prius_sdc_pkg/traffsign"
utils_module= "prius_sdc_pkg/utils"

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, lane_module, models_module, object_module, traffsign_module, utils_module],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,'launch'), glob('launch/*')),
        (os.path.join('share', package_name,'worlds'), glob('worlds/*')),
        (os.path.join('lib', package_name), glob('scripts/*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='phucphan',
    maintainer_email='phucphan1421@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'recorder_node = prius_sdc_pkg.video_recorder:main',
            'driver_node =   prius_sdc_pkg.driving_node:main',
            'spawner_node =  prius_sdc_pkg.sdf_spawner:main'

        ],
    },
)
