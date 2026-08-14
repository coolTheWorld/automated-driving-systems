"""carla_bridge 的 ament_python 打包说明."""
from setuptools import setup

package_name = 'carla_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/carla_sim.launch.py']),
        ('share/' + package_name + '/config', ['config/carla_bridge_params.yaml']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='孙帅',
    maintainer_email='suns1377154414@gmail.com',
    description='CARLA 侧翻译层（SPEC §4.1 环境 B）',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'carla_sidecar_node = carla_bridge.carla_sidecar_node:main',
        ],
    },
)
