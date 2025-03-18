from setuptools import find_packages, setup

package_name = 'drone_go_to'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tko',
    maintainer_email='kogucki@kth.se',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "drone_action_client= drone_go_to.droneActionClient:main",
            "drone_action_server= drone_go_to.droneActionServer:main",
        ],
    },
)
