from setuptools import find_packages, setup

package_name = 'teleop_dual_arm'

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
    maintainer='dev',
    maintainer_email='anshul.chauhan@rwth-aachen.de',
    description='Teleoperation package for dual-arm robot with gripper control.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_node = teleop_dual_arm.teleop_node:main',        # For teleop node
        ],
    },
)
