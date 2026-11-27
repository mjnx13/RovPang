from glob import glob

from setuptools import find_packages, setup

package_name = 'rovpang_arm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='1270161395@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'arm_controller = rovpang_arm.arm_controller:main',
		'arm_test = rovpang_arm.arm_test:main',
		'gripper_control = rovpang_arm.gripper_control:main',
		'arm_pose_server = rovpang_arm.arm_pose_server:main',
		'pick_place_server = rovpang_arm.pick_place_server:main',
        ],
    },
)
