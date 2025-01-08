from setuptools import find_packages, setup

package_name = 'kinova_python'

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
    maintainer='wego',
    maintainer_email='hj.ka@wego-robotics.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_mode_node = kinova_python.joint_mode:main',

            'gripper_node = kinova_python.gripper_test:main',

            'twist_mode_node = kinova_python.twist_mode:main',
            'twist_node = kinova_python.twist_test:main',

            'packaging_position = kinova_python.packaging:main',
            'zero_position = kinova_python.zero:main',
            'home_position = kinova_python.home:main',
        ],
    },
)
