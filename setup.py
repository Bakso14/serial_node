from setuptools import find_packages, setup

package_name = 'serial_node'

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
    maintainer='sayyid',
    maintainer_email='sayyidul98@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "node_serial_cmd_vel = serial_node.node_serial_cmd_vel:main",
            "node_serial_wheel = serial_node.node_serial_wheel:main"
        ],
    },
)
