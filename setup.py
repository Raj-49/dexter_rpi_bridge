from setuptools import setup, find_packages

package_name = 'dexter_rpi_bridge'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'adafruit-circuitpython-pca9685',
        'adafruit-blinka',
    ],
    zip_safe=True,
    maintainer='Raj',
    maintainer_email='raj@dexter-robotics.local',
    description='RPi hardware bridge for the Dexter dual-arm robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hardware_node = dexter_rpi_bridge.hardware_node:main',
        ],
    },
)
