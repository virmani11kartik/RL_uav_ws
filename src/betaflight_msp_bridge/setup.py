from setuptools import setup


package_name = 'betaflight_msp_bridge'


setup(
name=package_name,
version='0.1.0',
packages=[package_name],
data_files=[
('share/ament_index/resource_index/packages', ['resource/' + package_name]),
('share/' + package_name, ['package.xml']),
('share/' + package_name + '/launch', ['launch/main.launch.py']),
],
install_requires=['setuptools', 'pyserial'],
zip_safe=True,
    maintainer='kartik',
    maintainer_email='kartik8virmani@gmail.com',
    description='MSP bridge for Betaflight: RC write + telemetry for ROS 2',
    license='KV_1.0',
    entry_points={
        'console_scripts': [
            'msp_bridge = betaflight_msp_bridge.msp_bridge:main',
            'keyboard_teleop = betaflight_msp_bridge.keyboard_teleop:main',
            'safety_gate = betaflight_msp_bridge.safety_gate:main',
        ],
    },
)
