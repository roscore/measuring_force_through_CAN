from setuptools import setup

package_name = 'fsr_sensor'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='A simple inverse kinematics package',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'fsr_sensor = fsr_sensor.fsr_sensor:main'
        ],
    },
)

