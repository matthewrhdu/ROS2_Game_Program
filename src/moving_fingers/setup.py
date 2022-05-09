from setuptools import setup

package_name = 'moving_fingers'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='matthewrhdu',
    maintainer_email='matthewrhdu@mail.utoronto.ca',
    description='A Package using a publisher and subscriber to demonstrate a moving finger',
    license='Apache 1.2',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'brain = moving_fingers.brain:main',
            'sensor1 = moving_fingers.sensor1:main',
            'sensor2 = moving_fingers.sensor2:main',
            'sensor3 = moving_fingers.sensor3:main',
            'sensor4 = moving_fingers.sensor4:main',
            'sensor5 = moving_fingers.sensor5:main',
            'actuator1 = moving_fingers.actuator1:main',
            'actuator2 = moving_fingers.actuator2:main',
            'actuator3 = moving_fingers.actuator3:main',
            'actuator4 = moving_fingers.actuator4:main',
            'actuator5 = moving_fingers.actuator5:main',
        ],
    },
)
