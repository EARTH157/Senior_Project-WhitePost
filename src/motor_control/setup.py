from setuptools import find_packages, setup

package_name = 'motor_control'

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
    maintainer='raspi-earth',
    maintainer_email='raspi-earth@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'stepmotor1 = motor_control.stepper_joint1:main',
            'stepmotor2 = motor_control.stepper_joint2:main',
            'stepmotor3 = motor_control.stepper_joint3:main',
            'servodrive = motor_control.servo_driver_i2c:main',
            'joint1_node = motor_control.joint1_node:main',
            'joint2_node = motor_control.joint2_node:main',
            'joint3_node = motor_control.joint3_node:main',
        ],
    },
)
