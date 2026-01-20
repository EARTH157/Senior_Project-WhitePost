from setuptools import find_packages, setup

package_name = 'encoder_unit'

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
            'as5600_1 = encoder_unit.as5600_1:main', 
            'as5600_2 = encoder_unit.as5600_2:main', 
            'as5600_3 = encoder_unit.as5600_3:main', 
            'as5600_4 = encoder_unit.as5600_4:main', 
            'as5600_5 = encoder_unit.as5600_5:main', 
            'limit = encoder_unit.limit_switch_node:main',
        ],
    },
)
