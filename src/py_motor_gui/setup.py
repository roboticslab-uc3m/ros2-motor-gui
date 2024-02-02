from setuptools import find_packages, setup

package_name = 'py_motor_gui'

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
    maintainer='teo',
    maintainer_email='sanmasja@gmail.com',
    description='ROS2 Motor Gui for TEO',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motorGui = py_motor_gui.motorGuiLogic:main'
        ],
    },
)
