import os
from glob import glob
from setuptools import setup

package_name = 'keystroke'


setup(
    name=package_name,
    version='0.3.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'pynput'],
    zip_safe=True,
    maintainer='Dan Rose',
    maintainer_email='dan@digilabs.io',
    description='Capture keyboard input from the local computer and publish it to ROS',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keystroke_listen = keystroke.listen:main',
            'keystroke_arrows_to_twist = keystroke.arrows_to_twist:main'
        ],
    },
)
