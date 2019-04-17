from setuptools import setup

package_name = 'keystroke'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    py_modules=[
        'keystroke_listen',
    ],
    install_requires=['setuptools', 'pynput'],
    zip_safe=True,
    author='Dan Rose',
    author_email="dan@digilabs.io",
    keywords=['ROS', 'ROS2'],
    classifiers=[
        'Framework :: ROS2',
        'Intended Audience :: Developers',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='A node that does keyboard stuff',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keystroke_listen = keystroke_listen:main',
        ],
    },
)
