from setuptools import find_packages, setup

package_name = 'driving_square'

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
    maintainer='detch',
    maintainer_email='dmateedunsatits@olin.edu',
    description='It drives the robot in a square pattern.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'driving_square = driving_square.drive_square:main'
        ]
    },
)
