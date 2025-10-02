from setuptools import setup

package_name = 'px4_python_star_flight_mode'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nathaniel Handan',
    maintainer_email='handanfoun@gmail.com',
    description='Custom PX4 flight mode in Python to draw a star pattern',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'star_flight_mode = px4_python_star_flight_mode.star_flight_mode:main'
        ],
    },
)