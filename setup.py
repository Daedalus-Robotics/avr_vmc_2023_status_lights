from setuptools import setup

package_name = 'status_lights'

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
    maintainer='Arya Lohia',
    maintainer_email='alohia@ideaventionsacademy.org',
    description='Ros2 node for controlling status indicators',
    license='LGPL-3.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'status_lights_node = status_lights.status_lights:main',
        ],
    },
)
