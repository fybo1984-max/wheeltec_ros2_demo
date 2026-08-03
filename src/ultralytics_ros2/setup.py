from glob import glob

from setuptools import setup


package_name = 'ultralytics_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', ['launch/yolo.launch.py']),
        ('share/' + package_name + '/model', glob('model/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='WHEELTEC innovation workspace',
    maintainer_email='fybo1984-max@users.noreply.github.com',
    description='Warehouse person detection messages for semantic navigation.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detection_node = ultralytics_ros2.detection_node:main',
        ],
    },
)
