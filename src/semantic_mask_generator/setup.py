from glob import glob
import os

from setuptools import setup


package_name = 'semantic_mask_generator'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['README.md']),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml'),
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='WHEELTEC innovation workspace',
    maintainer_email='fybo1984-max@users.noreply.github.com',
    description='Generate map-aligned semantic risks from RGB-D and markers.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'semantic_mask_node = semantic_mask_generator.node:main',
        ],
    },
)
