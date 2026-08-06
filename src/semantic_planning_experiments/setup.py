from glob import glob
import os

from setuptools import setup


package_name = 'semantic_planning_experiments'

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
    description='Run controller-free Nav2 semantic planning A/B experiments.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'semantic_archive_audit = '
            'semantic_planning_experiments.archive_audit:main',
            'semantic_archive_register = '
            'semantic_planning_experiments.archive_registration:main',
            'semantic_archive_allocate_replacement = '
            'semantic_planning_experiments.archive_replacement:main',
            'semantic_collection_preflight = '
            'semantic_planning_experiments.collection_preflight:main',
            'semantic_live_input_readiness = '
            'semantic_planning_experiments.input_readiness:main',
            'semantic_experiment_evaluate = '
            'semantic_planning_experiments.experiment_evaluate:main',
            'semantic_experiment_plan = '
            'semantic_planning_experiments.experiment_manifest:main',
            'semantic_planning_ab = semantic_planning_experiments.runner:main',
            'semantic_paper_export = '
            'semantic_planning_experiments.paper_export:main',
            'semantic_protocol_freeze = '
            'semantic_planning_experiments.protocol_freeze:main',
            'semantic_planning_summary = '
            'semantic_planning_experiments.report_summary:main',
            'synthetic_rgbd_source = '
            'semantic_planning_experiments.synthetic_rgbd:main',
        ],
    },
)
