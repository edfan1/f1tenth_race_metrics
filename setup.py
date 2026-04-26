from setuptools import setup

package_name = 'f1tenth_race_metrics'

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
    maintainer='Ethan Fan',
    maintainer_email='ethanfan@seas.upenn.edu',
    description='f1tenth race metrics package for ESE 6510',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'f1tenth_race_metrics = f1tenth_race_metrics:main',
        ],
    },
)
