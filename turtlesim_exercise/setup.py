from setuptools import setup

package_name = 'turtlesim_exercise'

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
    maintainer='revaniko',
    maintainer_email='aniketos@gmx.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "turtle_controller = turtlesim_exercise.turtle_controller:main",
            "turtle_spawner = turtlesim_exercise.turtle_spawner:main",
        ],
    },
)
