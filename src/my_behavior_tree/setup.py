from setuptools import find_packages, setup

package_name = 'my_behavior_tree'

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
    maintainer='niweshsah',
    maintainer_email='sahniwesh@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_tree = my_behavior_tree.simple_tree:main',
            'turtlebot_btree = my_behavior_tree.turtlebot_btree:main',
            'frontier_exploration = my_behavior_tree.frontier_exploration:main',
            'frontier2 = my_behavior_tree.frontier2:main',
            'frontier_btTree = my_behavior_tree.frontier_btTree:main',
        ],
    },
)
