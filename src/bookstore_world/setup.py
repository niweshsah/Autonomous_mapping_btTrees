# from setuptools import find_packages, setup

# package_name = 'bookstore_world'

# setup(
#     name=package_name,
#     version='0.0.0',
#     packages=find_packages(exclude=['test']),
#     data_files=[
#         ('share/ament_index/resource_index/packages',
#             ['resource/' + package_name]),
#         ('share/' + package_name, ['package.xml']),
#     ],
#     install_requires=['setuptools'],
#     zip_safe=True,
#     maintainer='niweshsah',
#     maintainer_email='sahniwesh@gmail.com',
#     description='TODO: Package description',
#     license='TODO: License declaration',
#     tests_require=['pytest'],
#     entry_points={
#         'console_scripts': [
#         ],
#     },
# )


from setuptools import find_packages, setup
import os
import glob

package_name = 'bookstore_world'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob.glob('worlds/*.world')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='niweshsah',
    maintainer_email='sahniwesh@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
