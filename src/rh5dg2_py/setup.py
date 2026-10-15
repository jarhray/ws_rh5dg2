from setuptools import find_packages, setup

package_name = 'rh5dg2_py'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jerry',
    maintainer_email='jarhray@users.noreply.github.com',
    description='Python client and CLI for the RH5DG2 ROS 2 driver',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'hand_cli = rh5dg2_py.cli:main',
        'angle_demo = rh5dg2_py.angle_demo:main',
    ]},
)
