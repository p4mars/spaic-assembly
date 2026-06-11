from setuptools import find_packages, setup

package_name = 'navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/navigation.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tetris-assembly',
    maintainer_email='todo@todo.com',
    description='Move-to-location service wrapping Nav2 NavigateToPose.',
    license='TODO',
    entry_points={
        'console_scripts': [
            'move_to_server = navigation.move_to_server:main',
        ],
    },
)
