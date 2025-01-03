from setuptools import find_packages, setup

package_name = 'my_py_pkg'

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
    maintainer='harii',
    maintainer_email='harii@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "add_two_ints_server=my_py_pkg.add_two_ints_server:main",
            "hw_status_pub=my_py_pkg.hw_status_pub:main",
            "led_panel=my_py_pkg.led_panel:main",
            "battery=my_py_pkg.batter:main",
            "turtlesimcontroller=my_py_pkg.turtlesiimController:main",
            "scanModifier=my_py_pkg.scanmodifier:main",
            "datamodifier=my_py_pkg.dataModifier:main"
        ],
    },
)
