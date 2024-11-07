from setuptools import find_packages, setup

package_name = 'activity4_py_pkg'

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
    maintainer='predator',
    maintainer_email='francisferrir@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "battery = activity4_py_pkg.battery:main",
            "led_panel = activity4_py_pkg.led_panel:main"
        ],
    },
)
