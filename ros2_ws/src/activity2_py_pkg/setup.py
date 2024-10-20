from setuptools import find_packages, setup

package_name = 'activity2_py_pkg'

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
            "number_publisher = activity2_py_pkg.number_publisher:main",
            "number_counter = activity2_py_pkg.number_counter:main"
        ],
    },
)
