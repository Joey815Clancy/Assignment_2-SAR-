from setuptools import find_packages, setup

package_name = 'SAR_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='joeyc',
    maintainer_email='23375817@studentmail.ul.ie',
    description='SAR navigation package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_travel = SAR_navigation.robot_travel:main',
        ],
    },
)
