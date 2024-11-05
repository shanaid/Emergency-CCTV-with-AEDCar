from setuptools import setup

package_name = 'safety_package'

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
    maintainer='ability',
    maintainer_email='ek6015@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'laser_test = safety_package.laser_distance_test:main',
            # 'odom = safety_package.run_localization:main',
            'odom = safety_package.odometry:main',
            'ftc = safety_package.follow_the_carrot:main',
            'load_map = safety_package.load_map:main',
            'a_star_global = safety_package.a_star:main',
            'a_star_local = safety_package.a_star_local_path:main',
            'mapping = safety_package.run_mapping:main',
            'calibration = safety_package.ex_calib:main',
            'test_tracking = safety_package.path_tracking:main',
            'socket = safety_package.data_socket:main',
        ],
    },
)
