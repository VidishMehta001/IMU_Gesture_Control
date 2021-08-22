from setuptools import setup

package_name = 'gesture_imu'

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
    maintainer='mecarill',
    maintainer_email='mihkailkennerley@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_recorder = gesture_imu.imu_recorder:main',
            'imu_strict_recorder = gesture_imu.imu_strict_recorder:main',
            'imu_recorder_magnitute = gesture_imu.imu_recorder_magnitute:main'
        ],
    },
)
