from setuptools import setup

package_name = 'tello_mr2'

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
    maintainer='radek',
    maintainer_email='radekgoralewski@gmail.com',
    description='Package to control DJI Tello drone and collect data from it.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'save_imu_data = tello_mr2.save_imu_data:main',
            'save_video_imu_data = tello_mr2.save_video_imu_data:main'
        ],
    },
)
