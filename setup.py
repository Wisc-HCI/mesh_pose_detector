from setuptools import setup

package_name = 'pose_detector'

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
    maintainer='kevin',
    maintainer_email='kevinwelsh132@gmail.com',
    description='ROS node wrapper for OTF 6-dof pose detection. Depends on https://github.com/kpwelsh/Mesh-Pose-Detector',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'server = pose_detector.server:main'
        ],
    },
)
