from setuptools import find_packages, setup

package_name = 'wheel_link_tester'

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
    maintainer='linkchen',
    maintainer_email='3300387192@qq.com',
    description='Low-speed wheel link tester for real_runner.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'joint_default_pose = wheel_link_tester.joint_default_pose:main',
            'wheel_test = wheel_link_tester.wheel_test:main',
        ],
    },
)
