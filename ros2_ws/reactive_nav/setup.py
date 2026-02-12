from setuptools import find_packages, setup

package_name = 'reactive_nav'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='saman-aboutorab',
    maintainer_email='saman.aboutorab@gmail.com',
    description='Reactive navigation nodes: gap-following obstacle avoidance and LiDAR scan preprocessing',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
		'gap_follower = reactive_nav.gap_follower:main',    
        'scan_sanitizer = reactive_nav.scan_sanitizer:main',
        ],
    },
)
