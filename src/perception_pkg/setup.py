import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'perception_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'worlds'),
            glob(os.path.join('worlds', '*.world'))),
        (os.path.join('share', package_name, 'models'),
            glob(os.path.join('models', '*.sdf'))),
        (os.path.join('share', package_name, 'urdf'),
            glob(os.path.join('urdf', '*'))),
        # YOLO agirliklari: sign_detector bunlari ament_index uzerinden
        # ariyor, dolayisiyla share altina kurulmalari sart.
        (os.path.join('share', package_name, 'weights'),
            glob(os.path.join(package_name, 'weights', '*.pt'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ibrahim Tarik Solak',
    maintainer_email='ibrahimtariksolak.is@gmail.com',
    description='ROS2 perception package for autonomous vehicle: traffic sign detection and lane following',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sign_detector = perception_pkg.sign_detector:main',
            'serit_takip   = perception_pkg.serit_takip:main',
            'guvenlik      = perception_pkg.guvenlik:main',
        ],
    },
)
