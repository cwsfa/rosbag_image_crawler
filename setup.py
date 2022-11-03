import os
from glob import glob
from setuptools import setup

package_name = 'image_crawling'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')) # 파이썬 단일 패키지는 런시 설정을 추가 해줘야함. 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='turtle',
    maintainer_email='turtle@todo.todo',
    description='TODO: Package description',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bagimgset = image_crawling.rosbag_to_imgset:main',      # 했는데 왜 이러냐?
        ],
    },
)
