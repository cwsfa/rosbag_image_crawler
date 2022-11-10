from setuptools import setup

package_name = 'rosbag_image_crawler'

setup(
    name=package_name,
    version='0.1.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/image_crawler.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Shin',
    author_email='dev_shin@cwsfa.co.kr',
    maintainer=[
        'Shin',
        'leeeju',
        'Batbold'],
    maintainer_email=[
        'dev_shin@cwsfa.co.kr',
        'stu02@cwsfa.co.kr',
        'batbold@cwsfa.co.kr'],
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='The image crawler that extracts and stores image topics every set period from rosbag file.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_crawler = rosbag_image_crawler.image_crawler:main',
            'get_label = rosbag_image_crawler.get_label:main',
        ],
    },
)