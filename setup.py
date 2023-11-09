from setuptools import setup

package_name = 'ros2_test_communicate'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    py_modules=[
        'ros2_test_communicate.test_talker',
        'ros2_test_communicate.test_listener',
        'ros2_test_communicate.node_compressed_image_preview',
        'ros2_test_communicate.node_depth_image_preview',
        'ros2_test_communicate.node_hololens_depth_preview',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='htm2323',
    author_email="is0578ri@ed.ritsumei.ac.jp",
    maintainer='htm2323',
    maintainer_email="is0578ri@ed.ritsumei.ac.jp",
    description='TODO: Package description.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_talker= ros2_test_communicate.test_talker:main',
            'test_listener= ros2_test_communicate.test_listener:main',
            'compressed_image_preview = ros2_test_communicate.node_compressed_image_preview:main',
            'depth_image_preview = ros2_test_communicate.node_depth_image_preview:main',
            'hololens_depth_preview = ros2_test_communicate.node_hololens_depth_preview:main',
        ],
    },
)