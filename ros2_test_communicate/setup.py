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
        'ros2_test_communicate.node_action_display',
        'ros2_test_communicate.estHand_display',
        'ros2_test_communicate.node_action_convert_coodinate',
        'ros2_test_communicate.node_hl2_transform_broadcaster',
        'ros2_test_communicate.node_hl2_transform_listener'
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
            'hand_action_display = ros2_test_communicate.node_action_display:main',
            'hand_action_convert = ros2_test_communicate.node_action_convert_coodinate:main',
            'hl2_tf_listener = ros2_test_communicate.node_hl2_transform_listener:main'
        ],
    },
)