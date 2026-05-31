from glob import glob

from setuptools import find_packages, setup

package_name = 'RL_special'


data_files = [
    (
        'share/ament_index/resource_index/packages',
        ['resource/' + package_name],
    ),
    (
        'share/' + package_name,
        ['package.xml', 'README.md'],
    ),
    (
        'share/' + package_name + '/launch',
        glob('launch/*.launch.py'),
    ),
]

model_files = glob('models/*.onnx')
if model_files:
    data_files.append(
        (
            'share/' + package_name + '/models',
            model_files,
        )
    )

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kwon',
    maintainer_email='kwon@todo.todo',
    description='ONNX specialist policy inference node for cart alignment',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'specialist_policy_node = RL_special.specialist_policy_node:main',
            'debug_odometry_keyboard_node = RL_special.debug_odometry_keyboard_node:main',
        ],
    },
)
