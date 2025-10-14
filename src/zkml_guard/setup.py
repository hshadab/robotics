from setuptools import setup

package_name = 'zkml_guard'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/zkml_guard_demo.launch.py', 'launch/zkml_guard_proof.launch.py']),
        ('share/' + package_name + '/config', ['config/twist_mux.yaml', 'config/zkml_guard.params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zkml',
    maintainer_email='devnull@example.com',
    description='Proof-gated STOP for twist_mux via JOLT Atlas zkML (ROS 2, webcam)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'zkml_guard = zkml_guard.zkml_guard_node:main',
        ],
    },
)
