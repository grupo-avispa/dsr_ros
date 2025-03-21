from setuptools import find_packages, setup

package_name = 'dsr_python'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alberto Tudela',
    maintainer_email='ajtudela@gmail.com',
    description='Python API for seamless integration of ROS nodes with the DSR',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'example_listener = dsr_python.example_listener:main',
        ],
    },
)
