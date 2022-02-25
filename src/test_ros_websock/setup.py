from setuptools import setup

package_name = 'test_ros_websock'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='plantstoen',
    maintainer_email='plantstoen@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'webSockBridge = test_ros_websock.webSockBridge:main',
            'webSockBridge_hover = test_ros_websock.webSockBridge_hover:main',
            'test_subscriber = test_ros_websock.test_subscriber:main'        ],
    },
)
