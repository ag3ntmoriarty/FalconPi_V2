from setuptools import find_packages, setup

package_name = 'falconpi_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='dev@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
		'poscmd2mavros = falconpi_bridge.poscmd2mavros:main',
		'odom2ap_tf = falconpi_bridge.odom2ap_tf:main',
		'cmdvel2mavros = falconpi_bridge.cmdvel2mavros:main',
		'poscmd2ap_cmdvel = falconpi_bridge.poscmd2ap_cmdvel:main',
		'bench_vel_test = falconpi_bridge.bench_vel_test:main',

        ],
    },
)
