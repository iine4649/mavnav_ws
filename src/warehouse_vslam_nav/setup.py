from setuptools import find_packages, setup

package_name = 'warehouse_vslam_nav'

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
    maintainer='admin',
    maintainer_email='admin@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'vslam_to_mavros = warehouse_vslam_nav.vslam_to_mavros:main',
            'warehouse_scan = warehouse_vslam_nav.warehouse_scan:main',
            'utils = warehouse_vslam_nav.utils:main',
        ],
    },
)
