from setuptools import find_packages, setup

package_name = 'point_ratio'

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
    maintainer='henrique',
    maintainer_email='henrique@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'point_ratio = point_ratio.point_ratio:main',
            'point_ratio_kdtree = point_ratio.point_ratio_kdtree:main',
            'radar_text_marker = point_ratio.radar_text_marker:main',
        ],
    },
)
