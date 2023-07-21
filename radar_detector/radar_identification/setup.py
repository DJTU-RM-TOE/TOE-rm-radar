from setuptools import setup
from Cython.Build import cythonize

package_name = 'radar_identification'
utils = 'radar_identification/utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name,utils],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='evence',
    maintainer_email='evencewu@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'python_node = radar_identification.main:main'
        ],
    },
)
