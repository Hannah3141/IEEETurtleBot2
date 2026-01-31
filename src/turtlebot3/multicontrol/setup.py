from setuptools import find_packages, setup

package_name = 'multicontrol'

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
    maintainer='rats',
    maintainer_email='mjb2632@utulsa.edu',
    description='for multiclub mashup',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mynode = multicontrol.mynode:main'
        ],
    },
)
