from setuptools import setup
from glob import glob

package_name = 'cpluspod_joycontroller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.launch.xml')),
    ],
    py_modules=[
        'cpluspod_joycontroller.joy2twist',
        ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tmistudent',
    maintainer_email='shimo@ucl.nuee.nagoya-u.ac.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cpluspod_joycontroller = cpluspod_joycontroller.joy2twist:main'
        ],
    },
)
