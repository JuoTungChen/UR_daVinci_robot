from setuptools import setup

PACKAGE_NAME = 'eua_control'

setup(
    name=PACKAGE_NAME,
    version='0.1.0',
    packages=[PACKAGE_NAME, 'dynamixel'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml', 'launch/launch_eua_control.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kim Lindberg Schwaner',
    maintainer_email='kils@mmmi.sdu.dk',
    description='The eua_control package.',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller = eua_control.controller:main',
            'center_servos = eua_control.center_servos:main',
        ],
    },
)
