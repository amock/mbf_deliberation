from setuptools import setup

package_name = 'mbf_smach'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Alexander Mock',
    author_email='amock@uos.de',
    maintainer='Alexander Mock',
    maintainer_email='amock@uos.de',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: BSD 3-clause',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Examples for using MBF action interface via SMACH.',
    license='BDF 3-clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigate_to_pose = mbf_smach.navigate_to_pose:main'
        ],
    },
)
