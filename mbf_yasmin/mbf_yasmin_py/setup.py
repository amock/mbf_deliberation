from setuptools import setup

package_name = 'mbf_yasmin_py'

setup(
    name=package_name,
    version='0.20.4',
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
    description='Examples of minimal action clients calling MBF actions using Yasmin.',
    license='BSD 3-clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigate_to_goal = mbf_yasmin_py.navigate_to_goal:main'
        ],
    },
)
