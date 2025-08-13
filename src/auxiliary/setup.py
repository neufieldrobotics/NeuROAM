from setuptools import find_packages, setup

package_name = 'auxiliary'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='neuroam',
    maintainer_email='neuroam@northeastern.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'downsampler = auxiliary.downsampler:main'
        ],
    },
)
