from setuptools import find_packages, setup

package_name = 'spectacularAI_vio_test'

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
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spectacularAI_test_node = spectacularAI_vio_test.spectacularAI_vio_test:main',
            "specAI_vio_images = spectacularAI_vio_test.specAI_vio_images:main",
        ],
    },
)
