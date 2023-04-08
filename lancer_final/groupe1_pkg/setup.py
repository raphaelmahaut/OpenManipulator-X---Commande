from setuptools import setup

package_name = 'groupe1_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='montmartre',
    maintainer_email='montmartre@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'enregistrement = groupe1_pkg.enregistrement:main',
            'lancement = groupe1_pkg.lancement:main',
            'camera = groupe1_pkg.camera:main',
            'manuel = groupe1_pkg.manuel:main'

        ],
    },
)
