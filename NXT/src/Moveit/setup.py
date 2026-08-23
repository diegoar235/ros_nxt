from setuptools import setup

package_name = 'Moveit'  # ← Reemplaza por el nombre real de tu paquete

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
    maintainer='Diego',
    maintainer_email='tucorreo@example.com',
    description='Paquete con control de trayectoria para MoveIt',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_motion = src.simple_motion:main',
        ],
    },
)
