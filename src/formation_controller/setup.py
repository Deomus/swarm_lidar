from setuptools import setup

setup(
    name='formation_controller',
    version='0.1.0',
    packages=['formation_controller'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/formation_controller']),
        ('share/formation_controller', ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    entry_points={'console_scripts': []},
)
