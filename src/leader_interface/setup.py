from setuptools import setup

setup(
    name='leader_interface', version='0.1.0', packages=['leader_interface'],
    data_files=[('share/ament_index/resource_index/packages', ['resource/leader_interface']),
                ('share/leader_interface', ['package.xml']),
                ('share/leader_interface/config', ['config/leader.yaml'])],
    install_requires=['setuptools', 'numpy'], zip_safe=True,
    entry_points={'console_scripts': ['leader_node = leader_interface.node:main']},
)
