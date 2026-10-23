from setuptools import setup
setup(name='swarm_manager', version='0.1.0', packages=['swarm_manager'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/swarm_manager']),
                  ('share/swarm_manager', ['package.xml']),
                  ('share/swarm_manager/config', ['config/manager.yaml'])],
      install_requires=['setuptools'], zip_safe=True,
      entry_points={'console_scripts': ['manager_node = swarm_manager.node:main']})

