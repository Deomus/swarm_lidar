from setuptools import setup

setup(name='mission_planner', version='0.1.0', packages=['mission_planner'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/mission_planner']),
                  ('share/mission_planner', ['package.xml']),
                  ('share/mission_planner/config', ['config/mission.yaml'])],
      install_requires=['setuptools'], zip_safe=True,
      entry_points={'console_scripts': ['planner_node = mission_planner.node:main']})
