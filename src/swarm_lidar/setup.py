from glob import glob
from pathlib import Path
from setuptools import setup

model_files = sorted(path for path in Path('models').rglob('*')
                     if path.is_file() and '__pycache__' not in path.parts)

setup(name='swarm_lidar', version='0.1.0', packages=['swarm_lidar'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/swarm_lidar']),
                  ('share/swarm_lidar', ['package.xml']),
                  ('share/swarm_lidar/launch', glob('launch/*.launch.py')),
                  ('share/swarm_lidar/worlds', glob('worlds/*.sdf')),
                  ('share/swarm_lidar/config', glob('config/*.yaml')),
                  ('share/swarm_lidar/web', glob('web/*')),
                  ('share/swarm_lidar/rviz', glob('rviz/*.rviz')),
                  *[('share/swarm_lidar/' + str(path.parent), [str(path)])
                    for path in model_files]],
      install_requires=['setuptools', 'numpy', 'PyYAML'], zip_safe=True,
      entry_points={'console_scripts': ['sitl = swarm_lidar.sitl:main',
                                       'scenario_runner = swarm_lidar.scenario:main',
                                       'web_panel = swarm_lidar.web_panel:main',
                                       'training_map = swarm_lidar.training_map:main',
                                       'cloud_source = swarm_lidar.cloud_source:main']})

