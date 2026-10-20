from glob import glob
from setuptools import setup
setup(name='swarm_lidar', version='0.1.0', packages=['swarm_lidar'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/swarm_lidar']),
                  ('share/swarm_lidar', ['package.xml']),
                  ('share/swarm_lidar/launch', glob('launch/*.launch.py')),
                  ('share/swarm_lidar/worlds', glob('worlds/*.sdf')),
                  ('share/swarm_lidar/config', glob('config/*.yaml')),
                  *[('share/swarm_lidar/' + folder, glob(folder + '/*'))
                    for folder in glob('models/*')]],
      install_requires=['setuptools'], zip_safe=True,
      entry_points={'console_scripts': ['sitl = swarm_lidar.sitl:main']})

