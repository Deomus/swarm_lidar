from setuptools import setup
setup(name='autopilot_bridge', version='0.1.0', packages=['autopilot_bridge'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/autopilot_bridge']),
                  ('share/autopilot_bridge', ['package.xml']),
                  ('share/autopilot_bridge/config', ['config/bridge.yaml'])],
      install_requires=['setuptools', 'pymavlink'], zip_safe=True,
      entry_points={'console_scripts': ['bridge_node = autopilot_bridge.node:main',
                                       'gcs_heartbeat = autopilot_bridge.gcs:main']})

