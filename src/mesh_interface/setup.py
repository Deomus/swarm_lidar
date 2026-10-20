from setuptools import setup
setup(name='mesh_interface', version='0.1.0', packages=['mesh_interface'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/mesh_interface']),
                  ('share/mesh_interface', ['package.xml']),
                  ('share/mesh_interface/config', ['config/mesh.yaml'])],
      install_requires=['setuptools'], zip_safe=True,
      entry_points={'console_scripts': ['mesh_node = mesh_interface.node:main']})

