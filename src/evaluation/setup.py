from setuptools import setup

setup(name='evaluation', version='0.1.0', packages=['evaluation'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/evaluation']),
                  ('share/evaluation', ['package.xml']),
                  ('share/evaluation/config', ['config/evaluation.yaml'])],
      install_requires=['setuptools', 'numpy', 'matplotlib'], zip_safe=True,
      entry_points={'console_scripts': ['recorder_node = evaluation.record:main',
                                        'ground_truth_node = evaluation.ground_truth:main',
                                        'plot_metrics = evaluation.plots:main']})
