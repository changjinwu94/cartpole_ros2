from glob import glob
from setuptools import setup
setup(name='cartpole_hardware', version='0.2.0',
      packages=['cartpole_hardware'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/cartpole_hardware']),
                  ('share/cartpole_hardware', ['package.xml']),
                  ('share/cartpole_hardware/config', glob('config/*.yaml')),
                  ('share/cartpole_hardware/launch', glob('launch/*.py'))],
      install_requires=['setuptools', 'pyserial'], zip_safe=True,
      maintainer='Cartpole project', maintainer_email='maintainer@example.com',
      description='UNO R4 sensor bridge and FAULHABER RS232 cartpole control',
      license='MIT', entry_points={'console_scripts': ['cartpole = cartpole_hardware.node:main',
          'record = cartpole_hardware.record:main',
          'analyze = cartpole_hardware.analyze:main']})
