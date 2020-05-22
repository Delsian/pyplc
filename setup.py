from setuptools import setup, Extension
setup(name='pyplc',
      version='0.0.2',
      description='Python API for PLC modem',
      author='Eug Krashtan',
      author_email='eug.krashtan@gmail.com',
      ext_modules=[Extension('pyplc', ['src/pyplc.c','src/pyplc_hw.c','src/c_gpio.c'], include_dirs=['src'])],
      py_modules=['pyplc']
)