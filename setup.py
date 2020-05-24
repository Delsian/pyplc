"""
Copyright (c) 2020 Eug Krashtan

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

from setuptools import setup, Extension

classifiers = ['Development Status :: 2 - Pre-Alpha',
               'Operating System :: POSIX :: Linux',
               'License :: OSI Approved :: MIT License',
               'Intended Audience :: Developers',
               'Programming Language :: Python :: 3',
               'Programming Language :: C',
               'Topic :: Communications',
               'Topic :: Software Development',
               'Topic :: Home Automation',
               'Topic :: System :: Hardware :: Hardware Drivers'] 

setup(name='pyplc',
      version           ='0.1.1',
      description       ='Python API for PLC modem',
      author            ='Eug Krashtan',
      author_email      ='eug.krashtan@gmail.com',
      keywords          = 'Raspberry Pi PLC PL360',
      license           = 'MIT',
      ext_modules       =[Extension('pyplc', ['src/pyplc.c','src/pyplc_hw.c','src/pyplc_if.c'], include_dirs=['src'])],
      py_modules        =['pyplc'],
      classifiers       = classifiers
)