#!/usr/bin/env python

import os
import io
import re
from setuptools import setup, find_packages

VERSION = 1.0

requirements = [
    'certifi==2017.7.27.1',
    'chardet==3.0.4',
    'cycler==0.10.0',
    'idna==2.6',
    'matplotlib==2.0.2',
    'olefile==0.44',
    'opencv-python==3.2.0.7',
    'Pillow==4.2.1',
    'pygame==1.9.3',
    'pyparsing==2.2.0',
    'python-dateutil==2.6.1',
    'pytz==2017.2',
    'requests==2.18.4',
    'six==1.11.0',
    'u-msgpack-python==2.4.1',
    'urllib3==1.24.2',
    'pyfftw==0.10.4',
    'openpyxl',
    'numpy==1.13.1',
]

setup(
    # Metadata
    name='monodrive',
    version=VERSION,
    author='monoDrive',
    author_email='info@monodrive.io',
    url='https://github.com/monoDriveIO/PythonClient',
    description='Client for interacting with the monoDrive Driving Simulator.',
    long_description="test",
    license='BSD',

    # Package info
    packages=find_packages(exclude=('test',)),

    zip_safe=True,
    install_requires=requirements,

    data_files=[('configurations', ['configurations/demo.json', 'configurations/simulator.json', 'configurations/test.json'])]
)
