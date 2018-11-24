from setuptools import setup, find_packages
from codecs import open
from os import path

here = path.abspath(path.dirname(__file__))

with open(path.join(here, 'README.md'), encoding='utf-8') as f:
    long_description = f.read()

setup(
    name='tellopy',

    version='0.7.0.dev0',

    description='DJI Tello drone controller',
    url='https://github.com/hanyazou/TelloPy',
    author='Hanyazou',
    author_email='hanyazou@gmail.com',
    python_requires='>=2.7, !=3.0.*, !=3.1.*, !=3.2.*, !=3.3.*, !=3.4.*, <4',

    classifiers=[
        # How mature is this project? Common values are
        #   3 - Alpha
        #   4 - Beta
        #   5 - Production/Stable
        'Development Status :: 3 - Alpha',

        # Indicate who your project is intended for
        'Intended Audience :: Developers',
        'Topic :: Software Development :: Libraries',

        # Pick your license as you wish
        'License :: OSI Approved :: Apache Software License',

        # Specify the Python versions you support here. In particular, ensure
        # that you indicate whether you support Python 2, Python 3 or both.
        # 'Programming Language :: Python :: 2',
        'Programming Language :: Python :: 2.7',
        # 'Programming Language :: Python :: 3',
        # 'Programming Language :: Python :: 3.4',
        'Programming Language :: Python :: 3.5',
        'Programming Language :: Python :: 3.6',
    ],

    keywords='sample development robotics drone',

    packages=find_packages(exclude=['contrib', 'docs', 'tests', 'files']),

    install_requires=[
    ],

    project_urls={
        'Bug Reports': 'https://github.com/hanyazou/TelloPy/issues',
        'Say Thanks!': 'https://twitter.com/hanyazou',
        'Source': 'https://github.com/hanyazou/TelloPy',
    },
)
