from setuptools import setup

setup(
    name='toxicblend',
    version='0.2.0',
    description='Generated protobuffer files for https://crates.io/crates/toxicblend',
    url='https://crates.io/crates/toxicblend',
    author='Eadf',
    author_email='lacklustr@protonmail.com',
    license='AGPL-3.0',
    packages=['toxicblend'],
    install_requires=['protobuf>=3.15',
                      'grpcio>=1.37',
                      ],

    classifiers=[
        'Development Status :: 1 - Planning',
        'Intended Audience :: End Users/Desktop',
        'License :: OSI Approved :: GNU Affero General Public License v3 or later (AGPLv3+)',
        'Operating System :: OS Independent',
        'Topic :: Multimedia :: Graphics',
        'Programming Language :: Python :: 3.7',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
        'Programming Language :: Python :: 3.10',
    ],
)
