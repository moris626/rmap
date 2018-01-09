from setuptools import setup


setup(
    name='luftdaten_to_bufr',
    version='0.1.2',
    description='Python Distribution Utilities',
    author='Paolo Patruno',
    author_email='p-patruno@iperbole.bologna.it',
    url='https://github.com/r-map/rmap',
    packages=['luftdaten_to_bufr'],
    license="GNU GPL v2",
    entry_points={
        'console_scripts': [
            'luftdaten_to_bufr = luftdaten_to_bufr:main'
        ],
    },
)
