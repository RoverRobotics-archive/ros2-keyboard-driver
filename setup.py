from pathlib import Path
from xml.etree import ElementTree

from setuptools import setup

here = Path(__file__).parent
package = ElementTree.parse(here / 'package.xml')
name = package.findtext('name')

colcon_properties = dict(
    # should be distinct among PyPI packages so add a prefix
    name='ros2-{}'.format(name),
    version=package.findtext('version'),
    description=package.findtext('description'),

    author=package.find('author').text,
    author_email=package.find('author').attrib['email'],

    maintainer=package.find('maintainer').text,
    maintainer_email=package.find('maintainer').attrib['email'],
)

launchfiles = [str(p.relative_to(here)) for p in (here.rglob('*.launch.py'))]
assert launchfiles

setup(**colcon_properties,
      data_files=[
          (str(Path('share', name)), ['package.xml']),
          (str(Path('share', name, 'launch')), launchfiles),
      ])
