from glob import glob
from xml.etree import ElementTree

from setuptools import setup

package = ElementTree.parse('package.xml')
name = package.findtext('name')

properties_from_ros2_package = dict(
    name='ros2-{}' + name,
    version=package.findtext('version'),
    description=package.findtext('description'),

    author=package.find('author').text,
    author_email=package.find('author').attrib['email'],

    maintainer=package.find('maintainer').text,
    maintainer_email=package.find('maintainer').attrib['email'],
)
setup(**properties_from_ros2_package,
      data_files=[('share/' + name, ['package.xml', *glob('launch/*.launch.py')])],
      )
