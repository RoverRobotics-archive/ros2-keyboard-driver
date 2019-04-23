from glob import glob
import os
from xml.etree import ElementTree

from setuptools import setup

package = ElementTree.parse('package.xml')
name = package.findtext('name')

properties_from_ros2_package = dict(
    name='ros2-{}'.format(name),
    version=package.findtext('version'),
    description=package.findtext('description'),

    author=package.find('author').text,
    author_email=package.find('author').attrib['email'],

    maintainer=package.find('maintainer').text,
    maintainer_email=package.find('maintainer').attrib['email'],
)
setup(**properties_from_ros2_package,
      data_files=[
          (os.path.join('share', name), ['package.xml']),
          (os.path.join('share', name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
      ],
      )
