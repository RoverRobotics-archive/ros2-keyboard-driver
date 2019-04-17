from xml import etree
from setuptools import setup
from xml.etree import ElementTree

package = ElementTree.parse('package.xml')

properties_from_ros2_package = dict(
    name='ros2-' + package.findtext('name'),
    version=package.findtext('version'),
    description=package.findtext('description'),

    author=package.find('author').text,
    author_email=package.find('author').attrib['email'],

    maintainer=package.find('maintainer').text,
    maintainer_email=package.find('maintainer').attrib['email'],
)

setup(**properties_from_ros2_package)
