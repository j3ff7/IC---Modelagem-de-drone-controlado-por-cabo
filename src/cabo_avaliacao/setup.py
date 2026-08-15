import os
from glob import glob
from setuptools import find_packages, setup


package_name = 'cabo_avaliacao'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.json')),
        (os.path.join('share', package_name, 'docs'), glob('docs/*.md')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='joseubu',
    maintainer_email='nilson101704@gmail.com',
    description='Ambientes de avaliacao para os angulos locais do cabo.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'avaliador = cabo_avaliacao.avaliador:main',
            'gerar_mundos = cabo_avaliacao.gerar_mundos:main',
        ],
    },
)
