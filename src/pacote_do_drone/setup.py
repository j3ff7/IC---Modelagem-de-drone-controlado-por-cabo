import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'pacote_do_drone'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # --- COMEÇO DO QUE VOCÊ PRECISA ADICIONAR ---
        # Instala a pasta launch
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        
        # Instala a pasta worlds
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        
        # Instala os arquivos soltos na pasta models (como o cabo.sdf e cabo.xacro)
        (os.path.join('share', package_name, 'models'), glob('models/*.*')),
        
        # Instala o model.sdf e config do drone
        (os.path.join('share', package_name, 'models/meu_drone'), glob('models/meu_drone/*.*')),

        
        # Instala os arquivos 3D (.stl) das hélices e do corpo
        (os.path.join('share', package_name, 'models/meu_drone/meshes'), glob('models/meu_drone/meshes/*.*')),
        # --- FIM DO QUE VOCÊ PRECISA ADICIONAR ---
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seu_nome',
    maintainer_email='seu_email@todo.todo',
    description='Pacote do drone para Iniciação Científica',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensores = pacote_do_drone.sensores:main'
        ],
    },
)