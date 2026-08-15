import json
import os
import math

base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
caminho_json = os.path.join(base_dir, 'tether_parameters.json')
pasta_models = os.path.join(base_dir, 'models')
pasta_worlds = os.path.join(base_dir, 'worlds')
caminho_sdf = os.path.join(pasta_models, 'cabo.sdf')
caminho_world = os.path.join(pasta_worlds, 'my_world.sdf')
caminho_carretel_sdf = os.path.join(pasta_models, 'carretel', 'carretel.sdf')
caminho_drone_sdf = os.path.join(pasta_models, 'meu_drone', 'meu_drone.sdf')

os.makedirs(pasta_models, exist_ok=True)
os.makedirs(pasta_worlds, exist_ok=True)

with open(caminho_json, 'r') as f:
    params = json.load(f)

num_links = params['num_links']
length = params['length']
radius = params['radius']
mass = params['mass']
drone_x = params['drone_x']
drone_y = params['drone_y']
drone_z = params['drone_z']

comprimento_total = num_links * length
ancora_x = 0.0
ancora_y = 0.18
ancora_z = 0.33

yaw_base = math.atan2(drone_y - ancora_y, drone_x - ancora_x)
spawn_distance = comprimento_total
drone_spawn_x = ancora_x + spawn_distance * math.cos(yaw_base)
drone_spawn_y = ancora_y + spawn_distance * math.sin(yaw_base)
drone_spawn_z = ancora_z

print(f'Alvo original (JSON): ({drone_x:.4f}, {drone_y:.4f}, {drone_z:.4f})')
print(f'Spawn inicial do drone: ({drone_spawn_x:.4f}, {drone_spawn_y:.4f}, {drone_spawn_z:.4f})')
print(f'Cabo: {num_links} elos x {length:.4f} m = {comprimento_total:.4f} m')

ixx_segment = 0.5 * mass * radius ** 2
iyy_segment = (1.0 / 12.0) * mass * (3 * radius ** 2 + length ** 2)
izz_segment = iyy_segment
joint_limit_rad = 0.7
joint_damping = 0.08
joint_friction = 0.002
joint_spring_stiffness = 0.02

sdf = f'''<?xml version="1.0" ?>
<sdf version="1.8">
  <model name="cabo_flexivel">
    <link name="raiz_cabo">
      <inertial>
        <mass>0.02</mass>
        <inertia>
          <ixx>8e-7</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>8e-7</iyy><iyz>0</iyz><izz>8e-7</izz>
        </inertia>
      </inertial>
      <collision name="col_raiz"><geometry><sphere><radius>0.01</radius></sphere></geometry></collision>
      <visual name="vis_raiz"><geometry><sphere><radius>0.01</radius></sphere></geometry><material><ambient>0.8 0.1 0.1 1</ambient><diffuse>0.8 0.1 0.1 1</diffuse></material></visual>
    </link>
'''

parent_link = 'raiz_cabo'
for i in range(1, num_links + 1):
    segment_name = 'final_segment' if i == num_links else f'segment_{i}'
    dummy_name = f'dummy_{i}'
    joint_offset = 0.0 if i == 1 else length

    sensor_xml = ''
    if i == 1:
        sensor_xml = '''
        <sensor name="sensor_tensao_carretel" type="force_torque">
          <always_on>true</always_on>
          <update_rate>50</update_rate>
          <topic>/cabo/tensao_carretel</topic>
        </sensor>'''
    elif i == num_links:
        sensor_xml = '''
        <sensor name="sensor_tensao_drone" type="force_torque">
          <always_on>true</always_on>
          <update_rate>50</update_rate>
          <topic>/cabo/tensao_drone</topic>
        </sensor>'''

    sdf += f'''
    <joint name="joint_{i}_y" type="revolute">
      <pose relative_to="{parent_link}">{joint_offset} 0 0 0 0 0</pose>
      <parent>{parent_link}</parent>
      <child>{dummy_name}</child>
      <axis>
        <xyz>0 1 0</xyz>
        <limit><lower>-{joint_limit_rad}</lower><upper>{joint_limit_rad}</upper><effort>5</effort><velocity>10</velocity></limit>
        <dynamics>
          <damping>{joint_damping}</damping>
          <friction>{joint_friction}</friction>
          <spring_reference>0</spring_reference>
          <spring_stiffness>{joint_spring_stiffness}</spring_stiffness>
        </dynamics>
      </axis>{sensor_xml}
    </joint>
    <link name="{dummy_name}">
      <pose relative_to="joint_{i}_y">0 0 0 0 0 0</pose>
      <inertial><mass>0.001</mass><inertia><ixx>1e-7</ixx><ixy>0</ixy><ixz>0</ixz><iyy>1e-7</iyy><iyz>0</iyz><izz>1e-7</izz></inertia></inertial>
    </link>
    <joint name="joint_{i}_z" type="revolute">
      <pose relative_to="{dummy_name}">0 0 0 0 0 0</pose>
      <parent>{dummy_name}</parent>
      <child>{segment_name}</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit><lower>-{joint_limit_rad}</lower><upper>{joint_limit_rad}</upper><effort>5</effort><velocity>10</velocity></limit>
        <dynamics>
          <damping>{joint_damping}</damping>
          <friction>{joint_friction}</friction>
          <spring_reference>0</spring_reference>
          <spring_stiffness>{joint_spring_stiffness}</spring_stiffness>
        </dynamics>
      </axis>
    </joint>
    <link name="{segment_name}">
      <pose relative_to="joint_{i}_z">0 0 0 0 0 0</pose>
      <visual name="visual">
        <pose>{length / 2.0} 0 0 0 1.5708 0</pose>
        <geometry><cylinder><radius>{radius}</radius><length>{length}</length></cylinder></geometry>
        <material><ambient>0 0 0 1</ambient><diffuse>0 0 0 1</diffuse></material>
      </visual>
      <collision name="collision">
        <pose>{length / 2.0} 0 0 0 1.5708 0</pose>
        <geometry><cylinder><radius>{radius}</radius><length>{length * 0.9}</length></cylinder></geometry>
      </collision>
      <inertial>
        <pose>{length / 2.0} 0 0 0 0 0</pose>
        <mass>{mass}</mass>
        <inertia>
          <ixx>{ixx_segment}</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>{iyy_segment}</iyy><iyz>0</iyz><izz>{izz_segment}</izz>
        </inertia>
      </inertial>
    </link>
'''
    parent_link = segment_name

sdf += '''
    <link name="ponta_cabo">
      <pose relative_to="final_segment">{length} 0 0 0 0 0</pose>
      <inertial>
        <mass>0.001</mass>
        <inertia>
          <ixx>1e-7</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>1e-7</iyy><iyz>0</iyz><izz>1e-7</izz>
        </inertia>
      </inertial>
      <visual name="visual">
        <geometry><sphere><radius>0.008</radius></sphere></geometry>
        <material><ambient>0.9 0.05 0.05 1</ambient><diffuse>0.9 0.05 0.05 1</diffuse></material>
      </visual>
    </link>
    <joint name="joint_ponta_cabo" type="fixed">
      <parent>final_segment</parent>
      <child>ponta_cabo</child>
    </joint>
  </model>
</sdf>
'''.format(length=length)

with open(caminho_sdf, 'w') as f:
    f.write(sdf)

print('cabo.sdf gerado com cadeia revoluta estavel.')

world = f'''<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="mundo_ic">
    <gravity>0 0 -9.81</gravity>

    <physics name="1ms" type="ignored">
      <max_step_size>0.0004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-forcetorque-system" name="gz::sim::systems::ForceTorque"/>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry></collision>
        <visual name="visual"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry><material><ambient>0.8 0.8 0.8 1</ambient><diffuse>0.8 0.8 0.8 1</diffuse></material></visual>
      </link>
    </model>

    <model name="meu_carretel">
      <static>true</static>
      <link name="visual_link">
        <visual name="base_visual">
          <pose>0 0 0.01 0 0 0</pose>
          <geometry><box><size>0.4 0.7 0.02</size></box></geometry>
          <material><ambient>0.3 0.3 0.3 1</ambient><diffuse>0.3 0.3 0.3 1</diffuse></material>
        </visual>
        <visual name="haste_esq_visual">
          <pose>0 0.23 0.15 0 0 0</pose>
          <geometry><box><size>0.06 0.06 0.30</size></box></geometry>
          <material><ambient>0.2 0.2 0.8 1</ambient><diffuse>0.2 0.2 0.8 1</diffuse></material>
        </visual>
        <visual name="haste_dir_visual">
          <pose>0 -0.23 0.15 0 0 0</pose>
          <geometry><box><size>0.06 0.06 0.30</size></box></geometry>
          <material><ambient>0.2 0.2 0.8 1</ambient><diffuse>0.2 0.2 0.8 1</diffuse></material>
        </visual>
        <visual name="cilindro_visual">
          <pose>0 0 0.26 1.570796 0 0</pose>
          <geometry><cylinder><radius>0.07</radius><length>0.40</length></cylinder></geometry>
          <material><ambient>0.5 0.3 0.3 1</ambient><diffuse>0.5 0.3 0.3 1</diffuse></material>
        </visual>
      </link>
    </model>

    <model name="sistema_cabo_drone">
      <pose>0 0 0 0 0 0</pose>

      <link name="ancora_cabo">
        <pose>{ancora_x} {ancora_y} {ancora_z} 0 0 0</pose>
        <inertial>
          <mass>10</mass>
          <inertia>
            <ixx>1</ixx><ixy>0</ixy><ixz>0</ixz>
            <iyy>1</iyy><iyz>0</iyz><izz>1</izz>
          </inertia>
        </inertial>
        <visual name="visual">
          <geometry><sphere><radius>0.015</radius></sphere></geometry>
          <material><ambient>0.9 0.05 0.05 1</ambient><diffuse>0.9 0.05 0.05 1</diffuse></material>
        </visual>
      </link>

      <joint name="fixa_ancora_cabo_mundo" type="fixed">
        <parent>world</parent>
        <child>ancora_cabo</child>
      </joint>

      <include>
        <uri>file://{caminho_sdf}</uri>
        <name>cabo_dinamico</name>
        <pose>{ancora_x} {ancora_y} {ancora_z} 0 0 {yaw_base}</pose>
        <static>false</static>
      </include>

      <include>
        <uri>file://{caminho_drone_sdf}</uri>
        <name>meu_drone</name>
        <pose>{drone_spawn_x} {drone_spawn_y} {drone_spawn_z} 0 0 {yaw_base}</pose>
      </include>

      <joint name="ancora_carretel_cabo" type="fixed">
        <parent>ancora_cabo</parent>
        <child>cabo_dinamico::raiz_cabo</child>
      </joint>

      <joint name="cabo_drone_joint" type="fixed">
        <parent>cabo_dinamico::ponta_cabo</parent>
        <child>meu_drone::cabo_sensor_link</child>
      </joint>
    </model>
  </world>
</sdf>
'''

with open(caminho_world, 'w') as f:
    f.write(world)

print('my_world.sdf atualizado.')
