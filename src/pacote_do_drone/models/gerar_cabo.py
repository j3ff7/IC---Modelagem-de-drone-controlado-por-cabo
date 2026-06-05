import json
import subprocess
import os
import math

# ============================================================
# CAMINHOS
# ============================================================
caminho_json   = '/home/joseubu/IC/src/pacote_do_drone/tether_parameters.json'
pasta_models   = '/home/joseubu/IC/src/pacote_do_drone/models/'
pasta_worlds   = '/home/joseubu/IC/src/pacote_do_drone/worlds/'
caminho_urdf   = os.path.join(pasta_models, 'cabo.urdf')
caminho_sdf    = os.path.join(pasta_models, 'cabo.sdf')
caminho_world  = os.path.join(pasta_worlds, 'my_world.sdf')

os.makedirs(pasta_models, exist_ok=True)
os.makedirs(pasta_worlds, exist_ok=True)

# ============================================================
# LER PARÂMETROS
# ============================================================
with open(caminho_json, 'r') as f:
    params = json.load(f)

num_links = params["num_links"]
length    = params["length"]
radius    = params["radius"]
mass      = params["mass"]
drone_x   = params["drone_x"]
drone_y   = params["drone_y"]
drone_z   = params["drone_z"]

comprimento_total = num_links * length
ancora_z = 0.33
ancora_x = 0
ancora_y = 0.18

# Distância e direção agora são relativas à âncora
dist_2d = math.hypot(drone_x - ancora_x, drone_y - ancora_y)
dist_3d = math.hypot(dist_2d, drone_z - ancora_z) 

direcao_drone = math.atan2(drone_y - ancora_y, drone_x - ancora_x)
pitch_base = math.atan2(drone_z - ancora_z, dist_2d)

# ============================================================
# BUSCA BINÁRIA PARA CURVATURA (CATENÁRIA DISCRETA)
# ============================================================
target = dist_3d / comprimento_total
alpha = 0.0
delta_theta = 0.0

ALPHA_MAX = math.pi * 0.90  # limite seguro — sem loops

if target >= 1.0:
    alpha = 0.0
    delta_theta = 0.0
    print("Cabo esticado — sem catenária.")
elif target >= 2 / math.pi:
    low, high = 0.00001, math.pi
    for _ in range(50):
        mid = (low + high) / 2.0
        # CORREÇÃO: Usando a fórmula exata para links discretos
        razao_discreta = math.sin(mid) / (num_links * math.sin(mid / num_links))
        
        if razao_discreta > target:
            low = mid
        else:
            high = mid
            
    alpha = 2.0 * low
    delta_theta = alpha / num_links
    print(f"Catenária discreta calculada — alpha={math.degrees(alpha):.1f}°")
else:
    alpha = ALPHA_MAX
    delta_theta = alpha / num_links
    print(f"⚠️  Cabo muito frouxo — alpha clampado em {math.degrees(ALPHA_MAX):.1f}°")

yaw_base = direcao_drone

# ============================================================
# INÉRCIAS
# ============================================================
ixx_elo    = (1/2) * mass * radius**2
iyy_zz_elo = (1/12) * mass * (3 * radius**2 + length**2)
ixx_raiz   = (2/5) * 0.02 * (0.01**2)

# ============================================================
# CINEMÁTICA DIRETA
# ============================================================
# CORREÇÃO: + (delta_theta / 2.0) alinha a curvatura perfeitamente com a direção do drone
running_angle = pitch_base - (alpha / 2.0) + (delta_theta / 2.0)
x_tip_local = 0.0
z_tip_local = 0.0

for i in range(num_links): 
    x_tip_local += length * math.cos(running_angle)
    z_tip_local += length * math.sin(running_angle)
    running_angle += delta_theta

# Mapeamento 3D correto do Gazebo (mantendo sua lógica original de projeção)
drone_spawn_x = ancora_x + (x_tip_local * math.cos(yaw_base) + z_tip_local * math.sin(yaw_base))
drone_spawn_y = ancora_y + (x_tip_local * math.sin(yaw_base) - z_tip_local * math.cos(yaw_base))
drone_spawn_z = drone_z

print(f"  Origem do final_segment (mundo): ({drone_spawn_x:.4f}, {drone_spawn_y:.4f}, {drone_spawn_z:.4f})")
print(f"  Alvo do drone (JSON):            ({drone_x:.4f}, {drone_y:.4f}, {drone_z:.4f})")

# ============================================================
# GERAR CABO.SDF 
# ============================================================
raio_esfera = radius * 6.0  
ixx_esfera = (2/5) * mass * (raio_esfera**2)

sdf = f"""<?xml version="1.0" ?>
<sdf version="1.8">
  <model name="cabo_flexivel">
    <link name="raiz_cabo">
      <pose>0 0 0 0 0 0</pose>
      <inertial>
        <mass>0.02</mass>
        <inertia>
          <ixx>{ixx_raiz}</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>{ixx_raiz}</iyy><iyz>0</iyz><izz>{ixx_raiz}</izz>
        </inertia>
      </inertial>
      <collision name="col_raiz">
        <geometry><sphere><radius>0.01</radius></sphere></geometry>
      </collision>
    </link>
"""

# Inicializamos com a mesma correção de fase angular
running_angle = pitch_base - (alpha / 2.0) + (delta_theta / 2.0)
x_tip_local = 0.0
z_tip_local = 0.0

parent_link = "raiz_cabo"
for i in range(1, num_links + 1):
    nome_elo = "final_segment" if i == num_links else f"segment_{i}"
    pose_pitch = -running_angle
    
    sensor_xml = ""
    if i == 1:
        sensor_xml = """
        <sensor name="sensor_tensao_carretel" type="force_torque">
          <always_on>true</always_on>
          <update_rate>50</update_rate>
          <topic>/cabo/tensao_carretel</topic>
        </sensor>"""
    elif i == num_links:
        sensor_xml = """
        <sensor name="sensor_tensao_drone" type="force_torque">
          <always_on>true</always_on>
          <update_rate>50</update_rate>
          <topic>/cabo/tensao_drone</topic>
        </sensor>"""

    if i < num_links:
        geom_xml = f"<cylinder><radius>{radius}</radius><length>{length}</length></cylinder>"
        pose_vis_col = f"<pose>{length/2} 0 0 0 1.5708 0</pose>"
        cor_material = "<ambient>0 0 0 1</ambient><diffuse>0 0 0 1</diffuse>"
        ixx_str, iyy_str, izz_str = ixx_elo, iyy_zz_elo, iyy_zz_elo
        
        joint_xml = f"""
    <joint name="joint_{i}" type="universal">
      <parent>{parent_link}</parent>
      <child>{nome_elo}</child>
      <pose>0 0 0 0 0 0</pose>
      <axis>
        <xyz>0 1 0</xyz>
        <limit><lower>-2.0</lower><upper>2.0</upper></limit>
        <dynamics><damping>0.002</damping><friction>0.0</friction></dynamics>
      </axis>
      <axis2>
        <xyz>0 0 1</xyz>
        <limit><lower>-2.0</lower><upper>2.0</upper></limit>
        <dynamics><damping>0.002</damping><friction>0.0</friction></dynamics>
      </axis2>{sensor_xml}
    </joint>"""
    else:
        geom_xml = f"<sphere><radius>{raio_esfera}</radius></sphere>"
        pose_vis_col = f"<pose>{length/2} 0 0 0 0 0</pose>"
        cor_material = "<ambient>0.8 0.1 0.1 1</ambient><diffuse>0.8 0.1 0.1 1</diffuse>"
        ixx_str, iyy_str, izz_str = ixx_esfera, ixx_esfera, ixx_esfera
        
        joint_xml = f"""
    <joint name="joint_{i}" type="ball">
      <parent>{parent_link}</parent>
      <child>{nome_elo}</child>
      <pose>0 0 0 0 0 0</pose>{sensor_xml}
    </joint>"""

    sdf += f"""
    <link name="{nome_elo}">
      <pose>{x_tip_local:.6f} 0 {z_tip_local:.6f} 0 {pose_pitch:.6f} 0</pose>
      <visual name="visual">
        {pose_vis_col}
        <geometry>{geom_xml}</geometry>
        <material>{cor_material}</material>
      </visual>
      <collision name="collision">
        {pose_vis_col}
        <geometry>{geom_xml}</geometry>
      </collision>
      <inertial>
        <pose>{length/2} 0 0 0 0 0</pose>
        <mass>{mass}</mass>
        <inertia>
          <ixx>{ixx_str}</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>{iyy_str}</iyy><iyz>0</iyz><izz>{izz_str}</izz>
        </inertia>
      </inertial>
    </link>
    {joint_xml}
"""
    x_tip_local += length * math.cos(running_angle)
    z_tip_local += length * math.sin(running_angle)
    running_angle += delta_theta
    parent_link = nome_elo

sdf += f"""
    <link name="ponta_cabo">
      <pose>{x_tip_local:.6f} 0 {z_tip_local:.6f} 0 0 0</pose>
      <inertial>
        <mass>0.001</mass>
        <inertia>
          <ixx>1e-6</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>1e-6</iyy><iyz>0</iyz><izz>1e-6</izz>
        </inertia>
      </inertial>
    </link>
    <joint name="joint_ponta" type="fixed">
      <parent>final_segment</parent>
      <child>ponta_cabo</child>
      <pose>0 0 0 0 0 0</pose>
    </joint>
  </model>
</sdf>
"""

with open(caminho_sdf, "w") as f:
    f.write(sdf)

print(f"✓ cabo.sdf gerado direto com sucesso!")

# ============================================================
# GERAR my_world.sdf
# ============================================================
world = f"""<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="mundo_ic">

    <gravity>0 0 -9.81</gravity>

    <physics name="1ms" type="ignored">
      <max_step_size>0.0004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system"           name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system"     name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-forcetorque-system"       name="gz::sim::systems::ForceTorque"/>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material><ambient>0.8 0.8 0.8 1</ambient><diffuse>0.8 0.8 0.8 1</diffuse></material>
        </visual>
      </link>
    </model>

    <include>
      <uri>file:///home/joseubu/IC/src/pacote_do_drone/models/carretel/carretel.sdf</uri>
      <name>meu_carretel</name>
      <pose>0 0 0 0 0 0</pose> 
    </include>

    <include>
      <uri>file:///home/joseubu/IC/src/pacote_do_drone/models/cabo.sdf</uri>
      <name>cabo_dinamico</name>
      <pose>{ancora_x} {ancora_y} {ancora_z} 1.5708 0 {yaw_base}</pose>
      <static>false</static>
    </include>

    <joint name="ancora_carretel_cabo" type="ball">
      <parent>meu_carretel::cilindro_carretel</parent>
      <child>cabo_dinamico::raiz_cabo</child>
      <pose>0 0 0 0 0 0</pose>
    </joint>

    <include>
      <uri>file:///home/joseubu/IC/src/pacote_do_drone/models/meu_drone/meu_drone.sdf</uri>
      <name>meu_drone</name>
      <pose>{drone_spawn_x} {drone_spawn_y} {drone_spawn_z} 0 0 {yaw_base}</pose>
    </include>

    <joint name="cabo_drone_joint" type="ball">
      <parent>cabo_dinamico::ponta_cabo</parent>
      <child>meu_drone::base_link</child>
    </joint>

  </world>
</sdf>
"""

with open(caminho_world, "w") as f:
    f.write(world)

print(f"✓ Sucesso! Drone spawnado na posição exata requerida: ({drone_x:.4f}, {drone_y:.4f}, {drone_z:.4f})")