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

comprimento_total = num_links * length
ancora_z = 0.03

dist_xy = math.hypot(drone_x, drone_y)
direcao_drone = math.atan2(drone_y, drone_x)

# ============================================================
# BUSCA BINÁRIA PARA CURVATURA (CATENÁRIA)
# ============================================================
target = dist_xy / comprimento_total
alpha = 0.0
delta_theta = 0.0

ALPHA_MAX = math.pi * 0.90  # limite seguro — sem loops

if target >= 1.0:
    # Cabo esticado — reto, sem curva
    alpha = 0.0
    delta_theta = 0.0
    print("Cabo esticado — sem catenária.")

elif target >= 2 / math.pi:
    # Zona normal — catenária válida
    low, high = 0.00001, math.pi
    for _ in range(50):
        mid = (low + high) / 2.0
        if (math.sin(mid) / mid) > target:
            low = mid
        else:
            high = mid
    alpha = 2.0 * low
    delta_theta = alpha / num_links
    print(f"Catenária normal — alpha={math.degrees(alpha):.1f}°")

else:
    # Cabo muito frouxo — clamp para evitar loop
    alpha = ALPHA_MAX
    delta_theta = alpha / num_links
    print(f"⚠️  Cabo muito frouxo — alpha clampado em {math.degrees(ALPHA_MAX):.1f}°")
    print(f"   Curva aproximada. Reduza num_links ou aumente drone_x/drone_y para catenária exata.")

yaw_base = direcao_drone

# ============================================================
# INÉRCIAS
# ============================================================
ixx_elo    = (1/2) * mass * radius**2
iyy_zz_elo = (1/12) * mass * (3 * radius**2 + length**2)
ixx_raiz   = (2/5) * 0.02 * (0.01**2)

# ============================================================
# CINEMÁTICA DIRETA — origem do final_segment no frame local do cabo
# Itera num_links-1 vezes: para na ORIGEM do final_segment,
# que é onde a junta cabo_drone_joint se ancora
# ============================================================
running_angle = -(alpha / 2.0)
x_tip_local = 0.0
z_tip_local = 0.0

for i in range(1, num_links):  # num_links - 1 iterações
    x_tip_local += length * math.cos(running_angle)
    z_tip_local += length * math.sin(running_angle)
    running_angle += delta_theta

# Transformar do frame local do cabo para o frame do mundo
# Pose do cabo no mundo: roll=pi/2, pitch=0, yaw=yaw_base
# Rx(pi/2): (x, 0, z) -> (x, -z, 0)
# Rz(yaw):  rotaciona no plano XY
drone_spawn_x = x_tip_local * math.cos(yaw_base) + z_tip_local * math.sin(yaw_base)
drone_spawn_y = x_tip_local * math.sin(yaw_base) - z_tip_local * math.cos(yaw_base)
drone_spawn_z = ancora_z

print(f"  Origem do final_segment (mundo): ({drone_spawn_x:.4f}, {drone_spawn_y:.4f}, {drone_spawn_z:.4f})")
print(f"  Alvo do drone (JSON):            ({drone_x:.4f}, {drone_y:.4f})")

# ============================================================
# GERAR CABO.URDF
# ============================================================
urdf = f"""<?xml version="1.0"?>
<robot name="cabo_flexivel">
  <link name="raiz_cabo">
    <inertial>
      <mass value="0.02"/>
      <inertia ixx="{ixx_raiz}" ixy="0" ixz="0" iyy="{ixx_raiz}" iyz="0" izz="{ixx_raiz}"/>
    </inertial>
    <collision>
      <geometry><sphere><radius>0.01</radius></sphere></geometry>
    </collision>
  </link>
"""

parent_link = "raiz_cabo"
for i in range(1, num_links + 1):
    nome_elo = "final_segment" if i == num_links else f"segment_{i}"

    if i == 1:
        origem_x    = 0.0
        pitch_junta = -(alpha / 2.0)
    else:
        origem_x    = length
        pitch_junta = delta_theta

    urdf += f"""
  <link name="{nome_elo}">
    <visual>
      <geometry><cylinder radius="{radius}" length="{length}"/></geometry>
      <origin xyz="{length/2} 0 0" rpy="0 1.5708 0"/>
      <material name="black"><color rgba="0 0 0 1"/></material>
    </visual>
    <collision>
      <geometry><cylinder radius="{radius}" length="{length * 0.85}"/></geometry>
      <origin xyz="{length/2} 0 0" rpy="0 1.5708 0"/>
    </collision>
    <inertial>
      <mass value="{mass}"/>
      <origin xyz="{length/2} 0 0" rpy="0 1.5708 0"/>
      <inertia ixx="{ixx_elo}" ixy="0" ixz="0" iyy="{iyy_zz_elo}" iyz="0" izz="{iyy_zz_elo}"/>
    </inertial>
  </link>

  <joint name="joint_{i}" type="revolute">
    <parent link="{parent_link}"/>
    <child link="{nome_elo}"/>
    <origin xyz="{origem_x} 0 0" rpy="0 {pitch_junta} 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.0" upper="2.0" effort="5" velocity="10"/>
    <dynamics damping="0.002" friction="0.0"/>
  </joint>
"""
    parent_link = nome_elo

urdf += """</robot>"""

with open(caminho_urdf, "w") as f:
    f.write(urdf)

subprocess.run(['gz', 'sdf', '-p', caminho_urdf],
               stdout=open(caminho_sdf, "w"), check=True)

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

    <model name="ancora_chao">
      <static>true</static>
      <pose>0 0 {ancora_z} 0 0 0</pose>
      <link name="ancora_link">
        <visual name="vis">
          <geometry><sphere><radius>0.02</radius></sphere></geometry>
          <material><ambient>1 0 0 1</ambient></material>
        </visual>
      </link>
    </model>

    <include>
      <uri>file:///home/joseubu/IC/src/pacote_do_drone/models/cabo.sdf</uri>
      <name>cabo_dinamico</name>
      <pose>0 0 {ancora_z} 1.5708 0 {yaw_base}</pose>
      <static>false</static>
    </include>

    <joint name="ancora_chao_cabo" type="ball">
      <parent>world</parent>
      <child>cabo_dinamico::raiz_cabo</child>
    </joint>

    <include>
      <uri>file:///home/joseubu/IC/src/pacote_do_drone/models/meu_drone/meu_drone.sdf</uri>
      <name>meu_drone</name>
      <pose>{drone_spawn_x} {drone_spawn_y} {drone_spawn_z} 0 0 {yaw_base}</pose>
    </include>

    <!-- Drone spawnado exatamente na origem do final_segment:
         a junta não precisa mover nada, rotores ficam no lugar certo -->
    <joint name="cabo_drone_joint" type="ball">
      <parent>cabo_dinamico::final_segment</parent>
      <child>meu_drone::base_link</child>
    </joint>

  </world>
</sdf>
"""

with open(caminho_world, "w") as f:
    f.write(world)

print(f"✓ Sucesso! Drone spawnado em ({drone_spawn_x:.4f}, {drone_spawn_y:.4f}, {drone_spawn_z:.4f})")