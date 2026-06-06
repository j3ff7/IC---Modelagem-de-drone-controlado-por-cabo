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
ancora_x = 0.0
ancora_y = 0.18

# Distância 3D total
dist_2d = math.hypot(drone_x - ancora_x, drone_y - ancora_y)
dist_3d = math.hypot(dist_2d, drone_z - ancora_z) 
yaw_base = math.atan2(drone_y - ancora_y, drone_x - ancora_x)

# ============================================================
# CURVA DE BÉZIER 3D COM TAMANHO EXATO (ANTI-TRAVAMENTO)
# ============================================================
def calcular_bezier(p0, p1, p2, p3, t):
    u = 1.0 - t
    tt = t * t
    uu = u * u
    uuu = uu * u
    ttt = tt * t
    px = uuu * p0[0] + 3 * uu * t * p1[0] + 3 * u * tt * p2[0] + ttt * p3[0]
    py = uuu * p0[1] + 3 * uu * t * p1[1] + 3 * u * tt * p2[1] + ttt * p3[1]
    pz = uuu * p0[2] + 3 * uu * t * p1[2] + 3 * u * tt * p2[2] + ttt * p3[2]
    return (px, py, pz)

P0 = (ancora_x, ancora_y, ancora_z)
P3 = (drone_x, drone_y, drone_z)

dx_full = drone_x - ancora_x
dy_full = drone_y - ancora_y
dz_full = drone_z - ancora_z

print("Calculando curvatura ideal para evitar sobreposições...")

# 1. Busca Binária para encontrar a barriga (sag) que deixa a curva com o tamanho EXATO do cabo
low_sag = 0.0
high_sag = comprimento_total * 2.0
best_sag = 0.0

for _ in range(30): # 30 iterações garantem precisão milimétrica
    sag_z = (low_sag + high_sag) / 2.0
    
    p1_z = max(radius + 0.05, ancora_z - sag_z)
    p2_z = max(radius + 0.05, drone_z - sag_z)
    
    P1_teste = (ancora_x + dx_full * 0.333, ancora_y + dy_full * 0.333, p1_z)
    P2_teste = (ancora_x + dx_full * 0.666, ancora_y + dy_full * 0.666, p2_z)
    
    # Medir tamanho da curva gerada
    arc_len = 0.0
    prev_p = P0
    for step in range(1, 51):
        t = step / 50.0
        p_t = calcular_bezier(P0, P1_teste, P2_teste, P3, t)
        arc_len += math.hypot(p_t[0]-prev_p[0], p_t[1]-prev_p[1], p_t[2]-prev_p[2])
        prev_p = p_t
        
    if arc_len < comprimento_total:
        low_sag = sag_z # Curva está curta, precisa aprofundar a barriga
    else:
        high_sag = sag_z # Curva está longa, precisa achatar a barriga

# Configura os pontos finais de controle com a barriga perfeita encontrada
sag_z = (low_sag + high_sag) / 2.0
P1 = (ancora_x + dx_full * 0.333, ancora_y + dy_full * 0.333, max(radius + 0.05, ancora_z - sag_z))
P2 = (ancora_x + dx_full * 0.666, ancora_y + dy_full * 0.666, max(radius + 0.05, drone_z - sag_z))

# 2. Caminhar pela curva cravando os elos com a distância exata 'length'
pontos_cabo = [P0]
t_atual = 0.0
passo_t = 0.0001

for i in range(num_links):
    p_anterior = pontos_cabo[-1]
    achou_ponto = False
    
    while t_atual <= 1.0:
        t_atual += passo_t
        if t_atual > 1.0: 
            break
            
        p_teste = calcular_bezier(P0, P1, P2, P3, t_atual)
        dist = math.hypot(p_teste[0]-p_anterior[0], p_teste[1]-p_anterior[1], p_teste[2]-p_anterior[2])
        
        if dist >= length:
            pontos_cabo.append(p_teste)
            achou_ponto = True
            break
            
    # Trava antibug extrema: Se a matemática falhar na margem de erro e faltar curva, 
    # projeta o elo no ar na mesma direção do elo passado. NUNCA sobrepõe coordenadas!
    if not achou_ponto:
        if len(pontos_cabo) >= 2:
            p_ante = pontos_cabo[-2]
            dir_x = p_anterior[0] - p_ante[0]
            dir_y = p_anterior[1] - p_ante[1]
            dir_z = p_anterior[2] - p_ante[2]
        else:
            dir_x, dir_y, dir_z = dx_full, dy_full, dz_full
            
        norm = math.hypot(dir_x, dir_y, dir_z)
        if norm == 0: norm = 1.0
        
        p_ext = (
            p_anterior[0] + (dir_x/norm) * length,
            p_anterior[1] + (dir_y/norm) * length,
            p_anterior[2] + (dir_z/norm) * length
        )
        pontos_cabo.append(p_ext)

# ============================================================
# INÉRCIAS
# ============================================================
ixx_elo    = (1/2) * mass * radius**2
iyy_zz_elo = (1/12) * mass * (3 * radius**2 + length**2)
ixx_raiz   = (2/5) * 0.02 * (0.01**2)

raio_esfera = radius * 6.0  
ixx_esfera = (2/5) * mass * (raio_esfera**2)

# ============================================================
# GERAR CABO.SDF 
# ============================================================
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

parent_link = "raiz_cabo"

for i in range(1, num_links + 1):
    nome_elo = "final_segment" if i == num_links else f"segment_{i}"
    
    # Pegar os pontos do elo atual
    p_atual = pontos_cabo[i-1]
    # Se faltou ponto (cabo curto demais na matemática), usa o último conhecido
    p_prox = pontos_cabo[i] if i < len(pontos_cabo) else pontos_cabo[-1]
    
    # Calcular rotação 3D do cilindro para ele apontar para p_prox
    dx = p_prox[0] - p_atual[0]
    dy = p_prox[1] - p_atual[1]
    dz = p_prox[2] - p_atual[2]
    
    elo_yaw = math.atan2(dy, dx)
    elo_pitch = -math.atan2(dz, math.hypot(dx, dy))
    
    # As coordenadas X e Y do modelo principal agora começam de (0,0,0) global
    # Então subtraímos a âncora para manter o modelo relativo à raiz
    pose_x = p_atual[0] - ancora_x
    pose_y = p_atual[1] - ancora_y
    pose_z = p_atual[2] - ancora_z

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
      <pose>{pose_x:.6f} {pose_y:.6f} {pose_z:.6f} 0 {elo_pitch:.6f} {elo_yaw:.6f}</pose>
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
    parent_link = nome_elo

# Última ponta de fixação para o drone
p_final = pontos_cabo[-1]
ponta_x = p_final[0] - ancora_x
ponta_y = p_final[1] - ancora_y
ponta_z = p_final[2] - ancora_z

sdf += f"""
    <link name="ponta_cabo">
      <pose>{ponta_x:.6f} {ponta_y:.6f} {ponta_z:.6f} 0 0 0</pose>
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

print(f"✓ cabo.sdf gerado direto com sucesso usando Bézier!")

# ============================================================
# GERAR my_world.sdf
# ============================================================
# Note que a pose do cabo_dinamico agora não recebe mais Yaw extra, 
# pois a Bézier já posicionou os elos no espaço 3D global absoluto.
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
      <pose>{ancora_x} {ancora_y} {ancora_z} 0 0 0</pose>
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
      <pose>{drone_x} {drone_y} {drone_z} 0 0 {yaw_base}</pose>
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