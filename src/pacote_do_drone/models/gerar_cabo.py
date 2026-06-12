import json
import math
import os

# ============================================================
# CAMINHOS E PASTAS
# ============================================================
caminho_json   = '/home/joseubu/IC/src/pacote_do_drone/tether_parameters.json'
pasta_models   = '/home/joseubu/IC/src/pacote_do_drone/models/'
pasta_worlds   = '/home/joseubu/IC/src/pacote_do_drone/worlds/'
caminho_sdf    = os.path.join(pasta_models, 'cabo.sdf')
caminho_world  = os.path.join(pasta_worlds, 'my_world.sdf')

os.makedirs(pasta_models, exist_ok=True)
os.makedirs(pasta_worlds, exist_ok=True)

# ============================================================
# LER PARÂMETROS
# ============================================================
try:
    with open(caminho_json, 'r') as f:
        params = json.load(f)
except FileNotFoundError:
    print("Aviso: JSON não encontrado. Usando valores padrão de teste.")
    params = {
        "num_links": 250, "length": 0.01, "radius": 0.003, "mass": 0.002,
        "drone_x": 0.0, "drone_y": 0.0, "drone_z": 0.0,
        "voltas_desejadas": 3.75 
    }

num_links        = params["num_links"]
length           = params["length"]
radius           = params["radius"]
densidade_linear = 0.04  # kg por metro de cabo
mass = densidade_linear * length  # A massa diminui se o elo diminuir
voltas_input     = params.get("voltas_desejadas", 3.75)

# ============================================================
# PARÂMETROS DO CARRETEL (Geometria Fixa do seu SDF)
# ============================================================
spool_x = 0.0
spool_y_center = 0.0
spool_z = 0.26
spool_radius = 0.07

R_min = spool_radius + radius
R_eff = math.sqrt(R_min**2 + (length / 2)**2) + 0.002

start_y = 0.18 
passo_helice_y = radius * 2.2 
comprimento_total = num_links * length

Z_MINIMO_CHAO = 0.05  

# ============================================================
# AJUSTE E VALIDAÇÃO DAS VOLTAS (CORREÇÃO DA FOLGA EM METROS)
# ============================================================
base_voltas = math.floor(voltas_input)
voltas = base_voltas + 0.75
if voltas > voltas_input + 0.25:
    voltas -= 1.0
if voltas < 0.75: 
    voltas = 0.75

comprimento_uma_volta = math.sqrt((2 * math.pi * R_eff)**2 + passo_helice_y**2)

# MODIFICAÇÃO CRÍTICA: Trava de segurança definida em METROS (35 cm)
# Garante que o drone fique fora do carretel mesmo com elos muito pequenos
DIST_MIN_CONEXAO_METROS = 0.35  
min_elos_conexao = math.ceil(DIST_MIN_CONEXAO_METROS / length)
comprimento_max_carretel = comprimento_total - (min_elos_conexao * length)

# Reduz as voltas se o comprimento total do cabo não conseguir garantir os 35cm externos
while (voltas * comprimento_uma_volta) + length > comprimento_max_carretel and voltas > 0.75:
    voltas -= 1.0

theta_total = voltas * 2 * math.pi
print(f"Configuração definida: {voltas} voltas aplicadas no carretel.")

# ============================================================
# GERADOR DE PONTOS (VERTICAL + HÉLICE + EXTENSÃO PROTEGIDA)
# ============================================================
pontos_cabo = []

p0 = (spool_x + R_eff, start_y, spool_z - length)
p1 = (spool_x + R_eff, start_y, spool_z)
pontos_cabo.append(p0)
pontos_cabo.append(p1)

p_anterior = p1
theta_atual = 0.0

# MODIFICAÇÃO CRÍTICA: O passo do ângulo encolhe proporcionalmente ao elo
# Isso previne erros numéricos de amostragem em resoluções altas
passo_theta = min(0.001, length / (10 * R_eff))

print("1. Enrolando o cabo no carretel (Hélice)...")
while theta_atual <= theta_total and len(pontos_cabo) < num_links:
    theta_atual += passo_theta
    
    hx = spool_x + R_eff * math.cos(theta_atual)
    hy = start_y - (theta_atual / (2 * math.pi)) * passo_helice_y
    hz = spool_z + R_eff * math.sin(theta_atual)
    
    dist = math.hypot(hx - p_anterior[0], hy - p_anterior[1], hz - p_anterior[2])
    
    if dist >= length:
        razao = length / dist
        p_cravado = (
            p_anterior[0] + (hx - p_anterior[0]) * razao,
            p_anterior[1] + (hy - p_anterior[1]) * razao,
            p_anterior[2] + (hz - p_anterior[2]) * razao
        )
        pontos_cabo.append(p_cravado)
        p_anterior = p_cravado

if len(pontos_cabo) >= 2:
    p_ante_saida = pontos_cabo[-2]
    dir_x = p_anterior[0] - p_ante_saida[0]
    dir_y = p_anterior[1] - p_ante_saida[1]
    dir_z = p_anterior[2] - p_ante_saida[2]
else:
    dir_x, dir_y, dir_z = 1.0, 0.0, 0.0

norm = math.hypot(dir_x, dir_y, dir_z) or 1.0
vx, vy, vz = dir_x / norm, dir_y / norm, dir_z / norm

elos_restantes = num_links - len(pontos_cabo)
print(f"2. Estendendo {elos_restantes} elos restantes com trava de segurança contra o chão...")

while len(pontos_cabo) < num_links:
    proximo_z = p_anterior[2] + vz * length
    
    if proximo_z < Z_MINIMO_CHAO:
        vz_adaptado = 0.0
        norm_horizontal = math.hypot(vx, vy) or 1.0
        vx_adaptado = vx / norm_horizontal
        vy_adaptado = vy / norm_horizontal
        
        p_ext = (
            p_anterior[0] + vx_adaptado * length,
            p_anterior[1] + vy_adaptado * length,
            max(Z_MINIMO_CHAO, p_anterior[2])
        )
    else:
        p_ext = (
            p_anterior[0] + vx * length,
            p_anterior[1] + vy * length,
            proximo_z
        )
        
    pontos_cabo.append(p_ext)
    p_anterior = p_ext

p_final = pontos_cabo[-1]
print(f"   ✓ Drone configurado para nascer no chão seguro: ({p_final[0]:.3f}, {p_final[1]:.3f}, {p_final[2]:.3f})")

# ============================================================
# GERAR CABO.SDF
# ============================================================
ixx_elo    = (1/2) * mass * radius**2
iyy_zz_elo = (1/12) * mass * (3 * radius**2 + length**2)
ixx_raiz   = (2/5) * 0.02 * (0.01**2)

origem_modelo = pontos_cabo[0] 

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
    
    p_atual = pontos_cabo[i-1]
    p_prox = pontos_cabo[i] if i < num_links else pontos_cabo[-1]
    
    dx = p_prox[0] - p_atual[0]
    dy = p_prox[1] - p_atual[1]
    dz = p_prox[2] - p_atual[2]
    
    elo_yaw = math.atan2(dy, dx)
    elo_pitch = -math.atan2(dz, math.hypot(dx, dy))
    
    pose_x = p_atual[0] - origem_modelo[0]
    pose_y = p_atual[1] - origem_modelo[1]
    pose_z = p_atual[2] - origem_modelo[2]

    if i < num_links:
        geom_xml = f"<cylinder><radius>{radius}</radius><length>{length}</length></cylinder>"
        pose_vis_col = f"<pose>{length/2} 0 0 0 1.5708 0</pose>"
        cor_material = "<ambient>0 0 0 1</ambient><diffuse>0 0 0 1</diffuse>"
        
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
      </axis2>
    </joint>"""
    else:
        raio_esfera = radius * 4.0
        geom_xml = f"<sphere><radius>{raio_esfera}</radius></sphere>"
        pose_vis_col = f"<pose>{length/2} 0 0 0 0 0</pose>"
        cor_material = "<ambient>0.8 0.1 0.1 1</ambient><diffuse>0.8 0.1 0.1 1</diffuse>"
        
        joint_xml = f"""
    <joint name="joint_{i}" type="ball">
      <parent>{parent_link}</parent>
      <child>{nome_elo}</child>
      <pose>0 0 0 0 0 0</pose>
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
          <ixx>{ixx_elo}</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>{iyy_zz_elo}</iyy><iyz>0</iyz><izz>{iyy_zz_elo}</izz>
        </inertia>
      </inertial>
    </link>
    {joint_xml}
"""
    parent_link = nome_elo

p_final = pontos_cabo[-1]
ponta_x = p_final[0] - origem_modelo[0]
ponta_y = p_final[1] - origem_modelo[1]
ponta_z = p_final[2] - origem_modelo[2]

sdf += f"""
    <link name="ponta_cabo">
      <pose>{ponta_x:.6f} {ponta_y:.6f} {ponta_z:.6f} 0 0 0</pose>
      <inertial>
        <mass>0.001</mass>
        <inertia><ixx>1e-6</ixx><ixy>0</ixy><ixz>0</ixz><iyy>1e-6</iyy><iyz>0</iyz><izz>1e-6</izz></inertia>
      </inertial>
    </link>
    <joint name="joint_ponta" type="fixed">
      <parent>final_segment</parent>
      <child>ponta_cabo</child>
    </joint>
  </model>
</sdf>
"""

with open(caminho_sdf, "w") as f:
    f.write(sdf)
print("✓ cabo.sdf gerado com sucesso!")

# ============================================================
# GERAR my_world.sdf
# ============================================================
spawn_cabo_x = origem_modelo[0]
spawn_cabo_y = origem_modelo[1]
spawn_cabo_z = origem_modelo[2]

drone_spawn_x = p_final[0]
drone_spawn_y = p_final[1]
drone_spawn_z = p_final[2]

yaw_base = math.atan2(spool_y_center - drone_spawn_y, spool_x - drone_spawn_x)

world = f"""<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="mundo_ic">

    <gravity>0 0 -9.81</gravity>

    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
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
      <uri>file://{pasta_models}carretel/carretel.sdf</uri>
      <name>meu_carretel</name>
      <pose>0 0 0 0 0 0</pose> 
    </include>

    <include>
      <uri>file://{caminho_sdf}</uri>
      <name>cabo_dinamico</name>
      <pose>{spawn_cabo_x:.6f} {spawn_cabo_y:.6f} {spawn_cabo_z:.6f} 0 0 0</pose>
    </include>

    <joint name="ancora_carretel_cabo" type="fixed">
      <parent>meu_carretel::cilindro_carretel</parent>
      <child>cabo_dinamico::raiz_cabo</child>
    </joint>

    <include>
      <uri>file://{pasta_models}meu_drone/meu_drone.sdf</uri>
      <name>meu_drone</name>
      <pose>{drone_spawn_x:.6f} {drone_spawn_y:.6f} {drone_spawn_z:.6f} 0 0 {yaw_base:.6f}</pose>
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

print(f"✓ my_world.sdf gerado com caminhos e folgas adaptativas completas!")