import json
import os
import math
import sys

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
# FUNÇÕES AUXILIARES
# ============================================================

def clamp_min(valor, minimo):
    return max(float(valor), minimo)

def dist3(p, q):
    return math.sqrt(
        (p[0] - q[0]) ** 2 +
        (p[1] - q[1]) ** 2 +
        (p[2] - q[2]) ** 2
    )

def norm3(v):
    n = math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)
    if n < 1e-12:
        return (1.0, 0.0, 0.0)
    return (v[0] / n, v[1] / n, v[2] / n)

def yaw_pitch_do_segmento(p0, p1):
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    yaw = math.atan2(dy, dx)
    pitch = -math.atan2(dz, math.hypot(dx, dy))
    return yaw, pitch

def rotacionar_offset_yaw(x, y, yaw):
    c = math.cos(yaw)
    s = math.sin(yaw)
    return (
        c * x - s * y,
        s * x + c * y
    )

# ============================================================
# LER PARÂMETROS
# ============================================================

with open(caminho_json, 'r') as f:
    params = json.load(f)

num_links = max(3, int(params.get("num_links", 50)))
length = clamp_min(params.get("length", 0.05), 0.01)
radius = clamp_min(params.get("radius", 0.003), 0.001)
densidade_linear = float(params.get("densidade_linear", 0.04))

cabo_fim_x = float(params.get("cabo_fim_x", 1.0))

# ============================================================
# CONFIGURAÇÕES FÍSICAS DO CABO E CARRETEL
# ============================================================

comprimento_total = num_links * length

ancora_x = 0.0
ancora_y = 0.18
ancora_z = 0.26 

offset_conexao_drone_x = float(params.get("offset_conexao_drone_x", 0.0))
offset_conexao_drone_y = float(params.get("offset_conexao_drone_y", 0.0))
offset_conexao_drone_z = float(params.get("offset_conexao_drone_z", -0.01))

damping_junta = float(params.get("damping_junta", 0.1))
friction_junta = float(params.get("friction_junta", 0.05))
limite_junta_deg = float(params.get("limite_junta", 60.0))   # corrigido: chave do JSON é "limite_junta"

raio_carretel = float(params.get("raio_carretel", 0.07)) 
theta = math.radians(float(params.get("theta_enrolamento", 1.0)))  # padrão alterado para 1° conforme JSON

# ============================================================
# 1. CÁLCULO DAS VOLTAS E GERAÇÃO DA PARTE ENROLADA
# ============================================================

num_voltas_enroladas = float(params.get("num_voltas_enroladas", 1.0))

# Margem de segurança para evitar colisão com o carretel
seg_col_radius = 0.50 * radius
safety_margin = 0.002
min_dist_to_center = raio_carretel + seg_col_radius + safety_margin

val = (length * math.cos(theta)) / (2 * min_dist_to_center)
val = min(1.0, max(-1.0, val))
alpha_link = 2 * math.asin(val)

raio_efetivo = min_dist_to_center / math.cos(alpha_link / 2.0)

elos_por_volta = (2 * math.pi) / alpha_link
passo_y = length * math.sin(theta)

num_links_enrolados = int(num_voltas_enroladas * elos_por_volta)
num_links_livres = num_links - num_links_enrolados

if num_links_livres < 1:
    print(f"❌ Erro: As voltas exigem {num_links_enrolados} elos, mas o cabo só tem {num_links}. Aumente 'num_links'.")
    sys.exit(1)

y_inicio = ancora_y
angulo_saida = -math.pi / 2.0   # -90°, saída apontando para +X
angulo_atual = angulo_saida - (num_links_enrolados * alpha_link)
y_atual = y_inicio
pontos_cabo = []

for i in range(num_links_enrolados + 1):
    x = ancora_x + raio_efetivo * math.cos(angulo_atual)
    z = ancora_z + raio_efetivo * math.sin(angulo_atual)
    pontos_cabo.append((x, y_atual, z))
    angulo_atual += alpha_link
    y_atual -= passo_y

P0_bezier = pontos_cabo[-1]

# ============================================================
# 2. GERAÇÃO DA RETA PARA A PARTE LIVRE
# ============================================================

# Direção: ao longo do eixo X, na direção de cabo_fim_x
# (drone estará no mesmo Y e Z do ponto de saída)
dx = cabo_fim_x - P0_bezier[0]
if dx >= 0:
    direcao = (1.0, 0.0, 0.0)
else:
    direcao = (-1.0, 0.0, 0.0)

# Se a distância até o destino for menor que o comprimento disponível,
# o drone estará exatamente em cabo_fim_x; caso contrário, o ponto
# final será determinado pelo número de elos livres.
comprimento_livre_total = num_links_livres * length
dist_reta = abs(dx)

pontos_livres = []
for i in range(1, num_links_livres + 1):
    px = P0_bezier[0] + direcao[0] * (i * length)
    py = P0_bezier[1] + direcao[1] * (i * length)   # será 0.0
    pz = P0_bezier[2] + direcao[2] * (i * length)   # será 0.0
    pontos_livres.append((px, py, pz))

# Último ponto da parte livre
P3 = pontos_livres[-1]
pontos_cabo.extend(pontos_livres)

# ============================================================
# DIAGNÓSTICO DOS ELOS
# ============================================================

comprimentos_reais = [dist3(pontos_cabo[i-1], pontos_cabo[i]) for i in range(1, len(pontos_cabo))]

print("============================================================")
print("GERAÇÃO DO CABO HÍBRIDO (Espiral + Reta)")
print("============================================================")
print(f"Número de elos total: {num_links} (Enrolados: {num_links_enrolados} | Livres: {num_links_livres})")
print(f"Comprimento nominal de cada elo: {length:.6f} m")
print(f"Comprimento total nominal: {comprimento_total:.6f} m")
print(f"Ponto inicial na extremidade: ({pontos_cabo[0][0]:.4f}, {pontos_cabo[0][1]:.4f}, {pontos_cabo[0][2]:.4f})")
print(f"Ponto de saída do carretel: ({P0_bezier[0]:.4f}, {P0_bezier[1]:.4f}, {P0_bezier[2]:.4f})")
print(f"Ponto final usado: ({P3[0]:.4f}, {P3[1]:.4f}, {P3[2]:.4f})")
print("============================================================")

# ============================================================
# SPAWN DO DRONE
# ============================================================

# A direção do drone é a mesma do último segmento (ao longo do eixo X)
yaw_base = math.atan2(direcao[1], direcao[0])   # será 0 ou π
offset_world_x, offset_world_y = rotacionar_offset_yaw(
    offset_conexao_drone_x, offset_conexao_drone_y, yaw_base
)

p_final = pontos_cabo[-1]
drone_spawn_x = p_final[0] - offset_world_x
drone_spawn_y = p_final[1] - offset_world_y
drone_spawn_z = p_final[2] - offset_conexao_drone_z

# (opcional: imprimir a posição de spawn)
print(f"Posição de spawn do drone: ({drone_spawn_x:.4f}, {drone_spawn_y:.4f}, {drone_spawn_z:.4f})")

# ============================================================
# INÉRCIAS
# ============================================================

limite_inercia_minima = 1e-5
massa_raiz = 0.05
raio_raiz_visual = 0.02
ixx_raiz = max((2.0/5.0)*massa_raiz*raio_raiz_visual**2, limite_inercia_minima)

massa_ponta = 0.005
raio_ponta = max(2.5*radius, 0.006)
ixx_ponta = max((2.0/5.0)*massa_ponta*raio_ponta**2, limite_inercia_minima)

# ============================================================
# GERAÇÃO DO cabo.sdf
# ============================================================

limite_rad = math.radians(limite_junta_deg)
limite_xml = f"""
        <limit>
          <lower>{-limite_rad:.6f}</lower>
          <upper>{limite_rad:.6f}</upper>
        </limit>"""

sdf = f"""<?xml version="1.0" ?>
<sdf version="1.8">
  <model name="cabo_flexivel">
    <self_collide>false</self_collide>

    <link name="raiz_cabo">
      <pose>0 0 0 0 0 0</pose>
      <inertial>
        <mass>{massa_raiz}</mass>
        <inertia>
          <ixx>{ixx_raiz}</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>{ixx_raiz}</iyy><iyz>0</iyz><izz>{ixx_raiz}</izz>
        </inertia>
      </inertial>
      <visual name="visual_raiz">
        <geometry><sphere><radius>{raio_raiz_visual}</radius></sphere></geometry>
        <material>
          <ambient>0.1 0.1 0.1 1</ambient>
          <diffuse>0.1 0.1 0.1 1</diffuse>
        </material>
      </visual>
    </link>
"""

parent_link = "raiz_cabo"
for i in range(1, num_links+1):
    nome_elo = "final_segment" if i == num_links else f"segment_{i}"
    p_atual = pontos_cabo[i-1]
    p_prox = pontos_cabo[i]
    seg_len = dist3(p_atual, p_prox)

    yaw, pitch = yaw_pitch_do_segmento(p_atual, p_prox)
    pose_x = p_atual[0] - ancora_x
    pose_y = p_atual[1] - ancora_y
    pose_z = p_atual[2] - ancora_z

    mass_seg = max(densidade_linear * seg_len, 0.005 if i == num_links else 0.001)

    ixx_seg = max(0.5*mass_seg*radius**2, limite_inercia_minima)
    iyy_zz_seg = max((1.0/12.0)*mass_seg*(3*radius**2 + seg_len**2), limite_inercia_minima)

    collision_radius_seg = 0.50*radius
    collision_length_seg = max(0.001, 0.60*seg_len)

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

    visual_ponta_xml = ""
    if i == num_links:
        visual_ponta_xml = f"""
      <visual name="visual_ponta_vermelha">
        <pose>{seg_len:.6f} 0 0 0 0 0</pose>
        <geometry><sphere><radius>{raio_ponta}</radius></sphere></geometry>
        <material>
          <ambient>0.8 0.1 0.1 1</ambient>
          <diffuse>0.8 0.1 0.1 1</diffuse>
        </material>
      </visual>"""

    dynamics_xml = f"""
      <dynamics>
        <damping>{damping_junta}</damping>
        <friction>{friction_junta}</friction>
      </dynamics>"""

    sdf += f"""
    <link name="{nome_elo}">
      <pose>{pose_x:.6f} {pose_y:.6f} {pose_z:.6f} 0 {pitch:.6f} {yaw:.6f}</pose>
      <enable_wind>false</enable_wind>
      <self_collide>false</self_collide>

      <visual name="visual">
        <pose>{seg_len/2:.6f} 0 0 0 1.5708 0</pose>
        <geometry><cylinder><radius>{radius}</radius><length>{seg_len:.6f}</length></cylinder></geometry>
        <material>
          <ambient>0 0 0 1</ambient>
          <diffuse>0 0 0 1</diffuse>
        </material>
      </visual>
{visual_ponta_xml}
      <collision name="collision">
        <pose>{seg_len/2:.6f} 0 0 0 1.5708 0</pose>
        <geometry><cylinder><radius>{collision_radius_seg}</radius><length>{collision_length_seg:.6f}</length></cylinder></geometry>
      </collision>

      <inertial>
        <pose>{seg_len/2:.6f} 0 0 0 0 0</pose>
        <mass>{mass_seg}</mass>
        <inertia>
          <ixx>{ixx_seg}</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>{iyy_zz_seg}</iyy><iyz>0</iyz><izz>{iyy_zz_seg}</izz>
        </inertia>
      </inertial>
    </link>

    <joint name="joint_{i}" type="universal">
      <parent>{parent_link}</parent>
      <child>{nome_elo}</child>
      <pose>0 0 0 0 0 0</pose>
      <axis>
        <xyz>0 1 0</xyz>
        {limite_xml}
        {dynamics_xml}
      </axis>
      <axis2>
        <xyz>0 0 1</xyz>
        {limite_xml}
        {dynamics_xml}
      </axis2>
      {sensor_xml}
    </joint>
"""
    parent_link = nome_elo

ponta_x = p_final[0] - ancora_x
ponta_y = p_final[1] - ancora_y
ponta_z = p_final[2] - ancora_z

sdf += f"""
    <link name="ponta_cabo">
      <pose relative_to="final_segment">
        {seg_len:.6f} 0 0 0 0 0
      </pose>
      <inertial>
        <mass>{massa_ponta}</mass>
        <inertia>
          <ixx>{ixx_ponta}</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>{ixx_ponta}</iyy><iyz>0</iyz><izz>{ixx_ponta}</izz>
        </inertia>
      </inertial>
      <collision name="collision_ponta">
        <geometry><sphere><radius>{raio_ponta}</radius></sphere></geometry>
      </collision>
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

world = f"""<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="mundo_ic">
    <gravity>0 0 -9.81</gravity>

    <physics name="physics" type="ode">
      <max_step_size>0.0005</max_step_size>
      <real_time_update_rate>1000</real_time_update_rate>
      <real_time_factor>1.0</real_time_factor>
      <ode>
        <solver>
          <type>quick</type>
          <iters>800</iters>
          <sor>1.0</sor>
        </solver>
        <constraints>
          <cfm>1e-6</cfm>
          <erp>0.9</erp>
          <contact_max_correcting_vel>0.5</contact_max_correcting_vel>
          <contact_surface_layer>0.01</contact_surface_layer>
        </constraints>
      </ode>
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
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
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
  </world>
</sdf>
"""

with open(caminho_world, "w") as f:
    f.write(world)
print("✓ my_world.sdf gerado com sucesso!")