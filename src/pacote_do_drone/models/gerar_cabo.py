import json
import os
import math

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


def lerp(p, q, a):
    return (
        p[0] + (q[0] - p[0]) * a,
        p[1] + (q[1] - p[1]) * a,
        p[2] + (q[2] - p[2]) * a
    )


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

num_links = max(3, int(params.get("num_links", 40)))

# Valores mínimos para evitar elos fisicamente ruins no solver
length = clamp_min(params.get("length", 0.05), 0.01)
radius = clamp_min(params.get("radius", 0.003), 0.001)

# Cálculo da massa do elo
densidade_linear = float(params.get("densidade_linear", 0.04))
mass_elo = float(params.get("mass", densidade_linear * length))
mass_elo = max(mass_elo, 0.001)

# Parâmetros focados estritamente na meta geométrica do cabo
cabo_fim_x = float(params.get("cabo_fim_x", 1.0))
cabo_fim_y = float(params.get("cabo_fim_y", 0.18))
cabo_fim_z = float(params.get("cabo_fim_z", 0.35))

parent_ancora_carretel = params.get(
    "parent_ancora_carretel",
    "meu_carretel::base_link"
)

# ============================================================
# CONFIGURAÇÕES FÍSICAS DO CABO
# ============================================================

comprimento_total = num_links * length

ancora_x = 0.0
ancora_y = 0.18
ancora_z = 0.35

# Ponto de conexão do cabo no drone, em coordenadas locais do base_link.
offset_conexao_drone_x = float(params.get("offset_conexao_drone_x", 0.0))
offset_conexao_drone_y = float(params.get("offset_conexao_drone_y", 0.0))
offset_conexao_drone_z = float(params.get("offset_conexao_drone_z", -0.10))

# Colisões e dinâmicas
collision_radius = 0.50 * radius
collision_length = 0.60 * length
damping_junta = float(params.get("damping_junta", 0.05))
friction_junta = float(params.get("friction_junta", 0.001))
limite_junta = float(params.get("limite_junta", 2.0))
z_minimo_chao = max(radius + 0.02, 0.03)

# ============================================================
# CURVA DE BÉZIER 3D (Com Auto-Ajuste de Distância)
# ============================================================

P0 = (ancora_x, ancora_y, ancora_z)
P3 = (cabo_fim_x, cabo_fim_y, cabo_fim_z)

dx_full = cabo_fim_x - ancora_x
dy_full = cabo_fim_y - ancora_y

dist_2d = math.hypot(dx_full, dy_full)
dist_3d = dist3(P0, P3)

# 1. Determinar o limite de queda
z_base = min(ancora_z, cabo_fim_z)
queda_maxima = z_base - z_minimo_chao

# 2. Calcular a folga máxima permitida para não bater no chão
folga_maxima = max(0.0, queda_maxima * 2.0)
dist_3d_minima = comprimento_total - folga_maxima

# 3. Correção: Se a distância for muito curta para o cabo, esticamos o alvo
if dist_3d < dist_3d_minima:
    dz = cabo_fim_z - ancora_z
    
    # Evita raiz negativa caso a distância mínima seja menor que a própria altura (quase impossível aqui)
    if dist_3d_minima > abs(dz):
        novo_dist_2d = math.sqrt(dist_3d_minima**2 - dz**2)
        
        # Descobre o ângulo original no plano XY
        yaw_original = math.atan2(dy_full, dx_full)
        
        # Empurra o alvo para a nova distância, mantendo a direção
        cabo_fim_x = ancora_x + math.cos(yaw_original) * novo_dist_2d
        cabo_fim_y = ancora_y + math.sin(yaw_original) * novo_dist_2d
        
        # Atualiza as variáveis que serão usadas pelo resto do código
        dx_full = cabo_fim_x - ancora_x
        dy_full = cabo_fim_y - ancora_y
        P3 = (cabo_fim_x, cabo_fim_y, cabo_fim_z)
        dist_3d = dist3(P0, P3)
        dist_2d = math.hypot(dx_full, dy_full)
        
        print(f"⚠️ Alvo esticado automaticamente para (X: {cabo_fim_x:.2f}, Y: {cabo_fim_y:.2f}) para evitar atravessar o chão.")

# Agora calculamos a folga final de forma segura
folga = max(0.0, comprimento_total - dist_3d)
sag_z = folga * 0.50

p1_z = ancora_z - sag_z
p2_z = cabo_fim_z - sag_z

P1 = (ancora_x + dx_full * 0.333, ancora_y + dy_full * 0.333, p1_z)
P2 = (ancora_x + dx_full * 0.666, ancora_y + dy_full * 0.666, p2_z)

# ============================================================
# AMOSTRAR A CURVA E DISTRIBUIR OS ELOS POR COMPRIMENTO
# ============================================================

print("Calculando elos com Bézier e reamostragem por comprimento...")

num_amostras = max(2000, num_links * 200)

curva = []
for k in range(num_amostras + 1):
    t = k / num_amostras
    curva.append(calcular_bezier(P0, P1, P2, P3, t))

cum = [0.0]
for k in range(1, len(curva)):
    cum.append(cum[-1] + dist3(curva[k - 1], curva[k]))

comprimento_curva = cum[-1]

pontos_cabo = []
j = 1

for i in range(num_links + 1):
    alvo_s = i * length

    if alvo_s <= comprimento_curva:
        while j < len(cum) - 1 and cum[j] < alvo_s:
            j += 1

        s0 = cum[j - 1]
        s1 = cum[j]
        if abs(s1 - s0) < 1e-12:
            a = 0.0
        else:
            a = (alvo_s - s0) / (s1 - s0)

        p = lerp(curva[j - 1], curva[j], a)
        pontos_cabo.append(p)

    else:
        excesso = alvo_s - comprimento_curva
        direcao_final = norm3((
            curva[-1][0] - curva[-2][0],
            curva[-1][1] - curva[-2][1],
            curva[-1][2] - curva[-2][2]
        ))

        p = (
            curva[-1][0] + direcao_final[0] * excesso,
            curva[-1][1] + direcao_final[1] * excesso,
            curva[-1][2] + direcao_final[2] * excesso
        )

        if p[2] < z_minimo_chao:
            p = (p[0], p[1], z_minimo_chao)

        pontos_cabo.append(p)

# ============================================================
# DEFINIR SPAWN DO DRONE BASEADO NA PONTA REAL DO CABO
# ============================================================

p_final = pontos_cabo[-1]

yaw_base = math.atan2(cabo_fim_y - ancora_y, cabo_fim_x - ancora_x)

offset_world_x, offset_world_y = rotacionar_offset_yaw(
    offset_conexao_drone_x,
    offset_conexao_drone_y,
    yaw_base
)

# O drone nasce de forma que o ponto de conexão local fique na ponta do cabo.
drone_spawn_x = p_final[0] - offset_world_x
drone_spawn_y = p_final[1] - offset_world_y
drone_spawn_z = p_final[2] - offset_conexao_drone_z

print(f"Comprimento total do cabo: {comprimento_total:.4f} m")
print(f"Comprimento aproximado da Bézier: {comprimento_curva:.4f} m")
print(f"Meta final do Cabo JSON/Ajustada: ({cabo_fim_x:.4f}, {cabo_fim_y:.4f}, {cabo_fim_z:.4f})")
print(f"Ponta real física do cabo:  ({p_final[0]:.4f}, {p_final[1]:.4f}, {p_final[2]:.4f})")
print(f"Spawn calculado para o drone: ({drone_spawn_x:.4f}, {drone_spawn_y:.4f}, {drone_spawn_z:.4f})")

# ============================================================
# INÉRCIAS
# ============================================================

ixx_elo    = 0.5 * mass_elo * radius ** 2
iyy_zz_elo = (1.0 / 12.0) * mass_elo * (3.0 * radius ** 2 + length ** 2)

massa_raiz = 0.02
raio_raiz_visual = 0.01
ixx_raiz = (2.0 / 5.0) * massa_raiz * raio_raiz_visual ** 2

massa_ponta = 0.001
raio_ponta = max(2.5 * radius, 0.006)
ixx_ponta = (2.0 / 5.0) * massa_ponta * raio_ponta ** 2

# ============================================================
# GERAR CABO.SDF
# ============================================================

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
        <geometry>
          <sphere>
            <radius>{raio_raiz_visual}</radius>
          </sphere>
        </geometry>
        <material>
          <ambient>0.1 0.1 0.1 1</ambient>
          <diffuse>0.1 0.1 0.1 1</diffuse>
        </material>
      </visual>
    </link>
"""

parent_link = "raiz_cabo"

for i in range(1, num_links + 1):
    nome_elo = "final_segment" if i == num_links else f"segment_{i}"

    p_atual = pontos_cabo[i - 1]
    p_prox = pontos_cabo[i]

    yaw, pitch = yaw_pitch_do_segmento(p_atual, p_prox)

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

    sdf += f"""
    <link name="{nome_elo}">
      <pose>{pose_x:.6f} {pose_y:.6f} {pose_z:.6f} 0 {pitch:.6f} {yaw:.6f}</pose>

      <visual name="visual">
        <pose>{length / 2:.6f} 0 0 0 1.5708 0</pose>
        <geometry>
          <cylinder>
            <radius>{radius}</radius>
            <length>{length}</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0 0 0 1</ambient>
          <diffuse>0 0 0 1</diffuse>
        </material>
      </visual>

      <collision name="collision">
        <pose>{length / 2:.6f} 0 0 0 1.5708 0</pose>
        <geometry>
          <cylinder>
            <radius>{collision_radius}</radius>
            <length>{collision_length}</length>
          </cylinder>
        </geometry>
      </collision>

      <inertial>
        <pose>{length / 2:.6f} 0 0 0 0 0</pose>
        <mass>{mass_elo}</mass>
        <inertia>
          <ixx>{ixx_elo}</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>{iyy_zz_elo}</iyy><iyz>0</iyz><izz>{iyy_zz_elo}</izz>
        </inertia>
      </inertial>
    </link>

    <joint name="joint_{i}" type="universal">
      <parent>{parent_link}</parent>
      <child>{nome_elo}</child>
      <pose>0 0 0 0 0 0</pose>

      <axis>
        <xyz>0 1 0</xyz>
        <limit>
          <lower>{-limite_junta}</lower>
          <upper>{limite_junta}</upper>
        </limit>
        <dynamics>
          <damping>{damping_junta}</damping>
          <friction>{friction_junta}</friction>
        </dynamics>
      </axis>

      <axis2>
        <xyz>0 0 1</xyz>
        <limit>
          <lower>{-limite_junta}</lower>
          <upper>{limite_junta}</upper>
        </limit>
        <dynamics>
          <damping>{damping_junta}</damping>
          <friction>{friction_junta}</friction>
        </dynamics>
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
      <pose>{ponta_x:.6f} {ponta_y:.6f} {ponta_z:.6f} 0 0 0</pose>

      <inertial>
        <mass>{massa_ponta}</mass>
        <inertia>
          <ixx>{ixx_ponta}</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>{ixx_ponta}</iyy><iyz>0</iyz><izz>{ixx_ponta}</izz>
        </inertia>
      </inertial>

      <visual name="visual_ponta">
        <geometry>
          <sphere>
            <radius>{raio_ponta}</radius>
          </sphere>
        </geometry>
        <material>
          <ambient>0.8 0.1 0.1 1</ambient>
          <diffuse>0.8 0.1 0.1 1</diffuse>
        </material>
      </visual>
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
          <iters>150</iters>
          <sor>1.0</sor>
        </solver>

        <constraints>
          <cfm>1e-5</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>0.5</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
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
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>

        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
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

with open(caminho_world, "w") as f:
    f.write(world)