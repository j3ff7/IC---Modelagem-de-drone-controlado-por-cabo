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


def gerar_pontos_bezier(P0, P1, P2, P3, num_amostras):
    curva = []

    for k in range(num_amostras + 1):
        t = k / num_amostras
        curva.append(calcular_bezier(P0, P1, P2, P3, t))

    cum = [0.0]

    for k in range(1, len(curva)):
        cum.append(cum[-1] + dist3(curva[k - 1], curva[k]))

    return curva, cum


def comprimento_bezier(P0, P1, P2, P3, num_amostras):
    curva, cum = gerar_pontos_bezier(P0, P1, P2, P3, num_amostras)
    return cum[-1]


def montar_bezier_com_sag(P0, P3, sag):
    dx = P3[0] - P0[0]
    dy = P3[1] - P0[1]
    dz = P3[2] - P0[2]

    P1 = (
        P0[0] + 0.333 * dx,
        P0[1] + 0.333 * dy,
        P0[2] + 0.333 * dz - sag
    )

    P2 = (
        P0[0] + 0.666 * dx,
        P0[1] + 0.666 * dy,
        P0[2] + 0.666 * dz - sag
    )

    return P1, P2


def reamostrar_curva_por_comprimento(curva, cum, num_links, length, P3):
    pontos = []
    j = 1

    for i in range(num_links + 1):
        if i == num_links:
            pontos.append(P3)
            continue

        alvo_s = i * length

        while j < len(cum) - 1 and cum[j] < alvo_s:
            j += 1

        s0 = cum[j - 1]
        s1 = cum[j]

        if abs(s1 - s0) < 1e-12:
            a = 0.0
        else:
            a = (alvo_s - s0) / (s1 - s0)

        p = lerp(curva[j - 1], curva[j], a)
        pontos.append(p)

    return pontos


# ============================================================
# LER PARÂMETROS
# ============================================================

with open(caminho_json, 'r') as f:
    params = json.load(f)

num_links = max(3, int(params.get("num_links", 40)))

length = clamp_min(params.get("length", 0.05), 0.01)
radius = clamp_min(params.get("radius", 0.003), 0.001)

densidade_linear = float(params.get("densidade_linear", 0.04))

cabo_fim_x_original = float(params.get("cabo_fim_x", 1.0))
cabo_fim_y_original = float(params.get("cabo_fim_y", 0.18))
cabo_fim_z_original = float(params.get("cabo_fim_z", 0.35))

# ============================================================
# CONFIGURAÇÕES FÍSICAS DO CABO
# ============================================================

comprimento_total = num_links * length

ancora_x = 0.0
ancora_y = 0.18
ancora_z = 0.35

offset_conexao_drone_x = float(params.get("offset_conexao_drone_x", 0.0))
offset_conexao_drone_y = float(params.get("offset_conexao_drone_y", 0.0))
offset_conexao_drone_z = float(params.get("offset_conexao_drone_z", -0.10))

damping_junta = float(params.get("damping_junta", 0.05))
friction_junta = float(params.get("friction_junta", 0.001))
limite_junta = float(params.get("limite_junta", 2.0))

z_minimo_chao = max(radius + 0.02, 0.03)

# ============================================================
# AJUSTAR A BÉZIER PARA NÃO EXTRAPOLAR OS ÚLTIMOS ELOS
# ============================================================

P0 = (ancora_x, ancora_y, ancora_z)
P3_original = (cabo_fim_x_original, cabo_fim_y_original, cabo_fim_z_original)

num_amostras = max(4000, num_links * 250)

dist_reta_original = dist3(P0, P3_original)

dx_xy = cabo_fim_x_original - ancora_x
dy_xy = cabo_fim_y_original - ancora_y
dist_xy_original = math.hypot(dx_xy, dy_xy)

if dist_xy_original < 1e-9:
    dir_x = 1.0
    dir_y = 0.0
else:
    dir_x = dx_xy / dist_xy_original
    dir_y = dy_xy / dist_xy_original

z_base = min(ancora_z, cabo_fim_z_original)
sag_maximo = max(0.0, z_base - z_minimo_chao)

# Caso 1: o alvo está mais longe que o cabo. O cabo não alcança.
if dist_reta_original > comprimento_total:
    direcao_3d = norm3((
        P3_original[0] - P0[0],
        P3_original[1] - P0[1],
        P3_original[2] - P0[2]
    ))

    P3 = (
        P0[0] + direcao_3d[0] * comprimento_total,
        P0[1] + direcao_3d[1] * comprimento_total,
        P0[2] + direcao_3d[2] * comprimento_total
    )

    sag_final = 0.0

    print("⚠️ O alvo estava mais longe que o comprimento total do cabo.")
    print("⚠️ O ponto final foi aproximado para o cabo conseguir alcançar.")

else:
    # Primeiro tentamos manter o alvo original e só aumentar a barriga da curva.
    P1_max, P2_max = montar_bezier_com_sag(P0, P3_original, sag_maximo)
    comprimento_com_sag_maximo = comprimento_bezier(P0, P1_max, P2_max, P3_original, num_amostras)

    if comprimento_com_sag_maximo >= comprimento_total:
        # Dá para manter o alvo original.
        P3 = P3_original

        baixo = 0.0
        alto = sag_maximo

        for _ in range(60):
            meio = 0.5 * (baixo + alto)
            P1_teste, P2_teste = montar_bezier_com_sag(P0, P3, meio)
            comp_teste = comprimento_bezier(P0, P1_teste, P2_teste, P3, num_amostras)

            if comp_teste < comprimento_total:
                baixo = meio
            else:
                alto = meio

        sag_final = 0.5 * (baixo + alto)

        print("✓ O alvo original foi mantido.")
        print(f"✓ Sag ajustado automaticamente: {sag_final:.4f} m")

    else:
        # Nem com a barriga máxima a curva consome todo o comprimento do cabo.
        # Então o alvo precisa ser esticado na direção horizontal.
        sag_final = sag_maximo

        baixo = dist_xy_original
        alto = max(dist_xy_original + 0.10, comprimento_total)

        while True:
            P3_teste = (
                ancora_x + dir_x * alto,
                ancora_y + dir_y * alto,
                cabo_fim_z_original
            )

            P1_teste, P2_teste = montar_bezier_com_sag(P0, P3_teste, sag_final)
            comp_teste = comprimento_bezier(P0, P1_teste, P2_teste, P3_teste, num_amostras)

            if comp_teste >= comprimento_total:
                break

            alto *= 1.5

        for _ in range(70):
            meio = 0.5 * (baixo + alto)

            P3_teste = (
                ancora_x + dir_x * meio,
                ancora_y + dir_y * meio,
                cabo_fim_z_original
            )

            P1_teste, P2_teste = montar_bezier_com_sag(P0, P3_teste, sag_final)
            comp_teste = comprimento_bezier(P0, P1_teste, P2_teste, P3_teste, num_amostras)

            if comp_teste < comprimento_total:
                baixo = meio
            else:
                alto = meio

        dist_xy_final = 0.5 * (baixo + alto)

        P3 = (
            ancora_x + dir_x * dist_xy_final,
            ancora_y + dir_y * dist_xy_final,
            cabo_fim_z_original
        )

        print("⚠️ O cabo era longo demais para terminar no alvo original sem atravessar o chão.")
        print("⚠️ O ponto final foi esticado na direção horizontal.")
        print(f"⚠️ Distância XY original: {dist_xy_original:.4f} m")
        print(f"⚠️ Distância XY ajustada: {dist_xy_final:.4f} m")

P1, P2 = montar_bezier_com_sag(P0, P3, sag_final)

curva, cum = gerar_pontos_bezier(P0, P1, P2, P3, num_amostras)
comprimento_curva = cum[-1]

pontos_cabo = reamostrar_curva_por_comprimento(
    curva=curva,
    cum=cum,
    num_links=num_links,
    length=length,
    P3=P3
)

# ============================================================
# DIAGNÓSTICO DOS ELOS
# ============================================================

comprimentos_reais = [
    dist3(pontos_cabo[i - 1], pontos_cabo[i])
    for i in range(1, len(pontos_cabo))
]

print("============================================================")
print("DIAGNÓSTICO DO CABO")
print("============================================================")
print(f"Número de elos: {num_links}")
print(f"Comprimento nominal de cada elo: {length:.6f} m")
print(f"Comprimento total nominal: {comprimento_total:.6f} m")
print(f"Comprimento aproximado da Bézier final: {comprimento_curva:.6f} m")
print(f"Menor elo real: {min(comprimentos_reais):.6f} m")
print(f"Maior elo real: {max(comprimentos_reais):.6f} m")
print(f"Ponto final original: ({cabo_fim_x_original:.4f}, {cabo_fim_y_original:.4f}, {cabo_fim_z_original:.4f})")
print(f"Ponto final usado:     ({P3[0]:.4f}, {P3[1]:.4f}, {P3[2]:.4f})")
print("============================================================")

# ============================================================
# DEFINIR SPAWN DO DRONE BASEADO NA PONTA REAL DO CABO
# ============================================================

p_final = pontos_cabo[-1]

yaw_base = math.atan2(P3[1] - ancora_y, P3[0] - ancora_x)

offset_world_x, offset_world_y = rotacionar_offset_yaw(
    offset_conexao_drone_x,
    offset_conexao_drone_y,
    yaw_base
)

drone_spawn_x = p_final[0] - offset_world_x
drone_spawn_y = p_final[1] - offset_world_y
drone_spawn_z = p_final[2] - offset_conexao_drone_z

print(f"Ponta real física do cabo:  ({p_final[0]:.4f}, {p_final[1]:.4f}, {p_final[2]:.4f})")
print(f"Spawn calculado para o drone: ({drone_spawn_x:.4f}, {drone_spawn_y:.4f}, {drone_spawn_z:.4f})")

# ============================================================
# INÉRCIAS GERAIS
# ============================================================

limite_inercia_minima = 1e-5

massa_raiz = 0.02
raio_raiz_visual = 0.01

ixx_raiz_calc = (2.0 / 5.0) * massa_raiz * raio_raiz_visual ** 2
ixx_raiz = max(ixx_raiz_calc, limite_inercia_minima)

massa_ponta = 0.001
raio_ponta = max(2.5 * radius, 0.006)

ixx_ponta_calc = (2.0 / 5.0) * massa_ponta * raio_ponta ** 2
ixx_ponta = max(ixx_ponta_calc, limite_inercia_minima)

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

    seg_len = dist3(p_atual, p_prox)

    if seg_len < 1e-6:
        raise ValueError(
            f"Elo {i} com comprimento quase zero. "
            f"Verifique a geração dos pontos do cabo."
        )

    yaw, pitch = yaw_pitch_do_segmento(p_atual, p_prox)

    pose_x = p_atual[0] - ancora_x
    pose_y = p_atual[1] - ancora_y
    pose_z = p_atual[2] - ancora_z

    mass_seg = max(densidade_linear * seg_len, 0.001)

    ixx_seg_calc = 0.5 * mass_seg * radius ** 2
    iyy_zz_seg_calc = (1.0 / 12.0) * mass_seg * (3.0 * radius ** 2 + seg_len ** 2)

    ixx_seg = max(ixx_seg_calc, limite_inercia_minima)
    iyy_zz_seg = max(iyy_zz_seg_calc, limite_inercia_minima)

    collision_radius_seg = 0.50 * radius
    collision_length_seg = max(0.001, 0.60 * seg_len)

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
      <!-- Esfera vermelha desenhada diretamente no final do último elo.
           Assim ela fica garantidamente na ponta visual do cabo. -->
      <visual name="visual_ponta_vermelha">
        <pose>{seg_len:.6f} 0 0 0 0 0</pose>
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
"""

    sdf += f"""
    <link name="{nome_elo}">
      <pose>{pose_x:.6f} {pose_y:.6f} {pose_z:.6f} 0 {pitch:.6f} {yaw:.6f}</pose>

      <enable_wind>false</enable_wind>
      <self_collide>false</self_collide>

      <visual name="visual">
        <pose>{seg_len / 2:.6f} 0 0 0 1.5708 0</pose>
        <geometry>
          <cylinder>
            <radius>{radius}</radius>
            <length>{seg_len:.6f}</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0 0 0 1</ambient>
          <diffuse>0 0 0 1</diffuse>
        </material>
      </visual>
{visual_ponta_xml}
      <collision name="collision">
        <pose>{seg_len / 2:.6f} 0 0 0 1.5708 0</pose>
        <geometry>
          <cylinder>
            <radius>{collision_radius_seg}</radius>
            <length>{collision_length_seg:.6f}</length>
          </cylinder>
        </geometry>
      </collision>

      <inertial>
        <pose>{seg_len / 2:.6f} 0 0 0 0 0</pose>
        <mass>{mass_seg}</mass>
        <inertia>
          <ixx>{ixx_seg}</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>{iyy_zz_seg}</iyy><iyz>0</iyz><izz>{iyy_zz_seg}</izz>
        </inertia>
      </inertial>
    </link>

    <joint name="joint_{i}" type="ball">
      <parent>{parent_link}</parent>
      <child>{nome_elo}</child>
      <pose>0 0 0 0 0 0</pose>
  {sensor_xml}
    </joint>
"""

    parent_link = nome_elo

ponta_x = p_final[0] - ancora_x
ponta_y = p_final[1] - ancora_y
ponta_z = p_final[2] - ancora_z

# A ponta_cabo continua existindo como link físico para você poder usar em juntas,
# sensores ou conexão futura com o drone. Mas a esfera vermelha visual foi colocada
# no final_segment para não aparecer deslocada.
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

      <collision name="collision_ponta">
        <geometry>
          <sphere>
            <radius>{raio_ponta}</radius>
          </sphere>
        </geometry>
      </collision>
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
          <iters>400</iters>
          <sor>1.0</sor>
        </solver>

        <constraints>
          <cfm>1e-4</cfm>
          <erp>0.8</erp>
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
      <static>true</static>
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