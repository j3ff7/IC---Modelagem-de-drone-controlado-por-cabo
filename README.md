# Drone controlado por cabo

Este branch (`shared`) consolida uma baseline para simular no Gazebo/Ignition um drone conectado a um cabo flexivel discretizado, com foco em:

- hovering em waypoints conhecidos;
- diagnostico da interacao drone-tether;
- medicao de azimuth/elevation do cabo no frame do drone;
- comparacao com casos estaticos de postes.

O fluxo principal usa os pacotes em `src/`. Os diretorios `Gazebo/`, `Chrono/`, `Coppelia/` e `models/` contem material legado ou experimental.

## Build

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select pacote_do_drone cabo_avaliacao
source install/setup.bash
```

## Baseline Fisica

Drone em `src/pacote_do_drone/models/meu_drone/meu_drone.sdf`:

```text
base_link              1.500 kg
4 rotores              4 x 0.005 kg = 0.020 kg
cabo_sensor_link       0.020 kg
cabo_azimuth_link      0.010 kg
massa total SDF        1.550 kg
```

Cabo em `src/pacote_do_drone/tether_parameters.json`, gerado por `src/pacote_do_drone/models/gerar_cabo.py`:

```text
comprimento_total_m      2.500 m
num_links                50
length por segmento      0.050 m
densidade_linear_kg_m    0.060 kg/m
massa por segmento       0.003000 kg
massa dos segmentos      0.150 kg
links auxiliares         ~0.006 kg
massa dinamica total     ~0.156 kg
connection_type          ball
initial_shape            sine_slack horizontal
ancora                   (0.0, 0.0, 0.33) m
spawn padrao drone       (2.0, 0.0, 0.33) m
folga geometrica inicial 0.500 m
```

O parametro legado `mass` foi mantido consistente com a massa por segmento, mas a fonte principal agora e `densidade_linear_kg_m`.

```bash
python3 src/pacote_do_drone/models/gerar_cabo.py
ign sdf -k src/pacote_do_drone/models/cabo.sdf
```

## Controlador

O controlador `pacote_do_drone/movimento_circular.py` publica `/meu_drone/cmd_vel`. A referencia e uma sequencia de waypoints, mas o comando final e uma velocidade.

Defaults atuais do launch:

```text
tolerancia_posicao       0.12 m
tolerancia_altura        0.10 m
ganho_posicao_xy         0.8
ganho_altura             1.5
ganho_integral_xy        0.05
ganho_integral_z         0.08
ganho_velocidade_xy      1.4
ganho_velocidade_z       0.45
limite_vel_xy            0.35 m/s
limite_vel_z             0.50 m/s
cmd_vel_frame            body
janela_tangente_metros   0.15 m
```

O controlador usa `/clock` para derivadas, integracao, hovering e logs. Com 50 links, o RTF pode ficar perto de `0.08-0.10`; interprete sempre `t_sim`, nao tempo de parede.

## Teste Vertical

```bash
ros2 launch pacote_do_drone start_sim.launch.py \
  controlador_trajetoria:=true hover_metrics:=true \
  usar_cabo:=true prender_ancora:=true headless:=true \
  waypoints_file:=config/trajetoria_teste_z100.json \
  metricas_target_x:=2.0 metricas_target_y:=0.0 metricas_target_z:=1.0 \
  metricas_inicio_s:=0.0 metricas_duracao_s:=7.0 \
  metricas_janela_final_s:=2.0
```

Resultado observado com `rho=0.06 kg/m`: hover estavel, `roll_max~0.02 deg`, `pitch_max~0.45 deg`, `T_mean/max~0.33/0.38 N`, sem saturacao persistente.

## Teste N

`config/trajetoria_sensor_n.json`:

```text
WP0: (2.0, 0.0, 1.0)
WP1: (0.0, 1.0, 2.0)
```

```bash
ros2 launch pacote_do_drone start_sim.launch.py \
  controlador_trajetoria:=true hover_metrics:=true \
  usar_cabo:=true prender_ancora:=true headless:=true \
  waypoints_file:=config/trajetoria_sensor_n.json \
  metricas_target_x:=0.0 metricas_target_y:=1.0 metricas_target_z:=2.0 \
  metricas_inicio_s:=0.0 metricas_duracao_s:=34.0 \
  metricas_janela_final_s:=5.0 \
  log_periodo:=2.0 metricas_log_periodo_s:=2.0
```

Resultado observado:

```text
sequencia concluida: sim
pos_mean final       (-0.052, 1.040, 1.926) m
err_mean/rms/max     0.100 / 0.101 / 0.126 m
roll_max             0.89 deg
pitch_max            0.20 deg
T_mean/max           1.10 / 1.15 N
sat_xyz              0.0 / 0.0 / 0.0 %
az tangente drone    -29.36 +/- 0.90 deg
el tangente drone     34.91 +/- 0.75 deg
```

Antes da integral pequena, o mesmo caso estacionava perto de `z=1.87 m` para referencia `z=2.0 m` e nao entrava no criterio. Isso indica erro estacionario contra a carga/tensao do cabo, nao uma falha de frame ou saturacao de comando.

## Sensor De Angulo

Convencao no frame local do drone/sensor:

```text
x: frente
y: esquerda
z: cima
azimuth   = atan2(y, x)
elevation = atan2(-z, sqrt(x^2 + y^2))
```

Consequencias:

```text
cabo descendo verticalmente: elevation = 90 deg
cabo horizontal para frente: azimuth = 0 deg, elevation = 0 deg
cabo horizontal para esquerda: azimuth = 90 deg
cabo horizontal para direita: azimuth = -90 deg
```

Tangente local do cabo no lado do drone:

```bash
ros2 topic echo /cabo/azimuth_graus
ros2 topic echo /cabo/elevation_graus
ros2 topic echo /cabo/drone/azimuth_graus
ros2 topic echo /cabo/drone/elevation_graus
```

Reta ideal sensor-ancora, expressa no frame do drone:

```bash
ros2 topic echo /cabo/azimuth_ancora_graus
ros2 topic echo /cabo/elevation_ancora_graus
ros2 topic echo /cabo/drone/reta_ancora/azimuth_graus
ros2 topic echo /cabo/drone/reta_ancora/elevation_graus
```

Tangente fisica no lado da ancora/carretel, calculada no frame global:

```bash
ros2 topic echo /cabo/ancora/azimuth_graus
ros2 topic echo /cabo/ancora/elevation_graus
```

Monitor no terminal:

```bash
ros2 run pacote_do_drone cabo_monitor
ros2 run pacote_do_drone cabo_monitor -- --csv /tmp/cabo_angulos.csv
```

## Casos Estaticos Com Postes

```bash
ros2 launch cabo_avaliacao avaliar_cabo.launch.py caso:=s modo_cabo:=reto
ros2 launch cabo_avaliacao avaliar_cabo.launch.py caso:=s modo_cabo:=articulado
ros2 launch cabo_avaliacao avaliar_cabo.launch.py caso:=c1 modo_cabo:=catenaria
```

Use estes casos antes de alterar frames, sinais ou convencoes de azimuth/elevation.

## Validacoes De Desenvolvimento

```bash
python3 -m py_compile \
  src/pacote_do_drone/models/gerar_cabo.py \
  src/pacote_do_drone/launch/start_sim.launch.py \
  src/pacote_do_drone/pacote_do_drone/hover_metrics.py \
  src/pacote_do_drone/pacote_do_drone/movimento_circular.py

python3 src/pacote_do_drone/models/gerar_cabo.py
ign sdf -k src/pacote_do_drone/models/cabo.sdf

source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select pacote_do_drone cabo_avaliacao
```

## Cuidados

Nao rode varias simulacoes ao mesmo tempo:

```bash
pgrep -af 'ign gazebo|parameter_bridge|movimento_circular|sensores'
pkill -f 'ros2 launch pacote_do_drone'
pkill -f 'ign gazebo'
pkill -f 'parameter_bridge'
```

## Repositorio Remoto

```text
SSH:  git@github.com:j3ff7/Drone_controlado_por_cabo.git
HTTP: https://github.com/j3ff7/Drone_controlado_por_cabo.git
```
