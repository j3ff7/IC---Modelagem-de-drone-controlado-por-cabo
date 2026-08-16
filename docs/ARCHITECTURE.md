# Architecture

Este documento descreve apenas a arquitetura confirmada no estado atual do repositório.

## Visão Geral

O projeto é um workspace ROS 2 Humble com dois pacotes Python principais:

- `pacote_do_drone`: simulação dinâmica do drone com cabo no Gazebo/Ignition.
- `cabo_avaliacao`: geração de mundos estáticos com postes para validar os cálculos de ângulo do cabo sem a dinâmica do drone.

Diretórios `Gazebo/`, `Chrono/`, `Coppelia/` e `models/` contêm material legado ou experimental. O fluxo principal atual usa `src/pacote_do_drone` e `src/cabo_avaliacao`.

```mermaid
flowchart TD
  launch[start_sim.launch.py] --> gz[Gazebo/Ignition mundo_ic]
  launch --> bridge[ros_gz_bridge]
  launch --> sensores[sensores / LeitorCabo]
  launch --> ctrl[movimento_circular / ControladorTrajetoriaDrone]
  gerador[models/gerar_cabo.py] --> cabo_sdf[models/cabo.sdf]
  params[tether_parameters.json] --> gerador
  cabo_sdf --> gz
  drone_sdf[models/meu_drone/meu_drone.sdf] --> gz
  bridge --> sensores
  bridge --> ctrl
  sensores --> topicos_angulos[/cabo/*_graus]
  ctrl --> cmd[/meu_drone/cmd_vel]
  cmd --> gz
```

## `pacote_do_drone`

### Modelos E Configuração

- `models/meu_drone/meu_drone.sdf`: modelo do drone, rotores, links do sensor do cabo e plugins Gazebo.
- `models/gerar_cabo.py`: gera `models/cabo.sdf` e `worlds/my_world.sdf` a partir de `tether_parameters.json`.
- `models/cabo.sdf`: cabo discretizado gerado; não deve ser tratado como fonte primária manual.
- `tether_parameters.json`: parâmetros do cabo e âncora.
- `config/*.json`: trajetórias e tabelas esperadas para validação.

Parâmetros físicos atuais confirmados:

```text
Drone:
  base_link              1.500 kg
  4 rotores              4 x 0.005 kg
  cabo_sensor_link       0.020 kg
  cabo_azimuth_link      0.010 kg
  massa total aproximada 1.550 kg

Cabo:
  num_links              40
  length por segmento    0.050 m
  comprimento total      2.000 m
  massa por segmento     0.007375 kg
  dummy_mass             0.0001 kg por dummy
  root_mass              0.0005 kg
  tip_mass               0.0005 kg
  massa total            0.300 kg
  âncora                 (0.0, 0.0, 0.33) m
```

### Launch Principal

`src/pacote_do_drone/launch/start_sim.launch.py` monta um mundo diagnóstico em tempo de launch. Recursos relevantes:

- `usar_cabo`: inclui ou remove o cabo dinâmico.
- `headless`: executa Gazebo sem GUI.
- `spawn_x`, `spawn_y`, `spawn_z`, `spawn_yaw`: sobrescrevem o spawn do drone.
- `controlador_trajetoria`: inicia o controlador de waypoints.
- `cmd_vel_frame`: frame usado pelo comando de velocidade; o padrão do launch atual é `body`.
- `log_periodo`: controla a frequência dos logs compactos do controlador.

Quando `initial_end_x/y/z` estão presentes em `tether_parameters.json`, o launch usa esses valores como spawn padrão do drone.

### Controlador

Entry point:

```text
ros2 run pacote_do_drone movimento_circular
```

Classe principal: `ControladorTrajetoriaDrone` em `pacote_do_drone/movimento_circular.py`.

Responsabilidades:

- carregar waypoints de `waypoints_file` ou do parâmetro `waypoints`;
- controlar posição por velocidade em `/meu_drone/cmd_vel`;
- aplicar amortecimento por velocidade estimada por diferença de posição;
- manter tempo de hovering antes de avançar waypoint;
- publicar logs com posição, referência, erro, RPY, comando, saturação, tensões e rotação dos rotores.

Entradas:

- `/meu_drone/odom` (`nav_msgs/Odometry`)
- `/cabo/tensao_drone` (`geometry_msgs/WrenchStamped`)
- `/cabo/tensao_carretel` (`geometry_msgs/WrenchStamped`)
- `/world/mundo_ic/model/sistema_cabo_drone/model/meu_drone/joint_state` (`sensor_msgs/JointState`)

Saída:

- `/meu_drone/cmd_vel` (`geometry_msgs/Twist`)

### Sensor E Cálculo Dos Ângulos

Entry point:

```text
ros2 run pacote_do_drone sensores
```

Classe principal: `LeitorCabo` em `pacote_do_drone/sensores.py`.

Responsabilidades:

- ler odometria do drone;
- ler poses publicadas pelo Gazebo;
- estimar a tangente do cabo do lado do drone e da âncora;
- publicar azimuth/elevation em graus;
- publicar também leituras diretas das juntas do sensor simulado, quando disponíveis.

Tópicos principais:

```text
/cabo/azimuth_graus
/cabo/elevation_graus
/cabo/azimuth_ancora_graus
/cabo/elevation_ancora_graus
/cabo/drone/azimuth_graus
/cabo/drone/elevation_graus
/cabo/drone/reta_ancora/azimuth_graus
/cabo/drone/reta_ancora/elevation_graus
/cabo/ancora/azimuth_graus
/cabo/ancora/elevation_graus
/cabo/azimuth_joint_graus
/cabo/elevation_joint_graus
```

Observação: os tópicos históricos `/cabo/azimuth_graus` e `/cabo/elevation_graus` representam a estimativa do lado do drone, mas seus nomes são ambíguos. Os aliases `/cabo/drone/*` são mais explícitos.

### Convenção De Frames E Ângulos

Frame local do sensor/drone:

```text
x: frente
y: esquerda
z: cima
```

Definição matemática:

```text
azimuth   = atan2(y, x)
elevation = atan2(-z, sqrt(x^2 + y^2))
```

Consequências:

- cabo descendo verticalmente no frame local: `elevation = 90 deg`;
- cabo horizontal para frente: `azimuth = 0 deg`, `elevation = 0 deg`;
- cabo horizontal para a esquerda: `azimuth = 90 deg`;
- cabo horizontal para a direita: `azimuth = -90 deg`.

## `cabo_avaliacao`

Este pacote cria mundos estáticos para validar os ângulos do cabo em geometrias conhecidas.

Entry points:

```text
ros2 run cabo_avaliacao gerar_mundos
ros2 run cabo_avaliacao avaliador
ros2 launch cabo_avaliacao avaliar_cabo.launch.py
```

Componentes:

- `cenarios.py`: carrega configurações, calcula valores esperados e catenária.
- `gerar_mundos.py`: gera mundos SDF em `/tmp/cabo_avaliacao_worlds`.
- `avaliador.py`: compara ângulos medidos em Gazebo com valores esperados.
- `config/postes_padrao.json`: oito direções cardeais/intercardeais e caso `c1`.

Modos do cabo:

- `reto`: segmento único reto, útil para validar sinais.
- `articulado`: carrega o mesmo `cabo.sdf` usado no drone, mas estático.
- `catenaria`: gera uma aproximação geométrica de catenária por segmentos.

## Dependências Externas

Confirmadas pelos manifests e launch files:

- ROS 2 Humble;
- `rclpy`;
- `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `std_msgs`, `tf2_msgs`;
- `ros_gz_sim`;
- `ros_gz_bridge`;
- Gazebo/Ignition com plugins `Physics`, `SceneBroadcaster`, `Sensors`, `ForceTorque`, `MulticopterVelocityControl`, `MulticopterMotorModel`, `OdometryPublisher`, `JointStatePublisher`.

## Testes

Testes unitários existentes:

- `src/pacote_do_drone/test/test_angulos_cabo.py`
- `src/cabo_avaliacao/test/test_cenarios.py`

Também há testes padrão gerados por template em `pacote_do_drone/test/test_copyright.py`, `test_flake8.py` e `test_pep257.py`.
