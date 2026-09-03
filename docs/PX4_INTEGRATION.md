# Integracao PX4 SITL

Este documento registra a primeira etapa da integracao incremental do PX4 ao projeto Tethered Drone. A baseline atual do ROS 2/Gazebo permanece preservada; o PX4 foi adicionado como trilha paralela para validar primeiro um quadrotor padrao sem tether.

## Estado Inicial Do Repositorio

Comandos executados antes da integracao:

```bash
git status
git branch --show-current
git log -1 --oneline
```

Resultado:

```text
branch: shared
HEAD:   a73b816 Valida baseline cardinal do tether
```

O working tree ja continha alteracoes nao commitadas antes desta etapa. Nenhuma delas foi revertida ou substituida.

## Estrutura Atual Do Projeto

Fluxo ROS 2/Gazebo ativo:

```text
src/pacote_do_drone
  launch/start_sim.launch.py
  models/meu_drone/meu_drone.sdf
  models/gerar_cabo.py
  models/cabo.sdf
  tether_parameters.json
  pacote_do_drone/movimento_circular.py
  pacote_do_drone/sensores.py

src/cabo_avaliacao
  launch/avaliar_cabo.launch.py
  cabo_avaliacao/cenarios.py
  cabo_avaliacao/gerar_mundos.py
```

Arquitetura atual:

```text
controlador ROS 2 proprio
  -> /meu_drone/cmd_vel
  -> Gazebo MulticopterVelocityControl
  -> meu_drone.sdf
  -> ball joint
  -> tether atual
```

Ponto de insercao do PX4:

```text
PX4 SITL
  -> controladores internos PX4
  -> modelo Gazebo x500
  -> ponto de conexao futuro
  -> ball joint
  -> tether atual
```

Nesta primeira etapa, o tether nao foi conectado ao PX4.

## Versao PX4 Selecionada

Hardware real validado anteriormente:

```text
PX4_FMU_V3
PX4 v1.14.4
commit: 1555f2bd2229544c43966ab5f94879c41d8e1e01
```

Verificacao no upstream:

```bash
git ls-remote --tags --heads https://github.com/PX4/PX4-Autopilot.git \
  | grep -E '1555f2bd2229544c43966ab5f94879c41d8e1e01|v1\.14\.4|release/1\.14'
```

Resultado relevante:

```text
1555f2bd2229544c43966ab5f94879c41d8e1e01 refs/heads/release/1.14
4eb5668b38ab0217e7deffb58af5504237740be2 refs/tags/v1.14.4
1555f2bd2229544c43966ab5f94879c41d8e1e01 refs/tags/v1.14.4^{}
```

Conclusao: o commit informado existe no upstream e corresponde ao commit resolvido da tag anotada `v1.14.4`. A branch upstream `release/1.14` tambem aponta para esse commit.

## Clone PX4 Local

O PX4 foi clonado dentro do projeto:

```bash
mkdir -p px4
git clone https://github.com/PX4/PX4-Autopilot.git px4/PX4-Autopilot
cd px4/PX4-Autopilot
git checkout 1555f2bd2229544c43966ab5f94879c41d8e1e01
git submodule update --init --recursive
```

Estado do clone:

```text
remote: https://github.com/PX4/PX4-Autopilot.git
HEAD:   1555f2bd2229544c43966ab5f94879c41d8e1e01
tag:    v1.14.4
estado: detached HEAD
tamanho local apos build/teste: ~2.8 GB
```

O diretorio `px4/PX4-Autopilot/` e ignorado pelo Git deste repositorio. O projeto principal registra apenas a versao e os comandos.

## Ambiente Local Verificado

Ambiente observado:

```text
Ubuntu:        22.04.5 LTS jammy
Kernel:        5.19.0-45-generic
ROS 2:         Humble
Python:        3.10.12
GCC:           11.4.0
CMake:         3.22.1
Gazebo Classic: 11.14.0
Gazebo Sim:     7.9.0
Ignition:       6.18.0
```

Dependencias Python basicas verificadas:

```text
em, jinja2, yaml, numpy, serial, toml, kconfiglib, jsonschema: ok
```

Observacao de compatibilidade:

PX4 v1.14.4 contem integracoes para `gazebo-classic` e tambem `Tools/simulation/gz`, incluindo o modelo `x500`. O projeto tethered drone atual usa Gazebo Sim/Ignition via ROS 2. Para a migracao futura, o caminho mais natural e usar o modelo PX4 `gz_x500`, pois ele ja roda no mesmo ecossistema Gazebo Sim instalado.

## Build PX4 SITL

Comando:

```bash
cd /home/lima/codes/ic/drone-cabo/px4/PX4-Autopilot
make px4_sitl_default
```

Resultado:

```text
PX4 version: v1.14.4
target: px4_sitl_default
binario gerado: build/px4_sitl_default/bin/px4
resultado: build OK
```

## Teste PX4 SITL Sem Tether

Simulacao iniciada com:

```bash
cd /home/lima/codes/ic/drone-cabo/px4/PX4-Autopilot
HEADLESS=1 PX4_GZ_MODEL=x500 make px4_sitl gz_x500
```

O PX4 iniciou com:

```text
SYS_AUTOSTART=4001
world: Tools/simulation/gz/worlds/default.sdf
model name: x500_0
simulation model: x500
MAVLink UDP local: 14550
```

Comandos usados no shell `pxh>`:

```text
commander status
listener vehicle_local_position 1
commander arm
commander takeoff
listener vehicle_local_position 1
listener vehicle_attitude 1
commander status
commander land
shutdown
```

Resultados observados:

```text
Teste A - inicializacao: OK
  PX4 iniciou, Gazebo Sim default abriu em modo server/headless.

Teste B - arm: OK
  Armed by internal command
  Arm state: Armed

Teste C - decolagem: OK
  navigator: Using minimum takeoff altitude: 2.50 m
  commander: Takeoff detected

Teste D - hover aproximado: OK
  vehicle_local_position:
    x = -0.045 m
    y = -0.119 m
    z = -1.469 m  (NED; equivalente a ~1.47 m acima da origem local)
    dist_bottom = 1.950 m

  vehicle_attitude:
    roll  = -0.1 deg
    pitch = -0.3 deg
    yaw   = 89.5 deg

  commander status:
    Arm state: Armed
    navigation mode: AUTO_LOITER
    failsafe: no

Teste E - pouso: OK
  Landing at current position
  Landing detected
  Disarmed by landing
```

Um teste alternativo via MAVLink UDP tambem conseguiu heartbeat, arm e ACK de takeoff, mas nao subiu ate a altitude desejada usando apenas `MAV_CMD_NAV_TAKEOFF`. Para reproducibilidade, os comandos oficiais do shell PX4 acima sao a baseline desta etapa.

## Comandos Reproduziveis

Preparar PX4:

```bash
cd /home/lima/codes/ic/drone-cabo
mkdir -p px4
git clone https://github.com/PX4/PX4-Autopilot.git px4/PX4-Autopilot
cd px4/PX4-Autopilot
git checkout 1555f2bd2229544c43966ab5f94879c41d8e1e01
git submodule update --init --recursive
```

Compilar:

```bash
cd /home/lima/codes/ic/drone-cabo/px4/PX4-Autopilot
make px4_sitl_default
```

Iniciar simulação:

```bash
cd /home/lima/codes/ic/drone-cabo/px4/PX4-Autopilot
HEADLESS=1 PX4_GZ_MODEL=x500 make px4_sitl gz_x500
```

No shell `pxh>`:

```text
commander status
commander arm
commander takeoff
listener vehicle_local_position 1
listener vehicle_attitude 1
commander land
shutdown
```

Encerrar processos remanescentes, se necessario:

```bash
pkill -TERM -f 'PX4-Autopilot/build/px4_sitl_default/bin/px4'
pkill -TERM -f 'PX4-Autopilot/Tools/simulation/gz/worlds/default.sdf'
```

## Proxima Integracao Com Tether

Modelo PX4 escolhido para a proxima etapa:

```text
Tools/simulation/gz/models/x500/model.sdf
```

Link candidato para conexao:

```text
model x500
  link base_link
```

Massa do `base_link` no modelo PX4 `x500`:

```text
2.0 kg
```

Menor modificacao proposta:

1. Criar uma variante externa `x500_tethered`, sem alterar o modelo upstream diretamente.
2. Copiar ou sobrepor o modelo `x500` em uma pasta do projeto tethered drone.
3. Adicionar um link pequeno de conexao abaixo do `base_link`, por exemplo `tether_mount_link`.
4. Fixar `tether_mount_link` ao `base_link` por junta fixed curta.
5. Conectar o cabo atual ao `tether_mount_link` por `ball joint`.
6. Manter o tether como modelo separado gerado por `src/pacote_do_drone/models/gerar_cabo.py`.
7. Garantir que a junta `ball` esteja dentro do mesmo mundo Gazebo Sim onde o `x500` e o tether sao carregados.

Arquitetura futura:

```text
PX4 gz_x500
  -> x500::base_link
  -> tether_mount_link
  -> ball joint
  -> sistema_cabo_drone / cabo discretizado atual
  -> ancora
```

Se a forca do tether for aplicada por uma junta fisica do Gazebo ao link do modelo PX4, o PX4 deve perceber indiretamente essa perturbacao pela dinamica do veiculo, sensores simulados e estimador, sem precisar modificar os controladores PX4 nesta fase.

## Pendencias

- Automatizar arm/takeoff/land sem depender do shell interativo `pxh>`.
- Definir se a variante `x500_tethered` ficara em `src/pacote_do_drone/models` ou em uma nova pasta `px4_overlay/`.
- Decidir como gerar o mundo conjunto PX4 + tether sem duplicar o gerador atual.
- Integrar topicos PX4/uORB/MAVLink/ROS 2 para metricas equivalentes as atuais.
- Validar se o RTF permanece aceitavel com `x500 + tether N=50`.
