# Guia rapido: PX4 SITL + Gazebo

Este guia mostra como iniciar a simulacao PX4 + Gazebo deste projeto a partir de um terminal novo.

## 1. Entrar no projeto

```bash
cd /home/lima/codes/ic/drone-cabo
```

Verifique o branch, se desejar:

```bash
git branch --show-current
```

Branch esperado nesta baseline:

```text
shared
```

## 2. Ativar ROS 2 e workspace do projeto

O PX4 SITL com `gz_x500` roda sem depender diretamente dos nos ROS 2 deste projeto. Mesmo assim, para manter o ambiente do projeto carregado e permitir inspecao ROS 2 quando necessario, rode:

```bash
set +u
source /opt/ros/humble/setup.bash
source /home/lima/codes/ic/drone-cabo/install/setup.bash
set -u
```

Se o workspace ainda nao foi compilado, compile antes:

```bash
cd /home/lima/codes/ic/drone-cabo
set +u
source /opt/ros/humble/setup.bash
set -u
colcon build --symlink-install --packages-select pacote_do_drone cabo_avaliacao
set +u
source /home/lima/codes/ic/drone-cabo/install/setup.bash
set -u
```

## 3. Entrar no PX4-Autopilot

```bash
cd /home/lima/codes/ic/drone-cabo/px4/PX4-Autopilot
```

Confira a versao PX4:

```bash
git rev-parse HEAD
git describe --tags --exact-match HEAD
```

Resultado esperado:

```text
1555f2bd2229544c43966ab5f94879c41d8e1e01
v1.14.4
```

## 4. Build do PX4 SITL

```bash
cd /home/lima/codes/ic/drone-cabo/px4/PX4-Autopilot
make px4_sitl_default
```

O binario esperado e:

```text
/home/lima/codes/ic/drone-cabo/px4/PX4-Autopilot/build/px4_sitl_default/bin/px4
```

## 5. Iniciar PX4 SITL com Gazebo

Modelo usado:

```text
gz_x500
```

Para iniciar com interface grafica do Gazebo:

```bash
cd /home/lima/codes/ic/drone-cabo/px4/PX4-Autopilot
PX4_GZ_MODEL=x500 make px4_sitl gz_x500
```

Para iniciar sem interface grafica:

```bash
cd /home/lima/codes/ic/drone-cabo/px4/PX4-Autopilot
HEADLESS=1 PX4_GZ_MODEL=x500 make px4_sitl gz_x500
```

Quando a inicializacao terminar, o terminal deve mostrar:

```text
pxh>
```

Os comandos do PX4 sao digitados nesse mesmo terminal, depois do prompt `pxh>`.

## 6. Comandos basicos no PX4

No terminal `pxh>`:

```text
commander status
commander arm
commander takeoff
listener vehicle_local_position 1
listener vehicle_attitude 1
commander land
shutdown
```

Interpretacao rapida:

- `commander status`: mostra estado de arm, modo de voo e failsafe.
- `commander arm`: arma o drone.
- `commander takeoff`: decola.
- `listener vehicle_local_position 1`: mostra uma amostra da posicao local.
- `listener vehicle_attitude 1`: mostra uma amostra da atitude.
- `commander land`: pousa.
- `shutdown`: encerra o PX4.

## 7. ROS 2, bridges e agentes

Nesta baseline PX4 inicial, nao ha bridge ROS 2/PX4 obrigatoria e nao ha agente uXRCE-DDS configurado no projeto.

Para este teste, basta rodar:

```bash
PX4_GZ_MODEL=x500 make px4_sitl gz_x500
```

ou:

```bash
HEADLESS=1 PX4_GZ_MODEL=x500 make px4_sitl gz_x500
```

Os pacotes ROS 2 atuais do projeto continuam disponiveis para a simulacao antiga com tether, mas ainda nao foram conectados ao PX4 nesta etapa.

## 8. Verificar se esta funcionando

### Verificar PX4

No `pxh>`:

```text
commander status
```

Procure por:

```text
Arm state: Standby
in failsafe: no
```

Apos `commander arm`:

```text
Arm state: Armed
```

### Verificar Gazebo

Em outro terminal:

```bash
ps -o pid,ppid,stat,cmd -C gz -C px4 -C ruby
```

Deve aparecer pelo menos um processo `px4` e um processo `gz sim`.

Tambem e possivel listar topicos Gazebo:

```bash
gz topic -l | head -40
```

### Verificar ROS 2

Em outro terminal:

```bash
set +u
source /opt/ros/humble/setup.bash
source /home/lima/codes/ic/drone-cabo/install/setup.bash
set -u
ros2 node list
ros2 topic list
```

Observacao: nesta baseline PX4 inicial, pode nao haver nos ROS 2 especificos do PX4, pois a ponte ROS 2/PX4 ainda nao foi configurada.

## 9. Encerrar a simulacao

Preferencialmente, no `pxh>`:

```text
commander land
shutdown
```

Se algum processo ficar preso, rode em outro terminal:

```bash
pkill -TERM -f 'PX4-Autopilot/build/px4_sitl_default/bin/px4'
pkill -TERM -f 'PX4-Autopilot/Tools/simulation/gz/worlds/default.sdf'
pkill -TERM -f 'gz sim'
```

Confirme que encerrou:

```bash
ps -o pid,ppid,stat,cmd -C px4 -C gz -C ruby
```

## 10. Troubleshooting

### O prompt `pxh>` nao aparece

Recompile e rode novamente:

```bash
cd /home/lima/codes/ic/drone-cabo/px4/PX4-Autopilot
make px4_sitl_default
PX4_GZ_MODEL=x500 make px4_sitl gz_x500
```

### A janela do Gazebo nao abre

Teste o modo sem GUI:

```bash
cd /home/lima/codes/ic/drone-cabo/px4/PX4-Autopilot
HEADLESS=1 PX4_GZ_MODEL=x500 make px4_sitl gz_x500
```

Verifique se o `gz sim` esta rodando:

```bash
ps -o pid,ppid,stat,cmd -C gz -C px4 -C ruby
```

### Ja existe uma simulacao antiga rodando

Encerre processos antigos:

```bash
pkill -TERM -f 'PX4-Autopilot/build/px4_sitl_default/bin/px4'
pkill -TERM -f 'gz sim'
```

Depois rode novamente:

```bash
cd /home/lima/codes/ic/drone-cabo/px4/PX4-Autopilot
PX4_GZ_MODEL=x500 make px4_sitl gz_x500
```

### Erro ao usar `source` com `unbound variable`

Se o shell estiver com `set -u`, use:

```bash
set +u
source /opt/ros/humble/setup.bash
source /home/lima/codes/ic/drone-cabo/install/setup.bash
set -u
```

### `ros2 topic list` nao mostra topicos do PX4

Isso e esperado nesta etapa. O PX4 SITL com `gz_x500` foi validado sem ponte ROS 2/PX4 obrigatoria. A integracao ROS 2/PX4 via uXRCE-DDS/`px4_msgs` ainda e uma etapa futura.

## 11. Sequencia completa recomendada

Terminal 1:

```bash
cd /home/lima/codes/ic/drone-cabo
set +u
source /opt/ros/humble/setup.bash
source /home/lima/codes/ic/drone-cabo/install/setup.bash
set -u

cd /home/lima/codes/ic/drone-cabo/px4/PX4-Autopilot
make px4_sitl_default
PX4_GZ_MODEL=x500 make px4_sitl gz_x500
```

No `pxh>`:

```text
commander status
commander arm
commander takeoff
listener vehicle_local_position 1
listener vehicle_attitude 1
commander land
shutdown
```

Terminal 2, para inspecao:

```bash
ps -o pid,ppid,stat,cmd -C px4 -C gz -C ruby
gz topic -l | head -40
```
