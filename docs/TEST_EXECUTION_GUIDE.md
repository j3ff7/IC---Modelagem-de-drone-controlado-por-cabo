# Guia de execucao dos testes PX4/Gazebo

Comandos para reproduzir os testes ja executados no branch `shared`.

## Terminal 1 - preparar ambiente

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

Build PX4 SITL:

```bash
cd /home/lima/codes/ic/drone-cabo/px4/PX4-Autopilot
make px4_sitl_default
```

## Teste Etapa 0 - X500 puro

No Terminal 1:

```bash
cd /home/lima/codes/ic/drone-cabo/px4/PX4-Autopilot
PX4_GZ_MODEL=x500 make px4_sitl gz_x500
```

No prompt `pxh>`:

```text
commander status
commander arm
commander takeoff
listener vehicle_local_position 1
listener vehicle_attitude 1
commander land
shutdown
```

## Teste Etapa 3A - tether livre com 5 links

No Terminal 1:

```bash
cd /home/lima/codes/ic/drone-cabo
./tools/generate_x500_tethered.py --links 5 --length 0.30 --rho 0.06 --radius 0.003 --initial-axis z
GZ_SIM_RESOURCE_PATH=/home/lima/codes/ic/drone-cabo/src/pacote_do_drone/models:/home/lima/codes/ic/drone-cabo/px4/PX4-Autopilot/Tools/simulation/gz/models gz sdf -k src/pacote_do_drone/models/x500_tethered/model.sdf
```

Iniciar PX4 + Gazebo:

```bash
cd /home/lima/codes/ic/drone-cabo/px4/PX4-Autopilot
export GZ_SIM_RESOURCE_PATH=/home/lima/codes/ic/drone-cabo/src/pacote_do_drone/models:$GZ_SIM_RESOURCE_PATH
PX4_GZ_MODEL=x500_tethered make px4_sitl gz_x500
```

No prompt `pxh>`:

```text
commander status
commander arm
commander takeoff
listener vehicle_local_position 1
listener vehicle_attitude 1
commander land
shutdown
```

## Teste Etapa 3B - tether livre com 10 links

No Terminal 1:

```bash
cd /home/lima/codes/ic/drone-cabo
./tools/generate_x500_tethered.py --links 10 --length 0.30 --rho 0.06 --radius 0.003 --initial-axis z
GZ_SIM_RESOURCE_PATH=/home/lima/codes/ic/drone-cabo/src/pacote_do_drone/models:/home/lima/codes/ic/drone-cabo/px4/PX4-Autopilot/Tools/simulation/gz/models gz sdf -k src/pacote_do_drone/models/x500_tethered/model.sdf
```

Iniciar PX4 + Gazebo:

```bash
cd /home/lima/codes/ic/drone-cabo/px4/PX4-Autopilot
export GZ_SIM_RESOURCE_PATH=/home/lima/codes/ic/drone-cabo/src/pacote_do_drone/models:$GZ_SIM_RESOURCE_PATH
PX4_GZ_MODEL=x500_tethered make px4_sitl gz_x500
```

No prompt `pxh>`:

```text
commander status
commander arm
commander takeoff
listener vehicle_local_position 1
listener vehicle_attitude 1
commander land
shutdown
```

## Teste Etapa 4 - tether completo livre com 50 links

No Terminal 1:

```bash
cd /home/lima/codes/ic/drone-cabo
./tools/generate_x500_tethered.py --links 50 --length 2.50 --rho 0.06 --radius 0.003 --initial-axis z
GZ_SIM_RESOURCE_PATH=/home/lima/codes/ic/drone-cabo/src/pacote_do_drone/models:/home/lima/codes/ic/drone-cabo/px4/PX4-Autopilot/Tools/simulation/gz/models gz sdf -k src/pacote_do_drone/models/x500_tethered/model.sdf
```

Iniciar PX4 + Gazebo:

```bash
cd /home/lima/codes/ic/drone-cabo/px4/PX4-Autopilot
export GZ_SIM_RESOURCE_PATH=/home/lima/codes/ic/drone-cabo/src/pacote_do_drone/models:$GZ_SIM_RESOURCE_PATH
PX4_GZ_MODEL=x500_tethered make px4_sitl gz_x500
```

No prompt `pxh>`:

```text
commander status
commander arm
commander takeoff
listener vehicle_local_position 1
listener vehicle_attitude 1
commander status
commander land
commander status
shutdown
```

## Teste Etapa 5 - tether completo ancorado

Estado atual: tentativa bloqueada por topologia de juntas no Gazebo/DART. O comando abaixo reproduz a falha registrada no plano de integracao; nao e um caso aprovado para voo.

No Terminal 1:

```bash
cd /home/lima/codes/ic/drone-cabo
./tools/generate_x500_tethered.py --links 50 --length 2.50 --rho 0.06 --radius 0.003 --initial-axis z --anchored --anchor-x 0 --anchor-y 0 --anchor-z -2.57
GZ_SIM_RESOURCE_PATH=/home/lima/codes/ic/drone-cabo/src/pacote_do_drone/models:/home/lima/codes/ic/drone-cabo/px4/PX4-Autopilot/Tools/simulation/gz/models gz sdf -k src/pacote_do_drone/models/x500_tethered/model.sdf
```

Iniciar PX4 + Gazebo para observar o erro de startup:

```bash
cd /home/lima/codes/ic/drone-cabo/px4/PX4-Autopilot
export GZ_SIM_RESOURCE_PATH=/home/lima/codes/ic/drone-cabo/src/pacote_do_drone/models:$GZ_SIM_RESOURCE_PATH
PX4_GZ_MODEL=x500_tethered make px4_sitl gz_x500
```

Erro esperado:

```text
Asked to create a joint between links [tether_anchor_link] as parent and [tether_link_50] as child,
but the child link already has a parent joint of type [BallJoint].
```

Depois de reproduzir essa tentativa, regenere a configuracao aprovada da Etapa 4 para deixar o modelo em estado executavel:

```bash
cd /home/lima/codes/ic/drone-cabo
./tools/generate_x500_tethered.py --links 50 --length 2.50 --rho 0.06 --radius 0.003 --initial-axis z
```

## Teste de arquitetura A - tether ancorado independente

No Terminal 1, gere o tether independente:

```bash
cd /home/lima/codes/ic/drone-cabo
./tools/generate_tether_anchor_chain.py --links 50 --length 2.50 --rho 0.06 --radius 0.003 --initial-axis z
GZ_SIM_RESOURCE_PATH=/home/lima/codes/ic/drone-cabo/src/pacote_do_drone/models:/home/lima/codes/ic/drone-cabo/px4/PX4-Autopilot/Tools/simulation/gz/models gz sdf -k src/pacote_do_drone/models/tether_anchor_chain/model.sdf
```

Inicie o X500 puro:

```bash
cd /home/lima/codes/ic/drone-cabo/px4/PX4-Autopilot
PX4_GZ_MODEL=x500 make px4_sitl gz_x500
```

No Terminal 2, insira o tether independente:

```bash
cd /home/lima/codes/ic/drone-cabo
gz service -s /world/default/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 1000 --req 'sdf_filename: "/home/lima/codes/ic/drone-cabo/src/pacote_do_drone/models/tether_anchor_chain/model.sdf" name: "tether_anchor_chain" allow_renaming: true'
gz model -m tether_anchor_chain -l
gz topic -e -t /stats
```

No prompt `pxh>`:

```text
commander status
commander arm
commander takeoff
listener vehicle_attitude 1
commander land
shutdown
```

## Probe - ball joint entre modelos

Validar e executar o mundo minimo:

```bash
cd /home/lima/codes/ic/drone-cabo
gz sdf -k src/pacote_do_drone/worlds/inter_model_ball_probe.sdf
gz sim -s -r --iterations 1000 src/pacote_do_drone/worlds/inter_model_ball_probe.sdf
```

Resultado esperado: o mundo inicia sem erro bloqueante. Isso confirma `ball joint` inter-model predeclarado no `world`.

Criacao runtime de `<joint>` via `/world/create` nao e suportada nesta versao. O erro esperado e:

```text
Expected exactly one top-level <model>, <light> or <actor> on SDF.
```

## Experimento Etapa 5 - force-based com 5 links ancorados

No Terminal 1, compile o plugin e gere o tether:

```bash
cd /home/lima/codes/ic/drone-cabo
./tools/build_tether_force_plugin.sh
./tools/generate_tether_anchor_chain.py --links 5 --length 2.50 --rho 0.06 --radius 0.003 --initial-axis folded_ground --force-constraint --stiffness 5 --damping 0.5 --max-force 3
GZ_SIM_RESOURCE_PATH=/home/lima/codes/ic/drone-cabo/src/pacote_do_drone/models:/home/lima/codes/ic/drone-cabo/px4/PX4-Autopilot/Tools/simulation/gz/models gz sdf -k src/pacote_do_drone/models/tether_anchor_chain/model.sdf
```

Inicie o X500 puro com o caminho do plugin:

```bash
cd /home/lima/codes/ic/drone-cabo/px4/PX4-Autopilot
export GZ_SIM_RESOURCE_PATH=/home/lima/codes/ic/drone-cabo/src/pacote_do_drone/models:$GZ_SIM_RESOURCE_PATH
export GZ_SIM_SYSTEM_PLUGIN_PATH=/home/lima/codes/ic/drone-cabo/build/gz_plugins:$GZ_SIM_SYSTEM_PLUGIN_PATH
PX4_GZ_MODEL=x500 make px4_sitl gz_x500
```

No Terminal 2, insira o tether com a ancora junto ao solo:

```bash
cd /home/lima/codes/ic/drone-cabo
gz service -s /world/default/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 1000 --req 'sdf_filename: "/home/lima/codes/ic/drone-cabo/src/pacote_do_drone/models/tether_anchor_chain/model.sdf" name: "tether_anchor_chain" allow_renaming: false pose: {position: {z: 0.035}}'
gz topic -e -t /cabo/conexao/error
gz topic -e -t /cabo/conexao/force
gz topic -e -t /cabo/conexao/stats
./tools/collect_tether_force_stats.py --samples 300 --timeout 30
gz topic -e -t /stats
```

No prompt `pxh>`:

```text
commander status
commander arm
commander takeoff
listener vehicle_local_position 1
listener vehicle_attitude 1
commander status
commander land
commander status
shutdown
```

## Terminal 2 - acompanhar Gazebo

```bash
cd /home/lima/codes/ic/drone-cabo
gz topic -l
gz topic -e -t /stats
gz model -m x500_tethered_0 -l
gz model -m x500_tethered_0 -j
```

Topicos uteis:

```bash
gz topic -e -t /world/default/pose/info
gz topic -e -t /world/default/dynamic_pose/info
gz topic -e -t /world/default/model/x500_tethered_0/link/base_link/sensor/imu_sensor/imu
gz topic -e -t /world/default/model/x500_tethered_0/link/base_link/sensor/air_pressure_sensor/air_pressure
```

Processos:

```bash
ps -o pid,ppid,stat,cmd -C px4 -C gz -C ruby
```

Logs PX4:

```bash
find /home/lima/codes/ic/drone-cabo/px4/PX4-Autopilot/build/px4_sitl_default/rootfs/log -type f -name '*.ulg' | sort | tail
ulog_info /home/lima/codes/ic/drone-cabo/px4/PX4-Autopilot/build/px4_sitl_default/rootfs/log/2026-09-04/18_08_50.ulg
```

## Encerrar simulacao

Preferencialmente, no prompt `pxh>`:

```text
commander land
shutdown
```

Se sobrar processo:

```bash
pkill -TERM -f 'PX4-Autopilot/build/px4_sitl_default/bin/px4'
pkill -TERM -f 'PX4-Autopilot/Tools/simulation/gz/worlds/default.sdf'
pkill -TERM -f 'gz sim'
```
