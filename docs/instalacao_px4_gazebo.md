# Instalacao e configuracao do PX4 com Gazebo

Este documento descreve como instalar, configurar e executar o PX4 SITL com Gazebo dentro da pasta do projeto Tethered Drone.

O objetivo desta etapa e manter a baseline atual do projeto intacta e adicionar o PX4 como uma trilha paralela de simulacao:

```text
drone-cabo/
  src/                  # ROS 2, tether, sensores e testes atuais
  docs/                 # documentacao tecnica, notas e guias do branch
  px4/
    PX4-Autopilot/      # clone local do PX4, ignorado pelo Git deste repo
```

## 1. Versao PX4 usada

A versao escolhida foi a mesma validada no Pixhawk:

```text
PX4 version: v1.14.4
PX4 commit: 1555f2bd2229544c43966ab5f94879c41d8e1e01
PX4 upstream: https://github.com/PX4/PX4-Autopilot.git
```

O commit corresponde a:

```text
refs/heads/release/1.14
refs/tags/v1.14.4^{}
```

## 2. Ambiente observado

Ambiente local usado nesta configuracao:

```text
Ubuntu:          22.04.5 LTS
ROS 2:           Humble
Python:          3.10.12
GCC:             11.4.0
CMake:           3.22.1
Gazebo Classic:  11.14.0
Gazebo Sim:      7.9.0
Ignition:        6.18.0
```

Verificacao rapida:

```bash
lsb_release -a
python3 --version
gcc --version | head -1
cmake --version | head -1
gazebo --version
gz sim --version
ign gazebo --versions
```

## 3. Clonar o PX4 dentro do projeto

Entre no projeto:

```bash
cd /home/lima/codes/ic/drone-cabo
```

Crie a pasta local para o PX4:

```bash
mkdir -p px4
```

Clone o PX4:

```bash
git clone https://github.com/PX4/PX4-Autopilot.git px4/PX4-Autopilot
```

Entre no PX4:

```bash
cd /home/lima/codes/ic/drone-cabo/px4/PX4-Autopilot
```

Faça checkout do commit validado:

```bash
git checkout 1555f2bd2229544c43966ab5f94879c41d8e1e01
```

Inicialize os submodules:

```bash
git submodule update --init --recursive
```

Confira a versao:

```bash
git rev-parse HEAD
git describe --tags --exact-match HEAD
git status --short --branch
```

Resultado esperado:

```text
1555f2bd2229544c43966ab5f94879c41d8e1e01
v1.14.4
## HEAD (no branch)
```

## 4. Evitar versionar o PX4 dentro do projeto

O clone `px4/PX4-Autopilot/` deve ser ignorado pelo Git do projeto `drone-cabo`.

A regra usada em `.gitignore` e:

```text
px4/PX4-Autopilot/
```

Verifique:

```bash
cd /home/lima/codes/ic/drone-cabo
git status --short --ignored px4 | head
```

Resultado esperado:

```text
?? px4/
!! px4/PX4-Autopilot/
```

Ou seja: a pasta `px4/` pode conter documentacao pequena do projeto, mas o clone completo do PX4 nao entra no Git deste repositorio.

## 5. Compilar o PX4 SITL

Entre no PX4:

```bash
cd /home/lima/codes/ic/drone-cabo/px4/PX4-Autopilot
```

Compile o SITL:

```bash
make px4_sitl_default
```

Resultado esperado:

```text
PX4 version: v1.14.4
target: px4_sitl_default
binario: build/px4_sitl_default/bin/px4
```

## 6. Rodar PX4 com Gazebo sem interface grafica

Para executar em modo headless:

```bash
cd /home/lima/codes/ic/drone-cabo/px4/PX4-Autopilot
HEADLESS=1 PX4_GZ_MODEL=x500 make px4_sitl gz_x500
```

Esse comando inicia:

```text
PX4 SITL
Gazebo Sim server
modelo x500
mundo default do PX4
```

O terminal deve abrir o shell do PX4:

```text
pxh>
```

## 7. Rodar PX4 com a interface do Gazebo

Para abrir a simulacao com a janela do Gazebo:

```bash
cd /home/lima/codes/ic/drone-cabo/px4/PX4-Autopilot
PX4_GZ_MODEL=x500 make px4_sitl gz_x500
```

Use este modo quando quiser visualizar o quadrotor `x500` no Gazebo.

## 8. Comandos basicos no shell PX4

Quando aparecer:

```text
pxh>
```

Verifique o estado:

```text
commander status
```

Arme o drone:

```text
commander arm
```

Decole:

```text
commander takeoff
```

Leia a posicao local:

```text
listener vehicle_local_position 1
```

Leia a atitude:

```text
listener vehicle_attitude 1
```

Pouse:

```text
commander land
```

Encerre o PX4:

```text
shutdown
```

## 9. Teste minimo esperado

Sequencia recomendada:

```text
commander status
commander arm
commander takeoff
listener vehicle_local_position 1
listener vehicle_attitude 1
commander status
commander land
shutdown
```

Resultado observado na validacao inicial:

```text
Arm: OK
Takeoff: OK
Hover aproximado: OK
Land: OK
Failsafe: no
```

Durante o hover sem tether:

```text
vehicle_local_position:
  x = -0.045 m
  y = -0.119 m
  z = -1.469 m  # NED, aproximadamente 1.47 m acima da origem local

vehicle_attitude:
  roll  = -0.1 deg
  pitch = -0.3 deg
  yaw   = 89.5 deg

commander:
  Arm state: Armed
  navigation mode: AUTO_LOITER
  failsafe: no
```

## 10. Encerrar processos remanescentes

Se a simulacao travar ou uma execucao anterior ficar aberta:

```bash
pkill -TERM -f 'PX4-Autopilot/build/px4_sitl_default/bin/px4'
pkill -TERM -f 'PX4-Autopilot/Tools/simulation/gz/worlds/default.sdf'
pkill -TERM -f 'gz sim'
```

Confira:

```bash
ps -o pid,ppid,stat,cmd -C px4 -C gz -C ruby
```

## 11. Modelo usado

O modelo usado nesta etapa e:

```text
PX4 model: gz_x500
SDF: px4/PX4-Autopilot/Tools/simulation/gz/models/x500/model.sdf
link principal: base_link
massa do base_link: 2.0 kg
```

Este modelo foi escolhido porque ja e suportado pelo PX4/Gazebo e deve ser a base mais simples para conectar o tether em uma etapa futura.

## 12. Integracao futura com o tether

Nesta etapa, o cabo ainda nao foi conectado ao PX4.

A integracao futura deve seguir a ideia:

```text
PX4 gz_x500
  -> x500::base_link
  -> tether_mount_link
  -> ball joint
  -> cabo discretizado atual
  -> ancora
```

Menor modificacao sugerida:

1. Criar uma variante externa do modelo `x500`, chamada `x500_tethered`.
2. Nao modificar diretamente o PX4 upstream.
3. Adicionar um link pequeno abaixo do `base_link`, por exemplo `tether_mount_link`.
4. Fixar esse link ao `base_link` por uma junta `fixed`.
5. Conectar o cabo atual ao `tether_mount_link` com uma junta `ball`.
6. Manter o cabo gerado por `src/pacote_do_drone/models/gerar_cabo.py`.
7. Manter sensores, metricas e topicos ROS 2 no projeto `drone-cabo`.

Assim, as forcas do tether entram naturalmente na dinamica do modelo controlado pelo PX4, sem alterar os controladores PX4 nesta primeira fase.

## 13. Relacao com ROS 2

Neste primeiro milestone, o PX4 foi validado isoladamente, sem conectar ROS 2.

Na proxima etapa, o ROS 2 deve ser usado para:

- instrumentacao;
- coleta de metricas;
- comparacao com a baseline atual;
- leitura de topicos/telemetria;
- eventualmente comandos offboard.

Para comunicacao ROS 2 com PX4, o caminho recomendado e usar a infraestrutura PX4 baseada em uXRCE-DDS/`px4_msgs`, mas isso ainda nao foi configurado nesta etapa.

## 14. Comandos resumidos

Instalar/clonar:

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

Rodar com GUI:

```bash
cd /home/lima/codes/ic/drone-cabo/px4/PX4-Autopilot
PX4_GZ_MODEL=x500 make px4_sitl gz_x500
```

Rodar headless:

```bash
cd /home/lima/codes/ic/drone-cabo/px4/PX4-Autopilot
HEADLESS=1 PX4_GZ_MODEL=x500 make px4_sitl gz_x500
```

Comandos no `pxh>`:

```text
commander status
commander arm
commander takeoff
listener vehicle_local_position 1
listener vehicle_attitude 1
commander land
shutdown
```
