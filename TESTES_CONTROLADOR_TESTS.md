# Auditoria do controlador no branch tests

Este documento registra a auditoria minima da base de tempo do controlador existente no branch `tests`.

## Base de tempo

O launch `pacote_do_drone start_sim.launch.py` faz bridge de `/clock`:

```bash
ros2 launch pacote_do_drone start_sim.launch.py
```

Os scripts legados de controle sao:

```text
src/pacote_do_drone/scripts/test_moviment.py
src/pacote_do_drone/scripts/test_tensão.py
```

Antes da correcao, ambos usavam `self.dt` fixo para integrais e derivadas. Isso podia mudar o comportamento dinamico quando o RTF da simulacao fosse diferente de 1.

Depois da correcao, os dois scripts assinam `/clock` e calculam `dt` a partir de tempo simulado:

```text
dt = (t_sim_atual - t_sim_anterior)
```

Tempo de parede nao deve ser usado para integrais, derivadas, temporizacao fisica ou maquina de estados.

## Teste automatizado

Execute:

```bash
cd /home/lima/codes/ic/drone-cabo
source /opt/ros/humble/setup.bash
python3 -m pytest -q src/pacote_do_drone/test/test_time_base.py
```

Esse teste verifica que os controladores legados nao usam `time.time()`, `time.monotonic()`, `time.perf_counter()` ou `datetime`, e que nao voltaram a usar `self.dt` fixo no PID.

## Testes manuais minimos e logging

Terminal 1:

```bash
cd /home/lima/codes/ic/drone-cabo
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select pacote_do_drone
source install/setup.bash
ros2 launch pacote_do_drone start_sim.launch.py
```

Terminal 2:

```bash
cd /home/lima/codes/ic/drone-cabo
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Para isolar o controlador do tether, use o launch headless sem cabo:

```bash
cd /home/lima/codes/ic/drone-cabo
source /opt/ros/humble/setup.bash
source install/pacote_do_drone/share/pacote_do_drone/package.bash
ros2 launch pacote_do_drone controller_no_tether.launch.py
```

O controlador legado `test_moviment.py` aceita os parametros:

```text
x_alvo
y_alvo
altura_alvo
duracao_teste
csv_path
log_csv_periodo
```

Cada CSV registra:

```text
t_sim, dt_sim, t_wall, rtf,
x_ref, y_ref, z_ref,
x, y, z,
erro_x, erro_y, erro_z,
vx, vy, vz,
cmd_x, cmd_y, cmd_z,
roll, pitch, yaw,
I_x, I_y, I_z
```

Os plots sao gerados com:

```bash
python3 tools/plot_controller_test.py <csv> --name <nome> --out-dir results/controller_tests
```

### Teste A - waypoint unico vertical

Objetivo:

```text
spawn -> (2.0, 0.0, 0.60)
```

Comando:

```bash
python3 src/pacote_do_drone/scripts/test_moviment.py --ros-args -p x_alvo:=2.0 -p y_alvo:=0.0 -p altura_alvo:=0.60 -p duracao_teste:=12.0 -p csv_path:=results/controller_tests/vertical_z060.csv
```

Resultado: FAIL no controlador basico, porque o eixo Z converge, mas X/Y divergem com saturacao.

### Teste B - waypoint vertical maior

Objetivo:

```text
spawn -> (2.0, 0.0, 1.00)
```

Comando:

```bash
python3 src/pacote_do_drone/scripts/test_moviment.py --ros-args -p x_alvo:=2.0 -p y_alvo:=0.0 -p altura_alvo:=1.00 -p duracao_teste:=12.0 -p csv_path:=results/controller_tests/vertical_z100.csv
```

Resultado: FAIL pelo mesmo motivo do Teste A.

### Teste C - deslocamento em X

Objetivo:

```text
spawn -> (3.0, 0.0, 0.60)
```

Comando:

```bash
python3 src/pacote_do_drone/scripts/test_moviment.py --ros-args -p x_alvo:=3.0 -p y_alvo:=0.0 -p altura_alvo:=0.60 -p duracao_teste:=12.0 -p csv_path:=results/controller_tests/step_x.csv
```

Resultado: FAIL. O movimento nao permanece predominante em X; ha forte acoplamento e divergencia em Y.

### Teste D - deslocamento em Y

Objetivo:

```text
spawn -> (2.0, 1.0, 0.60)
```

Comando:

```bash
python3 src/pacote_do_drone/scripts/test_moviment.py --ros-args -p x_alvo:=2.0 -p y_alvo:=1.0 -p altura_alvo:=0.60 -p duracao_teste:=12.0 -p csv_path:=results/controller_tests/step_y.csv
```

Resultado: FAIL. O movimento nao permanece predominante em Y; ha forte acoplamento e divergencia em X/Y.

### Teste E - deslocamento combinado XY

Objetivo:

```text
spawn -> (3.0, 1.0, 0.60)
```

Comando:

```bash
python3 src/pacote_do_drone/scripts/test_moviment.py --ros-args -p x_alvo:=3.0 -p y_alvo:=1.0 -p altura_alvo:=0.60 -p duracao_teste:=12.0 -p csv_path:=results/controller_tests/step_xy.csv
```

Resultado: FAIL. O plano XY nao converge de forma confiavel.

## Metricas obtidas

| Teste | RMSE x [m] | RMSE y [m] | RMSE z [m] | erro final 3D [m] | overshoot z [m] | convergencia z < 0.10 m [s] | RTF medio | Resultado |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| A vertical_z060 | 0.297 | 0.460 | 0.121 | 1.562 | 0.039 | 11.62 | 1.000 | FAIL |
| B vertical_z100 | 0.294 | 0.442 | 0.190 | 1.660 | 0.061 | 11.58 | 1.000 | FAIL |
| C step_x | 1.766 | 1.661 | 0.349 | 4.389 | 0.580 | 11.96 | 1.000 | FAIL |
| D step_y | 1.386 | 1.544 | 0.103 | 3.143 | 0.088 | 10.88 | 1.000 | FAIL |
| E step_xy | 1.324 | 1.514 | 0.041 | 1.545 | 0.103 | 7.16 | 1.000 | FAIL |

CSV gerados:

```text
results/controller_tests/vertical_z060.csv
results/controller_tests/vertical_z100.csv
results/controller_tests/step_x.csv
results/controller_tests/step_y.csv
results/controller_tests/step_xy.csv
results/controller_tests/vertical_z060_slow.csv
```

PNGs gerados por teste:

```text
<nome>_position_x.png
<nome>_position_y.png
<nome>_position_z.png
<nome>_errors.png
<nome>_commands.png
<nome>_attitude.png
<nome>_trajectory_xy.png
```

## Validacao da base de tempo

O caso `vertical_z060` foi repetido com o mundo:

```text
controller_no_tether_slow.sdf
```

Esse mundo usa:

```text
real_time_factor = 0.25
real_time_update_rate = 250
```

Comando da simulacao slow:

```bash
ros2 launch pacote_do_drone controller_no_tether_slow.launch.py
```

Comando do controlador:

```bash
python3 src/pacote_do_drone/scripts/test_moviment.py --ros-args -p x_alvo:=2.0 -p y_alvo:=0.0 -p altura_alvo:=0.60 -p duracao_teste:=12.0 -p csv_path:=results/controller_tests/vertical_z060_slow.csv
```

Comparacao:

```bash
python3 tools/plot_controller_test.py results/controller_tests/vertical_z060.csv --compare-low results/controller_tests/vertical_z060_slow.csv --compare-name vertical_z060_rtf --axis z --out-dir results/controller_tests
```

Resultado da comparacao em Z:

```text
RTF alto:  rtf_medio = 1.000
RTF baixo: rtf_medio = 0.250
RMSE entre z(t_sim) alto e baixo = 0.022 m
```

Plot:

```text
results/controller_tests/vertical_z060_rtf_compare_z.png
```

Conclusao: a base de tempo ficou coerente. A falha remanescente e de tracking horizontal/controlador, nao de RTF.

## Topicos uteis

```bash
ros2 topic echo /clock
ros2 topic echo /meu_drone/odom
ros2 topic echo /meu_drone/cmd_vel
ros2 topic echo /cabo/tensao_drone
ros2 topic echo /cabo/tensao_carretel
```

## Comparacao com shared

| Aspecto | shared | tests |
| --- | --- | --- |
| fonte de tempo | `/clock` via subscription | `/clock` via subscription apos correcao |
| calculo de dt | diferenca entre tempos simulados | diferenca entre tempos simulados apos correcao |
| integrador | usa `dt` simulado | usa `dt` simulado apos correcao |
| derivada | usa `dt` simulado ou velocidades derivadas de odometria em tempo simulado | usa `dt` simulado apos correcao |
| tempo_hover | implementado no controlador de trajetoria | nao implementado no controlador legado |
| tempo_estabilizacao | implementado no controlador de trajetoria | nao implementado no controlador legado |
| timeouts | baseados em tempo simulado quando ligados a dinamica | nao ha timeouts dinamicos relevantes nos scripts legados |
| use_sim_time | `/clock` bridged e usado diretamente pelo controlador | `/clock` bridged e usado diretamente pelos scripts |

## Diagnostico do controlador legado

O controlador basico de `tests` nao usa sequenciador de missao. Ele aplica diretamente uma referencia fixa.

Durante os testes A-E, o eixo Z apresentou resposta razoavel inicialmente, mas o plano XY entrou em oscilacao crescente, com comandos saturando em `cmd_x/cmd_y = +/-1.0 m/s`. Mesmo em testes verticais, X/Y divergiram apos alguns segundos.

Foram feitas duas correcoes estruturais locais sem alterar ganhos:

```text
1. cmd_vel XY passou a ser transformado de frame global para frame body usando yaw.
2. termo derivativo XY passou a usar -velocidade medida por odometria para referencia constante.
```

Essas correcoes nao foram suficientes para aprovar o tracking horizontal.

## Classificacao

```text
BASE DE TEMPO: CORRETA
CONTROLADOR BASICO: NAO APROVADO
RASTREAMENTO VERTICAL: PARCIAL / NAO APROVADO COMO HOVER COMPLETO
RASTREAMENTO X: NAO APROVADO
RASTREAMENTO Y: NAO APROVADO
INVARIANCIA A RTF: APROVADA PARA O EIXO Z
INFRAESTRUTURA DE WAYPOINTS/HOVER: AINDA AUSENTE
```

O controlador basico nao foi reprovado pela falta de sequenciador. Ele foi reprovado porque as referencias simples A-E nao convergiram de forma robusta no plano XY.
