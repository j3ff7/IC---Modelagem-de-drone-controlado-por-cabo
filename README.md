# Drone controlado por cabo

Este branch organiza a simulacao do drone conectado a um cabo flexivel no Gazebo e a infraestrutura de testes para validar:

- estabilidade do controlador de posicao em hovering;
- efeito da tensao do cabo sobre o controle;
- medicao de azimuth e elevation do cabo no frame do drone;
- comparacao com casos geometricos conhecidos.

O foco atual e validar primeiro casos simples com um unico waypoint, especialmente com cabo folgado (`slack tether`). Somente depois desses casos ficarem confiaveis faz sentido retomar trajetorias com multiplos waypoints.

## Pacotes

- `pacote_do_drone`: simulacao principal com drone, cabo, carretel, controlador e leitores de sensor.
- `cabo_avaliacao`: mundos estaticos com postes para validar a convencao de angulos sem a dinamica do drone.

## Build

Na raiz do workspace:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select pacote_do_drone cabo_avaliacao
source install/setup.bash
```

## Simulacao principal

Rodar sem controlador:

```bash
ros2 launch pacote_do_drone start_sim.launch.py
```

Rodar com controlador de trajetoria:

```bash
ros2 launch pacote_do_drone start_sim.launch.py \
  controlador_trajetoria:=true \
  waypoints_file:=config/trajetoria_slack_hover_centro.json
```

O controlador atual publica velocidade em `/meu_drone/cmd_vel`, mas a velocidade e gerada a partir do erro de posicao ate um waypoint. Ele usa:

- controle proporcional de posicao;
- amortecimento por velocidade estimada por diferenca de posicao;
- termo integral desativado por padrao;
- controle opcional de heading;
- histerese de chegada para nao resetar hovering por pequenos overshoots.

Parametros uteis:

```bash
controlador_trajetoria:=true
waypoints_file:=config/trajetoria_slack_hover_centro.json
tolerancia_posicao:=0.18
tolerancia_altura:=0.15
histerese_chegada:=1.6
tempo_estabilizacao:=1.0
tempo_hover:=10.0
controlar_heading:=false
heading_fixo:=0.0
cmd_vel_frame:=body
```

O plugin `gz::sim::systems::MulticopterVelocityControl` interpreta as velocidades lineares do `Twist` no frame do corpo do veiculo e `angular.z` como taxa de yaw. Quando `cmd_vel_frame:=body`, o controlador calcula a velocidade desejada no mundo e converte `x/y` para o frame do drone antes de publicar em `/meu_drone/cmd_vel`.

## Parametros fisicos atuais

Os arquivos usados pelo launch do pacote ficam em `src/pacote_do_drone`. O diretorio legado `Gazebo/` ainda existe, mas nao e a fonte principal da simulacao rodada por `ros2 launch pacote_do_drone start_sim.launch.py`.

Drone, em `src/pacote_do_drone/models/meu_drone/meu_drone.sdf`:

```text
base_link              1.500 kg
4 rotores              4 x 0.005 kg = 0.020 kg
cabo_sensor_link       0.020 kg
cabo_azimuth_link      0.010 kg
massa total SDF        1.550 kg
distancia max. rotores 0.364 m
velocityGain           4.0 4.0 6.0
attitudeGain           2.0 3.0 0.15
angularRateGain        4.0 4.0 0.5
motorConstant          1.5e-03
momentConstant         0.090
maxRotVelocity         1000 rad/s
```

Cabo, em `src/pacote_do_drone/tether_parameters.json` e gerado por `src/pacote_do_drone/models/gerar_cabo.py`:

```text
num_links              50
length por segmento    0.050 m
comprimento nominal    2.500 m
raio                   0.002 m
massa por segmento     0.005880 kg
massa dos 50 segmentos 0.294 kg
50 links dummy         50 x 0.0001 kg = 0.005 kg
raiz_cabo              0.0005 kg
ponta_cabo             0.0005 kg
massa total SDF cabo   0.300 kg
ancora                 10.000 kg, fixa ao mundo
connection_type        ball (baseline fisica provisoria)
razao cabo/drone       0.194
```

O valor `mass` do JSON e a massa de cada segmento cilindrico do cabo. A massa dinamica total do cabo inclui tambem `dummy_*`, `raiz_cabo` e `ponta_cabo`.

A conexão `ball` transmite força no ponto de conexão e permite orientação passiva do tether, evitando o momento artificial observado com a conexão `fixed`. A opção `fixed` permanece disponível apenas para diagnóstico em `tether_parameters.json`.

## Diagnostico de estabilidade

O launch possui opcoes para isolar controlador, spawn e tether sem alterar permanentemente o mundo:

```bash
usar_cabo:=true|false
prender_ancora:=true|false
velocity_test:=true|false
vz_cmd:=0.25
spawn_x:=0.0
spawn_y:=0.0
spawn_z:=2.0
spawn_yaw:=0.0
headless:=true
cmd_vel_frame:=body
```

Waypoint minimo usado nos testes:

```bash
waypoints_file:=config/trajetoria_hover_z2_unico.json
```

Sequencia recomendada:

```bash
# 1. Drone sem cabo, spawn original calculado pelo cabo.
ros2 launch pacote_do_drone start_sim.launch.py \
  controlador_trajetoria:=true \
  waypoints_file:=config/trajetoria_hover_z2_unico.json \
  usar_cabo:=false \
  cmd_vel_frame:=body \
  headless:=true

# 2. Drone sem cabo, spawn proximo ao waypoint.
ros2 launch pacote_do_drone start_sim.launch.py \
  controlador_trajetoria:=true \
  waypoints_file:=config/trajetoria_hover_z2_unico.json \
  usar_cabo:=false \
  spawn_x:=0.0 spawn_y:=0.0 spawn_z:=2.0 spawn_yaw:=0.0 \
  cmd_vel_frame:=body \
  headless:=true

# 3. Drone com cabo, spawn proximo ao waypoint.
ros2 launch pacote_do_drone start_sim.launch.py \
  controlador_trajetoria:=true \
  waypoints_file:=config/trajetoria_hover_z2_unico.json \
  usar_cabo:=true \
  spawn_x:=0.0 spawn_y:=0.0 spawn_z:=2.0 spawn_yaw:=0.0 \
  cmd_vel_frame:=body \
  headless:=true

# 4. Drone com cabo, spawn original.
ros2 launch pacote_do_drone start_sim.launch.py \
  controlador_trajetoria:=true \
  waypoints_file:=config/trajetoria_hover_z2_unico.json \
  usar_cabo:=true \
  cmd_vel_frame:=body \
  headless:=true
```

Resultados observados nesta revisao:

```text
Sem cabo + spawn original: concluiu; erro final aprox. (0.04, 0.01, 0.00) m; roll/pitch max ~4.8 deg.
Sem cabo + spawn proximo:  concluiu; hover estavel em (0, 0, 2) m; roll/pitch ~0 deg.
Com cabo + spawn proximo:  concluiu; erro final aprox. (0.03, 0.00, 0.10) m; tensao no drone ate ~0.35 N.
Com cabo + spawn original: nao concluiu no tempo testado; pitch perto de +/-90 deg por longos periodos; tensao no drone ate ~6 N.
```

A causa mais provavel da falha de hovering nos testes anteriores e a combinacao de erro inicial grande com o cabo inicialmente quase esticado, o que induz atitudes muito grandes no drone. Nessas atitudes, o comando vertical do controlador de velocidade fica pouco efetivo. Antes de ajustar ganhos, valide sempre o caso sem cabo e o caso com cabo usando spawn proximo ao waypoint.

### Teste aberto de velocidade vertical

Para separar o controlador de posicao da resposta do plugin do multicopter, use o no `velocity_test`. Ele publica um `Twist` constante em `/meu_drone/cmd_vel`, mede odometria, RPY, rotores, forca/momento na conexao cabo-drone e margem das juntas internas do cabo. A duração e os logs usam tempo simulado via `/clock`; `t_wall` aparece apenas para indicar o custo real/RTF.

```bash
ros2 launch pacote_do_drone start_sim.launch.py \
  velocity_test:=true \
  controlador_trajetoria:=false \
  usar_cabo:=true \
  prender_ancora:=true \
  headless:=true \
  spawn_x:=2.0 spawn_y:=0.0 spawn_z:=0.33 spawn_yaw:=0.0 \
  vz_cmd:=0.25 \
  velocity_test_duracao:=8.0 \
  log_periodo:=0.5 \
  janela_tangente_metros:=0.15
```

Variantes importantes:

```bash
# Sem cabo: resposta vertical esperada do drone/controlador interno.
usar_cabo:=false

# Cabo conectado ao drone, mas raiz livre: separa efeito da ancora fixa.
usar_cabo:=true prender_ancora:=false
```

Resultado atual do caso acima com tether `ball`, `L=2.5 m` e massa total `0.30 kg`: o comando chega ao Gazebo (`cmd_z_pub=0.25`) e o drone sobe de `z ~= 0.33 m` para `z ~= 0.60 m` em cerca de `2 s` simulados, com `pitch` pequeno e `|M|=0`.

Sem tether, o mesmo teste roda próximo do tempo real. Com o tether de 50 links, o RTF cai bastante; por isso avalie sempre `t_sim`, não o tempo de parede.

### Métricas de hover

Para registrar estabilidade de posição, tensão e ângulos sem poluir o terminal:

```bash
ros2 launch pacote_do_drone start_sim.launch.py \
  controlador_trajetoria:=true \
  hover_metrics:=true \
  waypoints_file:=config/trajetoria_teste_z100.json \
  metricas_target_x:=2.0 metricas_target_y:=0.0 metricas_target_z:=1.0 \
  metricas_inicio_s:=0.0 \
  metricas_duracao_s:=10.0 \
  metricas_janela_final_s:=5.0 \
  headless:=true
```

O nó imprime `RTF`, erro, tensão e um resumo final com média/desvio de posição, roll/pitch, tensão, azimuth e elevation. O sensor usa por padrão uma janela física de `0.15 m` perto da ponta do cabo (`janela_tangente_metros`) para estimar a tangente local; se esse valor for `0`, ele volta ao fallback `janela_tangente_links`.

### Diagnostico da geometria inicial do cabo

O gerador `src/pacote_do_drone/models/gerar_cabo.py` aceita parametros opcionais em `src/pacote_do_drone/tether_parameters.json`:

```json
"initial_shape": "sine_slack",
"initial_end_x": 2.0,
"initial_end_y": 0.0,
"initial_end_z": 0.33
```

Esses campos definem a extremidade superior usada para gerar a geometria inicial do cabo e o spawn padrao do drone no launch diagnostico. Com a configuracao permanente atual:

```text
ancora         = (0.0, 0.0, 0.33) m
spawn drone    = (2.0, 0.0, 0.33) m
comprimento    = 2.5 m
folga inicial  = 0.5 m
```

Para evitar que a folga atravesse o chao, o cabo nao e inicializado em uma senoidal vertical. O gerador usa uma senoidal horizontal no lado `+y`:

```text
x(s) = 2 s
y(s) = A sin(pi s)
z(s) = 0.33 m
```

A amplitude `A` e calculada numericamente para que a cadeia tenha o comprimento configurado. Para a configuracao atual, `A ~= 0.6921 m`, `z_min = 0.330 m` e o erro de comprimento e praticamente zero.

Essa curvatura inicial e aplicada com `<initial_position>` nas juntas revolutas do cabo, com `spring_reference` igual ao angulo inicial. Tentativas anteriores de colocar a curvatura como pose/rotacao estrutural dos links causavam abort do DART.

Gerar o cabo e imprimir os pontos de verificacao:

```bash
python3 src/pacote_do_drone/models/gerar_cabo.py
```

O gerador imprime os pontos `0`, `N/4`, `N/2`, `3N/4` e `N`. Na configuracao atual, eles devem ficar todos em `z = 0.330 m`, com o ponto central deslocado em `+y`.

Testes comparativos recentes:

```text
L=2.0 m:
  folga = 0.0 m
  tensao maxima drone/ancora = 6.32 / 6.92 N
  pitch maximo = 86.6 deg

L=2.5 m:
  folga = 0.5 m
  amplitude lateral = 0.6921 m
  tensao maxima drone/ancora = 2.62 / 2.86 N
  pitch maximo = 30.2 deg

L=3.0 m:
  folga = 1.0 m
  amplitude lateral = 1.0482 m
  tensao maxima drone/ancora = 8.30 / 36.32 N
  pitch maximo = 15.2 deg
```

O melhor caso de assentamento ate agora e `L=2.5 m`, mas ele ainda nao e um hover nivelado perfeito. O proximo problema a investigar e o pitch sustentado de aproximadamente 30 graus antes de retomar subidas ou trajetorias.

Teste de assentamento recomendado:

```bash
ros2 launch pacote_do_drone start_sim.launch.py \
  controlador_trajetoria:=true \
  usar_cabo:=true \
  headless:=true \
  waypoints_file:=config/trajetoria_assentamento_spawn.json \
  cmd_vel_frame:=body \
  tempo_hover:=5.0 \
  log_periodo:=0.5
```

Teste de subida curta, ainda diagnostico:

```bash
ros2 launch pacote_do_drone start_sim.launch.py \
  controlador_trajetoria:=true \
  usar_cabo:=true \
  headless:=true \
  waypoints_file:=config/trajetoria_subida_curta_spawn.json \
  cmd_vel_frame:=body \
  tempo_hover:=5.0 \
  limite_vel_xy:=0.25 \
  limite_vel_z:=0.25 \
  log_periodo:=0.5
```

Resultado observado na subida curta com `L=2.5 m`: o drone nao atingiu `z = 0.60 m` dentro da janela testada; houve saturacao frequente do comando vertical, tensao maxima de `3.59/4.76 N` e pitch maximo de `36.5 deg`.

## Casos com cabo folgado

O cabo nominal do branch atual tem `2.5 m`, mas varios casos historicos de validacao angular foram definidos para o cabo de `2.0 m`. Antes de usar os casos slack direcionais abaixo como comparacao quantitativa, confirme o comprimento ativo em `tether_parameters.json` e regenere o cabo.

Caso inicial de hovering, sem controle de heading:

```bash
ros2 launch pacote_do_drone start_sim.launch.py \
  controlador_trajetoria:=true \
  waypoints_file:=config/trajetoria_slack_hover_centro.json
```

Casos slack para validar angulos com heading conhecido (`heading_fixo = pi/2 rad`):

```text
config/trajetoria_slack_e.json
config/trajetoria_slack_ne.json
config/trajetoria_slack_n.json
config/trajetoria_slack_nw.json
config/trajetoria_slack_w.json
config/trajetoria_slack_sw.json
config/trajetoria_slack_s.json
config/trajetoria_slack_se.json
```

Exemplo:

```bash
ros2 launch pacote_do_drone start_sim.launch.py \
  controlador_trajetoria:=true \
  waypoints_file:=config/trajetoria_slack_s.json
```

Valores geometricos esperados:

```bash
src/pacote_do_drone/config/angulos_slack_esperados.json
```

Resumo, para os casos com raio XY de 1.0 m, `base_link_z = 2.0` e sensor 5 cm abaixo do `base_link`:

```text
caso   azimuth [deg]   elevation [deg]
e          90.00            58.31
ne        135.00            58.31
n         180.00            58.31
nw       -135.00            58.31
w         -90.00            58.31
sw        -45.00            58.31
s           0.00            58.31
se         45.00            58.31
```

No caso `slack_hover_centro`, o cabo esta geometricamente vertical em relacao a ancora. A elevacao esperada e 90 graus e o azimuth e indefinido.

Os casos direcionais usam primeiro o waypoint intermediario `(0.0, 0.0, 0.5)` e depois o waypoint final em raio horizontal de 1.0 m. No caso `trajetoria_slack_n.json`, o waypoint final e `(0.0, 1.0, 2.0)`. Com ancora em `(0.0, 0.0, 0.33)` e sensor 5 cm abaixo do `base_link`, a distancia reta sensor-ancora no ponto final e aproximadamente `1.904 m`, deixando cerca de `0.096 m` de folga em relacao ao comprimento nominal de `2.0 m`.

O waypoint intermediario em `z = 0.5 m` e usado como passagem curta, com `tempo_hover = 1.0 s` nos casos direcionais. Ele e geometricamente seguro para o drone, mas nao deve ser usado como ponto de repouso estatico longo do cabo: se o cabo de 2.0 m fosse deixado em equilibrio com as duas extremidades quase alinhadas verticalmente, haveria folga suficiente para tocar o chao.

Teste atual com cabo de 300 g: `trajetoria_slack_n.json` ainda nao atingiu o waypoint intermediario. O gerador atual inicializa o drone em `(2.0, 0.0, 0.33)`, com o cabo reto no comprimento total, enquanto o waypoint intermediario pedido e `(0.0, 0.0, 0.5)`. Assim, o primeiro movimento nao e uma subida vertical a partir do spawn; ele exige um deslocamento horizontal de aproximadamente 2 m em direcao a ancora. Em uma janela de teste de 125 s, o controlador permaneceu no `alvo[0]`, nao publicou `Avancando para waypoint 1`, teve erro XY ate cerca de `2.00 m`, `z` entre `0.06` e `0.39 m`, tensao maxima no drone de `6.40 N`, roll/pitch maximos de aproximadamente `156.6/86.9 deg` e saturacao XY em 59 de 100 amostras. O proximo ajuste recomendado e tornar a condicao inicial do cabo/drone compatível com o primeiro waypoint, ou criar uma rotina de spawn/acomodacao especifica para testes slack.

Para sanidade do controlador, use primeiro:

```bash
ros2 launch pacote_do_drone start_sim.launch.py \
  controlador_trajetoria:=true \
  waypoints_file:=config/trajetoria_slack_hover_centro.json
```

Depois avance para casos laterais. Uma melhoria futura importante e gerar a geometria inicial do cabo ja compatível com o waypoint de teste ou levar o drone ate o ponto por uma fase de acomodacao antes de medir os angulos.

## Convencao do sensor de angulo

Os angulos sao definidos no frame local do drone/sensor:

- `x`: frente;
- `y`: esquerda;
- `z`: cima;
- `azimuth = atan2(y, x)`;
- `elevation = atan2(-z, sqrt(x^2 + y^2))`.

Com essa convencao:

- cabo descendo verticalmente: `elevation = 90 deg`;
- cabo horizontal para frente: `azimuth = 0 deg`, `elevation = 0 deg`;
- cabo horizontal para a esquerda: `azimuth = 90 deg`;
- cabo horizontal para a direita: `azimuth = -90 deg`.

Os topicos principais do drone sao calculados expressando a tangente local do cabo no frame do drone. Eles medem a orientacao do cabo no lado do drone, perto da ponta presa ao sensor:

```bash
ros2 topic echo /cabo/azimuth_graus
ros2 topic echo /cabo/elevation_graus
ros2 topic echo /cabo/drone/azimuth_graus
ros2 topic echo /cabo/drone/elevation_graus
```

Os quatro topicos acima publicam o mesmo par de valores. Os nomes sem `/drone/` foram mantidos por compatibilidade.

Diagnosticos geometricos usando o vetor direto sensor-ancora, tambem expresso no frame do drone:

```bash
ros2 topic echo /cabo/azimuth_ancora_graus
ros2 topic echo /cabo/elevation_ancora_graus
ros2 topic echo /cabo/drone/reta_ancora/azimuth_graus
ros2 topic echo /cabo/drone/reta_ancora/elevation_graus
```

Apesar do nome antigo conter `ancora`, esses topicos nao representam a tangente fisica do cabo no carretel. Eles representam a reta ideal do sensor do drone ate a ancora fixa. Os nomes com `/drone/reta_ancora/` sao os mais explicitos.

Tangente fisica no lado da ancora/carretel, calculada no frame global:

```bash
ros2 topic echo /cabo/ancora/azimuth_graus
ros2 topic echo /cabo/ancora/elevation_graus
```

Para esses topicos da ancora, `x` e `y` sao os eixos globais do mundo e a elevacao e positiva para cima. Portanto eles nao devem ser comparados diretamente com a elevacao do drone sem levar em conta o frame e o sentido do vetor.

Diagnosticos das juntas simplificadas:

```bash
ros2 topic echo /cabo/azimuth_joint_graus
ros2 topic echo /cabo/elevation_joint_graus
```

Monitor no terminal:

```bash
ros2 run pacote_do_drone cabo_monitor
```

A saida resumida usa:

```text
Drone tangente: az=... deg el=... deg | Drone reta->ancora: az=... deg el=... deg | Ancora tangente: az=... deg el=... deg
```

Gravar CSV:

```bash
ros2 run pacote_do_drone cabo_monitor -- --csv /tmp/cabo_angulos.csv
```

## Observacao sobre o joystick fisico

No sistema real, o sensor sera implementado como um joystick de 2 graus de liberdade preso ao drone, com o cabo conectado ao eixo movel.

Na simulacao atual, o requisito principal e medir de forma consistente a orientacao do cabo em relacao ao frame do drone. Uma tentativa de representar literalmente a cadeia `drone -> joystick 2 GDL -> cabo -> ancora -> mundo -> drone` cria uma malha cinematica fechada no Gazebo/DART, que nao e aceita por joints SDF comuns. Por isso, a validacao numerica atual deve priorizar os topicos geometricos:

```bash
/cabo/azimuth_graus
/cabo/elevation_graus
/cabo/drone/azimuth_graus
/cabo/drone/elevation_graus
/cabo/drone/reta_ancora/azimuth_graus
/cabo/drone/reta_ancora/elevation_graus
/cabo/ancora/azimuth_graus
/cabo/ancora/elevation_graus
```

Os topicos `*_joint_graus` continuam disponiveis como diagnostico, mas nao devem ser tratados como a referencia principal enquanto nao houver uma constraint/plugin especifico para representar a conexao fisica sem fechar a arvore de joints.

## Casos equivalentes aos postes

Tambem existem arquivos que reproduzem as posicoes dos testes estaticos dos postes:

```text
config/trajetoria_poste_e.json
config/trajetoria_poste_ne.json
config/trajetoria_poste_n.json
config/trajetoria_poste_nw.json
config/trajetoria_poste_w.json
config/trajetoria_poste_sw.json
config/trajetoria_poste_s.json
config/trajetoria_poste_se.json
config/trajetoria_poste_c1.json
```

Esses casos sao uteis para comparar convencao de angulos, mas podem tensionar mais o cabo. Use primeiro os casos `slack_*`.

## Avaliacao estatica com postes

Gerar mundos:

```bash
ros2 run cabo_avaliacao gerar_mundos --todos --modo-cabo reto
```

Rodar um caso:

```bash
ros2 launch cabo_avaliacao avaliar_cabo.launch.py caso:=s modo_cabo:=reto
```

Com cabo articulado:

```bash
ros2 launch cabo_avaliacao avaliar_cabo.launch.py caso:=s modo_cabo:=articulado
```

Com catenaria estatica aproximada:

```bash
ros2 launch cabo_avaliacao avaliar_cabo.launch.py caso:=c1 modo_cabo:=catenaria
```

## Cuidados

Nao rode varias simulacoes ao mesmo tempo, porque elas publicam nos mesmos topicos. Antes de repetir um teste:

```bash
pgrep -af 'ign gazebo|parameter_bridge|movimento_circular|sensores'
```

Se necessario, encerre processos antigos:

```bash
pkill -f 'ros2 launch pacote_do_drone'
pkill -f 'ign gazebo'
pkill -f 'parameter_bridge'
```

## Repositorio remoto

Remote SSH atual:

```bash
git@github.com:j3ff7/Drone_controlado_por_cabo.git
```

Endereco HTTP equivalente:

```bash
https://github.com/j3ff7/Drone_controlado_por_cabo.git
```
