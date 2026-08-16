# Testes do sensor de angulo do cabo

Este pacote cria mundos simples para validar os calculos de azimuth e elevation do cabo antes de usar o sensor na simulacao com drone.

## Convencao de angulos

O vetor do cabo e expresso no referencial local do sensor/drone:

- `x`: frente
- `y`: esquerda
- `z`: cima

Com essa convencao:

- `azimuth = atan2(y, x)`
- `elevation = atan2(-z, sqrt(x^2 + y^2))`
- cabo descendo verticalmente no referencial do sensor: `elevation = 90 graus`
- cabo horizontal para frente: `azimuth = 0 graus`, `elevation = 0 graus`
- cabo horizontal para a esquerda: `azimuth = 90 graus`
- cabo horizontal para a direita: `azimuth = -90 graus`

Nos mundos de avaliacao, o sensor do poste usa `sensor_yaw_graus = 90`, entao o eixo local `x` do sensor aponta para o norte do mundo e o eixo local `y` aponta para oeste.

## Arquivo de configuracao

O arquivo padrao fica em:

```bash
src/cabo_avaliacao/config/postes_padrao.json
```

Campos principais:

```json
{
  "cabo_comprimento": 2.0,
  "poste_altura": 1.2,
  "ancora": [0.0, 0.0, 0.05],
  "sensor_yaw_graus": 90.0,
  "casos": {
    "s": {"x": 0.0, "y": -1.6}
  }
}
```

Para criar um teste novo, copie esse JSON, mude `poste_altura` e as coordenadas dos casos em `casos`, e passe o caminho com `config:=...`.

## Build

Na raiz do workspace:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select pacote_do_drone cabo_avaliacao
source install/setup.bash
```

## Gerar mundos sem abrir o Gazebo

Gerar um caso:

```bash
ros2 run cabo_avaliacao gerar_mundos --caso s --modo-cabo reto
```

Gerar todos os casos:

```bash
ros2 run cabo_avaliacao gerar_mundos --todos --modo-cabo articulado
```

Usar configuracao customizada:

```bash
ros2 run cabo_avaliacao gerar_mundos --caso teste1 --modo-cabo articulado --config /caminho/para/postes_custom.json
```

Os mundos e a tabela de angulos esperados sao escritos em:

```bash
/tmp/cabo_avaliacao_worlds
```

## Rodar avaliacao no Gazebo

Cabo rigido de referencia:

```bash
ros2 launch cabo_avaliacao avaliar_cabo.launch.py caso:=s modo_cabo:=reto
```

Cabo articulado com links e joints do `cabo.sdf`:

```bash
ros2 launch cabo_avaliacao avaliar_cabo.launch.py caso:=s modo_cabo:=articulado
```

Representacao geometrica de catenaria para casos com folga:

```bash
ros2 launch cabo_avaliacao avaliar_cabo.launch.py caso:=c1 modo_cabo:=catenaria config:=/caminho/para/postes_custom.json
```

Com configuracao customizada:

```bash
ros2 launch cabo_avaliacao avaliar_cabo.launch.py caso:=teste1 modo_cabo:=articulado config:=/caminho/para/postes_custom.json
```

O avaliador imprime linhas como:

```text
OK | sensor sensor_cabo | az   90.00 esper   90.00 erro    0.00 | el   36.87 esper   36.87 erro    0.00
```

## Modos do cabo

`reto`: cria um unico segmento reto entre a ancora e o sensor. E o melhor modo para conferir sinais e convencao de angulos.

`articulado`: carrega o mesmo `cabo.sdf` usado na simulacao com drone, com links e joints, mas de forma estatica. Esse modo valida nomes, frames e a tangente do `final_segment`; ele nao calcula barriga. Se o poste estiver mais perto que o comprimento nominal do cabo, o cabo continuara reto e passara alem do sensor.

`catenaria`: cria uma catenaria estatica aproximada por varios segmentos entre a ancora e o sensor. Esse modo usa `cabo_comprimento`, `poste_altura`, `ancora` e a posicao do caso no JSON. O ultimo segmento e usado como tangente local no sensor para calcular os angulos esperados.

## Topicos publicados

```bash
ros2 topic echo /cabo_avaliacao/azimuth_graus
ros2 topic echo /cabo_avaliacao/elevation_graus
ros2 topic echo /cabo_avaliacao/erro_azimuth_graus
ros2 topic echo /cabo_avaliacao/erro_elevation_graus
```

Se o `rqt_plot` estiver instalado:

```bash
rqt_plot /cabo_avaliacao/azimuth_graus/data /cabo_avaliacao/elevation_graus/data
```

Se nao estiver:

```bash
sudo apt install ros-humble-rqt-plot
```

## Simulacao com drone

Rodar a simulacao principal:

```bash
ros2 launch pacote_do_drone start_sim.launch.py
```

Rodar a simulacao principal com controlador de sequencia de waypoints:

```bash
ros2 launch pacote_do_drone start_sim.launch.py \
  controlador_trajetoria:=true \
  waypoints_file:=config/trajetoria_hover_ancora.json \
  tempo_hover:=10.0 \
  repetir:=false \
  tolerancia_posicao:=0.18 \
  heading_fixo:=0.0
```

O arquivo padrao `src/pacote_do_drone/config/trajetoria_hover_ancora.json` define um unico waypoint em `(0.0, 0.0, 1.6)`, verticalmente acima da ancora do cabo, que fica em `x=0, y=0` no plano. Esse e o teste de sanidade inicial: o drone deve convergir para o ponto e permanecer nele.

Depois que esse caso estiver estavel, use o arquivo `src/pacote_do_drone/config/trajetoria_drone_padrao.json`, que define dois pontos simetricos em torno da ancora:

```bash
ros2 launch pacote_do_drone start_sim.launch.py \
  controlador_trajetoria:=true \
  waypoints_file:=config/trajetoria_drone_padrao.json \
  tempo_hover:=3.0 \
  repetir:=true
```

Para passar os pontos diretamente pela linha de comando:

```bash
ros2 launch pacote_do_drone start_sim.launch.py \
  controlador_trajetoria:=true \
  waypoints:='[[0.8, 0.0, 1.6], [-0.8, 0.0, 1.6]]' \
  waypoints_file:='' \
  tempo_hover:=5.0 \
  repetir:=true
```

Cada waypoint pode ser uma lista `[x, y, z]` ou um objeto `{"x": 0.8, "y": 0.0, "z": 1.6}`. Se `z` for omitido, o controlador usa `altura_trajetoria`. Quando `waypoints` e `waypoints_file` forem informados ao mesmo tempo, a lista passada em `waypoints` tem prioridade.

O controlador so avanca para o proximo ponto depois que o drone fica dentro das tolerancias de posicao, altura e velocidade durante o tempo de hovering. Ele usa controle PD em posicao, com termo integral opcional para ensaios posteriores. Se o drone se comportar como se o comando de velocidade estivesse no frame do corpo, rode com:

No caso de sanidade com um waypoint, o controle de heading fica desativado por padrao (`controlar_heading:=false`) para evitar acoplamento entre yaw e posicao enquanto a resposta basica ainda esta sendo avaliada. O amortecimento usa velocidade estimada por diferenca de posicao (`usar_velocidade_por_diferenca:=true`), pois o `twist` da odometria pode ser ambiguo quanto ao frame. O controlador tambem usa histerese de chegada (`histerese_chegada:=1.6`) para nao zerar o tempo de hovering por pequenos overshoots causados pela dinamica do cabo.

```bash
ros2 launch pacote_do_drone start_sim.launch.py controlador_trajetoria:=true cmd_vel_frame:=body
```

## Validacao com drone nas posicoes dos postes

Para reproduzir com o drone os casos estaticos dos postes, use um arquivo de waypoint por vez. Nesses arquivos, o drone usa `heading_fixo = pi/2 rad`, entao o eixo local `x` do drone aponta para o norte do mundo e o eixo local `y` aponta para oeste, igual ao sensor dos postes.

O ponto de conexao do cabo fica 5 cm abaixo do `base_link`. Por isso, os arquivos comandam `base_link_z = 1.58`, fazendo o sensor/conexao ficar em `z = 1.53`, isto e, 1.20 m acima da ancora do carretel em `z = 0.33`.

Exemplo para o caso norte:

```bash
ros2 launch pacote_do_drone start_sim.launch.py \
  controlador_trajetoria:=true \
  waypoints_file:=config/trajetoria_poste_n.json
```

Troque o sufixo do arquivo para testar cada caso:

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

Angulos esperados para comparacao:

```text
caso   azimuth [deg]   elevation [deg]
e          90.00            36.87
ne        135.00            36.87
n         180.00            36.87
nw       -135.00            36.87
w         -90.00            36.87
sw        -45.00            36.87
s           0.00            36.87
se         45.00            36.87
c1        135.00            37.65
```

A mesma tabela esta em:

```bash
src/pacote_do_drone/config/angulos_postes_esperados.json
```

Topicos principais:

```bash
ros2 topic echo /cabo/azimuth_graus
ros2 topic echo /cabo/elevation_graus
```

Topicos diagnosticos:

```bash
ros2 topic echo /cabo/azimuth_ancora_graus
ros2 topic echo /cabo/elevation_ancora_graus
ros2 topic echo /cabo/azimuth_joint_graus
ros2 topic echo /cabo/elevation_joint_graus
```

Use primeiro os diagnosticos da ancora para conferir a geometria nominal. Em seguida compare os topicos principais, que usam a tangente local do cabo perto do sensor quando os links do cabo estao disponiveis.

Observacao sobre a representacao fisica do joystick: o Gazebo/DART nao aceita, com joints SDF comuns, uma malha cinemática fechada do tipo `base_link do drone -> joystick 2 GDL -> ponta do cabo -> cabo -> ancora -> mundo -> drone`. Ao tentar representar o sensor exatamente como uma cadeia presa ao drone e ao cabo, o simulador recusa o modelo porque o link terminal passa a ter dois joints pais. Por isso, a validacao inicial deve usar os topicos geometricos calculados no frame do drone (`/cabo/azimuth_graus` e `/cabo/elevation_graus`). Os topicos `*_joint_graus` permanecem como diagnostico da articulacao simplificada, mas nao devem ser a referencia principal de validacao numerica ate trocarmos a conexao por uma constraint/plugin adequado.

O mesmo calculo de azimuth/elevation e usado pelo drone e pelo avaliador dos postes. No drone, os topicos sao:

```bash
ros2 topic echo /cabo/azimuth_graus
ros2 topic echo /cabo/elevation_graus
```

Para monitorar no terminal:

```bash
ros2 run pacote_do_drone cabo_monitor
```

## Cuidados

Nao rode varios mundos `cabo_avaliacao` ao mesmo tempo, porque todos usam o mesmo nome de mundo e os mesmos topicos. Se os valores parecerem misturados, cheque processos antigos:

```bash
pgrep -af 'ign gazebo|parameter_bridge|ros2 launch cabo_avaliacao|avaliador'
```

Encerre os processos antigos antes de repetir o teste.
