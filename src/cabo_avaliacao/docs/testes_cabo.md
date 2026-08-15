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
