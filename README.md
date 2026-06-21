# Modelagem de Drone Controlado por Cabo

Projeto de Iniciação Científica voltado à modelagem, simulação e análise de um drone controlado por cabo, também conhecido como *tethered drone*. O objetivo é representar o comportamento dinâmico do drone, do cabo flexível e do sistema de ancoragem/carretel em ambiente de simulação.

O desenvolvimento principal do projeto está concentrado na pasta:

```text
src/pacote_do_drone
```

Para manter compatibilidade com os caminhos usados nos scripts, recomenda-se clonar este repositório com o nome de pasta `IC`.

---

## Objetivos do projeto

* Modelar um drone conectado a um cabo flexível.
* Simular o comportamento do cabo no Gazebo.
* Integrar a simulação com ROS 2.
* Ler dados de tensão e ângulos do cabo por tópicos ROS.
* Permitir ajustes de parâmetros físicos do cabo por arquivo JSON.
* Servir como base para estudos de controle, carretel, guincho e dinâmica de drones controlados por cabo.

---

## Tecnologias utilizadas

* Ubuntu Linux
* ROS 2 Jazzy
* Gazebo Sim
* `ros_gz_sim`
* `ros_gz_bridge`
* Python 3
* Colcon
* SDF/URDF/Xacro

---

## Estrutura principal

```text
IC/
├── README.md
├── src/
│   └── pacote_do_drone/
│       ├── launch/
│       │   └── start_sim.launch.py
│       ├── models/
│       │   ├── gerar_cabo.py
│       │   ├── cabo.sdf
│       │   ├── cabo.urdf
│       │   ├── cabo.urdf.xacro
│       │   ├── Gazebo/
│       │   └── meu_drone/
│       ├── pacote_do_drone/
│       │   ├── __init__.py
│       │   └── sensores.py
│       ├── resource/
│       ├── test/
│       ├── worlds/
│       │   └── my_world.sdf
│       ├── package.xml
│       ├── setup.cfg
│       ├── setup.py
│       └── tether_parameters.json
```

---

## Arquivos importantes

| Arquivo                                           | Função                                                    |
| ------------------------------------------------- | --------------------------------------------------------- |
| `src/pacote_do_drone/models/gerar_cabo.py`        | Gera os arquivos do cabo e atualiza o mundo de simulação. |
| `src/pacote_do_drone/tether_parameters.json`      | Define parâmetros físicos e geométricos do cabo.          |
| `src/pacote_do_drone/worlds/my_world.sdf`         | Mundo usado na simulação do Gazebo.                       |
| `src/pacote_do_drone/launch/start_sim.launch.py`  | Inicializa o Gazebo e a ponte ROS-Gazebo.                 |
| `src/pacote_do_drone/pacote_do_drone/sensores.py` | Nó ROS que lê tensão e ângulos do cabo.                   |
| `src/pacote_do_drone/models/meu_drone/`           | Modelo do drone utilizado na simulação.                   |

---

## Parâmetros do cabo

Os parâmetros principais ficam em:

```text
src/pacote_do_drone/tether_parameters.json
```

Exemplo:

```json
{
  "num_links": 40,
  "length": 0.05,
  "radius": 0.002,
  "mass": 0.002,
  "drone_x": 0.5,
  "drone_y": 0.2
}
```

| Parâmetro   | Significado                          |
| ----------- | ------------------------------------ |
| `num_links` | Número de elos/segmentos do cabo.    |
| `length`    | Comprimento de cada elo, em metros.  |
| `radius`    | Raio do cabo, em metros.             |
| `mass`      | Massa de cada elo, em kg.            |
| `drone_x`   | Posição desejada do drone no eixo X. |
| `drone_y`   | Posição desejada do drone no eixo Y. |

Sempre que alterar esses parâmetros, execute novamente o script de geração do cabo.

---

## Instalação

### 1. Clone o repositório como `IC`

Use o comando abaixo para que o repositório seja clonado diretamente com o nome da pasta `IC`:

```bash
git clone https://github.com/j3ff7/IC---Modelagem-de-drone-controlado-por-cabo.git IC
cd IC
```

Com isso, a estrutura ficará parecida com:

```text
/home/joseubu/IC/
└── src/
    └── pacote_do_drone/
```

Isso ajuda a manter compatibilidade com caminhos usados no projeto, como:

```text
/home/joseubu/IC/src/pacote_do_drone/...
```

---

### 2. Caso já tenha clonado com o nome original

Se você já clonou o repositório com o nome completo:

```text
IC---Modelagem-de-drone-controlado-por-cabo
```

você pode apenas renomear a pasta:

```bash
mv IC---Modelagem-de-drone-controlado-por-cabo IC
cd IC
```

---

### 3. Instale as dependências principais

Exemplo para ROS 2 Jazzy:

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions
sudo apt install ros-jazzy-ros-gz ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge
```

Também é necessário ter o Gazebo Sim instalado e configurado corretamente.

---

### 4. Compile o workspace

Na raiz do repositório:

```bash
cd ~/IC
colcon build --symlink-install
```

Depois carregue o ambiente:

```bash
source install/setup.bash
```

Para evitar repetir esse comando em todo terminal, adicione ao `~/.bashrc`:

```bash
echo "source ~/IC/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## Atenção sobre caminhos absolutos

Alguns arquivos do projeto podem conter caminhos absolutos do ambiente de desenvolvimento, por exemplo:

```text
/home/joseubu/IC/src/pacote_do_drone/...
```

Por isso, recomenda-se clonar o repositório como `IC`, conforme mostrado anteriormente:

```bash
git clone https://github.com/j3ff7/IC---Modelagem-de-drone-controlado-por-cabo.git IC
```

Assim, os caminhos absolutos continuam funcionando no ambiente esperado.

Em outro computador ou com outro nome de usuário, será necessário ajustar esses caminhos nos scripts.

Arquivos que podem precisar de ajuste:

```text
src/pacote_do_drone/models/gerar_cabo.py
src/pacote_do_drone/launch/start_sim.launch.py
```

Uma melhoria futura recomendada é substituir caminhos absolutos por caminhos relativos ao pacote ROS, usando `get_package_share_directory`.

---

## Como executar

### 1. Entrar na pasta do projeto

```bash
cd ~/IC
```

---

### 2. Carregar o ambiente do ROS 2

```bash
source /opt/ros/jazzy/setup.bash
```

---

### 3. Gerar o cabo e o mundo

Sempre que os parâmetros do cabo forem alterados, execute:

```bash
python3 src/pacote_do_drone/models/gerar_cabo.py
```

Esse script gera ou atualiza arquivos como:

```text
src/pacote_do_drone/models/cabo.urdf
src/pacote_do_drone/models/cabo.sdf
src/pacote_do_drone/worlds/my_world.sdf
```

---

### 4. Compilar o projeto

```bash
colcon build --symlink-install
```

Depois carregue o ambiente do workspace:

```bash
source install/setup.bash
```

---

### 5. Iniciar a simulação

```bash
ros2 launch pacote_do_drone start_sim.launch.py
```

Esse comando abre o Gazebo com o mundo configurado e inicia a ponte entre Gazebo e ROS.

---

### 6. Ler sensores do cabo

Em outro terminal:

```bash
cd ~/IC
source install/setup.bash
ros2 run pacote_do_drone sensores
```

O nó `sensores` lê os tópicos de tensão e ângulos do cabo e imprime os valores no terminal.

---

## Tópicos principais

A simulação utiliza uma ponte entre Gazebo e ROS para publicar e receber dados nos seguintes tópicos:

| Tópico               | Tipo ROS                          | Função                              |
| -------------------- | --------------------------------- | ----------------------------------- |
| `/tensao_cabo`       | `geometry_msgs/msg/WrenchStamped` | Tensão/força no cabo.               |
| `/angulos_cabo`      | `sensor_msgs/msg/JointState`      | Ângulos das juntas do cabo.         |
| `/meu_drone/cmd_vel` | `geometry_msgs/msg/Twist`         | Comando de velocidade para o drone. |

Para verificar os tópicos ativos:

```bash
ros2 topic list
```

Para visualizar dados de um tópico:

```bash
ros2 topic echo /tensao_cabo
ros2 topic echo /angulos_cabo
```

---

## Fluxo recomendado de uso

Use este fluxo após clonar o projeto como `IC`:

```bash
cd ~/IC

source /opt/ros/jazzy/setup.bash

python3 src/pacote_do_drone/models/gerar_cabo.py

colcon build --symlink-install

source install/setup.bash

ros2 launch pacote_do_drone start_sim.launch.py
```

Em outro terminal, para ler os sensores:

```bash
cd ~/IC

source install/setup.bash

ros2 run pacote_do_drone sensores
```

---

## Problemas comuns

### O pacote não é encontrado pelo ROS 2

Verifique se o workspace foi compilado e se o ambiente foi carregado:

```bash
cd ~/IC
colcon build --symlink-install
source install/setup.bash
```

---

### O Gazebo não encontra modelos

Verifique se os arquivos de modelos existem em:

```text
src/pacote_do_drone/models/
```

Também confira se o modelo do drone está em:

```text
src/pacote_do_drone/models/meu_drone/
```

---

### Erro por caminho absoluto

Se aparecer erro envolvendo caminhos como:

```text
/home/joseubu/IC/...
```

verifique se o repositório realmente está na pasta correta:

```bash
cd ~/IC
pwd
```

O resultado esperado é algo como:

```text
/home/joseubu/IC
```

Se o repositório estiver com outro nome, renomeie:

```bash
cd ~
mv IC---Modelagem-de-drone-controlado-por-cabo IC
```

---

### Alterei o JSON, mas a simulação não mudou

Depois de alterar:

```text
src/pacote_do_drone/tether_parameters.json
```

execute novamente:

```bash
cd ~/IC
python3 src/pacote_do_drone/models/gerar_cabo.py
colcon build --symlink-install
source install/setup.bash
```

---

### O comando `ros2 launch` não funciona

Verifique se o ROS 2 foi carregado:

```bash
source /opt/ros/jazzy/setup.bash
```

e se o workspace foi carregado:

```bash
source ~/IC/install/setup.bash
```

Depois tente novamente:

```bash
ros2 launch pacote_do_drone start_sim.launch.py
```

---

## Comandos úteis

Listar tópicos ROS:

```bash
ros2 topic list
```

Ver dados de tensão do cabo:

```bash
ros2 topic echo /tensao_cabo
```

Ver dados dos ângulos das juntas:

```bash
ros2 topic echo /angulos_cabo
```

Executar o nó de sensores:

```bash
ros2 run pacote_do_drone sensores
```

Executar o launch principal:

```bash
ros2 launch pacote_do_drone start_sim.launch.py
```

Gerar novamente os arquivos do cabo:

```bash
python3 src/pacote_do_drone/models/gerar_cabo.py
```
