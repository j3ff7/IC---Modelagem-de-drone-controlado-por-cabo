# Análise técnica de referência: marsupial_simulator_ros2

## 1. Escopo, proveniência e convenções de evidência

Análise em 2026-09-04, para transferência de conhecimento ao projeto X500/PX4 + tether. Nenhum arquivo de `drone-cabo` foi inspecionado ou modificado; não houve integração, commit, push ou instalação de dependência no projeto principal.

| Item | Identificação |
|---|---|
| Repositório | https://github.com/robotics-upo/marsupial_simulator_ros2 |
| Checkout independente utilizado | `/home/lima/codes/ic/refs/marsupial_simulator_ros2` |
| Branch | `main` |
| Commit analisado | `d9046774cada2f0b679fb0dfdc1857516fc36936` |
| Data do commit | 2026-07-24 16:42:59 +0200 |
| Estado inicial | Sem mudanças rastreadas; `paper/` não rastreado |
| Pacote local | `marsupial_simulator_ros2`, versão declarada 1.0.0, licença MIT |
| Artigo local | `paper/2412.12776v3.pdf`, arXiv:2412.12776v3, 28 Jul 2025, 20 páginas |
| Título | Physical simulation of Marsupial UAV-UGV Systems Connected by a Variable-Length Hanging Tether |
| Autores | José E. Maese, Fernando Caballero, Luis Merino |
| SHA256 do PDF | `6e58b341b2474e822d3e9df7c203635507387f092eb5e1eeb2c98ae171e6ecda` |

O checkout já estava em local independente compatível com o solicitado, fora de `drone-cabo`; não foi necessário duplicá-lo. Não havia `.codegraph/`; empregaram-se leitura de código, buscas `rg`, parsing XML, renderização Jinja em memória e inspeção do artigo. O README atual anuncia publicação IEEE RA-P, mas **o documento efetivamente analisado é a versão arXiv local de 2025**, anterior ao commit. Não equiparar versões.

Convenções usadas em todo o documento:

- **CONFIRMADO PELO CÓDIGO**: declaração, fluxo alcançável ou cálculo reproduzível a partir dos arquivos. Não significa execução física bem-sucedida.
- **CONFIRMADO PELO ARTIGO**: afirmação do PDF, citada por seção/página; não valida automaticamente o código atual.
- **INFERÊNCIA**: consequência técnica explicitamente derivada; requer teste quando envolve comportamento dinâmico.
- **HIPÓTESE**: proposta de adaptação ou explicação ainda não demonstrada.

Arquivos relativos neste documento pertencem ao commit acima. O índice `analysis_evidence/source_index.txt` localiza classes, funções e interfaces com linhas. O inventário `analysis_evidence/model_inventory.json` conserva links, joints, inércias, colisões, sensores e plugins dos SDF/URDF/worlds; evita depender apenas de tabelas resumidas. `analysis_evidence/paper-extracted.txt` contém extração textual do PDF com separadores de página.

## 2. Resultado central para a decisão de arquitetura

**CONFIRMADO PELO CÓDIGO:** a referência usa **Gazebo Classic e ODE**, com três modelos físicos independentes: `rs_robot` (UGV, suportes e tambor), `tether` (cadeia articulada) e `sjtu_drone` (UAV). Após spawn, um WorldPlugin externo cria dois joints entre modelos. O comprimento físico da cadeia não muda: um tambor gira e a distribuição entre cabo enrolado e cabo livre muda por dinâmica e contato.

**A referência não demonstra uma solução de árvore sem loops para DART.** O próprio UGV contém um loop: `box_central` é filho de **dois joints revolute**, cujos pais estão fixos ao mesmo chassi. Além disso, a conexão runtime UAV → `link_final` acrescenta um parent ao endpoint que já é child na cadeia. Não existe rotina para inverter ou reconstruir a árvore do cabo.

Não confundir três questões:

1. O tether isolado é uma cadeia sem ciclo.
2. UAV—tether—UGV, reduzidos a três corpos/modelos abstratos, formam um caminho com duas conexões; isso não garante uma árvore orientada admissível pela API DART.
3. A topologia detalhada do carretel **contém efetivamente um ciclo não orientado**. A solução opera no contexto de constraints ODE, não eliminando esse ciclo.

**Recomendação:** reutilizar conceitos de cadeia discretizada e interfaces de controle/medição, não o artifício de attach Classic nem o duplo parent do carretel. Para o problema relatado no X500, avaliar uma única cadeia orientada estação → cabo → UAV, ou uma constraint/força de endpoint específica para a versão do backend, com validação separada. Não presumir que separar SDFs basta.

> **Claude Code review: CONFIRMADO.** Segunda leitura independente de `models/rs_robot/rs_robot.sdf:775-813` (SDF) e `urdf/rs_robot.urdf:285-299` (URDF, o mesmo repositório mantém as duas descrições em paralelo) reproduz exatamente a mesma dupla filiação: `joint_izquierdo` e `joint_derecho` declaram `<child>box_central</child>` em ambos os formatos. O SDF ainda carrega um comentário de autor explicando a intenção (não encontrado citado no texto original do Codex): `rs_robot.sdf:771-774` — "Rodamiento doble coaxial con joint_derecho: ambos joints deben tener parámetros idênticos para que el tambor gire libremente sin par restaurador. spring_stiffness=0 es crítico...". Isso mostra que o loop é uma decisão consciente de projeto (dois rolamentos coaxiais modelando fisicamente o eixo do tambor), não um erro de geração de SDF — reforça a leitura do Codex de que não há mecanismo de reparenting nem eliminação de ciclo, apenas paralelismo cinemático tolerado pelo solver ODE.
>
> Adicionalmente, reproduzi de forma independente (execução Python, não apenas leitura) o gerador de geometria de `tether.sdf.jinja` fora do Jinja (replicando as fórmulas do §5) e obtive a mesma distância total entre origens **17.862585799065698 m** e a mesma origem final de `link_final` **(0.454824026, -0.0042, -0.148732200)** relatadas pelo Codex, confirmando por execução (não só leitura) os números do §5 abaixo.

## 3. Arquitetura, packages e dependências

```text
world ODE + gazebo_ros init/factory + WorldPlugin link_attacher
  ├─ spawn rs_robot.sdf
  │    ├─ ros2_control → juntas de direção, rodas, joint_izquierdo
  │    ├─ plugin_ugv → pose de referência
  │    └─ joint_state publisher + lidar
  ├─ spawn sjtu_drone.sdf
  │    └─ plugin_drone externo → controle por força/torque + estados/sensores
  └─ spawn tether.sdf
       ├─ 125 links / 124 joints universal → dinâmica no ODE
       ├─ 125 LiftDragPlugin → forças aerodinâmicas por link
       └─ tether_position_publisher → snapshots de poses
attach_tether.py → /attach → dois joints runtime entre modelos
trajectory_follower.py → alvos UAV/UGV/comprimento
  ├─ uav_trajectory_follower.py → cmd_vel UAV
  └─ ugv_theter_trajectory_follower.py → rodas, direção, tambor e estimativa L
rosbag → scripts CSV → gráficos/catenária/distância de obstáculos
```

Há **um package ROS 2 no repositório**, ament_cmake, C++14. CMake compila somente `libplugin_ugv.so` e `libtether_position_publisher.so`; instala scripts Python como executáveis e diretórios de recursos. Não gera os dois `.msg` existentes: faltam `rosidl_generate_interfaces` e dependências de geração. Portanto `target_publisher.py`, que importa esses tipos, não representa uma interface construída por este pacote.

Dependências do Dockerfile:

| Origem | Branch/configuração | Papel / reprodutibilidade |
|---|---|---|
| `osrf/ros:humble-desktop` | tag não fixada por digest | ROS 2 Humble / desktop; instalação Classic via apt |
| `noshluk2/sjtu_drone` | `ros2` | Packages description, control, bringup; `libplugin_drone.so` |
| `davidorchansky/gazebo_ros_link_attacher` | `humble-devel` | serviço e WorldPlugin de constraints |
| `ros-simulation/gazebo_ros2_control` | `humble` | backend ros2_control; README aponta também ros-controls |
| apt | sem versões fixadas | libgazebo-dev, gazebo_ros_pkgs, xacro, ros2_control/controllers, scipy, colcon, imu-tools, joint-state-publisher, xterm, Git LFS |

Dependências adicionais usadas mas não inteiramente declaradas no `package.xml`: gazebo_dev, gazebo_msgs, gazebo_ros_link_attacher, ament_index_python, launch/launch_ros, robot_state_publisher, joy, teleop_twist_keyboard, Jinja2, NumPy, PyYAML; avaliação usa pandas, matplotlib, scipy, tqdm, pycatenary, rosbag2_py e, em scripts de nuvem, Open3D. Plugins `libBuildOctomapPlugin.so`, `libgazebo_ros_velodyne_laser.so` e `libgazebo_ros_attach.so` são referenciados, mas não implementados neste repositório e sua resolução não foi demonstrada no ambiente.

Para fechar a evidência de dependências críticas, foram clonadas somente para leitura em `/home/lima/codes/ic/refs/marsupial_analysis_support/`:

- link_attacher: `2879cf838565a2603bf03ba4f1ea202965ad0304` (`humble-devel`).
- sjtu_drone: `354c71a630d89793de6180d0dfc7824b4de3fba9` (`ros2`).

Esses SHAs são as versões inspecionadas em 2026-09-04; **não se afirma que sejam as versões utilizadas pelos autores nos experimentos**, pois o Dockerfile não as fixa. Links imutáveis: [implementação attach](https://github.com/davidorchansky/gazebo_ros_link_attacher/blob/2879cf838565a2603bf03ba4f1ea202965ad0304/src/gazebo_ros_link_attacher.cpp), [dinâmica UAV](https://github.com/noshluk2/sjtu_drone/blob/354c71a630d89793de6180d0dfc7824b4de3fba9/sjtu_drone_description/src/plugin_drone_private.cpp).

## 4. Fluxo completo de inicialização e atualização

### Launch básico: `launch/marsupial_simulation.launch.py:15`

`generate_launch_description()` escolhe `theatre.world` por default, via `get_package_share_directory`. Instancia servidor e cliente de `gazebo_ros`, um `controller_manager/ros2_control_node`, carregamento dos três controllers e robot_state_publisher.

Posições default dos modelos, antes da dinâmica:

| Modelo | Spawn XYZ em metros |
|---|---|
| rs_robot | (0, 0, 0.3) |
| sjtu_drone | (0.2, 0.01, 0.7) |
| tether | (-0.25, 0, 0.625) |

UGV e UAV são disparados em paralelo pelo launch. Ao terminar o processo de spawn UGV, TimerAction espera 0.5 s e inicia spawn tether. Ao terminar o processo de spawn tether, mais 0.5 s antecedem `ros2 run ... attach_tether.py`. **OnProcessExit não testa exit code**. Não há barreira explícita de sucesso do UAV, pausa física durante a montagem ou rollback de uma conexão parcial.

`attach_tether.py:11`, função `attach`, monta request, espera resposta até 10 s e tenta até três vezes por endpoint. Antes disso espera disponibilidade de `/attach` indefinidamente. Ordem: UAV-base/link_final; depois tambor/link_0. Falha retorna exit 1; caso a segunda conexão falhe, a primeira não é desfeita.

O SDF é carregado já gerado. **Jinja não executa durante launch**. O cabo passa a ser atualizado pelo solver em cada passo; não há ROS node que crie links continuamente. O publisher de tether é exclusivamente observacional, acionado por mudanças de targets, não por cada passo físico.

O objeto de spawn `teatro` existe no launch básico mas sua inclusão final está comentada; portanto escolher `theatre.world` não implica que a malha teatral externa seja spawnada por esse launch. `delayed_spawn_uav_node` também é declarado sem participação na lista retornada.

### Modos adicionais

| Launch | Fluxo ativo e observações |
|---|---|
| `marsupial_manual_simulation.launch.py` | Repete spawn/attach; defaults UGV (-3,0,0.3); UAV y=0; inclui teatro, joy, teleop UAV em xterm e `ugv_control.launch.py` |
| `marsupial_experiment.launch.py` | **Não inicia o simulador**. Publica takeoff, posições iniciais após 2 s; inicia TrajectoryPublisher com `mission` após 2 s; seguidores UAV e UGV/tether após 10 s; grava bag imediatamente |
| `marsupial_to_point.launch.py` | Também pressupõe simulação rodando; publica takeoff/targets e depois inicia `ugv_to_point.py` e `uav_to_point.py` |
| `ugv_state_publisher.launch.py` | Lê `urdf/rs_robot.urdf` e publica robot_description/TF via robot_state_publisher; não cria corpos no Gazebo |
| `ugv_control.launch.py` | Inicia controle por joystick, não teclado; use_sim_time declarado mas não transmitido ao Node |
| `multi_drone_simulation.launch.py` | Experimental: dois UAVs usando mesmo namespace hardcoded no SDF; tether linear; attach script conecta apenas UAV1 a link_0, não conecta UAV2 |
| `ugv_sim_controller.launch.py`, `ugv_sim_keyboard.launch.py` | Referenciam `theatre.model` e `marsupial_simulator_ros2.launch.py`, ausentes; não são entrada íntegra para esta simulação |
| `bag_record.py` | Utilitário de gravação, não componente físico |

Riscos de startup: carregamento de controllers não sequenciado com readiness; controller_manager standalone coexistindo com manager do plugin; caminho absoluto de YAML; URDF do UGV também contém dupla filiação de `box_central`; `mission` é LaunchConfiguration sem DeclareLaunchArgument/default próprio (passar `mission:=test1`). CLI de targets usa QoS default, enquanto seguidores exigem TRANSIENT_LOCAL; os publishers periódicos do TrajectoryPublisher têm QoS correspondente, mas a variante to_point depende de corrigir publicação/readiness para entregar os alvos aos nós tardios.

## 5. Modelo físico do tether: valores realmente ativos

**CONFIRMADO PELO CÓDIGO / parsing do SDF**, não a tabela do README:

| Arquivo / função | Parâmetro | Valor ativo/default | Função física |
|---|---|---|---|
| `models/tether/tether.sdf.jinja:4` | number_elements | 125 | Número total de links, incluindo endpoint |
| template `link`, SDF | links | link_0…link_123, link_final | 125 corpos rígidos |
| template `joint` | joints | joint_1…joint_124, universal | 124 conexões com dois eixos rotacionais |
| template | cl | 0.15 m | Comprimento nominal do trecho helicoidal |
| template | cr | 0.004 m | Raio das colisões cilíndricas |
| template | sr | 0.009 m | Parâmetro nominal de esfera; esfera efetiva é apenas visual |
| template | cr_visual / sr_visual | 0.008 / 0.018 m | Visual deliberadamente mais espesso |
| template | m=0.01*cl | 0.0015 kg em **todos** os 125 links | Massa; o trecho final não recalcula m |
| soma de inertials | massa total | 0.1875 kg | Total explícito incluindo endpoint sem cilindro |
| macro inertial | ixx=iyy=izz | 0.01 kg m², produtos zero | Inércia artificial constante |
| macro joint, ambos eixos | damping | 0.1 | Amortecimento rotacional, não amortecimento axial |
| idem | friction | 0.0 | Atrito estático do joint |
| idem | spring_stiffness / reference | 0.01 / 0 | Mola angular em torno da configuração de referência |
| idem | axis / axis2 | (0,1,0) / (0,0,1) | Duas rotações; use_parent_model_frame=true |
| idem | limites | Nenhum `<limit>` explícito | Valores efetivos dependem do parser/engine; não inventar limites físicos calibrados |
| idem | ode/cfm_damping | 1 | Tratamento numérico específico do ODE, legado |
| macro collision | pose | (0.1,0,0,0,1.570790,0) | Deslocamento fixo do cilindro de colisão |
| macro element_visual | pose x | cl/2 | Visual centrado no meio do segmento |
| macro collision | contact/ode/min_depth | 0.001 m | Camada/profundidade mínima de contato |
| macro collision | mu, mu2 | 1.0, 1.0 | Atrito de contato do cabo |
| links | gravity | true | Peso em cada corpo |
| modelo e links tether | self_collide | Não declarado | Não há ativação explícita de auto-colisão |

Universal tem dois graus de liberdade de rotação; não há grau de liberdade de extensão axial. A rigidez 0.01 é aplicada a coordenadas angulares: interpretar como rigidez torsional/flexional equivalente, não como EA nem como N/m de uma mola axial. A unidade mecânica coerente é N m/rad para stiffness e N m s/rad para damping. A tabela do artigo imprime N/m e Ns/m; isso não altera a natureza dos joints no código. [Semântica SDFormat de joints](https://sdformat.org/tutorials/specification/spec_model_kinematics/).

### Discretização e comprimento: quatro quantidades diferentes

A tabela do artigo, N×cl do template, geometria cilíndrica e comprimento estimado pelo controller **não são intercambiáveis**.

- `125*0.15 = 18.75 m` é um produto nominal, não o comprimento da cadeia efetivamente gerada.
- Para n=0…114, a origem segue uma hélice poligonal; para n=115…124, segue interpolação em direção a `drone_point=(0.52,-0.02,-0.16)`.
- Há 115 distâncias entre origens de aproximadamente 0.150004800 m e 9 de 0.068003756433 m: soma **17.862585799 m** entre `link_0` e `link_final` na configuração SDF.
- O passo helicoidal inclui deslocamento lateral 0.0012 m por link; por isso a distância entre origens não é exatamente 0.15 m.
- Colisões: 115 cilindros de 0.15 m + 8 de 0.068003756433 m = **123 cilindros**. `link_123` possui visual cilíndrico mas não colisão; `link_final` possui apenas esfera visual, com massa/inércia.
- O comprimento livre em operação não foi medido geometricamente pelo controller; `/cable_length[0]` é estimativa de encoder ou integração de comando, começando em 1.0 m no follower experimental.

Fórmulas do gerador:

```text
Δα = 2 asin(cl/(2 radius)), radius=0.14 m
x_n = radius sin(n Δα)
y_n = n winch_lenght/number_elements, winch_lenght=0.15 m
z_n = radius cos(n Δα)
P_115 = início da transição para o UAV
P_n = P_115 + (drone_point - P_115)*(n-115)/10, n=115…124
pitch = asin(-vz), yaw=atan2(vy,vx), roll=0
```

> **Claude Code review: CONFIRMADO por execução independente.** Reimplementei estas fórmulas em um script Python isolado (fora do Jinja, sem reutilizar `jinja_gen.py`) e integrei as 124 distâncias entre origens consecutivas de `link_0` a `link_final`. Resultado: soma total **17.862585799065698 m**, **115** segmentos de **0.150004800...** m e **9** segmentos de **0.06800375643304235 m** (não 10, pois o décimo "gap" curto é o segmento 115→116 dentro do trecho já recalculado, coerente com 124 gaps totais = 115+9). Também confirmo `cl` recalculado nos 10 últimos links (`n=115..124`) como **0.06800375643304235 m constante** — mas a massa `m` usada em `inertial(m)` permanece a constante global `0.01*0.15=0.0015 kg`, pois é fixada **antes** do laço e nunca depende do `cl` recalculado dentro do `for`. Isto é evidência direta (execução), não inferência, de que a "elasticidade"/variação geométrica dos 10 últimos elementos não altera a massa por elemento — achado já apontado pelo Codex, agora com reprodução numérica independente.

A última origem está em **90%** da interpolação, não em drone_point: `link_final=(0.454824026,-0.0042,-0.148732200)`. No spawn básico resulta `(0.204824026,-0.0042,0.476267800)` no mundo; relativa ao UAV inicial `(0.2,0.01,0.7)`, offset `(0.004824026,-0.0142,-0.223732200)`. São coordenadas pré-dinâmica, não medição do joint após attach. O comentário do controller calcula offset a partir do drone_point e usa `(0.07,-0.03,-0.235)`, que **não coincide** com o endpoint gerado.

### Inércias, contatos e limitações físicas

A macro `inertial` contém fórmulas cilíndricas comentadas: Izz=m r²/2 e Ixx=Iyy=m l²/12+m r²/4. Substitui-as por 0.01, alegando segfault ao variar especificações. Para m=0.0015, r=0.004 e l=0.15, as fórmulas dariam Izz=1.2e-8 e Ixx=Iyy=2.8185e-6 kg m²: os valores ativos são aproximadamente 833333 e 3548 vezes maiores. Isso altera a dinâmica; não é uma aproximação quantitativa pequena. O tensor está na origem do link, sem deslocamento de COM para cl/2.

A colisão deslocada x=0.1 e visual em cl/2 diferem em 0.025 m nos elementos longos e cerca de 0.066 m nos curtos. Esferas não participam de contato. Não há material de cabo extensível, ruptura, deformação contínua, modelo constitutivo de torção completo, limite de tensão nem self-contact explícito que sustente afirmações sobre nós físicos. A ausência de self_collide implica uso dos defaults do parser, não simulação demonstrada de cabo-cabo.

### Arrasto

`lift_drag` instancia incondicionalmente **125** `libLiftDragPlugin.so`, área=2 cr cl (0.0012 m² nos longos), densidade 1.2041 kg/m³, cda=1.2535816618911175, cda_stall=1.4326647564469914, cla=0, a0=0, alpha_stall=0, cp=(0,0,0), forward=(0,1,0), upward=(0,0,1). Inclusive `link_final` recebe plugin embora não tenha cilindro físico. Este é arrasto por plugin externo, não tensão de endpoint; vento comandado por ROS não foi encontrado.

### Geração e parâmetros obsoletos

`scripts/jinja_gen.py`, `parse_var`, avalia expressões e passa variáveis ao template. Entretanto o template faz `{% set number_elements=125 %}`, `{% set cl=0.15 %}` etc. incondicionalmente, e não usa `enable_drag`. **Verificado em memória:** render default é byte a byte igual ao SDF rastreado; passar number_elements=60, cl=0.1, enable_drag=False produz exatamente o mesmo resultado. Os exemplos CLI documentados não funcionam para essa configuração.

Os parâmetros devem hoje ser alterados no template e o SDF regenerado antes de reinstalar os recursos. `y_el`, `y_tetha` e macro `y_element` não são invocados; cálculos `input_angle`, `n_helices`, `paso` não determinam as poses finais. O comentário “203 para 20 m, 101 para 10 m” não é confiável para cl=0.15 atual. A condição comentada de fixed_to_world não cria joint de ancoragem.

## 6. Topologia mecânica completa e endpoints

### Grafo físico: não existe uma árvore global única sem ciclo

```text
rs_robot/base_footprint
  └─ base_joint [fixed] → base_link
       ├─ fl_steering_joint [revolute] → fl_steering_link
       │    └─ fl_wheel_joint [revolute] → fl_wheel_link
       ├─ fr_steering_joint [revolute] → fr_steering_link
       │    └─ fr_wheel_joint [revolute] → fr_wheel_link
       ├─ rl_steering_joint [revolute] → rl_steering_link
       │    └─ rl_wheel_joint [revolute] → rl_wheel_link
       ├─ rr_steering_joint [revolute] → rr_steering_link
       │    └─ rr_wheel_joint [revolute] → rr_wheel_link
       ├─ joint_izquierdo_to_ugv [fixed] → box_plano_izquierdo
       │    └─ joint_izquierdo [revolute] ─┐
       ├─ joint_derecho_to_ugv [fixed] → box_plano_derecho
       │    └─ joint_derecho [revolute] ───┴→ box_central (MESMO link)
       ├─ joint_plataforma_to_ugv [fixed] → plataforma_central
       ├─ joint_plataforma_to_uav_1 [fixed] → plataforma_uav_1
       └─ joint_plataforma_to_uav_2 [fixed] → plataforma_uav_2

box_central ── attach runtime ─→ tether/link_0
  └─ joint_1 [universal] → link_1
       └─ joint_2 [universal] → link_2
            ...
             └─ joint_123 [universal] → link_123
                  └─ joint_124 [universal] → link_final
                                              ↑
sjtu_drone/base_link ── attach runtime ─────────┘
```

Setas dos attaches indicam ordem l1/l2 de `Attach/Load`, não uma árvore DART reconstruída. O primeiro modelo passado é UAV no endpoint livre e UGV no endpoint enrolado. Não há reparenting explícito dos links internos. `SetModel(m2)` muda associação do joint no plugin; não inverte os parents da cadeia.

Raízes declaradas: `base_footprint` no UGV; `link_0` no cabo antes de attach; `base_link` único no UAV. UGV tem 16 links/16 joints (6 fixed, 10 revolute); cabo 125/124; UAV 1/0. Ground station é móvel: rodas fazem contato com o chão, não há fixed joint UGV→world. O cabo não se liga diretamente a world. A plataforma de decolagem é geometria de suporte do UAV, **não joint de pouso/engate**.

> **Claude Code review: CONFIRMADO.** Grep direto em `urdf/rs_robot.urdf:225-307` mostra a mesma topologia do SDF: `link name="box_central"` (linha 225), `joint name="joint_izquierdo"` com `<child link="box_central"/>` (linhas 285-287) e `joint name="joint_derecho"` com `<child link="box_central"/>` (linhas 293-295). Ou seja, o URDF usado por `ugv_state_publisher.launch.py`/`robot_state_publisher` **não é uma árvore válida por si só** (um URDF canônico exige um único parent por link); isso é consistente com o aviso do Codex de que TF não deve ser usado como prova de topologia do solver, e reforça que a duplicação não é artefato só do gerador Jinja do SDF, mas uma decisão replicada manualmente em dois formatos de descrição independentes.

### Conexão runtime: evidência da dependência

No SHA externo fixado, `GazeboRosLinkAttacher::attach`, aproximadamente linhas 112–151, executa:

```cpp
j.joint = this->physics->CreateJoint("revolute", m1);
j.joint->Attach(l1, l2);
j.joint->Load(l1, l2, ignition::math::Pose3d());
j.joint->SetModel(m2);
j.joint->SetUpperLimit(0, 0);
j.joint->SetLowerLimit(0, 0);
j.joint->Init();
```

É **revolute travado em zero**, usado como conexão rígida; não `CreateJoint("fixed")`, ball joint ou força elástica de endpoint. A pose relativa explícita do joint é identidade; não há parâmetro de offset no request. Não confundir com a posição do corpo após integração e correção de constraints.

> **Claude Code review: CONFIRMADO.** Repeti a leitura do checkout somente-leitura em `/home/lima/codes/ic/refs/marsupial_analysis_support/gazebo_ros_link_attacher` e confirmei `git rev-parse HEAD` = `2879cf838565a2603bf03ba4f1ea202965ad0304` (mesmo SHA citado pelo Codex) e o trecho de código linha a linha idêntico ao transcrito.
>
> **Claude Code addition:** o próprio arquivo carrega, logo acima do `SetModel`, um comentário histórico do autor original explicando **por que** `SetModel(m2)` é necessário e o que acontece sem ele — evidência direta do motivo pelo qual a referência não pode simplesmente criar o `CreateJoint` diretamente ligando as duas árvores sem essa chamada extra: *"If SetModel is not done we get: Internal Program Error - assertion (this->GetParentModel() != __null) ... An entity without a parent model should not happen"* e *"If SetModel is given the same model than CreateJoint given, Gazebo crashes with: assertion (self->inertial != __null) ... Inertial pointer is NULL"* (`gazebo_ros_link_attacher.cpp:112-128` no SHA acima). Isso documenta uma limitação de baixo nível específica do Gazebo Classic/ODE (o link recém-anexado a outro modelo precisa de um "dono" de modelo consistente antes de poder publicar sua pose), e é evidência adicional de que a solução inteira é um contorno de uma limitação de API do motor Classic — não uma técnica de topologia generalizável a engines diferentes (DART/Gazebo Sim têm modelos de propriedade de entidade e de detecção de ciclos completamente distintos), reforçando a recomendação do Codex de não portar esse mecanismo tal como está.

O WorldPlugin anuncia `attach` e `detach` (`gazebo_ros_link_attacher/srv/Attach` em ambos), resolve modelos via `ModelByName` e links via `GetLink`. Guarda joints para reutilização por par ordenado; reattach usa `Attach`, detach usa `Detach`. **A referência só chama attach na inicialização**, não detach durante recolhimento. Há funcionalidade destacável no plugin Classic, mas **não o sistema `gz::sim::systems::DetachableJoint`**. Os comentários de `SetModel` citam falhas históricas de ODE; não provam portabilidade.

## 7. Resposta objetiva ao problema de loop cinemático

| Questão | Resposta com evidência |
|---|---|
| Cabo e veículo no mesmo SDF/model? | Não; três `spawn_entity.py` no launch básico |
| Cabo e ground station separados? | Sim; tether e rs_robot; tambor pertence ao rs_robot |
| Joints criados runtime? | Sim, WorldPlugin externo via duas requests de attach_tether.py |
| DetachableJoint de Gazebo Sim? | Não; há `/detach` Classic disponível, não chamado pelo fluxo |
| Constraint plugin? | Sim: joint revolute travado em zero, criado pelo physics engine |
| Forças diretas substituem ligação? | Não nos endpoints; forças do UAV/arrasto têm outra finalidade |
| Árvore enraizada na estação e UAV child? | Não há tal reconstrução; request passa UAV como l1 e link_final como l2 |
| Como elimina segundo parent? | Não elimina no código; usa API de constraints Classic/ODE |
| Existe loop mesmo sem cabo? | Sim: dois rolamentos do tambor conectam o mesmo child ao chassi por dois caminhos |
| Compatibilidade com problema DART relatado? | Não demonstrada; copiar mecanismo não constitui solução |

**CONFIRMADO PELO ARTIGO:** não há descrição de DART, reparenting, algoritmo para romper loops ou detachable joint. O artigo descreve fisicamente acoplamento, cabo e tambor (§3.1, pp.6–8), não a restrição de representação da API.

**INFERÊNCIA:** o funcionamento descrito pelos autores apoia-se no regime de constraints e contatos ODE. Não é correto concluir que múltiplos modelos em qualquer engine aceitam automaticamente múltiplos parents.

A documentação de [DetachableJoint no Gazebo Sim 9](https://gazebosim.org/api/sim/9/detachablejoints.html) exige topologia em árvore e explicita restrições de contato no reattach. O [tutorial de loop no Classic](https://get.gazebosim.org/tutorials?tut=kinematic_loop) distingue o grafo SDFormat da árvore URDF. **Limite de escopo:** não afirmar que DART, em todas as versões e APIs, jamais suporte constraints de loop. O [changelog atual de Gazebo Physics](https://gazebosim.org/libs/physics/) já menciona suporte non-tree em AttachFixedJoint; é necessário verificar as versões efetivamente instaladas no X500 antes de decidir sobre recursos novos. Nenhuma versão do projeto principal foi auditada nesta tarefa.

## 8. Plataforma, carretel e variação de comprimento

### Hardware virtual no modelo ativo

`models/rs_robot/rs_robot.sdf:575–845` contém o carretel integrado:

| Elemento | Geometria / parâmetros | Papel |
|---|---|---|
| box_central | cilindro raio 0.1 m, comprimento 0.4 m; pose (-0.25,0,0.35,0,1.5708,1.5708) | Tambor rotativo real em termos de corpo/colisão |
| box_plano_izquierdo/derecho | boxes 0.025×0.25×0.35 m, y=±0.2 | Suportes laterais fixos |
| plataforma_central | box 0.9×0.45×0.01 m, z=0.18 | Base superior |
| plataforma_uav_1/2 | boxes 0.4×0.05×0.02 m, (0.2,±0.14,0.2) | Apoio de decolagem |
| joint_izquierdo/derecho | revolute, axis=(0,0,1), mesma pose, limites ±1e16 rad | Dois rolamentos coaxiais declarados |
| ambos joints | damping=0.05, friction=0, stiffness=0, reference=0 | Não há mola restauradora do tambor |
| ros2_control joint_izquierdo | velocity command, min/max ±3.14 | Atuação só do rolamento esquerdo |

Tambor, suportes e plataformas não têm `<inertial>` explícito. Não tratá-los como massa zero: defaults do SDF/backend podem contribuir massa/inércia. Direções têm limites ±2.1 rad, effort=5, velocidade=6.28 rad/s; rodas ±1e16 rad, effort=1.5, velocidade=20 rad/s. Ambos grupos têm damping=0.005, friction=0.02 e stiffness=0. Esses limites físicos diferem dos limites de interfaces ros2_control. A soma **explicitamente declarada** dos links UGV é 275 kg (base_link 175 kg + quatro steering de 15 kg + quatro rodas de 10 kg), excluindo defaults dos links restantes. Inventário conserva os tensores individuais. Rodas têm 4 direções e 4 velocidades comandadas; o follower põe todas as direções iguais para translação tipo crab, não implementa planejamento holonômico geral ou Ackermann completo.

> **Claude Code review: CONFIRMADO, com adição.** `rs_robot.sdf:775-813` confirma `damping=0.05`/`friction=0.0`/`spring_stiffness=0`/`spring_reference=0` em ambos os joints do tambor, e limites `<lower>-1e16</lower><upper>1e16</upper>` — exatamente como relatado.
>
> **Claude Code addition:** o bloco `ros2_control` (`rs_robot.sdf:866-914`) declara explicitamente `state_interface name="effort"` tanto para `joint_izquierdo` quanto para as quatro rodas (`fl_wheel_joint`, etc.), além de `velocity`. Isto é evidência de código adicional (linha exata) para a afirmação já feita pelo Codex na §9 de que "effort é interface declarada; não é usado pelo algoritmo para calculá-la" — a interface existe no XML e é potencialmente populável pelo hardware plugin `gazebo_ros2_control/GazeboSystem`, mas nenhum script Python (`ugv_theter_trajectory_follower.py`, `attach_tether.py`, etc.) assina ou lê esse campo `effort` do tópico `/joint_states`; portanto, mesmo que o plugin popule o valor de esforço/torque do rolamento, ele fica **sem consumidor** no fluxo ativo. Isso é uma pista concreta para X500/PX4: se um plugin de hardware equivalente publicar `effort` no winch, a instrumentação de tensão poderia começar por esse campo, em vez de criar um sensor novo do zero.

Contato do tambor e suportes: mu=mu2=0, enquanto cabo tem 1.0. Logo não assumir mecanismo de capstan por atrito calibrado; o endpoint preso transmite movimento e o tambor fornece obstáculo geométrico. Auto-colisão entre links do cabo não é ativada. A exatidão do enrolamento multicamada não foi validada.

### Operação

Não se criam/destroem links, não se alteram tamanhos, não se ativam joints sucessivos. Toda a cadeia permanece simulada e tem massa constante. `joint_izquierdo` recebe velocidade; o corpo do tambor gira e move a ligação de `link_0`. O restante redistribui-se por gravidade, constraints e colisões. Há portanto **carretel geométrico**, mas não modelo completo de motor, encoder físico, guia de distribuição de espiras, raio variável por camada ou tensão medida.

`models/winch/winch.sdf` é variante independente não spawnada no fluxo principal; não usá-la como fonte dos parâmetros ativos.

### Controle ativo do experimento

`scripts/ugv_theter_trajectory_follower.py`, classe `UGVController`, métodos `_joint_state_cb`, `calculate_winch_velocity`, `control_loop`:

```text
D = || (pose_UAV + offset_UAV) - (pose_UGV + offset_winch) ||
Ltarget = D * tether_coef + safety_margin
error = Ltarget - Lestimate
v = Kp error + Ki integral(error dt) + Kd Δerror/dt
ω = clamp(v / radius, -3.14, +3.14)
Lestimate = L0 - wind_sign*(θ - θ0)*effective_radius
fallback sem θ0: Lestimate += ω*effective_radius*dt
```

Defaults: modo RTTA (`use_tether_trajectory=False`), coef=1.0, margem=0.0 m, L0 inicial=1.0 m; radius=effective_radius=0.1 m; wind_sign=-1; Kp=2.5, Ki=0, Kd=0.1; integral limitada ±5; dt mínimo 0.001 s; timer 0.02 s. Não são ROS parameters declarados: são constantes Python. Limite corresponde a velocidade linear nominal máxima 0.314 m/s se raio efetivo correto.

Ao primeiro JointState contendo posição não-NaN de joint_izquierdo, captura θ0 e L0. Não há unwrap explícito, limite inferior/superior de comprimento, detecção de cabo acabado, sensor de escoamento nem verificação de escorregamento. O erro é calculado **antes** de atualizar L pelo encoder nessa chamada, introduzindo defasagem de uma iteração. A calibração de sinal alegada em comentário não foi repetida nesta análise.

Em PTR, `target_length_callback` converte o valor planejado em coeficiente dividindo pela distância entre **alvos**, com offsets adicionais (-0.5 x, +0.9 z), e aplica coeficiente à distância **atual**. Não é simplesmente atribuir Ltarget=Lyaml em toda iteração. O publisher avança waypoints usando o target dinâmico de `/cable_length[1]`, não erro contra YAML. RTTA default não mantém 5% de folga como artigo: coef=1 sem margem visa a distância reta estimada.

Offsets são somados em coordenadas mundo sem rotação pelos quaternions; tornam-se inconsistentes quando UAV/UGV inclinam ou giram. O steering experimental usa yaw para transformar direção ao corpo, mas isso não corrige os offsets de endpoints.

### Outra variante ainda alcançável

`ugv_to_point.py` é usado por `marsupial_to_point.launch.py`, portanto não deve ser descartado como arquivo morto. Inicializa L=0.6 m, coef=1.1, margem=0.3 m, PID=(10,0.1,1), limite=10; integra diretamente saída de controle×dt usando `time.time()`, e envia essa saída como velocidade angular sem conversão por raio. Mistura unidades e estima a partir de comando; `/cable_length` tem só dois valores, ao contrário dos três do follower de experimento. Não reutilizar essa implementação diretamente.

## 9. Forças, tensão, sensores e observabilidade

**Não foi encontrada medição ROS de tensão do tether.** Não há force_torque sensor no cabo, wrench publisher, GetForceTorque no código local, load cell, nem conversão torque→tensão. `effort` em ros2_control é interface declarada; não equivale a sensor de tensão e não é usado pelo algoritmo para calculá-la.

> **Claude Code review: CONFIRMADO, com verificação cruzada no artigo.** Busquei `tension|torque|force sensor|load cell|wrench` em `analysis_evidence/paper-extracted.txt`: a palavra "tension" aparece apenas de forma qualitativa, nunca como grandeza medida ou publicada — p.ex. p.7 "these interactions create local tension variations that propagate along the tether" e p.11 "Fig. 5: Examples of tether-obstacle collisions with varying degrees of tension" / "significantly increasing tension". Não há nenhuma ocorrência de "torque", "force sensor", "load cell" ou "wrench" no texto extraído do PDF. Isso é evidência adicional (artigo, não só código) de que a tensão é tratada apenas como fenômeno qualitativo observado visualmente pelos autores, nunca como sinal instrumentado — reforça a conclusão do Codex e eleva a confiança de "NÃO ENCONTRADA" de "não documentado no código" para "não documentado nem no código nem no artigo".

As reações dos joints e contatos pertencem ao solver. A transmissão mecânica aos veículos decorre das conexões runtime. O código local não extrai essas forças. O plugin SJTU externo aplica `AddRelativeForce` e `AddRelativeTorque` para propulsão/controle, não para substituir o acoplamento físico. LiftDragPlugin aplica aerodinâmica aos links.

| Grandeza | Origem / arquivo / função | Tópico e unidade | Frequência / limitação |
|---|---|---|---|
| Tensão / endpoint wrench | Não implementado localmente | Nenhum | Não medida, nem inferida pelo controller |
| Comprimento estimado | UGVController, encoder virtual ou integração | `/cable_length`, Float64MultiArray, m | timer 50 Hz nominal, apenas com poses/target disponíveis |
| Comprimento alvo / distância | Mesmo cálculo | índices [1]/[2] no follower experimental | Distância entre pontos aproximados; não comprimento de curva |
| Rotação do tambor | `libgazebo_ros_joint_state_publisher.so`, SDF UGV | `/joint_states`, position rad, velocity rad/s | SDF pede 60 Hz; broadcaster ros2_control também pode publicar |
| Pose de todos os links | TetherPositionPublisher::publishLinks | `/tether_positions`, PoseArray, posição m/quaternion | Evento de mudança de target >1e-4; sem taxa periódica |
| Pose UGV | UGVSimpleController::Update, GetLink()->WorldPose | `/ugv_gt_pose`, Pose | A cada 10 passos: 25 Hz no theatre de 4 ms; 100 Hz nos worlds de 1 ms |
| Pose/vel/acel UAV | plugin_drone externo, UpdateDynamics | `/sjtu_drone/gt_pose`, `/gt_vel`, `/gt_acc` sob namespace UAV | Atualização de dinâmica; aceleração por diferença de velocidade/dt |
| IMU UAV | SDF sensor_imu / gazebo_ros_imu_sensor | namespace sjtu_drone, Imu | 625 Hz pedidos, engine limita taxa efetiva |
| GPS UAV | SDF gps / gazebo_ros_gps_sensor | `/sjtu_drone/gps/data` via `~/out:=data`, NavSatFix | 30 Hz pedidos; resolução de namespace depende do plugin |
| Câmeras UAV | gazebo_ros_camera | camera_bottom/front e respectivos camera_info sob sjtu_drone | 15 / 60 Hz, frame bottom_cam_link/front_cam_link |
| Sonar UAV | gazebo_ros_ray_sensor | sonar sob sjtu_drone, Range, m | 5 Hz, frame sonar_link |
| Lidars UGV/UAV | gazebo_ros_velodyne_laser | PointCloud2; SDF usa topicName velodyne_scan | 10 Hz; scripts esperam paths diferentes, confirmar runtime |

Os tópicos lidar usados pelo recorder são `/rs_robot/velodyne_plugin/out` e `/sjtu_drone/velodyne_plugin/out`. O SDF contém tags de convenções diferentes (`topicName`, `frameName`, remapping `output:=velodyne_scan` no UGV). Sem a biblioteca resolvida/executada não se confirma qual nome final ela publica. O inventário registra XML completo dos sensores, inclusive namespaces, ruído, resoluções e alcance.

O publisher de tether ordena numericamente link_0…link_123,link_final; frame_id=`tether`, porém os valores são obtidos por **WorldPose**. Isso é inconsistência de frame, não transformação automática. Não transporta nomes por pose, forças ou ângulos dos joints. O consumidor precisa conhecer a ordem; quaternions permitem recuperar orientação dos corpos, não diretamente uma decomposição angular robusta dos dois eixos de cada joint.

Snapshot é capturado quando chega novo alvo, possivelmente antes do sistema estabilizar. Republishing de alvo idêntico a 1 Hz não causa publicação adicional. Não usar como stream de dinâmica a 1 Hz nem como detector de regime estacionário. A leitura de física ocorre em callback ROS, não callback WorldUpdateBegin; sincronização/thread-safety é risco a validar ao portar.

### Catenária e “inferência de força”

`tether_comparison.py::calculate_catenary` usa pycatenary com peso_por_metro×9.8 e EA=1.5e5 N, `floor=False`, para comparação geométrica offline. O default do script é 0.1 kg/m, diferente do 0.01 kg/m nominal dos trechos longos do SDF. Filtra cabo próximo ao winch por threshold 0.25 m, soma distâncias entre amostras selecionadas e usa cKDTree na comparação. **Não publica nem usa tensão calculada de catenária no controle.** Nome `CatenaryElastic` no pós-processamento não torna o cabo SDF axialmente elástico.

Um estimador T≈τ/r exigiria separar inércia do tambor, atrito, contato e reação do segundo rolamento; isso é **HIPÓTESE de instrumentação futura**, não implementação encontrada.

## 10. Interfaces ROS 2, serviços, actions e TF

Tabela de fluxo nominal, não lista descoberta em uma simulação executada:

| Tópico | Tipo ROS | Publisher | Subscriber | Finalidade |
|---|---|---|---|---|
| `/target_position_uav` | geometry_msgs/Pose | TrajectoryPublisher / CLI | DroneController, UGVController experimental, TetherPositionPublisher | Alvo e gatilho de observação |
| `/target_position_ugv` | geometry_msgs/Pose | TrajectoryPublisher / CLI | UGVController, TetherPositionPublisher | Alvo e gatilho de observação |
| `/target_length_tether` | std_msgs/Float64 | TrajectoryPublisher | UGVController experimental | Planejamento; ignorado no RTTA default |
| `/sjtu_drone/cmd_vel` | geometry_msgs/Twist | DroneController/teleop | plugin_drone | Comando de voo |
| `/sjtu_drone/takeoff`, `/land`, `/reset` (todos sob sjtu_drone) | std_msgs/Empty | CLI/launch | plugin_drone | Estado de voo |
| `/sjtu_drone/posctrl`, `/sjtu_drone/dronevel_mode` | std_msgs/Bool | Cliente externo | plugin_drone | Alternância de modos; defaults na dependência |
| `/sjtu_drone/gt_pose` | geometry_msgs/Pose | plugin_drone | Seguidores, planner e logger | Feedback UAV essencial |
| `/ugv_gt_pose` | geometry_msgs/Pose | plugin_ugv | UGVController, planner e logger | Feedback UGV essencial |
| `/forward_position_controller/commands` | std_msgs/Float64MultiArray | UGVController/manual | ForwardCommandController | [fl,fr,rl,rr] steering, rad |
| `/forward_velocity_controller/commands` | std_msgs/Float64MultiArray | UGVController/manual | ForwardCommandController | [fl,fr,rl,rr,winch], rad/s |
| `/joint_states` | sensor_msgs/JointState | plugin joint-state + broadcaster | robot_state_publisher, UGVController experimental | Encoder virtual e TF |
| `/cable_length` | std_msgs/Float64MultiArray | UGVController | TrajectoryPublisher/logger | Estimativa e meta; **não força** |
| `/tether_positions` | geometry_msgs/PoseArray | plugin tether | bag; consumidores antigos incompatíveis | Observação, não integração física |
| `/sjtu_drone/cmd_mode`, `/sjtu_drone/state` | std_msgs/String, std_msgs/Int8 | plugin_drone | Clientes de diagnóstico | Modo e estado de voo |
| `/sjtu_drone/odom` | nav_msgs/Odometry | plugin_drone, se pub_odom=true | Clientes de navegação | Publicação desativada por default no SDF inspecionado |
| `/trajectory_progress` | std_msgs/String | TrajectoryPublisher | bag/usuário | Status nominal 10 Hz |
| `/joy` | sensor_msgs/Joy | joy_node | ugv_control_controller.py | Controle manual |
| `/cmd_vel` | geometry_msgs/Twist | teleop externo | ugv_control_keyboard.py | Alternativa manual, não default do launch ugv_control |
| `/tf`, `/tf_static` | tf2_msgs/TFMessage | robot_state_publisher | visualização/consumidores | Descrição UGV; integridade a validar |
| `/clock` | rosgraph_msgs/Clock | integração Gazebo/ROS | Nodes com use_sim_time | Tempo simulado |

Serviços implementados no plugin externo: `/attach` e `/detach`, ambos `gazebo_ros_link_attacher/srv/Attach`, request com quatro strings model_name_1/link_name_1/model_name_2/link_name_2, resposta bool ok. Serviço de spawn utilizado: `/spawn_entity`, gazebo_msgs/srv/SpawnEntity, oferecido pela factory. `scripts/spawn_ugv.py` é cliente alternativo; launch principal usa o executável padrão gazebo_ros. Serviços de controller_manager são dependências de load/configure/switch de controllers; não há serviço próprio de comprimento/tensão.

**Actions:** nenhum servidor/cliente ROS action próprio encontrado. `launch.actions` são ações do framework launch, não ROS actions. Mensagens custom `TargetPoseUGV.msg` e `TargetPoseUAV.msg` estão sem geração no build.

**TFs:** URDF UGV tem frames base_footprint/base_link, steering, rodas e winch, mas repete o child box_central nos dois rolamentos; não é uma árvore URDF válida garantida. Não há robot_state_publisher para tether ou UAV no launch básico nem broadcasts de todos os links do cabo. O frame `tether` de PoseArray não vem acompanhado de TF explícito. Labels de frames dos sensores não demonstram que existam transforms correspondentes. Pose ground truth sem Header também não declara frame/timestamp. Não usar TF como evidência de topologia do solver.

## 11. Artigo × implementação: correspondências e divergências

### Cadeia articulada com contatos

**Descrição no artigo:** §3.1, pp.7–8: cabo multielementos, flexibilidade configurável e interação com obstáculos; origem em modelo airborne de RigidWing/sitl_gazebo.

**Implementação:** `tether.sdf.jinja`, macros link/joint/collision, SDF gerado, ODE nos worlds. **Correspondência: PARCIAL** (mecanismo multibody presente; parâmetros e detalhes numéricos diferentes). Gravidade, colisões e articulações existem; auto-contato e tensão publicada não demonstrados. “Elasticidade” não se traduz em extensão axial no SDF.

### Parâmetros default

**Descrição no artigo:** Tabela 1, p.7: 123 elementos enrolados + 10 livres, comprimentos 0.15/0.05 m, massa 0.01 kg por seção, damping 0.05, stiffness 0.01.

**Implementação:** `tether.sdf.jinja:4–23`, `inertial` e laço de geração. **Correspondência: PARCIAL**. São 125 links **totais**, últimos 10 interpolados; 115 intervalos longos + 9 curtos entre origens; curto≈0.068 m; massa=0.0015 kg; damping=0.1. README ainda apresenta outra contagem, 125 enrolados + 10 livres e massa 0.01. Nenhuma dessas tabelas substitui parsing do SDF.

### Tambor e plataforma

**Descrição no artigo:** §3.1, p.7: cilindro gira para enrolar/desenrolar, com plataforma frontal para takeoff.

**Implementação:** `rs_robot.sdf:575–845` e UGVController. **Correspondência: TOTAL quanto à existência do mecanismo**, PARCIAL quanto à fidelidade/calibração: dois rolamentos fecham loop, inércias do conjunto ausentes, atrito do tambor zero e raio efetivo fixo. O artigo não explica a API de attach.

### Folga e modos RTTA/PTR

**Descrição no artigo:** §5.2 p.15: L 5% maior que distância; §5.3 pp.16–17: adaptação em tempo real versus referência planejada.

**Implementação:** `ugv_theter_trajectory_follower.py::calculate_winch_velocity/target_length_callback`. **Correspondência: PARCIAL**: coef default=1.0, não 1.05; PTR desativado e tratado por coeficiente; variante to_point usa 1.1+0.3 m. Sincronização de waypoint considera alvo dinâmico, não necessariamente comprimento YAML.

### Dinâmica do UAV

**Descrição no artigo:** §3.1 pp.6–7: adaptação principalmente do thrust Z para peso extra; restante inalterado.

**Implementação:** `sjtu_drone.sdf:4–22,248–279`; massa=2.954 kg contra comentário anterior 1.477; inércias também duplicadas em relação às comentadas; maxForce=1800. **Correspondência: PARCIAL**: código atual contém mudanças além de thrust. Plugin simplificado de força/torque, sem PX4, mixer ou rotores articulados neste SDF.

### Forças e tensão

**Descrição no artigo:** §3.1 p.8 e §5.1 pp.11–12: forças de contato, tensão e efeitos sobre os veículos.

**Implementação:** joints/colisões ODE e attach; nenhum sensor de tensão. **Correspondência: PARCIAL** para transmissão mecânica; **NÃO ENCONTRADA** para interface de medição quantitativa de tensão. O artigo não fornece evidência de load cell simulada; não atribuir-lhe uma alegação que não faz.

### Métricas e comparação com catenária

**Descrição no artigo:** §4 pp.9–10: soma de deslocamentos, incrementos positivos/negativos de L, erro de catenária; §5.1 pp.10–12: discretizações e erros médios abaixo de 1% nos cenários reportados.

**Implementação:** get_bag_data*, plot_bag_data*, tether_comparison.py, min_distance_calculator.py. **Correspondência: PARCIAL**: pipeline existe, mas formato de tether mudou de PoseStamped para PoseArray e consumidores não acompanharam. Há um CSV tether_data rastreado, mas faltam os CSVs complementares UAV/UGV e bags necessários à reprodução completa; não reproduzidos os números do paper.

### Validação real e desempenho

**Descrição no artigo:** §5.3 pp.15–17: comparação no teatro; §5.4 p.18: 100→700 elementos, laptop RTF 0.92→0.14, desktop 0.99→0.19; problemas junto ao winch e limite empírico de elemento 0.2 m (§5.1).

**Implementação:** cenários, missões test0…test5, scripts de gráfico, malha teatro armazenada apenas como ponteiro Git LFS neste checkout. **Correspondência: PARCIAL**: ferramentas/figuras disponíveis; reprodução exata não demonstrada para este commit. Mudanças de passo físico, massa/inércia, controladores e telemetria tornam imprópria a transferência automática dos resultados.

> **Claude Code addition — classificação por mecanismo (item 7 do protocolo de auditoria):**
> - **Limite de 0.2 m por elemento:** citação textual do artigo (`analysis_evidence/paper-extracted.txt:489-491`): *"the winch model used in our simulation does not support tether element lengths greater than 0.2 m due to mechanical constraints"*. Busquei em `tether.sdf.jinja` e em `jinja_gen.py` qualquer `assert`, `if cl > 0.2` ou validação equivalente e **não encontrei nenhuma** — o template aceita qualquer `cl` sem checagem. Classificação: **SOMENTE NO ARTIGO** (o limite é uma observação empírica dos autores sobre o comportamento do solver perto do tambor, não uma trava de software). Relevante para X500/PX4: se o carretel for reimplementado, um limite de discretização análogo deve ser descoberto experimentalmente, não assumido a partir de um valor herdado sem checagem.
> - **"Artefato puramente visual junto ao winch":** citação textual (`paper-extracted.txt:472-477`): *"this is purely a visual artifact and does not impact the overall behaviour of the catenary"*. O código não contém nenhuma métrica automatizada que distinga "artefato visual" de "instabilidade real" (não há detector de energia, RTF por região, ou verificação de convergência perto do tambor). Classificação: **SOMENTE NO ARTIGO** — é uma afirmação qualitativa dos autores, não verificável a partir do código estático desta análise, e o próprio Codex já sinalizava (§13) que essa alegação "não foi validada para este commit"; concordo com essa cautela e a reforço: tratar como hipótese não confirmada ao planejar o carretel do X500.

### Resolução de loops DART

**Descrição no artigo:** não encontrada.

**Implementação:** constraints runtime ODE e loop interno do tambor. **Correspondência: NÃO ENCONTRADA** para solução DART; qualquer arquitetura X500 proposta é trabalho novo.

## 12. Código ativo, legado e mapa de arquivos

Classificação é por alcançabilidade no fluxo analisado, não por nome/data apenas. “LEGADO” indica variante fora do caminho principal ou interface obsoleta, não prova de que nunca foi usada.

| Arquivo ou família explicitamente agrupada | Função / situação | Importância | Reutilização potencial |
|---|---|---|---|
| `launch/marsupial_simulation.launch.py` | Bootstrap básico efetivo | CRÍTICO | Sequência conceitual; corrigir readiness |
| `launch/marsupial_manual_simulation.launch.py` | Bootstrap manual com teatro/joy | IMPORTANTE | Referência de operação |
| `launch/marsupial_experiment.launch.py` | Seguimento+bag separado | CRÍTICO | Adaptar orquestração de experimento |
| `launch/marsupial_to_point.launch.py` | Alternativa ativa com outro controller | IMPORTANTE | Somente referência; QoS/startup |
| `launch/ugv_state_publisher.launch.py` | URDF/TF | IMPORTANTE | Adaptar árvore válida |
| `launch/ugv_control.launch.py` | Joystick | SECUNDÁRIO | Referência manual |
| `launch/ugv_sim_controller.launch.py`, `ugv_sim_keyboard.launch.py` | Paths ausentes | LEGADO | Não recomendado |
| `launch/multi_drone_simulation.launch.py` | Experimento linear incompleto | LEGADO | Não tomar por conexão UAV-UAV funcional |
| `launch/bag_record.py` | Gravação | SECUNDÁRIO | Adaptar |
| `models/tether/tether.sdf` | Geometria física carregada | CRÍTICO | Referência numérica, não copiar inércia |
| `models/tether/tether.sdf.jinja` | Gerador da cadeia helicoidal | CRÍTICO | Adaptar profundamente |
| `scripts/jinja_gen.py` | Render offline; overrides ineficazes neste template | CRÍTICO | Adaptar API/validação |
| `scripts/attach_tether.py` | Conecta extremos via WorldPlugin | CRÍTICO | Referência de endpoints, não portar API Classic |
| `src/tether_position_publisher.cpp` | PoseArray por mudança de target | IMPORTANTE | Adaptar taxa, frame e nomes |
| `models/rs_robot/rs_robot.sdf` | UGV, tambor, control/sensores | CRÍTICO | Simplificar topologia |
| `models/rs_robot/config/controllers.yaml` | Ordem de comandos, controllers 250 Hz | CRÍTICO | Adaptar contratos |
| `urdf/rs_robot.urdf` | robot_description usado; dupla filiação do tambor | IMPORTANTE | Não copiar árvore |
| `src/plugin_ugv.cpp`, `include/plugin_ugv.h` | Ground truth, apesar do nome SimpleController | IMPORTANTE | Referência de medição |
| `models/sjtu_drone/sjtu_drone.sdf` | UAV de um link, sensores, plugin externo | CRÍTICO | Manter como referência, não substituir PX4 |
| `scripts/ugv_theter_trajectory_follower.py` | Controle experimental/estimativa por encoder | CRÍTICO | Adaptar e instrumentar |
| `scripts/trajectory_follower.py` | YAML, alvos latched, sincronização | IMPORTANTE | Adaptar esquema temporal |
| `scripts/uav_trajectory_follower.py` | Pose→velocidade | IMPORTANTE | Só referência, PX4 mantém controle |
| `scripts/ugv_to_point.py`, `uav_to_point.py` | Alternativas alcançáveis to_point | IMPORTANTE | Não copiar estimador winch com unidades ambíguas |
| `scripts/ugv_control_controller.py`, `ugv_control_keyboard.py` | Joystick / Twist manual | SECUNDÁRIO | Adaptar |
| `scripts/spawn_ugv.py` | Spawn por serviço, fora do launch principal | SECUNDÁRIO | Referência |
| `scripts/get_bag_data.py` | Bag→CSV; espera PoseStamped antigo | LEGADO | Corrigir antes de usar com PoseArray |
| `scripts/min_distance_calculator.py` | Lidar+tether; espera PoseStamped | LEGADO | Adaptar interface antes de usar |
| `scripts/get_bag_data_theatre.py`, `get_bag_trajectory.py` | Pós-processamento | SECUNDÁRIO | Adaptar paths/dados |
| `scripts/tether_comparison.py` | Catenária offline | IMPORTANTE | Adaptar parâmetros e indexação |
| `scripts/plot_bag_data.py`, `plot_bag_data_theatre.py`, `plot_comparison.py`, `plot_test3.py`, `plot_overview.py`, `plot_efficiency.py` | Gráficos e avaliação offline; alguns não instalados como executáveis | SECUNDÁRIO | Reaproveitar conceitos de métricas |
| `scripts/target_publisher.py`, `msg/TargetPoseUGV.msg`, `msg/TargetPoseUAV.msg` | Interface não gerada e não usada no fluxo ativo | LEGADO | Não recomendado |
| `models/tether/tether_original.sdf` | 20 links, 19 universal, massa explícita 0.18 kg | LEGADO | Histórico de modelagem |
| `models/tether/tether_lineal.sdf`, `tether_lineal.sdf.jinja`, `scripts/attach_tether_lineal.py` | 40 links, 39 universal, 3.9 m entre origens, 0.36 kg; somente um endpoint conectado | LEGADO | Teste simplificado após correções |
| `models/rs_robot/rs_robot_original.sdf`, `rs_robot_test_winch.sdf`, `urdf/rs_robot_original.urdf` | Variantes não spawnadas pelo fluxo principal | LEGADO | Histórico/teste |
| `models/winch/winch.sdf` | Carretel separado não spawnado | LEGADO | Geometria conceitual |
| `urdf/sjtu_drone.urdf` | Não utilizado pelo launch básico | SECUNDÁRIO | Referência descritiva |
| `worlds/theatre.world` | Default ODE 4 ms | CRÍTICO | Referência de solver |
| `worlds/stage_1.world` … `stage_7.world`, `stage_collisions.world` | Cenários alternativos ODE | IMPORTANTE | Adaptar obstáculos e testes |
| `models/teatro/teatro.sdf` | Ponteiro Git LFS de malha e placeholder X Y Z | SECUNDÁRIO | Não reproduzível sem recursos/correção |
| `optimized_path/test0.yaml` … `test5.yaml`, `teatro_*.yaml`, `optimized_path_*.yaml` | Alvos coordenados UAV/UGV/tether | IMPORTANTE | Reutilizar formato conceitual |
| `models/rs_robot/meshes/**`, meshes UAV, model.config | Recursos visuais/colisão | SECUNDÁRIO | Dependem de licença/geometria pretendida |
| `rviz/rviz_config.rviz` | Visualização; diretório não consta no install principal | SECUNDÁRIO | Não essencial à dinâmica |
| `README.md`, `docs/**`, `images/**` | Documentação/figuras demonstrativas | SECUNDÁRIO | Conceitos; parâmetros não autoritativos |
| `CMakeLists.txt`, `package.xml`, `Dockerfile` | Build/dependências | CRÍTICO | Referência da stack, corrigir reprodutibilidade |
| `simulation_data/tether_data.csv` | Amostra de poses do tether, sem conjunto UAV/UGV completo | SECUNDÁRIO | Referência de formato antigo |
| `.gitattributes` | STL via Git LFS; verificar materialização | IMPORTANTE | Reprodução dos assets |
| `paper/2412.12776v3.pdf` | Contexto científico local | IMPORTANTE | Critérios de avaliação |

Parâmetros obsoletos confirmados: `bodyName` e `maxForce` do `ugv_plugin` não são lidos por `UGVSimpleController::Load`; GetLink() é chamado sem nome e o plugin só publica pose. Seu log fala em ~100 Hz, mas no world default são 25 Hz de simulação. Não interpretar esse plugin como força motriz do UGV.

### Recursos rastreados que as regras de ignore ocultam em buscas comuns

A auditoria final usou também `git ls-files`, pois `.gitignore` contém nomes de arquivos que continuam rastreados. Existem missões `teatro_mission`, `teatro_trajectory` e `optimized_path_*`, além de test0…test5, e `simulation_data/tether_data.csv` (7 amostras, 610 colunas). As 610 colunas correspondem a tempo mais 203 posições XYZ, outra discretização em relação aos 125 links ativos; esse CSV não valida o SDF atual. As missões e seus tamanhos estão em `analysis_evidence/resources.json`. Não atribuir a falta do conjunto completo de dados à inexistência desse CSV isolado.

Neste checkout há 16 ponteiros Git LFS não materializados, listados no mesmo inventário. A malha `models/teatro/mesh/teatro.stl` existe como **ponteiro de 134 bytes**, com conteúdo real de 128671884 bytes indicado pelo LFS, não como malha carregável. Usar `git lfs pull` na cópia destinada à execução. O `.sdf` teatral também contém tamanho literal `X Y Z` em visual: materializar LFS não resolve esse placeholder. Não foi necessário baixar a malha de aproximadamente 129 MB para determinar joints ou o mecanismo do tether.

## 13. Parâmetros numéricos dos worlds e custo computacional

| World | Engine | max_step_size | real_time_update_rate | Configuração adicional |
|---|---|---|---|---|
| theatre | ODE | 0.004 s | 250 Hz | quick, 80 iterações, SOR 1.0, CFM 0, ERP 0.2, contact_max_correcting_vel 100, surface_layer 0.001 |
| stage_1, stage_5 | ODE | 0.001 s | 1000 Hz | Defaults do engine para campos omitidos |
| stage_2,3,4,6,7,collisions | ODE | 0.002 s | 500 Hz | Defaults do engine para campos omitidos |

Todos pedem real_time_factor=1; isso é **meta**, não resultado observado. Frequências físicas em sim-time diferem das frequências de parede quando RTF<1. Controllers Python usam timer 50 Hz, mas launch experimental não passa `use_sim_time` aos seguidores; get_clock() será wall/ROS-time sem clock override ativado. to_point usa time.time explicitamente. Logo queda de RTF pode dessintonizar integração de L/comandos e física.

Custo básico: 125 corpos leves com 124 joints de dois eixos, 123 cilindros de colisão e 125 plugins de arrasto, além do UGV e sensores GPU. Reduzir passo aumenta custo; aumentar discretização acrescenta constraints, contatos e callbacks. Enrolamento concentra geometrias e razões de massa elevadas (link cabo 0.0015 kg versus base UGV 175 kg). Inércias artificialmente grandes, colisões deslocadas e dual bearing podem aumentar problemas numéricos. O artigo relata degradação de RTF e comportamento irregular junto ao winch; a alegação de que isso é apenas visual não foi validada para este commit.

## 14. Execução e validação realizadas

### Ambiente detectado

ROS 2 Humble em `/opt/ros/humble`; Gazebo Classic `11.14.0` (pacote gazebo11 11.14.0-1~beta1). libsdformat9 9.9.1 e bibliotecas 12/13 também instaladas; presença múltipla não fixa a versão usada por cada binário. DART 6.12.1 instalado, **não selecionado pelos worlds da referência**. colcon e docker disponíveis. O shell tinha overlay MoveIt; build foi isolado em diretório de suporte, sem alterar esse overlay.

Consulta ao ament index: gazebo_ros, gazebo_ros2_control, gazebo_ros_link_attacher, sjtu_drone, controller_manager e velodyne_gazebo_plugins não encontrados. Ter o executável gazebo não implica disponibilidade da stack ROS/Gazebo do projeto.

### Comando de build realmente executado

```bash
colcon --log-base /home/lima/codes/ic/refs/marsupial_analysis_support/log build \
  --base-paths /home/lima/codes/ic/refs/marsupial_simulator_ros2 \
  --build-base /home/lima/codes/ic/refs/marsupial_analysis_support/build \
  --install-base /home/lima/codes/ic/refs/marsupial_analysis_support/install \
  --cmake-args -DBUILD_TESTING=OFF
```

**Resultado:** falha CMake em `find_package(gazebo_dev REQUIRED)`, CMakeLists.txt:25; 0 packages concluídos. Log preservado em `analysis_evidence/build.log`. Não foram instalados pacotes de sistema nem alterada implementação para mascarar falha.

`ros2 launch marsupial_simulator_ros2 marsupial_simulation.launch.py --show-args` também foi tentado: pacote não instalado/encontrado, como esperado. Não houve execução integrada, takeoff observado, lista de tópicos de uma instância da referência, medição de RTF ou validação de estabilidade/força. A tabela de interfaces deste relatório é derivada de código.

### Validações que puderam ser concluídas

- Parsing XML dos modelos/worlds/URDF e reconstrução das arestas com identificação do child duplicado.
- Contagem de links/joints, massa explícita, geometrias e distâncias entre origens do tether.
- Render Jinja em memória: idêntico ao SDF rastreado; overrides citados não modificam saída.
- `gz sdf -k models/tether/tether.sdf`: **Valid**.
- `gz sdf -k models/rs_robot/rs_robot.sdf`: warnings sobre ros2_control não definido no schema e erros de nomes não únicos dentro das interfaces dos joints. Esses diagnósticos são de validação SDF local, não observação de crash do solver.
- Leitura integral do texto do PDF e confronto das seções técnicas/validação com os fluxos atuais.

Os outputs estão em `analysis_evidence/validation.log` e `generator-check.txt`. Build não atingiu install. O diretório `simulation_data` existe e contém `tether_data.csv`; não é causa da falha do build.

### Receita de reprodução futura, em workspace independente

Comandos abaixo são orientação derivada do Dockerfile/launch, **não uma execução bem-sucedida desta análise**:

```bash
mkdir -p /home/lima/codes/ic/refs/marsupial_runtime_ws/src
cd /home/lima/codes/ic/refs/marsupial_runtime_ws/src
git clone https://github.com/robotics-upo/marsupial_simulator_ros2.git
git -C marsupial_simulator_ros2 checkout d9046774cada2f0b679fb0dfdc1857516fc36936
git clone -b ros2 https://github.com/noshluk2/sjtu_drone.git
git clone -b humble-devel https://github.com/davidorchansky/gazebo_ros_link_attacher.git
git clone -b humble https://github.com/ros-simulation/gazebo_ros2_control.git
cd ..
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
# Resolver também dependências omitidas, plugins externos e paths identificados neste relatório.
# Corrigir somente nessa cópia experimental: YAML absoluto, materialização de arquivos Git LFS,
# URDF do carretel, readiness/manager e recursos opcionais antes de assumir reprodução.
colcon build --symlink-install
source install/setup.bash
export GAZEBO_MODEL_PATH="/home/lima/codes/ic/refs/marsupial_runtime_ws/src/marsupial_simulator_ros2/models:${GAZEBO_MODEL_PATH:-}"
ros2 launch marsupial_simulator_ros2 marsupial_simulation.launch.py world:=stage_1.world
```

Em outro terminal, source de `/opt/ros/humble/setup.bash` e do install do workspace, depois `ros2 launch marsupial_simulator_ros2 marsupial_experiment.launch.py mission:=test1`. Para controle manual, escolher `marsupial_manual_simulation.launch.py`, resolvendo antes teatro/joy/xterm. Não iniciar dois modos de controle simultaneamente sobre os mesmos tópicos.

Inspecionar `/attach`, resultados ok dos dois endpoints, `ros2 control list_controllers`, `ros2 topic list -t`, `/joint_states`, `ros2 topic info /tether_positions -v` e `ros2 topic echo /cable_length`. Esse é checklist de validação pendente, não lista de observações realizadas.

## 15. Decisões, pressupostos e riscos de transferência

1. **Engine:** APIs `gazebo::physics::ModelPtr`, ModelPlugin/WorldPlugin e `gazebo_ros::Node` são Classic. Gazebo Sim usa outro ciclo de vida e componentes; bibliotecas `.so` não são plug-and-play.
2. **Topologia:** duplo rolamento e attach ao child final não obedecem à árvore esperada no problema relatado. URDF e SDF também não são equivalentes para loops.
3. **Fidelidade:** massa por unidade de comprimento não uniforme na transição; inércias artificiais; colisões desalinhadas; COM na origem; parâmetros de contato não calibrados.
4. **Comprimento:** estimativa baseada em raio constante e L0 arbitrário não prova metragem livre. Não há limites de payout nem reserva de cabo monitorada.
5. **Tensão:** não é observada; controle de slack por distância não é controle de força. Com obstáculos, distância Euclidiana pode subestimar fortemente o caminho do cabo.
6. **Frames:** offsets sem rotação, endpoint comentado divergente do SDF, PoseArray com frame errado e TF incompleto.
7. **Tempo/QoS:** sim_time não propagado, publicação de alvos one-shot versus subscrição transient-local, posição de tether somente por mudança de target.
8. **Telemetry:** consumidores PoseStamped não suportam PoseArray atual; contratos Float64MultiArray variam por controller.
9. **Startup:** atrasos fixos sem sucesso explícito, manager duplicado, paths absolutos, recursos ausentes, dependências não fixadas.
10. **Aeronave:** SJTU simplificado não valida controle PX4/X500, saturação de motores, estimador ou convenções NED/ENU/FRD/FLU.
11. **Custo:** cadeia enrolada inteira custa simulação mesmo com pouco cabo exposto; escalar quantidade sem refazer parâmetros pode piorar precisão.
12. **Evidência científica:** validação do artigo está ligada a outra configuração; comparação geométrica de catenária não identifica corretamente inércia, transientes nem força em endpoints.

## 16. Classificação de portabilidade para Gazebo Sim + DART

> **Claude Code addition.** O documento original já discute dependências do Classic de forma dispersa (§3, §6, §15). Esta seção consolida, mecanismo a mecanismo, a classificação pedida explicitamente para a transferência ao X500/PX4 (Gazebo Sim + DART). Evidência: leitura direta do código listado; nenhuma execução em Gazebo Sim foi realizada nesta tarefa (não há Gazebo Sim/gz-sim instalado neste ambiente de análise, apenas Gazebo Classic 11.14.0 e DART 6.12.1 como biblioteca de terceiros não selecionada por nenhum world da referência — §14). Classificações abaixo são portanto análise estática de API, não teste comparativo.

| Mecanismo da referência | Dependência identificada | Classificação | Motivo |
|---|---|---|---|
| Cadeia articulada do tether (links+`universal` joints) | Conceito de multibody com joints de 2 DOF; **não** depende de API Classic específica | **PORTÁVEL COM ADAPTAÇÃO** | SDFormat `universal` joint é suportado pelo parser comum a Classic e Sim; a física real passa a ser resolvida por DART, cujo tratamento numérico de `cfm_damping`/`spring_stiffness` difere do ODE. Inércias artificiais (`I=0.01` fixo) e colisões deslocadas precisam ser recalculadas, não copiadas. |
| `libLiftDragPlugin.so` (arrasto por link) | Plugin Classic (`gazebo::ModelPlugin` sobre `physics::LinkPtr` da API antiga) | **PRECISA SER REIMPLEMENTADO** | Gazebo Sim usa a arquitetura `gz-sim::System` (ECS) com plugins novos; não há garantia de um `LiftDragPlugin` binário compatível pronto para uso — mesmo que exista um equivalente em `gz-sim`, a interface de carregamento/parâmetros SDF muda. Reimplementar como `System` do Gazebo Sim ou aplicar força equivalente via API ECS. |
| `libgazebo_ros_link_attacher` (`/attach`, `/detach`, joint revolute travado) | API Classic pura: `physics::ModelPtr`, `CreateJoint`, `SetModel` (ver comentário de crash documentado acima) | **INCOMPATÍVEL / NÃO RECOMENDADO** | Todas as chamadas são da árvore de classes Classic (`gazebo::physics::*`), inexistente no Gazebo Sim. O padrão equivalente do lado Sim é `gz::sim::systems::DetachableJoint`, que **exige** que o link a ser destacado seja filho em uma árvore já válida no SDF (documentação citada pelo Codex, §7) — não aceita o mesmo truque de `SetModel` para reatribuir parentesco entre modelos distintos pós-spawn. Não portar o binário nem a lógica; usar apenas como referência conceitual de "conectar modelos spawnados separadamente". |
| Duplo rolamento do tambor (`joint_izquierdo`/`joint_derecho` → mesmo `box_central`) | Loop cinemático não-árvore, tolerado pelo solver de constraints do ODE Classic | **INCOMPATÍVEL / NÃO RECOMENDADO** | DART é fundamentalmente baseado em árvores (`dart::dynamics::Skeleton` com um pai por corpo); loops fechados exigem constraints explícitas adicionais (`dart::constraint`), não dois joints declarados normalmente no SDF. Reimplementar como um único joint estrutural + (opcionalmente) uma peça visual/fixa não articulada do segundo "apoio", exatamente como o Codex já recomenda em "Mecanismo: carretel físico". |
| `ros2_control` / `gazebo_ros2_control::GazeboSystem` | Plugin de integração ros2_control com Classic | **PORTÁVEL COM ADAPTAÇÃO** | Existe uma via de integração equivalente para Gazebo Sim (`gz_ros2_control` / `ign_ros2_control`, nomes variam por distro); os `command_interface`/`state_interface` do SDF são conceitualmente reaproveitáveis, mas o `<plugin>` de hardware precisa apontar para o pacote do Gazebo Sim, não `gazebo_ros2_control`. Não presumir nome de pacote sem checar a distro instalada no X500. |
| `TetherPositionPublisher` / `UGVSimpleController` (plugins C++ locais deste repositório) | Herdam de `gazebo::ModelPlugin`, usam `event::Events::ConnectWorldUpdateBegin`, `physics::LinkPtr::WorldPose()` | **PRECISA SER REIMPLEMENTADO** | API de plugin e ciclo de eventos mudam por completo no Gazebo Sim (sistemas ECS com `PreUpdate`/`PostUpdate` e `EntityComponentManager`). A lógica (ordenar links, publicar `PoseArray`/`Pose` por evento) é perfeitamente portável como **conceito**, não como código-fonte. |
| Controle de winch em Python (`UGVController`, PID sobre `/joint_states`) | Somente rclpy/ROS 2 puro, nenhuma API Gazebo direta | **PORTÁVEL DIRETAMENTE** | Não toca em nenhuma API específica do motor de física; consome apenas tópicos ROS 2 padrão. Ainda assim, herda os problemas de unidade/offset já descritos (§8) — portável como esqueleto de nó, não como implementação pronta para uso sem correções. |
| Comparação com catenária offline (`tether_comparison.py`, `pycatenary`) | Python puro, pós-processamento de bag | **PORTÁVEL DIRETAMENTE** | Independente de motor de física; útil como está para qualquer novo pipeline de validação, inclusive X500/PX4. |
| `sjtu_drone`/`plugin_drone` (dinâmica do UAV) | Plugin Classic aplicando força/torque | **INCOMPATÍVEL / NÃO RECOMENDADO** (para este projeto) | Não é sobre portabilidade técnica per se, mas sobre escopo: o X500/PX4 já tem sua própria pilha de dinâmica/controle (PX4 SITL); substituí-la pelo controlador simplificado do SJTU regrediria fidelidade de voo. Mantido apenas como referência de arquitetura de acoplamento tether↔veículo aéreo. |

**Síntese:** nenhum dos mecanismos de conexão estrutural (attach, duplo rolamento) é portável como código; a cadeia multibody e o controle de alto nível (Python/ROS 2) são portáveis como conceito com adaptação de parâmetros físicos; os plugins C++ locais são portáveis como especificação de comportamento, exigindo reescrita completa para a API ECS do Gazebo Sim.

# Transferência para o projeto X500/PX4 + tether

Esta seção é uma proposta para uma sessão que conhece `drone-cabo`; não depende de conhecimento oculto sobre o código desse projeto. As hipóteses específicas de DART vêm do problema informado pelo usuário e devem ser confrontadas com sua versão/API.

## Mecanismo: cadeia multibody

**Como funciona na referência:** corpos rígidos cilíndricos com universal de dois eixos, gravidade, contato e arrasto; quantidade fixa. **Arquivos relevantes:** tether.sdf.jinja, tether.sdf. **Motivação:** representar catenária e colisões sem resolver um contínuo customizado. **Dependências:** engine com joints/contatos suportados e gerador SDF.

**Aplicabilidade ao X500/PX4:** boa como estratégia geral para cabo com colisões. **Classificação: ADAPTAR.** **Adaptação sugerida:** definir densidade linear física, massas e inércias consistentes para cada comprimento, COM e colisão no mesmo referencial; selecionar universal/ball conforme backend; explicitar self-contact desejado; validar primeiro poucos segmentos e extremos fixos, depois acoplar PX4. Não preservar I=0.01 por link sem estudo.

## Mecanismo: conexão de endpoints

**Como funciona na referência:** dois modelos de veículo e um de cabo, joints revolute bloqueados criados em runtime pelo WorldPlugin. **Arquivos relevantes:** attach_tether.py e dependência gazebo_ros_link_attacher::attach. **Motivação:** conectar modelos já spawnados sem um URDF monolítico. **Dependências:** API Classic/ODE.

**Aplicabilidade ao X500/PX4:** apenas princípio de desacoplar spawn e conexão. **Classificação: USAR COMO REFERÊNCIA.** **Adaptação sugerida:** para evitar segundo parent na mesma árvore, estudar **uma única hierarquia estação → cabo → UAV**, com somente uma ligação estrutural por child e sem outra ancoragem independente que feche ciclo. Confirmar que o X500 pode ser anexado como child sem afetar a integração PX4; não há demonstração disso na referência. Alternativamente usar constraint suportada pela versão de gz-physics que não demande reparenting. Separar modelos por si só não basta.

## Mecanismo: fechamento por forças quando a árvore não pode mudar

**Como funciona na referência:** este mecanismo **não existe**; forças de endpoint não substituem joints. **Arquivos relevantes:** ausência em src/ e attach_tether.py; fonte externa confirma joint estrutural. **Motivação:** hipótese para contornar limitação específica mantendo árvore atual. **Dependências:** acesso à pose/velocidade dos endpoints e aplicação de wrench no backend X500.

**Aplicabilidade ao X500/PX4:** alternativa experimental quando constraint estrutural suportada não é viável. **Classificação: USAR COMO REFERÊNCIA** para o problema, não reutilização de código. **Adaptação sugerida (HIPÓTESE):** acoplamento regularizado com forças iguais/opostas e torques r×F nos pontos corretos; distinguir conexão bilateral de cabo unilateral (não transmitir compressão onde não faz sentido); tratar dissipação, integração, rigidez numérica e erro de fechamento. Não alegar equivalência física a um joint exato ou estabilidade antes de testes de energia, carga estática e transientes.

## Mecanismo: carretel físico e comprimento variável

**Como funciona na referência:** toda a cadeia começa enrolada em torno de tambor, que gira preso ao UGV e a link_0. **Arquivos relevantes:** rs_robot.sdf, tether.sdf.jinja, ugv_theter_trajectory_follower.py. **Motivação:** variar comprimento livre mantendo discretização e massa constantes. **Dependências:** contatos robustos, solver capaz e reserva de cabo simulada.

**Aplicabilidade ao X500/PX4:** interessante se enrolamento real for objeto do estudo, caro para apenas reproduzir payout. **Classificação: ADAPTAR.** **Adaptação sugerida:** um único joint do tambor ao suporte; segundo apoio somente visual ou mesma peça rígida, sem segundo parent. Calibrar massa/inércia/raio efetivo e contabilizar reserva, comprimento exposto e total separadamente. Começar com cabo de comprimento fixo até resolver endpoints. Não recomendar criação/destruição de links como técnica extraída desta referência: ela não a implementa.

## Mecanismo: encoder e controle de comprimento

**Como funciona na referência:** θ de joint_states integrado por L=L0+rΔθ, fallback por comando, PID de comprimento com raio fixo. **Arquivos relevantes:** UGVController::_joint_state_cb/calculate_winch_velocity e controllers.yaml. **Motivação:** realimentar a rotação realmente simulada em vez de só comando. **Dependências:** sensor/estado do joint e tempo simulado.

**Aplicabilidade ao X500/PX4:** útil com instrumento equivalente do winch. **Classificação: ADAPTAR.** **Adaptação sugerida:** separar `L_total`, `L_free_measured`, `L_encoder_estimated`, `L_target`; inicializar geometricamente, unwrap angular, limitar reserva e garantir unidades rad/s↔m/s. Publicar mensagem com Header/campos nomeados. Rotacionar offsets do corpo, medir falha/deslizamento e usar sim_time. Não chamar esse PID de controle de tensão.

## Mecanismo: propulsão e controle UAV

**Como funciona na referência:** libplugin_drone aplica forças/torques em um único link e recebe cmd_vel, takeoff/land. **Arquivos relevantes:** sjtu_drone.sdf e plugin_drone_private.cpp externo. **Motivação:** voo simplificado para estudar sistema marsupial. **Dependências:** controlador SJTU/Classic.

**Aplicabilidade ao X500/PX4:** não reproduz arquitetura PX4. **Classificação: NÃO RECOMENDADO** substituir o controle/motores X500. **Adaptação sugerida:** preservar PX4, sensores e atuadores do X500; introduzir somente interação mecânica/força do tether, depois validar hover, saturação e recuperação.

## Mecanismo: observabilidade e validação

**Como funciona na referência:** poses ground truth, comprimento estimado, bags e catenária offline; sem wrench. **Arquivos relevantes:** tether_position_publisher.cpp, get_bag_data.py, tether_comparison.py, trajectory_follower.py. **Motivação:** separar efeitos de geometria, controle e desempenho. **Dependências:** logger compatível e referência analítica.

**Aplicabilidade ao X500/PX4:** forte valor metodológico. **Classificação: ADAPTAR.** **Adaptação sugerida:** publicação periódica em sim-time, frame world correto e IDs de links; sensores de reação nos dois endpoints; curvas T(t), L(t), erro de fechamento, energia e RTF. Validar catenária estática, cabo tenso, obstáculo, payout/retract, manobra e timestep convergence. Resultados de catenária do artigo são referência comparativa, não tolerância garantida do X500.

## Ordem recomendada de trabalho para a próxima sessão

1. Registrar versões exatas de Gazebo Sim, gz-physics/DART, PX4 e topology atual; localizar qual chamada tenta dar o segundo parent.
2. Fazer protótipo mínimo de dois endpoints e poucos links com somente um caminho estrutural; decidir árvore reorientada versus constraint adicional realmente suportada.
3. Instrumentar forças e erro de fechamento antes de incluir winch; confirmar hover e ação/reação com PX4 intacto.
4. Calibrar massa, inércia, discretização e passo; validar cabo livre/obstáculos.
5. Só então adicionar payout, reserva de cabo e controle; usar um rolamento estrutural no tambor.
6. Portar logging/métricas com contratos novos e sim-time, sem carregar as inconsistências da referência.

# HANDOFF PARA OUTRA SESSÃO CODEX

**Referência:** https://github.com/robotics-upo/marsupial_simulator_ros2 — checkout `/home/lima/codes/ic/refs/marsupial_simulator_ros2`, branch main.

**Commit:** `d9046774cada2f0b679fb0dfdc1857516fc36936`. Análise: 2026-09-04. Artigo: `paper/2412.12776v3.pdf`, arXiv v3 de 28/07/2025, anterior ao código.

**Estratégia de tether:** Gazebo Classic/ODE; cadeia fixa de 125 links, 124 universal; massa total explícita 0.1875 kg; inércias artificiais I=0.01 por eixo/link. Distância total entre origens inicial≈17.862586 m; não 125×0.15. Cilindros físicos raio 0.004 m, últimos segmentos≈0.068 m; 125 plugins de arrasto.

**Topologia:** três modelos (rs_robot com winch, tether, sjtu_drone). Tether interno link_0→…→link_123→link_final. Tambor tem dois revolute com mesmo child box_central, formando loop no UGV.

**Como evita loop cinemático:** **não evita por árvore**. Usa constraints Classic/ODE; não demonstra correção para segundo parent DART. Separar modelos não basta. Não copiar duplo rolamento nem presumir equivalência com DetachableJoint Gazebo Sim.

**Como conecta ao veículo:** `/attach` une sjtu_drone/base_link a tether/link_final; plugin externo cria revolute travado em zero. Não reorienta a árvore do cabo. Link-attacher inspecionado em `2879cf838565a2603bf03ba4f1ea202965ad0304`.

**Como conecta à ground station:** segundo `/attach` une rs_robot/box_central a tether/link_0. Estação móvel sobre rodas, sem joint cabo→world. Plataforma de decolagem é contato, não conexão rígida UAV→plataforma.

**Como trata comprimento variável:** gira joint_izquierdo de tambor raio 0.1 m; cadeia inteira permanece simulada; não cria/destrói links. Comprimento livre estimado de L0+rΔθ ou fallback por integração do comando. RTTA default coef=1.0, margem=0, não 5% de folga do artigo; PTR desativado por constante Python.

**Como mede tensão:** não mede nem publica; reações ficam no solver. JointState effort não é conversão implementada para tensão. Comparação de catenária é geométrica offline.

**Arquivos mais importantes:**

- `models/tether/tether.sdf.jinja` e `tether.sdf` — parâmetros, geometria e cadeia.
- `models/rs_robot/rs_robot.sdf` — tambor, loop, ros2_control e paths absolutos.
- `scripts/attach_tether.py` — endpoints e ordem de montagem.
- `scripts/ugv_theter_trajectory_follower.py` — controle/estimativa L.
- `launch/marsupial_simulation.launch.py` e `marsupial_experiment.launch.py` — fluxos separados.
- `src/tether_position_publisher.cpp` — PoseArray por evento, WorldPose com frame_id incorreto.
- `MARSUPIAL_TETHER_REFERENCE_ANALYSIS.md` e `analysis_evidence/` — análise e provas.

**Componentes recomendados para adaptar:** discretização com inércias físicas, conceito de encoder/controle L, logging periódico corrigido, comparação de catenária, testes com obstáculos.

**Componentes não recomendados:** attach Classic como solução DART, duplo parent do carretel, inércias 0.01 copiadas, controller to_point com mistura de unidades, substituição do PX4 por SJTU, consumidores PoseStamped para o publisher PoseArray.

**Principal recomendação para X500/PX4:** resolver primeiro topologia/constraint suportada mantendo PX4; estudar única árvore estação→cabo→UAV ou acoplamento específico do backend. Simplificar carretel para um joint e instrumentar wrench antes de comprimento variável. Checar versão: recursos non-tree recentes não podem ser presumidos presentes no projeto.

**Validação e limites:** parsing e render validados; override Jinja `--var` não surte efeito. Build tentado e bloqueado por gazebo_dev ausente; stack ROS/Gazebo adicional também ausente. Simulação integrada e estabilidade não validadas. Código principal permaneceu intacto. **Handoff documental: PRONTO; validação dinâmica: PENDENTE explicitamente delimitada.**

# REVISÃO CRUZADA — CODEX × CLAUDE CODE

Segunda análise independente realizada em 2026-09-04/05, mesmo checkout (`/home/lima/codes/ic/refs/marsupial_simulator_ros2`, branch `main`, commit `d9046774cada2f0b679fb0dfdc1857516fc36936` — **confirmado igual** ao documentado pelo Codex via `git log -1`, nenhuma divergência de versão). Metodologia: leitura direta dos arquivos-fonte citados pelo Codex (`tether.sdf.jinja`, `rs_robot.sdf`, `urdf/rs_robot.urdf`, `attach_tether.py`, `plugin_ugv.cpp/h`, `tether_position_publisher.cpp`, `ugv_theter_trajectory_follower.py`, `CMakeLists.txt`, `package.xml`, `marsupial_simulation.launch.py`, `multi_drone_simulation.launch.py`, `controllers.yaml`), reexecução independente em Python das fórmulas geométricas do gerador do tether (fora do Jinja), leitura do código-fonte externo já clonado pelo Codex em `marsupial_analysis_support/gazebo_ros_link_attacher` (mesmo SHA `2879cf838565a2603bf03ba4f1ea202965ad0304`), e busca textual dirigida em `analysis_evidence/paper-extracted.txt`.

**Informações do Codex confirmadas (sem alteração de conteúdo, apenas evidência adicional):**
- Engine Gazebo Classic/ODE, três modelos independentes (rs_robot, tether, sjtu_drone) — §2, §6.
- Loop cinemático no tambor do UGV (`box_central` como child de dois joints revolute) — §6, agora confirmado também no URDF (`urdf/rs_robot.urdf:225-299`), não só no SDF.
- Mecanismo de attach runtime via `gazebo_ros_link_attacher`, joint revolute travado em zero, sem `CreateJoint("fixed")` — §6, confirmado linha a linha no SHA externo.
- Todos os valores numéricos do tether (125 links, 124 joints, massa 0.0015 kg/link, massa total 0.1875 kg, 123 colisões cilíndricas, cr=0.004 m, distância total ≈17.862586 m, offset do endpoint do UAV) — §5, **reproduzidos por execução independente**, não apenas por leitura, com concordância a 9+ dígitos significativos.
- Ausência de qualquer medição/publicação de tensão no código — §9, agora também confirmada como ausente no artigo (busca textual não encontrou "load cell"/"force sensor"/"wrench"/"torque").
- Limite empírico de 0.2 m por elemento e "artefato visual" perto do winch — §11, ambos confirmados como citações textuais exatas do artigo, e confirmados como **não codificados** (sem checagem em `tether.sdf.jinja`/`jinja_gen.py`).
- Dependências ausentes do `package.xml` (gazebo_dev, gazebo_msgs, gazebo_ros_link_attacher, ament_index_python, robot_state_publisher, joy, jinja2, numpy, pyyaml) — §3, confirmado por leitura direta do `package.xml` e `CMakeLists.txt`.
- Sequenciamento de `marsupial_simulation.launch.py` (OnProcessExit sem checar exit code, `delayed_spawn_uav_node` declarado e não usado, `spawn_theatre` comentado) — §4, confirmado linha a linha.
- `multi_drone_simulation.launch.py` não conecta o segundo UAV (`sjtu_drone_2`) via `attach_tether_lineal.py` — §4, confirmado por leitura do script de attach linear.

**Informações complementadas pelo Claude Code:**
- Comentário de projeto em `rs_robot.sdf:771-774` que documenta explicitamente a intenção por trás do duplo rolamento e por que `spring_stiffness=0` é crítico — evidência de que o loop é intencional, não acidental.
- Comentário de crash histórico em `gazebo_ros_link_attacher.cpp` explicando tecnicamente por que `SetModel()` é necessário (assertions internas do Gazebo Classic/ODE) — evidência de que o mecanismo de attach é um contorno de limitação de API específica do Classic.
- Declaração explícita de `state_interface name="effort"` em `ros2_control` para `joint_izquierdo` e as quatro rodas (`rs_robot.sdf:866-914`) — interface potencialmente populável mas sem consumidor no fluxo Python ativo; caminho candidato para instrumentação futura de tensão.
- Citações textuais exatas do artigo para o limite de 0.2 m e o "artefato visual" do winch, com números de linha em `analysis_evidence/paper-extracted.txt`.

**Novos mecanismos identificados:** nenhum mecanismo estrutural novo além dos já documentados pelo Codex; as adições acima são aprofundamento de mecanismos já mapeados, não descoberta de fluxos alternativos de execução.

**Divergências encontradas:** nenhuma. Todos os números, caminhos de arquivo, classificações ATIVO/LEGADO e conclusões centrais revisados de forma independente coincidiram com o documento do Codex. Não foi necessário abrir nenhuma subseção "Divergência identificada".

**Divergências resolvidas:** N/A (nenhuma divergência encontrada).

**Divergências ainda abertas:** N/A.

**Lacunas do Codex preenchidas nesta revisão:** ausência de uma classificação explícita mecanismo-a-mecanismo de portabilidade Gazebo Sim/DART (item 17 do protocolo desta tarefa) — adicionada na nova §16 "Classificação de portabilidade para Gazebo Sim + DART", já que o documento original discutia portabilidade de forma qualitativa e dispersa (§3, §6, §15) mas sem uma tabela objetiva PORTÁVEL/ADAPTAR/REIMPLEMENTAR/INCOMPATÍVEL.

**Impactos para X500/PX4:**
- A confirmação por execução independente da geometria do tether eleva a confiança dos parâmetros de §5 de "leitura de código" para "comportamento reproduzido" (topo da hierarquia de evidências do protocolo), o que é relevante caso esses números venham a ser usados como ponto de partida (ainda que adaptados) para o tether do X500.
- A nova §16 fornece uma decisão rápida por mecanismo (portar/adaptar/reimplementar/descartar) que pode ser usada diretamente como checklist pela sessão Codex responsável pelo X500/PX4.
- A confirmação de que nem o código nem o artigo desta referência implementam ou descrevem qualquer solução para loops cinemáticos em árvores tipo-DART reforça, com maior confiança ainda, que a solução para o X500/PX4 é trabalho original — não deve ser procurada nesta referência.

**Status da consolidação:** CONFIRMAÇÃO AMPLA, sem reescrita de conclusões anteriores. Nenhuma informação do Codex foi removida, substituída ou contestada.
