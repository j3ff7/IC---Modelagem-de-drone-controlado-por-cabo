# Experiments

Este arquivo registra experimentos técnicos para evitar repetição de investigações já realizadas.

## Experimento: Validação Unitária Dos Ângulos Do Cabo

Data:
2026-08-15 ou anterior, conforme histórico do branch.

Objetivo:
Validar matematicamente as funções de azimuth/elevation.

Hipótese:
Com a convenção `x` frente, `y` esquerda, `z` cima, os casos geométricos simples devem ter valores previsíveis.

Configuração:
Testes em `src/pacote_do_drone/test/test_angulos_cabo.py`.

Parâmetros relevantes:

```text
azimuth   = atan2(y, x)
elevation = atan2(-z, sqrt(x^2 + y^2))
```

Alterações realizadas:
Testes cobrem extração de juntas, saturação perto de 90 graus, cabo horizontal/vertical, drone inclinado e vetor mundo transformado para frame do drone.

Resultado:
Testes estão presentes no repositório. Execução deve ser feita com `colcon test`.

Conclusão:
A base matemática da convenção angular está coberta por testes unitários.

Próxima ação:
Executar testes após mudanças em `cabo_angulos.py` ou `sensores.py`.

## Experimento: Mundos Estáticos Com Postes

Data:
Commit `d007c79 Add cable angle validation environments`.

Objetivo:
Validar os ângulos do cabo sem a dinâmica do drone.

Hipótese:
Mundos com postes em posições conhecidas permitem validar sinais, frames e convenção de azimuth/elevation.

Configuração:

- Pacote `cabo_avaliacao`.
- Configuração padrão em `src/cabo_avaliacao/config/postes_padrao.json`.
- Launch `ros2 launch cabo_avaliacao avaliar_cabo.launch.py caso:=s modo_cabo:=reto`.

Parâmetros relevantes:

```text
poste_altura = 1.2 m
cabo_comprimento = 2.0 m
ancora = (0.0, 0.0, 0.05) m
sensor_yaw_graus = 90 deg
```

Alterações realizadas:
Implementados modos `reto`, `articulado` e `catenaria`.

Resultado:
O modo `reto` valida sinais; o modo `catenaria` permite avaliar geometria com folga; o modo `articulado` carrega o cabo do drone de forma estática, mas não produz catenária dinâmica.

Conclusão:
Usar `cabo_avaliacao` antes de validar no drone quando houver mudança em frames ou sinais.

Próxima ação:
Manter tabelas esperadas sincronizadas com a convenção.

## Experimento: `cmd_vel_frame` Do Controlador

Data:
Contexto do chat anterior, antes desta documentação.

Objetivo:
Determinar se `/meu_drone/cmd_vel` deve ser interpretado no frame global ou no frame do corpo.

Hipótese:
O plugin `MulticopterVelocityControl` interpreta velocidades lineares no frame do corpo.

Configuração:
Comparações de comportamento do controlador com e sem transformação para frame `body`.

Parâmetros relevantes:

```text
cmd_vel_frame:=body
usar_velocidade_por_diferenca:=true
```

Alterações realizadas:
O controlador passou a converter a velocidade desejada do mundo para o frame do corpo quando `cmd_vel_frame == body`. O launch atual usa `body` por padrão.

Resultado:
O comportamento melhorou em relação às alternativas testadas.

Conclusão:
Preservar `cmd_vel_frame:=body` nos ensaios atuais.

Próxima ação:
Só reavaliar se houver mudança de plugin/modelo de drone.

## Experimento: Controlador Sem Cabo E Com Cabo Próximo Ao Waypoint

Data:
Contexto do chat anterior.

Objetivo:
Separar falhas do controlador de falhas causadas pelo tether.

Hipótese:
Se o controlador funcionar sem cabo e com cabo em condição moderada, a instabilidade do spawn original provavelmente vem da interação inicial com o tether.

Configuração:
Testes com `trajetoria_hover_z2_unico.json` e variações de `usar_cabo` e spawn.

Resultado:

```text
Sem cabo + spawn original:
  hover estável
  roll/pitch máx. ~4.8 deg

Sem cabo + spawn próximo:
  hover estável

Com cabo + spawn próximo:
  hover estável
  tensão máxima ~0.35 N

Com cabo + spawn original:
  instável
  pitch próximo de ±90 deg
  tensão máxima ~6 N
```

Conclusão:
O controlador básico não é a primeira suspeita; a condição inicial drone-tether deve ser investigada.

Próxima ação:
Criar condição inicial slack fisicamente compatível.

## Experimento: Aumento Temporário Do Comprimento Do Cabo

Data:
2026-08-15, contexto recente.

Objetivo:
Verificar se aumentar o comprimento do tether elimina a instabilidade do spawn original.

Hipótese:
Com `L=2.5 m` ou `L=3.0 m`, a folga geométrica reduziria picos de tensão.

Configuração:

```text
ancora = (0.0, 0.0, 0.33) m
spawn  = (2.0, 0.0, 0.33) m
massa total do cabo ~= 0.30 kg
```

Parâmetros relevantes:

```text
L=2.0 m: num_links=40, massa_segmento=0.007375 kg
L=2.5 m: num_links=50, massa_segmento=0.00588 kg
L=3.0 m: num_links=60, massa_segmento=0.004883 kg
```

Alterações realizadas:
Comprimentos foram aplicados temporariamente no JSON e o cabo foi regenerado para testes; a configuração permanente voltou para `L=2.0 m`.

Resultado:

```text
L=2.0 m:
  slack = 0.0 m
  z_min = 0.330 m
  simulação roda
  tensão máxima drone/âncora ~6.21/6.80 N
  pitch máximo ~86.6 deg

L=2.5 m:
  slack = 0.5 m
  z_min senoidal planar = -0.335 m
  tentativas de inicialização curvada causaram abort do DART

L=3.0 m:
  slack = 1.0 m
  z_min senoidal planar = -0.648 m
  tentativas de inicialização curvada causaram abort do DART
```

Conclusão:
Aumentar comprimento sem mudar a geometria inicial/spawn não resolve diretamente. Com extremidades a `z=0.33 m`, a folga adicional não cabe para baixo sem atravessar o solo.

Próxima ação:
Testar slack com extremidades mais altas ou spawn mais próximo da âncora.

## Experimento: Inicialização 3D Ou Curvada Do Cabo

Data:
2026-08-15, contexto recente.

Objetivo:
Criar uma geometria inicial com folga que conecte a âncora ao drone sem compressão geométrica no primeiro passo.

Hipótese:
Uma curva senoidal ou meandra 3D poderia representar o cabo relaxado e evitar impulso inicial.

Configuração:
Modificações temporárias em `models/gerar_cabo.py`, incluindo tentativas com poses absolutas dos links e rotações relativas estruturais nas juntas.

Resultado:

- Poses/rotações estruturais para formas curvadas causaram abort do DART ao construir juntas.
- Para `L=2.0 m`, a geometria permanece reta e roda porque os ângulos iniciais são zero.

Conclusão:
O encadeamento SDF atual não aceita facilmente curvatura inicial embutida dessa forma. É necessário redesenhar a inicialização do cabo ou usar outro modelo/solver.

Próxima ação:
Não insistir na mesma abordagem sem reprojetar a representação. Priorizar cenário fisicamente compatível com a geometria reta atual ou elevar as extremidades.

## Experimento: Inicialização Slack Horizontal Do Cabo

Data:
2026-08-16.

Objetivo:
Criar uma condição inicial com folga sem atravessar o solo, mantendo âncora e sensor do drone em `z = 0.33 m`.

Hipótese:
Como a folga vertical atravessa o chão, a folga pode ser acomodada no plano horizontal com uma curva simples:

```text
x(s) = 2 s
y(s) = A sin(pi s)
z(s) = 0.33 m
```

A amplitude `A` deve ser resolvida numericamente para que o comprimento poliangular da cadeia seja igual ao comprimento configurado do cabo.

Configuração:

```text
âncora = (0.0, 0.0, 0.33) m
spawn  = (2.0, 0.0, 0.33) m
massa total dinâmica do cabo ~= 0.30 kg
cmd_vel_frame:=body
controlador e ganhos inalterados
trajetoria_assentamento_spawn.json
```

Alterações realizadas:

- `src/pacote_do_drone/models/gerar_cabo.py` agora aceita `initial_shape: "sine_slack"` como senóide horizontal.
- A amplitude lateral é resolvida por bisseção.
- Os pontos de diagnóstico `0`, `N/4`, `N/2`, `3N/4` e `N` são impressos no terminal.
- A curvatura inicial é aplicada com `<initial_position>` nas juntas revolutas e `spring_reference` igual ao ângulo inicial. Isto evita aplicar a curvatura como rotação estrutural dos links, que causava abort do DART.

Resultado de geração:

```text
L=2.0 m:
  N = 40
  massa segmento = 0.007375 kg
  amplitude = 0.0000 m
  folga = 0.0000 m
  z_min = 0.3300 m

L=2.5 m:
  N = 50
  massa segmento = 0.005880 kg
  amplitude = 0.6921 m
  folga = 0.5000 m
  z_min = 0.3300 m

L=3.0 m:
  N = 60
  massa segmento = 0.004883 kg
  amplitude = 1.0482 m
  folga = 1.0000 m
  z_min = 0.3300 m
```

Resultado dinâmico no assentamento do spawn:

```text
L=2.0 m:
  tensão inicial drone/âncora = 0.20 / 0.40 N
  tensão máxima drone/âncora = 6.32 / 6.92 N
  pitch máximo = 86.6 deg
  deslocamento máximo = 0.31 m

L=2.5 m:
  tensão inicial drone/âncora = 0.18 / 0.33 N
  tensão máxima drone/âncora = 2.62 / 2.86 N
  pitch máximo = 30.2 deg
  deslocamento máximo = 0.09 m

L=3.0 m:
  tensão inicial drone/âncora = 0.16 / 0.29 N
  tensão máxima drone/âncora = 8.30 / 36.32 N
  pitch máximo = 15.2 deg
  deslocamento máximo = 0.07 m
```

Conclusão:
`L=2.5 m` é a melhor configuração entre as três para o assentamento inicial: remove o crash do DART e reduz o pico de tensão em relação a `L=2.0 m`. `L=3.0 m` cria uma curva lateral grande demais e produz pico muito alto na âncora, provavelmente por dinâmica/impacto da cadeia próxima ao carretel.

Teste de subida com `L=2.5 m`:

```text
trajetoria_subida_curta_spawn.json, z final 0.60 m:
  tensão máxima drone/âncora = 3.59 / 4.76 N
  pitch máximo = 36.5 deg
  erro final = 0.29 m
  saturação z = 23 amostras
  não atingiu o waypoint dentro da janela de teste

trajetoria_subida_z1_spawn.json, z final 1.00 m:
  tensão máxima drone/âncora = 2.68 / 15.79 N
  pitch máximo = 34.0 deg
  erro final = 0.71 m
  saturação z = 22 amostras
  não atingiu o waypoint dentro da janela de teste
```

Próxima ação:
Antes de trajetórias verticais, investigar por que a cadeia horizontal relaxada ainda induz pitch sustentado de aproximadamente 30 graus. Possíveis causas: torque da junta fixa entre `ponta_cabo` e `cabo_sensor_link`, referência de mola/damping nos yaw joints, contato/impulso dos primeiros segmentos perto da âncora ou incompatibilidade entre cabo inicialmente lateral e junta fixa ao sensor.

## Experimento: Diagnóstico Da Conexão Cabo-Drone

Data:
2026-08-16.

Objetivo:
Identificar qual elemento do tether ainda transmite força ou momento artificial ao drone no caso baseline com `L=2.5 m`.

Baseline:

```text
comprimento do cabo = 2.5 m
massa total = 0.30 kg
50 segmentos
comprimento por segmento = 0.05 m
massa por segmento = 0.00588 kg
initial_shape = sine_slack horizontal
amplitude lateral = 0.6921 m
cmd_vel_frame = body
controlador e ganhos inalterados
trajetoria_assentamento_spawn.json
```

Alterações realizadas:

- `tether_parameters.json` ganhou parâmetros diagnósticos:
  - `connection_type`: `fixed` ou `ball`;
  - `joint_spring_stiffness`;
  - `joint_damping`;
  - `joint_friction`;
  - `segment_collision`.
- `start_sim.launch.py` passa a ler `connection_type` e gerar a junta `cabo_drone_joint` como `fixed` ou `ball`.
- A junta de conexão publica `/cabo/conexao_drone`.
- O controlador registra `|F|`, `|M|`, `F=(Fx,Fy,Fz)` e `M=(Mx,My,Mz)` da conexão.
- O gerador imprime `tau_spring_inicial_max=0.0000 Nm`, pois `initial_position == spring_reference` nas juntas internas.

Resultados no assentamento do spawn:

```text
caso                 conexão  spring  damping  colisão  Tmax D/C [N]   |F|max [N]  |M|max [Nm]  pitch max [deg]
baseline_fixed       fixed    0.02    0.08     sim      3.72 / 4.88    0.99       0.044       31.2
conexao_ball         ball     0.02    0.08     sim      2.01 / 3.32    1.03       0.000        0.2
spring_zero          fixed    0.00    0.08     sim      2.78 / 2.97    0.96       0.042       31.4
damping_half         fixed    0.02    0.04     sim      2.04 / 2.32    1.33       0.050       30.3
damping_double       fixed    0.02    0.16     sim      5.72 / 8.29    5.74       0.039       30.9
no_segment_collision fixed    0.02    0.08     não      8.86 / 6.55    8.85       1.963       89.1
```

Interpretação:

- A junta `ball` praticamente elimina o momento na conexão e derruba o pitch máximo de aproximadamente `31 deg` para `0.2 deg`.
- Zerar a mola interna não reduz o pitch, logo a energia elástica inicial das juntas internas não é a causa principal.
- `initial_position` e `spring_reference` estão alinhados no gerador, logo `tau_spring(t=0)` é zero por construção.
- Dobrar o damping aumenta bastante as tensões e a força na conexão, então damping excessivo pode transmitir força artificial.
- Remover colisões dos segmentos piorou drasticamente o resultado neste modelo, com pitch próximo de `90 deg` e momento de quase `2 Nm`; portanto esse caminho está descartado como melhoria imediata.

Conclusão:
A causa dominante do pitch sustentado no assentamento é a transmissão de momento pela junta fixa `ponta_cabo -> cabo_sensor_link`, não a tensão axial isolada nem a mola interna inicial do cabo.

Teste de subida curta com conexão `ball`:

```text
trajetoria_subida_curta_spawn.json, z final 0.60 m:
  tensão máxima drone/âncora = 5.92 / 14.32 N
  |F|max conexão = 2.16 N
  |M|max conexão = 0.000 Nm
  roll/pitch máximo = 0.0 / 0.7 deg
  erro final = 0.24 m
  saturação z = 22 amostras
  resultado: não atingiu o waypoint dentro da janela testada
```

Conclusão da subida:
A conexão livre resolve a inclinação artificial, mas a subida vertical ainda é limitada por saturação em `z` e por forças/tensões do tether. Esse problema deve ser tratado separadamente, depois de consolidar uma conexão final que não transmita momento indevido ao drone.

Próxima ação:
Projetar uma conexão final fisicamente coerente para o sensor do cabo: ela deve transmitir força no ponto de conexão, permitir orientação passiva do tether e medir azimuth/elevation sem impor orientação rígida ao drone. A junta `ball` é um bom diagnóstico, mas ainda não representa sozinha o sensor final de 2 DOF.

## Experimento: Baseline Ball, Ângulos Locais E Subida Vertical

Data:
2026-08-16.

Objetivo:
Adotar `connection_type = ball` como baseline física provisória, verificar se a medição de ângulos continua viável e investigar a falha de subida vertical sem o momento artificial da conexão fixa.

Configuração baseline:

```text
L = 2.5 m
massa total do cabo = 0.30 kg
num_links = 50
length = 0.05 m
mass = 0.00588 kg
initial_shape = sine_slack horizontal
connection_type = ball
cmd_vel_frame = body
ganhos do controlador inalterados
```

### Validação Angular Com Ball

Método:
O sensor principal usa a direção local do tether obtida pela diferença entre a `ponta_cabo` e um segmento próximo ao drone, expressa no frame do drone. Foram comparadas duas janelas:

```text
Método A: janela_tangente_links = 1
Método B: janela_tangente_links = 3
```

Casos testados:
`E`, `N`, `W`, `S` e `NE`, todos com extremidade em raio horizontal de `2.0 m` e `z = 0.33 m`.

Resultados:

```text
janela = 1:
  azimuth medido: ~179.8 deg em todos os casos
  elevation medida: -10.7 a -12.0 deg

janela = 3:
  azimuth medido: ~179.8 deg em todos os casos
  elevation medida: -3.3 a -6.1 deg
```

Interpretação:
Com `connection_type = ball`, o cabo se reorienta passivamente após o início da simulação. Por isso, a tangente dinâmica medida não coincide com a tangente da curva senoidal inicial congelada. A comparação correta para o sensor é contra a geometria dinâmica dos segmentos, não contra a forma inicial. A janela de 3 links suaviza a estimativa local e reduz a inclinação aparente em relação ao uso de apenas 1 link.

Decisão operacional:
Usar `janela_tangente_links = 3` como estimativa local provisória do cabo no drone. A reta até a âncora continua disponível como diagnóstico, mas não deve ser usada como sensor local em cabo slack.

### Subida Vertical Sem Tether Vs Com Tether Ball

Trajetória:

```text
spawn = (2.0, 0.0, 0.33)
waypoint inicial = (2.0, 0.0, 0.33)
waypoint final = (2.0, 0.0, 0.60)
tempo_hover = 5.0 s
```

Sweep de `limite_vel_z`:

```text
caso                 limite_z  chegou  z_final/ref  vz_max  raw_z_max  cmd_z_max  sat_z   Tmax D/C [N]   Fz_raw max [N]  pitch max
sem tether           0.25      sim     0.60/0.60    0.24    0.17       0.17       0/22    0.00/0.00      0.00            0.0
sem tether           0.50      sim     0.60/0.60    0.17    0.13       0.13       0/22    0.00/0.00      0.00            0.0
sem tether           0.75      sim     0.60/0.60    0.17    0.14       0.14       0/22    0.00/0.00      0.00            0.0
tether ball          0.25      não     0.34/0.60    0.04    0.35       0.25       31/41   1.78/3.47      0.49            0.4
tether ball          0.50      não     0.37/0.60    0.03    0.35       0.35       0/40    7.83/14.73     0.49            0.3
tether ball          0.75      não     0.44/0.60    0.04    0.35       0.35       0/40    1.75/3.55      0.49            1.1
```

Localização da saturação:

- Para `limite_vel_z = 0.25`, a saturação ocorre no clamp do controlador: `raw_z_max = 0.35`, `cmd_z_max = 0.25`.
- Para `limite_vel_z = 0.50` e `0.75`, o clamp do controlador não satura (`cmd_z_max = raw_z_max = 0.35`), mas o drone ainda quase não sobe.
- Portanto, a falha com tether não é explicada apenas por `limite_vel_z`; há limitação posterior ao `cmd_vel` publicado ou carga/dinâmica do tether.

Força vertical do tether:
O maior `Fz` bruto medido na conexão foi aproximadamente `0.49 N` nos casos com settling. Esse valor é pequeno comparado ao peso do drone:

```text
peso do drone ~= 1.55 * 9.81 = 15.2 N
Fz_tether / peso ~= 3.2 %
```

Teste imediato vs settling:

```text
subida imediata, tether ball, limite_z=0.25:
  z_final/ref = 0.33/0.60
  vz_max = 0.03 m/s
  raw_z_max/cmd_z_max = 0.33/0.25
  sat_z = 25/25
  Tmax D/C = 1.66/4.04 N
  Fz_raw max = 1.08 N
  pitch max = 0.3 deg

settling + subida, tether ball, limite_z=0.25:
  z_final/ref = 0.34/0.60
  vz_max = 0.04 m/s
  raw_z_max/cmd_z_max = 0.35/0.25
  sat_z = 31/41
  Tmax D/C = 1.78/3.47 N
  Fz_raw max = 0.49 N
  pitch max = 0.4 deg
```

Conclusão do settling:
O settling reduz a componente vertical máxima medida na conexão, mas não resolve a incapacidade de subir.

Margem nominal de empuxo:
Pelos parâmetros do SDF:

```text
forceConstant = 1.5e-03
maxRotVelocity = 1000 rad/s
4 rotores
Tmax nominal = 4 * 1.5e-03 * 1000^2 = 6000 N
peso do drone ~= 15.2 N
T/W nominal ~= 394.6
```

Essa margem nominal é enorme e provavelmente não representa a limitação real do plugin durante o controle por velocidade. Nos logs, velocidades de rotor reportadas ficaram por volta de `11–12 rad/s`, muito abaixo de `maxRotVelocity`, então não há evidência de saturação dos rotores nesses ensaios.

Conclusão:
A conexão `ball` remove o torque artificial e preserva a possibilidade de medir a direção local do cabo por geometria dos segmentos. A subida vertical, porém, continua limitada mesmo sem saturação do clamp do controlador e com componente vertical de tether pequena. A próxima suspeita é a dinâmica interna do `MulticopterVelocityControl`, a interpretação do comando vertical quando há carga externa no modelo acoplado, ou a forma como o sistema multibody cabo-drone afeta o link/controlador usado pelo plugin.

Próxima ação:
Investigar a resposta do plugin para `cmd_vel.linear.z` com carga externa pequena: testar comandos verticais constantes sem controlador de posição, verificar o link usado como `comLinkName`, e comparar odometria/velocidade real do `base_link` com o comando publicado.

## Experimento: Trajetórias Slack Com Waypoint Intermediário

Data:
Contexto do chat anterior.

Objetivo:
Evitar tensionamento durante deslocamento inicial usando waypoint intermediário antes do ponto final.

Hipótese:
Uma subida ou passagem conservadora reduziria aceleração abrupta do tether.

Configuração:
Arquivos `trajetoria_slack_*.json` e `trajetoria_spawn_conservadora_z2.json`.

Resultado:
Resultados numéricos completos não estão confirmados no repositório. O README registra que `trajetoria_slack_n.json` não atingiu o waypoint intermediário em janela de 125 s em um ensaio anterior.

Conclusão:
A estratégia depende da consistência entre spawn, primeiro waypoint e geometria inicial do cabo.

Próxima ação:
Confirmar o conteúdo atual dos arquivos `trajetoria_slack_*.json`, pois há inconsistência documentada para o caso `n`.

## Experimento: Resposta Vertical Aberta Com Tether Ball

Data:
2026-08-17.

Objetivo:
Separar o controlador de posicao da resposta vertical do plugin `MulticopterVelocityControl` quando o drone esta acoplado ao cabo.

Hipótese:
Se o drone sobe corretamente sem tether ao receber `cmd_vel.linear.z > 0`, mas nao sobe com tether mesmo sem controlador de posicao, o problema esta depois da geracao de referencia: plugin, modelo fisico acoplado, restricoes do cabo ou parametrizacao do multirotor.

Configuração baseline:

```text
L = 2.5 m
massa total do cabo = 0.30 kg
num_links = 50
length = 0.05 m
mass = 0.00588 kg
initial_shape = sine_slack horizontal
connection_type = ball
cmd_vel_frame = body
janela_tangente_links = 3
spawn = (2.0, 0.0, 0.33) m
```

No de teste:

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
  log_periodo:=0.5
```

O no `velocity_test` publica `/meu_drone/cmd_vel` constante e registra:

```text
t, z, vz, az, cmd_z_pub, RPY, F_raw, F_body_est, |F|, |M|, rotores, margem das juntas do cabo
```

Resultados do sweep de comando vertical:

```text
caso                vz_cmd  dz [m]  z_final [m]  vz_mean [m/s]  vz_max [m/s]  |F|max [N]  |M|max [Nm]  pitch max [deg]
sem tether          0.10    0.568   0.896        0.089          0.101         0.00       0.000        0.0
tether livre        0.10   -0.029   0.299       -0.005          0.000         1.53       0.000        0.4
tether ancorado     0.10   -0.011   0.317       -0.002          0.003         4.11       0.000        0.1

sem tether          0.25    1.505   1.833        0.225          0.252         0.00       0.000        0.0
tether livre        0.25    0.001   0.329       -0.000          0.003         7.71       0.000        0.4
tether ancorado     0.25    0.020   0.348        0.001          0.007         2.00       0.000        0.2

sem tether          0.50    3.048   3.372        0.443          0.504         0.00       0.000        0.0
tether livre        0.50    0.066   0.394        0.008          0.016         4.79       0.000        0.4
tether ancorado     0.50    0.166   0.494        0.018          0.027         1.09       0.000        0.6
```

Teste adicional executado apos adicionar `JointStatePublisher` ao cabo:

```text
tether ancorado, vz_cmd=0.25:
  cmd_z_pub = 0.25 ate 8 s
  z = 0.328 m em t=2.15 s
  z = 0.336 m em t=8.00 s
  z = 0.362 m em t=12.70 s, apos comando zerar
  |F|max observado no trecho = 2.00 N
  |M| = 0.000 Nm
  pitch max observado = ~0.5 deg
  margem minima das juntas do cabo = ~64% no fim do ensaio
  juntas proximas do limite = 0
```

Sweep de massa/comprimento com `vz_cmd=0.25`:

```text
caso                 massa total  L [m]  ancora  dz [m]  z_final [m]  vz_mean [m/s]  |F|max [N]  pitch max [deg]
mass030_L25_anchor   0.30 kg      2.5    sim     0.038   0.366        0.004          0.99       0.3
mass010_L25_anchor   0.10 kg      2.5    sim     0.072   0.400        0.008          0.64       0.2
mass003_L25_anchor   0.03 kg      2.5    sim     0.113   0.441        0.012          0.17       0.0
mass030_L25_free     0.30 kg      2.5    nao     0.006   0.334        0.001          7.71       0.4
mass003_L25_free     0.03 kg      2.5    nao     0.058   0.386        0.004          0.23       0.0
mass030_L30_anchor   0.30 kg      3.0    sim     0.013   0.341        0.000          0.77       0.2
```

Interpretação das hipóteses:

```text
H1 - MulticopterVelocityControl afetado pelo multibody conectado: provavel.
Evidencia: sem tether o drone segue cmd_vel.z; com tether livre e ancorado a resposta vertical quase desaparece.

H2 - forcas/restricoes do solver nao totalmente capturadas por /cabo/conexao_drone: provavel.
Evidencia: mesmo quando a forca medida na conexao e pequena, a resposta vertical e fortemente alterada.

H3 - juntas internas do cabo em limite: improvavel neste ensaio.
Evidencia: margem dinamica minima maior que 60% e zero juntas proximas do limite no teste curto.

H4 - massa/inercia simples do cabo domina a falha: improvavel como causa unica.
Evidencia: reduzir massa total de 0.30 kg para 0.03 kg melhora pouco, mas nao recupera a subida normal.

H5 - parametrizacao/interpretacao de empuxo do modelo deve ser revisada: provavel como risco de modelo.
Evidencia: os parametros `forceConstant`/`maxRotVelocity` geram uma margem nominal absurdamente alta se interpretados diretamente, mas o mesmo drone sem tether sobe corretamente; portanto nao explica sozinho a diferenca com tether.

H6 - sinal ou frame de cmd_vel.z errado: improvavel.
Evidencia: documentacao oficial do Gazebo Sim 6 indica velocidade linear no frame do corpo; sem tether, `vz_cmd > 0` produz subida coerente.
```

Conclusão:
O problema atual nao deve ser tratado ajustando ganhos do controlador de posicao. O caso aberto mostra que o comando vertical chega ao Gazebo e funciona sem tether, mas perde efetividade quando qualquer cadeia do cabo esta acoplada ao drone, mesmo com raiz livre e sem momento na conexao `ball`.

Próxima ação:
Investigar a topologia de conexao entre `meu_drone::base_link`, `cabo_sensor_link` e o cabo. Um teste diagnostico recomendado e conectar o tether diretamente ao `base_link`, ou transformar o sensor em link filho do `base_link`, para verificar se o `MulticopterVelocityControl` e o `comLinkName=base_link` sao afetados pela cadeia externa ligada ao link do sensor. Tambem vale testar uma carga externa simples aplicada ao drone, sem multibody do cabo, para separar carga fisica de restricao articulada.

## Experimento: Reavaliacao Com Tempo Simulado

Data:
2026-08-17.

Objetivo:
Verificar se a falha de subida observada no experimento anterior era fisica ou causada por usar tempo de parede enquanto o Gazebo rodava lentamente com o cabo.

Hipótese:
Com o cabo de 50 links, o fator de tempo real cai bastante. Se `velocity_test` encerra o comando apos `8 s` de parede, mas a simulacao avancou muito menos que `8 s` fisicos, o drone parece nao subir embora esteja respondendo corretamente em tempo simulado.

Alterações realizadas:

- Bridge de `/clock` em `start_sim.launch.py`.
- `velocity_test.py` passou a usar tempo simulado para:
  - duracao do comando;
  - periodo de log;
  - derivadas `vz` e `az`.
- `movimento_circular.py` passou a usar tempo simulado para:
  - derivadas de velocidade por diferenca;
  - integradores;
  - tempo de hovering;
  - periodo de log.

Teste aberto com tether desacoplado visualmente:

```text
tether presente, sem joint com drone, vz_cmd=0.25:
  t_sim=0.50 s, t_wall=5.55 s, z=0.357 m, vz=0.187 m/s
  t_sim=1.00 s, t_wall=13.45 s, z=0.470 m, vz=0.238 m/s
  t_sim=1.50 s, t_wall=25.10 s, z=0.594 m, vz=0.249 m/s
```

Interpretação:
A presença do cabo sem conexão nao impede a subida. Ela reduz drasticamente o RTF.

Teste aberto com tether conectado por `ball`, raiz livre:

```text
vz_cmd=0.25:
  t_sim=0.50 s, t_wall=8.90 s, z=0.337 m, vz=0.126 m/s
  t_sim=1.00 s, t_wall=19.90 s, z=0.418 m, vz=0.184 m/s
  t_sim=1.50 s, t_wall=31.75 s, z=0.510 m, vz=0.186 m/s
  t_sim=2.00 s, t_wall=42.85 s, z=0.599 m, vz=0.176 m/s
  pitch < 1.5 deg
  |M| conexao = 0.000 Nm
```

Teste aberto com tether conectado por `ball`, raiz ancorada:

```text
vz_cmd=0.25:
  t_sim=0.50 s, t_wall=5.25 s, z=0.341 m, vz=0.139 m/s
  t_sim=1.00 s, t_wall=15.75 s, z=0.426 m, vz=0.184 m/s
  t_sim=1.50 s, t_wall=27.40 s, z=0.519 m, vz=0.184 m/s
  t_sim=2.00 s, t_wall=38.20 s, z=0.609 m, vz=0.177 m/s
  t_sim=2.51 s, t_wall=46.85 s, z=0.634 m, vz=-0.024 m/s
  pitch < 1.0 deg
  |M| conexao = 0.000 Nm
```

Teste com controlador de posicao:

```text
trajetoria_subida_curta_spawn.json
spawn=(2.0, 0.0, 0.33)
ref final=(2.0, 0.0, 0.60)
tempo_hover=2.0 s
tolerancia_altura=0.15 m

resultado:
  transicao WP0->WP1 em t_sim ~2.2 s
  z=0.47 m em t_sim=4.0 s
  z=0.51 m em t_sim=5.0 s
  sequencia concluida dentro da tolerancia configurada
```

Com tolerancia vertical mais apertada apenas para diagnostico:

```text
tolerancia_altura=0.05 m
tempo_hover=1.0 s

resultado parcial antes do timeout real:
  z=0.54 m em t_sim=6.01 s
  ref=0.60 m
  erro_z=0.06 m
  subida continua e comandos coerentes
```

Conclusão:
A hipótese estrutural forte sobre `MulticopterVelocityControl` e topologia `base_link/cabo_sensor_link` nao foi confirmada como causa da falha vertical. A causa dominante dos resultados anteriores foi a base de tempo incorreta dos testes e do controlador em uma simulacao com RTF baixo. Em tempo simulado, `cmd_vel.z > 0` produz subida coerente do `base_link` com tether `ball`, massa `0.30 kg` e `L=2.5 m`.

Próxima ação:
Manter `/clock` bridged e usar `t_sim` nos testes. Para melhorar usabilidade, reduzir custo computacional do cabo ou aumentar timeouts reais dos ensaios automatizados. So retomar investigacao estrutural se uma falha persistir quando medida em tempo simulado.

## Experimento: Estabilização E Métricas Em Tempo Simulado

Data:
2026-08-17.

Objetivo:
Consolidar o controlador com tempo simulado, adicionar critério robusto de chegada ao waypoint e iniciar medições quantitativas de hover/ângulos.

Alterações realizadas:

- Auditoria de tempo no pacote ativo `src/pacote_do_drone`.
- `sensores.py` e `cabo_monitor.py` passaram a usar `/clock` para cadência de log/CSV quando disponível.
- `sensores.py` passou a aceitar `janela_tangente_metros`; com o padrão atual `0.15 m`, a janela equivale a 3 links no cabo baseline de `length=0.05 m`.
- `movimento_circular.py` ganhou `tempo_estabilizacao`, separado de `tempo_hover`.
- Logs compactos do controlador agora incluem `RTF`.
- Criado o nó `hover_metrics`, que mede:
  - média/desvio de posição;
  - erro médio/RMS/máximo;
  - roll/pitch máximos;
  - tensão média/máxima;
  - média/desvio/min/max de azimuth/elevation.
- Criados arquivos de teste:
  - `trajetoria_teste_z060.json`;
  - `trajetoria_teste_z100.json`;
  - `trajetoria_teste_z150.json`;
  - `trajetoria_teste_z200.json`;
  - `trajetoria_sensor_n/s/e/w.json`.

Auditoria de tempo:

```text
movimento_circular.py:
  dinamica, derivadas, integradores, estabilizacao, hover e logs usam /clock.

velocity_test.py:
  duracao do comando, derivadas e logs usam /clock.

sensores.py:
  publicacao de angulos nao usa dt; log usa /clock.

cabo_monitor.py:
  monitor/CSV usam /clock quando disponivel.

hover_metrics.py:
  estatisticas usam /clock; tempo de parede e usado apenas para RTF.
```

Teste vertical `z=0.60 m`:

```text
spawn=(2.0, 0.0, 0.33)
waypoints=(2.0,0.0,0.33) -> (2.0,0.0,0.60)
tempo_estabilizacao=1.0 s
tempo_hover=2.0 s
tolerancia_altura=0.10 m

resultado:
  transicao WP0->WP1 em t_sim~3.2 s
  t_sim=4.01 s: z=0.39 m, erro_z=0.21 m
  t_sim=5.01 s: z=0.47 m, erro_z=0.13 m
  t_sim=6.01 s: z=0.51 m, erro_z=0.09 m
  estado=estabilizando no waypoint final
  RTF observado: ~0.04 a 0.07
  pitch max observado no trecho: ~1.2 deg
  tensao maxima no trecho: ~1.70 N no carretel, ~0.35 N na conexao
```

Teste vertical `z=1.00 m`:

```text
spawn=(2.0, 0.0, 0.33)
waypoints=(2.0,0.0,0.33) -> (2.0,0.0,1.00)
tempo_estabilizacao=1.0 s
tempo_hover=2.0 s
tolerancia_altura=0.10 m

resultado parcial ate timeout real:
  transicao WP0->WP1 em t_sim~3.2 s
  t_sim=4.00 s: z=0.50 m, erro_z=0.50 m
  t_sim=5.01 s: z=0.73 m, erro_z=0.27 m
  t_sim=6.01 s: z=0.83 m, erro_z=0.17 m
  t_sim=9.01 s: z=0.88 m, erro_z=0.12 m
  RTF observado: ~0.04 a 0.07
  pitch max observado no trecho: ~0.9 deg
  tensao maxima no trecho: ~1.70 N no carretel, ~0.70 N na conexao
```

Métricas coletadas no teste `z=1.00 m`, janela final antes do timeout real:

```text
RTF_med=0.05
pos_mean=(1.994, 0.004, 0.860) m
pos_std=(0.028, 0.000, 0.012) m
err_mean/rms/max=0.143/0.143/0.178 m
roll_max=0.01 deg
pitch_max=0.90 deg
T_mean/max=0.63/0.66 N
az_mean/std/min/max=179.86/0.03/179.82/179.92 deg
el_mean/std/min/max=33.17/3.70/28.96/38.94 deg
```

Interpretação:
O controlador está fisicamente coerente e estável, mas a convergência vertical para alturas maiores é lenta com os ganhos atuais e sem termo integral. Como a orientação do pedido foi não alterar ganhos, isso fica registrado como característica atual da baseline.

Testes preparados mas ainda não concluídos nesta rodada:

```text
z=1.5 m
z=2.0 m
hover 10 s simulado em (2.0,0.0,1.0)
hover 30 s simulado em (2.0,0.0,1.0)
casos N/S/E/W em raio 1.0 m, z=2.0 m
sweep 20/30/40/50 links
comparacao janela_tangente_metros=0.05 vs 0.15
```

Motivo:
Com 50 links, o RTF observado ficou tipicamente em `0.04-0.07`; testes de 30 s simulados exigem timeouts reais muito maiores.

Próxima ação:
Rodar as baterias longas com `hover_metrics` e timeouts reais dimensionados por RTF, ou primeiro executar o sweep de discretização para encontrar uma configuração com RTF melhor sem degradar ângulos/tensão.

## Experimento: Janela Física E Teste Curto De Convergência

Data:
2026-08-17.

Objetivo:
Usar uma janela física para estimar a tangente local do cabo e comparar a subida vertical para `z=1.0 m` com e sem tether, mantendo a baseline física e os ganhos atuais.

Alterações:

- `sensores.py` agora recebe `janela_tangente_metros`.
- Padrão atual: `0.15 m`.
- Com a baseline `length=0.05 m/link`, isso equivale a `3` links.
- `janela_tangente_links` permanece como fallback quando `janela_tangente_metros <= 0`.

Caso sem cabo:

```text
usar_cabo=false
spawn=(2.0,0.0,0.33)
waypoints=(2.0,0.0,0.33) -> (2.0,0.0,1.0)
RTF~1.00

resultado:
  transicao WP0->WP1: t_sim~3.85 s
  entrada em hover no WP1: t_sim~8.0 s
  sequencia concluida: t_sim~9.4 s
  erro final na janela de metricas: ~0.03 m RMS
  roll/pitch max: ~0.02/0.00 deg
```

Métricas sem cabo:

```text
RTF_med=1.00
pos_mean=(2.000,0.004,0.975) m
pos_std=(0.000,0.001,0.017) m
err_mean/rms/max=0.025/0.030/0.066 m
T_mean/max=0.00/0.00 N
```

Caso com cabo baseline:

```text
usar_cabo=true
connection_type=ball
L=2.5 m
m_cabo~0.30 kg
janela_tangente_metros=0.15
RTF~0.05

resultado:
  transicao WP0->WP1: t_sim~3.3 s
  t_sim~4.0 s: z~0.51 m, erro~0.50 m
  t_sim~5.0 s: z~0.73 m, erro~0.27 m
  t_sim~6.0 s: z~0.83 m, erro~0.18 m
  cmd_z_raw abaixo de limite_vel_z depois da transicao
  roll/pitch pequenos, sem indicio de tombamento
```

Métricas com cabo, janela curta `t_sim=4.0-5.0 s`:

```text
RTF_med=0.05
pos_mean=(2.060,0.004,0.643) m
pos_std=(0.006,0.000,0.064) m
err_mean/rms/max=0.362/0.367/0.496 m
roll_max=0.02 deg
pitch_max=1.26 deg
T_mean/max=0.42/0.47 N
az_mean/std/min/max=179.88/0.01/179.85/179.90 deg
el_mean/std/min/max=21.20/4.95/11.71/29.63 deg
```

Interpretação:
A comparação confirma que o controlador e o comando `cmd_vel_frame=body` funcionam no caso sem cabo. Com cabo, a resposta permanece estável, sem saturação vertical dominante e com atitude pequena, mas a convergência vertical fica mais lenta e o custo computacional ainda é alto. O próximo diagnóstico deve priorizar redução de RTF/discretização antes de alterar ganhos.
