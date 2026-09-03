# Current Status

## Atualizacao 2026-08-27

Foi iniciada a integracao incremental com PX4, preservando a baseline atual do tethered drone.

PX4 configurado localmente:

```text
local                 px4/PX4-Autopilot/
remote                https://github.com/PX4/PX4-Autopilot.git
tag                   v1.14.4
commit                1555f2bd2229544c43966ab5f94879c41d8e1e01
estado                detached HEAD
modelo testado        gz_x500
```

Resultados:

```text
make px4_sitl_default                 OK
HEADLESS=1 PX4_GZ_MODEL=x500 make px4_sitl gz_x500  OK
commander arm                         OK
commander takeoff                     OK
hover aproximado sem tether           OK
commander land                        OK
```

Telemetria observada durante hover sem tether:

```text
vehicle_local_position:
  x = -0.045 m
  y = -0.119 m
  z = -1.469 m  (NED; ~1.47 m acima da origem local)

vehicle_attitude:
  roll  = -0.1 deg
  pitch = -0.3 deg
  yaw   = 89.5 deg

commander:
  Arm state       Armed
  navigation mode AUTO_LOITER
  failsafe        no
```

Documentacao detalhada:

```text
docs/PX4_INTEGRATION.md
px4/README.md
```

A integracao com o tether ainda nao foi feita. O proximo passo recomendado e criar uma variante externa `x500_tethered`, baseada em `Tools/simulation/gz/models/x500/model.sdf`, adicionando apenas um ponto de conexao fisico ao `base_link` para receber a `ball joint` do cabo atual.

## Atualizacao 2026-08-20

A baseline atual foi congelada para os testes cardeais do drone:

```text
comprimento_total_m      2.500 m
num_links                50
length por segmento      0.050 m
densidade_linear_kg_m    0.060 kg/m
massa por segmento       0.003000 kg
massa dos segmentos      0.150 kg
links auxiliares         ~0.006 kg
massa dinamica total     ~0.156 kg
connection_type          ball
initial_shape            sine_slack horizontal
cmd_vel_frame            body
janela_tangente_metros   0.15 m
```

O controlador foi mantido com a logica validada. Foram adicionados apenas parametros/logs de diagnostico para os limites do integrador e metricas separadas de erro `xy` e erro `z`.

Resultados headless dos casos cardeais, todos com `usar_cabo:=true`, `prender_ancora:=true`, `cmd_vel_frame:=body` e mesma configuracao fisica:

```text
Caso  alvo final          pos_mean final          err_rms  roll/pitch max   T max   sat_xyz
N     ( 0.0,  1.0,2.0)   (-0.052,  1.039,1.927)  0.100 m  0.89/0.20 deg   1.15 N  0/0/0 %
S     ( 0.0, -1.0,2.0)   (-0.051, -1.030,1.923)  0.099 m  0.95/0.23 deg   1.18 N  0/0/0 %
E     ( 1.0,  0.0,2.0)   ( 0.951,  0.001,1.933)  0.084 m  0.02/0.36 deg   1.07 N  0/0/0 %
W     (-1.0,  0.0,2.0)   (-1.024,  0.001,1.922)  0.081 m  0.03/0.84 deg   1.16 N  0/0/0 %
```

Conclusao atual:

```text
CONTROLADOR PARA TESTES ESTATICOS: APROVADO
SENSOR DE ELEVATION: VALIDADO COMO TANGENTE LOCAL, pendente comparacao redundante independente
SENSOR DE AZIMUTH: VALIDADO COMO TANGENTE LOCAL, pendente comparacao redundante independente
SISTEMA PRONTO PARA PROXIMA ETAPA: SIM, para pontos estaticos; ainda nao para trajetorias agressivas
```

Observacao importante sobre o sensor: `/cabo/azimuth_graus` e `/cabo/elevation_graus` representam a tangente local do cabo no lado do drone, estimada com a ponta do cabo e um segmento dentro da janela fisica de aproximadamente `0.15 m`, expressa no frame do drone. Esses valores nao devem ser comparados diretamente com a reta sensor-ancora quando o cabo esta frouxo ou ainda acomodando. O caso W mostrou acomodacao lenta da tangente local depois que o drone ja havia concluido o hover.

## Atualização 2026-08-17

A baseline fisica atual foi revisada para uma densidade linear mais realista:

```text
comprimento_total_m      2.500 m
num_links                50
length por segmento      0.050 m
densidade_linear_kg_m    0.060 kg/m
massa por segmento       0.003000 kg
massa dos segmentos      0.150 kg
links auxiliares         ~0.006 kg
massa dinamica total     ~0.156 kg
connection_type          ball
initial_shape            sine_slack horizontal
cmd_vel_frame            body
janela_tangente_metros   0.15 m
```

O teste vertical com essa massa concluiu de forma estavel, sem saturacao persistente:

```text
target                 (2.0, 0.0, 1.0) m
pos_mean final         (1.988, 0.005, 0.856) m
err_mean/rms/max       0.145 / 0.150 / 0.244 m
roll_max/pitch_max     0.02 / 0.45 deg
T_mean/max             0.33 / 0.38 N
sat_xyz                0.0 / 0.0 / 0.0 %
```

O teste N sem integral pequena melhorou, mas ficou `NO-GO`: estabilizou perto de `(0.02, 0.98, 1.87) m` para referencia `(0.0, 1.0, 2.0) m`, com erro medio final `~0.13 m`, sem saturacao persistente e com atitude pequena. A causa provavel era erro estacionario contra a carga/tensao do cabo.

Com defaults revisados do controlador (`ganho_altura=1.5`, `ganho_integral_xy=0.05`, `ganho_integral_z=0.08`, `ganho_velocidade_xy=1.4`, `limite_vel_xy=0.35`, tolerancias `0.12/0.10 m`), o caso N concluiu:

```text
pos_mean final         (-0.052, 1.040, 1.926) m
err_mean/rms/max       0.100 / 0.101 / 0.126 m
roll_max/pitch_max     0.89 / 0.20 deg
T_mean/max             1.10 / 1.15 N
sat_xyz                0.0 / 0.0 / 0.0 %
az/el tangente drone   -29.36 +/- 0.90 deg / 34.91 +/- 0.75 deg
```

Diagnostico atual: o controlador e a configuracao fisica estao proximos de uma baseline funcional para um waypoint cardeal com tether conectado. A validacao angular completa N/S/E/W ainda precisa ser executada nesta nova baseline; portanto o sensor no drone esta parcialmente validado por estabilidade e coerencia dinamica, mas ainda nao por varredura cardeal completa.

## Current Goal

Criar uma base confiável para simular e validar um drone conectado a cabo no Gazebo, separando:

- estabilidade do controlador de posição;
- efeitos físicos do tether;
- medição de azimuth/elevation do cabo no frame do drone.

O objetivo imediato de desenvolvimento é obter um caso simples e reproduzível:

```text
spawn original
+ tether conectado
+ tether inicialmente compatível/slack
+ waypoint único
+ hover estável
```

O marco intermediario atual foi resolvido: a aparente falta de resposta vertical com tether vinha de diagnosticos em tempo de parede enquanto a simulação com cabo rodava muito abaixo de tempo real. Com `/clock`, `cmd_vel.linear.z > 0` produz subida coerente também com tether `ball`.

## What Works

- O workspace compila com:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select pacote_do_drone cabo_avaliacao
```

- Testes unitários existem para:
  - funções de ângulo do cabo em `pacote_do_drone`;
  - cenários e catenária geométrica em `cabo_avaliacao`.
- `cabo_avaliacao` fornece mundos estáticos com postes e modos `reto`, `articulado` e `catenaria`.
- A massa atual do cabo está consistente em torno de `0.156 kg` para `L=2.5 m`.
- O controlador de waypoints publica em `/meu_drone/cmd_vel` e possui logging compacto para posição, referência, erro, RPY, comando, `cmd_z_raw`, saturação, tensões, força/momento da conexão cabo-drone e rotores.
- O cálculo de ângulos no frame do drone possui testes unitários, incluindo cabo vertical com drone nivelado e com pitch de 10 graus.

## In Progress

- Diagnóstico da condição inicial drone-tether.
- Construção de trajetórias simples com um waypoint por vez.
- Inicialização horizontal com folga via `<initial_position>` das juntas.
- Documentação persistente do estado atual.

## Known Problems

- Com a configuração reta antiga:

```text
âncora = (0.0, 0.0, 0.33) m
spawn  = (2.0, 0.0, 0.33) m
L      = 2.0 m
```

o cabo nasce sem folga geométrica (`slack = 0.0 m`) e reproduz o pico de tensão alto.

- A configuração atual usa `L=2.5 m`, folga horizontal no lado `+y`, `N=50`, massa total dinâmica de `~0.156 kg` e `connection_type=ball`.
- No teste de assentamento em `L=2.5 m` com conexão fixa, o pico de tensão cai bastante em relação a `L=2.0 m`, mas ainda aparece pitch sustentado de aproximadamente `30 deg`. Com conexão `ball`, esse pitch cai para menos de `1 deg`.
- Aumentar para `L=3.0 m` cria uma curva lateral grande e gerou pico muito alto na âncora.
- Uma senóide vertical entre as extremidades atuais atravessaria o chão:

```text
L=2.5 m: z_min = -0.335 m
L=3.0 m: z_min = -0.648 m
```

- Tentativas antigas de embutir meandra 3D ou rotações estruturais nas juntas do cabo causaram abort do DART durante construção das juntas. A versão atual evita isso usando `<initial_position>` nas juntas.
- O arquivo `README.md` diz que `trajetoria_slack_n.json` tem waypoint final em `(0.0, 1.0, 2.0)`, mas o arquivo atual contém waypoints verticais em `(0.0,0.0,1.0)` e `(0.0,0.0,2.0)`. Esta inconsistência precisa ser confirmada antes de usar esse caso como “N”.
- Existem muitas alterações não commitadas no branch `shared`; uma nova sessão deve verificar `git status` antes de editar.

## Current Hypotheses

- A instabilidade principal com spawn original é causada por condição inicial desfavorável do tether, não primariamente por ganhos do controlador.
- O pico de tensão aparece porque o cabo nasce esticado ou em configuração incompatível com a referência/solo.
- A folga horizontal reduz o pico inicial, mas a junta fixa `ponta_cabo -> cabo_sensor_link` transmite momento artificial ao drone durante o assentamento.
- A conexão `ball` praticamente elimina esse momento e reduz o pitch máximo para menos de `1 deg`; ela agora é a baseline física provisória.
- `velocity_test` agora usa `/clock`; com tether `ball`, `L=2.5 m`, `vz_cmd=0.25`, o drone sobe de `z ~= 0.33 m` para `z ~= 0.60 m` em cerca de `2 s` simulados, com pitch menor que `1.5 deg`, momento nulo e juntas longe do limite.
- O controlador de posição também usa `/clock` para derivadas, integradores, estabilização, hovering e logs.
- `tempo_estabilizacao` foi separado de `tempo_hover`.
- `hover_metrics` coleta estatísticas de posição, tensão, atitude e azimuth/elevation em janelas de tempo simulado.
- `sensores.py` usa `janela_tangente_metros=0.15` por padrão para estimar a tangente local em comprimento físico, equivalente a 3 links no cabo baseline.
- A massa dos segmentos do tether agora é derivada de `densidade_linear_kg_m=0.06 kg/m` e `comprimento_total_m=2.5 m`, resultando em `0.150 kg` nos segmentos e `~0.156 kg` incluindo links auxiliares.

## Constraints / Do Not Change

- Não alterar ganhos do controlador para mascarar a condição inicial sem experimento registrado.
- Preservar `cmd_vel_frame:=body` nos ensaios atuais.
- Não alterar modelo físico do drone para DJI Matrice 100 por enquanto.
- Não alterar a convenção angular sem atualizar testes, tabelas esperadas e documentação.
- Não editar manualmente `models/cabo.sdf` como fonte primária; ele é gerado por `models/gerar_cabo.py`.
- Interpretar testes com cabo em tempo simulado (`t_sim`), não em tempo de parede, porque o RTF pode cair para cerca de `0.04-0.10` com o cabo de 50 links.
- Com a baseline física de `25 g`, o teste vertical `z=1.0 m` concluiu hover com `pitch_max~0.06 deg`, `T_mean/max~0.10/0.10 N`, `RTF~0.10` e sem saturação.
- O teste N com `25 g` foi `NO-GO` por convergência lateral: forças baixas, sem saturação persistente e roll/pitch moderados, mas erro 3D RMS de cerca de `0.25 m` na janela final.

## Recent Results

Resultados quantitativos recentes obtidos em simulação headless:

```text
L=2.0 m, assentamento no spawn:
  slack inicial: 0.0 m
  tensão inicial drone/âncora: 0.20 / 0.40 N
  tensão máxima drone/âncora: 6.32 / 6.92 N
  instante do pico: ~1.05 s
  pitch máximo: ~86.6 deg
  deslocamento máximo: ~0.31 m
```

Resultados com slack horizontal no spawn:

```text
L=2.5 m:
  num_links: 50
  massa por segmento: 0.00588 kg
  folga geométrica: 0.5 m
  amplitude lateral: 0.6921 m
  z_min: 0.330 m
  tensão inicial drone/âncora: 0.18 / 0.33 N
  tensão máxima drone/âncora: 2.62 / 2.86 N
  pitch máximo: ~30.2 deg
  deslocamento máximo: ~0.09 m

L=3.0 m:
  num_links: 60
  massa por segmento: 0.004883 kg
  folga geométrica: 1.0 m
  amplitude lateral: 1.0482 m
  z_min: 0.330 m
  tensão inicial drone/âncora: 0.16 / 0.29 N
  tensão máxima drone/âncora: 8.30 / 36.32 N
  pitch máximo: ~15.2 deg
  deslocamento máximo: ~0.07 m
```

Resultados de subida com `L=2.5 m`:

```text
z final 0.60 m:
  tensão máxima drone/âncora: 3.59 / 4.76 N
  pitch máximo: ~36.5 deg
  erro final: ~0.29 m
  resultado: não atingiu o waypoint na janela testada

z final 1.00 m:
  tensão máxima drone/âncora: 2.68 / 15.79 N
  pitch máximo: ~34.0 deg
  erro final: ~0.71 m
  resultado: não atingiu o waypoint na janela testada
```

Resultados do diagnóstico da conexão cabo-drone:

```text
baseline fixed:
  conexão: fixed
  spring/damping internos: 0.02 / 0.08
  tensão máxima drone/âncora: 3.72 / 4.88 N
  |F|max conexão: 0.99 N
  |M|max conexão: 0.044 Nm
  pitch máximo: ~31.2 deg

conexão ball:
  conexão: ball
  spring/damping internos: 0.02 / 0.08
  tensão máxima drone/âncora: 2.01 / 3.32 N
  |F|max conexão: 1.03 N
  |M|max conexão: 0.000 Nm
  pitch máximo: ~0.2 deg

spring zero:
  conexão: fixed
  spring/damping internos: 0.00 / 0.08
  tensão máxima drone/âncora: 2.78 / 2.97 N
  |M|max conexão: 0.042 Nm
  pitch máximo: ~31.4 deg

damping 0.04:
  conexão: fixed
  tensão máxima drone/âncora: 2.04 / 2.32 N
  |M|max conexão: 0.050 Nm
  pitch máximo: ~30.3 deg

damping 0.16:
  conexão: fixed
  tensão máxima drone/âncora: 5.72 / 8.29 N
  |F|max conexão: 5.74 N
  pitch máximo: ~30.9 deg

sem colisão dos segmentos:
  conexão: fixed
  tensão máxima drone/âncora: 8.86 / 6.55 N
  |M|max conexão: 1.963 Nm
  pitch máximo: ~89.1 deg
```

Resultado da subida curta com conexão `ball`:

```text
z final 0.60 m:
  tensão máxima drone/âncora: 5.92 / 14.32 N
  |F|max conexão: 2.16 N
  |M|max conexão: 0.000 Nm
  pitch máximo: ~0.7 deg
  erro final: ~0.24 m
  resultado: não atingiu o waypoint na janela testada; saturação z persistiu
```

Resultados antigos do teste aberto de velocidade vertical, antes de usar `/clock`, estão superados porque mediam duração em tempo de parede. Resultado válido atual:

```text
velocity_test, tether ball livre, vz_cmd=0.25:
  t_sim=0.50 s, t_wall=8.90 s, z=0.337 m, vz=0.126 m/s
  t_sim=1.00 s, t_wall=19.90 s, z=0.418 m, vz=0.184 m/s
  t_sim=1.50 s, t_wall=31.75 s, z=0.510 m, vz=0.186 m/s
  t_sim=2.00 s, t_wall=42.85 s, z=0.599 m, vz=0.176 m/s
  |M| conexao = 0.000 Nm

velocity_test, tether ball ancorado, vz_cmd=0.25:
  t_sim=0.50 s, t_wall=5.25 s, z=0.341 m, vz=0.139 m/s
  t_sim=1.00 s, t_wall=15.75 s, z=0.426 m, vz=0.184 m/s
  t_sim=1.50 s, t_wall=27.40 s, z=0.519 m, vz=0.184 m/s
  t_sim=2.00 s, t_wall=38.20 s, z=0.609 m, vz=0.177 m/s
  |M| conexao = 0.000 Nm
```

Diagnostico atualizado:

```text
causa pouco provavel: ganhos do controlador de posicao
causa pouco provavel: sinal/frame de cmd_vel.z
causa pouco provavel: saturacao das juntas do cabo no ensaio curto
causa pouco provavel como unica explicacao: massa simples do cabo
causa confirmada da falsa falha vertical: uso de tempo de parede em simulação com RTF baixo
```

Proximos testes recomendados:

```text
1. Continuar validando trajetorias em tempo simulado.
2. Medir e reduzir o custo computacional/RTF do cabo de 50 links.
3. Retomar investigacao estrutural apenas se a falha persistir quando medida por t_sim.
4. Para testes automatizados com tether, usar timeouts reais muito maiores que a duracao simulada desejada.
```

Resultados recentes de controlador com tether `ball`, `L=2.5 m`, massa `0.30 kg`:

```text
z=0.60 m:
  chegou a erro_z~0.09 m em t_sim~6.0 s
  entrou em estabilizacao no waypoint final
  RTF~0.04-0.07
  pitch <= ~1.2 deg no trecho observado

z=1.00 m:
  chegou a z~0.88 m em t_sim~9.0 s
  erro_z~0.12 m no timeout real
  RTF_med~0.05
  pos_std final ~(0.028,0.000,0.012) m
  az_std~0.03 deg, el_std~3.70 deg na janela final observada
```

Resultados abaixo sao anteriores a correcao por `/clock` e ficam mantidos apenas como historico do diagnostico contaminado por tempo de parede:

```text
sem tether, limite_vel_z=0.25:
  chegou ao waypoint: sim
  z_final/ref: 0.60 / 0.60 m
  vz_max: 0.24 m/s
  raw_z_max/cmd_z_max: 0.17 / 0.17 m/s
  saturação z: 0/22

sem tether, limite_vel_z=0.50:
  chegou ao waypoint: sim
  z_final/ref: 0.60 / 0.60 m
  raw_z_max/cmd_z_max: 0.13 / 0.13 m/s
  saturação z: 0/22

sem tether, limite_vel_z=0.75:
  chegou ao waypoint: sim
  z_final/ref: 0.60 / 0.60 m
  raw_z_max/cmd_z_max: 0.14 / 0.14 m/s
  saturação z: 0/22

tether ball, limite_vel_z=0.25:
  chegou ao waypoint: não
  z_final/ref: 0.34 / 0.60 m
  vz_max: 0.04 m/s
  raw_z_max/cmd_z_max: 0.35 / 0.25 m/s
  saturação z: 31/41
  Fz conexão máx. abs.: 0.49 N

tether ball, limite_vel_z=0.50:
  chegou ao waypoint: não
  z_final/ref: 0.37 / 0.60 m
  vz_max: 0.03 m/s
  raw_z_max/cmd_z_max: 0.35 / 0.35 m/s
  saturação z: 0/40
  Fz conexão máx. abs.: 0.49 N

tether ball, limite_vel_z=0.75:
  chegou ao waypoint: não
  z_final/ref: 0.44 / 0.60 m
  vz_max: 0.04 m/s
  raw_z_max/cmd_z_max: 0.35 / 0.35 m/s
  saturação z: 0/40
  Fz conexão máx. abs.: 0.49 N
```

Diagnóstico de saturação em `z`:

```text
limite_vel_z=0.25:
  a saturação ocorre no clamp do controlador

limite_vel_z=0.50 e 0.75:
  o controlador não satura, mas o drone ainda não sobe adequadamente
```

Logo, a limitação vertical com tether não é explicada apenas por `limite_vel_z`. A força vertical medida na conexão é pequena em relação ao peso do drone (`~0.49 N` contra `~15.2 N`), então o próximo foco deve ser a resposta do plugin/modelo acoplado ao `cmd_vel.linear.z`.

Validação angular com conexão `ball`:

```text
janela_tangente_links=1:
  azimuth ~179.8 deg
  elevation -10.7 a -12.0 deg

janela_tangente_links=3:
  azimuth ~179.8 deg
  elevation -3.3 a -6.1 deg
```

Com `ball`, a direção dinâmica do cabo perto do drone não coincide com a curva inicial senoidal congelada. A janela de 3 links é a estimativa local provisória preferida por reduzir ruído na elevação.

Diagnóstico atual:

```text
causa confirmada para pitch no assentamento:
  momento artificial transmitido pela conexão fixa cabo-drone

causas descartadas como causa dominante do pitch:
  mola interna inicial do cabo
  erro entre initial_position e spring_reference
  self-collision dos segmentos como explicação principal

problema ainda aberto:
  subida vertical continua limitada por tensão/força do tether e saturação em z mesmo com conexão ball
```

Resultados anteriores do controlador:

```text
Sem cabo + spawn original: hover estável, roll/pitch máx. ~4.8 deg.
Sem cabo + spawn próximo: hover estável.
Com cabo + spawn próximo: hover estável, tensão máxima ~0.35 N.
Com cabo + spawn original: instável, pitch próximo de ±90 deg, tensão máxima ~6 N.
```

## Next Steps

1. Manter `connection_type=ball` como baseline provisória; usar `fixed` apenas para diagnóstico.
2. Consolidar o cálculo de azimuth/elevation usando direção local por janela física, inicialmente `janela_tangente_metros=0.15` (`3` links no cabo baseline de `0.05 m/link`).
3. Ajustar de forma conservadora a convergência lateral do controlador ou a estratégia de aproximação antes dos testes N/S/E/W completos.
4. Manter o modelo do drone inalterado; não há evidência objetiva de insuficiência física com tether de `25 g`.

## Last Known Good State

Último commit remoto conhecido no branch atual:

```text
f945daa (shared, origin/shared) Adiciona janela fisica para sensor do cabo
```

Observação: após a correção por `/clock`, o próximo commit deverá atualizar esta referência.
