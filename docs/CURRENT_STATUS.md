# Current Status

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
- A massa atual do cabo está consistente em torno de `0.30 kg` para `L=2.5 m`.
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

- A configuração atual usa `L=2.5 m`, folga horizontal no lado `+y`, `N=50`, massa total dinâmica de `0.30 kg` e `connection_type=ball`.
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

## Constraints / Do Not Change

- Não alterar ganhos do controlador para mascarar a condição inicial sem experimento registrado.
- Preservar `cmd_vel_frame:=body` nos ensaios atuais.
- Não alterar modelo físico do drone para DJI Matrice 100 por enquanto.
- Não alterar a convenção angular sem atualizar testes, tabelas esperadas e documentação.
- Não editar manualmente `models/cabo.sdf` como fonte primária; ele é gerado por `models/gerar_cabo.py`.

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

Resultados atualizados de subida vertical, comparando sem tether e tether ball:

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
2. Consolidar o cálculo de azimuth/elevation usando direção local dos últimos segmentos, inicialmente `janela_tangente_links=3`.
3. Investigar por que o drone com tether não responde ao `cmd_vel.linear.z` como no caso sem tether, mesmo quando o clamp do controlador não satura.
4. Testar comandos verticais constantes sem controlador de posição e verificar a resposta do `MulticopterVelocityControl` no modelo multibody cabo-drone.

## Last Known Good State

Último commit remoto conhecido no branch atual:

```text
d1ac633 (HEAD -> shared, origin/shared) Ajusta controle de trajetoria do drone
```

Observação: o worktree atual possui alterações não commitadas e arquivos não rastreados relevantes, incluindo `README.md`, configs adicionais e estes documentos.
