# Decisions

Este arquivo registra decisões técnicas relevantes já inferíveis pelo repositório ou pelo contexto disponível nesta sessão.

## 2026-08-27 Integrar PX4 Como Trilha Paralela Inicial

Contexto:
O hardware real foi validado com PX4 v1.14.4 no Pixhawk/PX4_FMU_V3, commit `1555f2bd2229544c43966ab5f94879c41d8e1e01`. Foi solicitado iniciar a migracao para PX4 preservando a baseline atual do tethered drone.

Decisão:
Adicionar o PX4 como workspace local paralelo em `px4/PX4-Autopilot/`, ignorado pelo Git deste repositorio, fixado inicialmente no commit/tag:

```text
PX4 tag     v1.14.4
PX4 commit  1555f2bd2229544c43966ab5f94879c41d8e1e01
modelo SITL gz_x500
```

Justificativa:
O commit informado existe no upstream e corresponde a `refs/tags/v1.14.4^{}` e a branch `release/1.14`. Isso mantem a simulacao proxima da versao ja validada no Pixhawk, sem trocar imediatamente o controlador/tether atual.

Consequências:
O repositorio principal continua guardando o tether, sensores, metricas e documentacao. O PX4 inteiro nao deve ser versionado dentro deste repo. A proxima integracao deve criar uma variante externa do `x500` com ponto de conexao para o cabo, preservando o PX4 upstream sem modificacoes nesta fase.

## 2026-08-20 Congelar Baseline Para Testes Cardeais Estáticos

Contexto:
Os testes N/S/E/W foram executados com a mesma configuracao fisica, os mesmos ganhos e `cmd_vel_frame:=body`. O objetivo era verificar se havia assimetria grosseira ou falha do controlador antes de prosseguir para trajetorias mais complexas.

Decisão:
Manter a baseline atual para novos testes estaticos:

```text
L=2.5 m
N=50
densidade_linear_kg_m=0.06
connection_type=ball
initial_shape=sine_slack horizontal
janela_tangente_metros=0.15
cmd_vel_frame=body
```

Justificativa:
Os quatro casos cardeais concluem a sequencia, mantem atitude pequena, tensao maxima em torno de `1.1-1.2 N`, erro final RMS menor ou igual a `0.10 m` e sem saturacao persistente na janela final.

Consequências:
Nao retunar ganhos por direcao. A proxima frente deve ser validar redundantemente a medicao angular da tangente local e estudar a acomodacao lenta do cabo, especialmente no caso W.

## 2026-08-17 Revisar Densidade Linear Para `0.06 kg/m`

Contexto:
A baseline anterior derivada de `0.01 kg/m` deixou o cabo dinamico com apenas `0.031 kg` incluindo auxiliares. Foi solicitado revisar para `rho_linear = 0.06 kg/m`, mantendo `L=2.5 m`, `N=50`, `connection_type=ball`, `initial_shape=sine_slack horizontal`, `cmd_vel_frame=body`, `/clock` e `janela_tangente_metros=0.15`.

Decisão:
Usar `densidade_linear_kg_m = 0.06 kg/m` como baseline atual do branch.

Valores resultantes:

```text
mass por segmento     0.003000 kg
massa segmentos       0.150 kg
auxiliares            ~0.006 kg
massa dinamica total  ~0.156 kg
```

Justificativa:
O teste vertical permanece estavel e o caso N passa a ser reproduzivel com um ajuste pequeno de integral/amortecimento do controlador. A nova massa preserva uma escala fisica mais plausivel que o cabo antigo de `0.30 kg`, sem ficar leve demais para os testes de interacao.

Consequências:
A decisão antiga de `0.01 kg/m` fica supersedida para a baseline ativa. Ela permanece no histórico apenas como experimento comparativo.

## 2026-08-17 Usar Integral Pequena Para Compensar Erro Estacionario Do Tether

Contexto:
Com `rho_linear = 0.06 kg/m`, o caso N sem integral ficava perto do alvo, com atitude pequena e sem saturacao persistente, mas estacionava em torno de `z=1.87 m` para referencia `z=2.0 m`.

Decisão:
Atualizar os defaults do launch para uma compensacao conservadora:

```text
ganho_altura        1.5
ganho_integral_xy   0.05
ganho_integral_z    0.08
ganho_velocidade_xy 1.4
limite_vel_xy       0.35 m/s
tolerancia_posicao  0.12 m
tolerancia_altura   0.10 m
```

Justificativa:
O caso N concluiu com erro medio final de aproximadamente `0.10 m`, `roll_max=0.89 deg`, `pitch_max=0.20 deg`, `Tmax=1.15 N` e sem saturacao persistente.

Consequências:
Essa e uma compensacao de erro estacionario causada pelo cabo, nao uma mudanca estrutural do controlador. Novos ajustes devem ser comparados contra essa baseline.

## 2026-08-15 Usar `src/pacote_do_drone` Como Fonte Principal Da Simulação Gazebo

Contexto:
O repositório possui diretórios legados `Gazebo/`, `Chrono/`, `Coppelia/` e também pacotes ROS 2 em `src/`.

Decisão:
O fluxo principal atual para ROS 2/Gazebo usa `src/pacote_do_drone`, especialmente `start_sim.launch.py`, `models/meu_drone`, `models/gerar_cabo.py`, `models/cabo.sdf` e `tether_parameters.json`.

Justificativa:
O README e os launch files atuais apontam para esse pacote como a simulação ativa.

Alternativas consideradas:
Usar diretamente `Gazebo/` legado ou outros simuladores. Justificativa não registrada no contexto disponível.

Consequências:
Mudanças funcionais da simulação principal devem priorizar `src/pacote_do_drone`. Diretórios legados devem ser tratados com cautela.

## 2026-08-15 Medir Ângulos Do Cabo No Frame Do Drone

Contexto:
O sensor simulado deve representar a orientação do cabo em relação ao drone, análogo a um sensor tipo joystick conectado ao drone.

Decisão:
Calcular azimuth/elevation a partir do vetor/tangente do cabo expresso no frame do drone.

Justificativa:
Se o cabo permanece vertical no mundo e o drone inclina, a medição deve mudar com a atitude do drone.

Alternativas consideradas:
Usar ângulos no frame global ou leitura direta das juntas. Essas leituras continuam disponíveis como diagnóstico, mas não são a grandeza principal.

Consequências:
As funções em `cabo_angulos.py` usam rotação inversa da orientação do drone para expressar vetores no frame local.

## 2026-08-15 Convenção De Ângulos

Contexto:
Foi necessário alinhar os testes com postes, o sensor do drone e a interpretação em robótica.

Decisão:
Usar frame local `x` frente, `y` esquerda, `z` cima, com:

```text
azimuth   = atan2(y, x)
elevation = atan2(-z, sqrt(x^2 + y^2))
```

Justificativa:
Com essa convenção, cabo descendo verticalmente no frame do sensor produz `elevation = 90 deg`.

Alternativas consideradas:
Usar elevação positiva para cima. Essa variante existe como função auxiliar para a tangente no lado da âncora, mas não é a convenção principal do sensor do drone.

Consequências:
Valores esperados em `angulos_postes_esperados.json` e `angulos_slack_esperados.json` dependem dessa convenção.

## 2026-08-15 Preservar `cmd_vel_frame:=body` Para O Controlador Atual

Contexto:
Testes indicaram que o plugin `MulticopterVelocityControl` interpreta o comando linear do `Twist` no frame do corpo do drone.

Decisão:
Usar `cmd_vel_frame:=body` para os testes atuais.

Justificativa:
Resultados prévios indicaram melhora e consistência quando o controlador converte velocidade desejada do mundo para o frame do corpo antes de publicar.

Alternativas consideradas:
`cmd_vel_frame:=world`, que permaneceu como opção de parâmetro, mas não deve ser usado como padrão dos ensaios atuais sem nova justificativa.

Consequências:
Alterações no controlador devem preservar essa configuração salvo experimento documentado.

## 2026-08-15 Validar Ângulos Primeiro Em Mundos Estáticos Com Postes

Contexto:
A dinâmica do drone e do cabo dificulta isolar erros de convenção angular.

Decisão:
Criar o pacote `cabo_avaliacao` para validar geometrias conhecidas com postes nos casos `n`, `s`, `e`, `w`, `ne`, `nw`, `se`, `sw` e customizações.

Justificativa:
Separar validação de sinais/frames da dinâmica do drone.

Alternativas consideradas:
Validar apenas no drone. Descartado pragmaticamente porque mistura erros do sensor com controle e tether.

Consequências:
Mudanças de convenção angular devem ser refletidas nos testes de `cabo_avaliacao` e nas tabelas esperadas.

## 2026-08-15 Não Ajustar Ganhos Antes De Resolver Condição Inicial Do Tether

Contexto:
O controlador básico funcionou sem cabo e também com cabo em condição moderada, mas falhou com spawn original e tether tensionado.

Decisão:
Investigar primeiro geometria e dinâmica inicial do cabo antes de alterar ganhos.

Justificativa:
Resultados de simulação indicaram pico de tensão significativo e pitch elevado associados à condição inicial.

Alternativas consideradas:
Ajustar ganhos empiricamente. Adiado para evitar mascarar o problema físico.

Consequências:
Experimentos devem registrar tensão inicial, pico de tensão, pitch/roll, erro e geometria inicial.

## 2026-08-15 Massa Total Do Cabo Ajustada Para Aproximadamente 300 g

Contexto:
Foi solicitado usar um cabo de massa total aproximadamente `0.30 kg`.

Decisão histórica supersedida:
Configuração atual usa `50` segmentos de `0.00588 kg`, `50` dummies de `0.0001 kg`, raiz `0.0005 kg` e ponta `0.0005 kg`, totalizando `0.300 kg`.

Justificativa:
Solicitação experimental e consistência com `tether_parameters.json`.

Alternativas consideradas:
Massa anterior não registrada com segurança neste documento.

Consequências:
Esta decisao foi supersedida em 2026-08-17 pela densidade linear `0.06 kg/m`. A baseline ativa nao e mais `0.30 kg`; para `L=2.5 m`, ela usa `0.150 kg` nos segmentos e aproximadamente `0.156 kg` incluindo links auxiliares. Mudancas futuras devem seguir a densidade linear ativa, salvo novo experimento documentado.

## 2026-08-17 Usar Tempo Simulado Em Testes E Controle

Contexto:
Com o cabo de 50 links presente, o Gazebo roda com fator de tempo real muito menor que 1. Os testes anteriores de `velocity_test` e parte da instrumentação do controlador usavam tempo de parede. Isso fazia o comando vertical ser encerrado depois de poucos segundos reais, antes de o simulador avançar tempo físico suficiente, criando a falsa impressão de que `cmd_vel.z` não produzia subida.

Decisão:
Usar `/clock` como base de tempo para:

```text
velocity_test.py:
  duracao do comando
  periodo de log
  derivadas vz/az

movimento_circular.py:
  derivadas de velocidade por diferença
  integradores
  tempo de hovering
  periodo de log
```

Justificativa:
Repetindo o teste aberto com tempo simulado, o drone com tether `ball`, massa `0.30 kg` e `L=2.5 m` sobe de forma coerente: com `vz_cmd=0.25 m/s`, o `base_link` foi de aproximadamente `z=0.33 m` para `z=0.60 m` em cerca de `2 s` simulados, com pitch pequeno e momento nulo na conexão.

Alternativas consideradas:
Continuar usando tempo de parede e aumentar timeouts reais. Isso preservaria o erro conceitual e manteria derivadas/hover dependentes do desempenho da simulação.

Consequências:
Ensaios com cabo devem ser interpretados em tempo simulado (`t_sim`). O tempo de parede (`t_wall`) continua útil apenas para estimar custo computacional/RTF.

## 2026-08-17 Separar Estabilização De Hover

Contexto:
Considerar chegada ao waypoint por uma amostra instantânea pode aceitar uma passagem transitória pelo alvo como se fosse hover.

Decisão:
O controlador passa a usar dois tempos em simulação:

```text
tempo_estabilizacao:
  tempo continuo dentro das tolerancias de posição e velocidade antes de iniciar hover

tempo_hover:
  tempo simulado de permanência após estabilização
```

Justificativa:
Isso torna os testes de sensor mais robustos, pois as estatísticas de azimuth/elevation devem ser coletadas depois que o drone estabilizou, não durante a aproximação.

Consequências:
Os arquivos de trajetória continuam definindo `tempo_hover`; `tempo_estabilizacao` é parâmetro de launch/controlador. A lógica e os ganhos do controlador foram preservados.

## 2026-08-17 Massa Do Tether Derivada Da Densidade Linear Física

Contexto:
A baseline anterior de `0.30 kg` para `L=2.5 m` representava um cabo aproximadamente 12 vezes mais pesado que a densidade linear física informada, `0.01 kg/m`.

Decisão:
A massa do tether é derivada de sua densidade linear física, `0.01 kg/m`, e do comprimento configurado. Para `L=2.5 m`, a massa nominal dos segmentos é `0.025 kg`.

Implementação:
`tether_parameters.json` define `densidade_linear_kg_m` e `comprimento_total_m`. Quando `densidade_linear_kg_m` está presente, `models/gerar_cabo.py` calcula a massa por segmento automaticamente:

```text
mass = densidade_linear_kg_m * comprimento_total_m / num_links
```

Para `50` segmentos:

```text
mass = 0.0005 kg por segmento
massa segmentos = 0.025 kg
massa auxiliares = 0.006 kg
massa dinâmica total = 0.031 kg
```

Justificativa:
O teste vertical com `25 g` concluiu hover, reduziu a tensão medida no drone de aproximadamente `0.42/0.47 N` para `0.10/0.10 N` média/máxima, eliminou a degradação de pitch e aproximou a resposta do caso sem tether.

Consequências:
O parâmetro legado `mass` permanece no JSON como valor compatível/diagnóstico, mas a densidade linear tem precedência. A baseline de `0.30 kg` permanece documentada apenas como histórico comparativo.

## 2026-08-16 Conexão Tether-Drone Deve Permitir Orientação Passiva

Contexto:
Com `L=2.5 m`, cabo horizontalmente folgado e controlador inalterado, a junta fixa `ponta_cabo -> cabo_sensor_link` reduziu o pico de tensão em relação ao cabo esticado, mas ainda induziu pitch sustentado de aproximadamente `31 deg`. Um sensor de força/torque na conexão mediu momento não nulo.

Decisão:
A conexão nominal entre tether e drone não deve ser `fixed`. Ela deve transmitir força no ponto de conexão, mas permitir orientação passiva do cabo, sem impor a orientação do último segmento ao corpo do drone. A junta `ball` passa a ser a baseline física provisória e `fixed` fica apenas como opção diagnóstica.

Justificativa:
No mesmo cenário, trocar apenas a conexão de `fixed` para `ball` reduziu o pitch máximo de `31.2 deg` para `0.2 deg` e o momento medido na conexão de `0.044 Nm` para aproximadamente `0.000 Nm`, mantendo a força transmitida.

Alternativas consideradas:
Manter conexão fixa e alterar mola/damping do cabo. Zerar `joint_spring_stiffness` não removeu o pitch, reduzir damping teve efeito pequeno, dobrar damping aumentou tensões, e remover colisões dos segmentos piorou drasticamente o comportamento.

Consequências:
O sensor de azimuth/elevation deve ser calculado a partir da direção local do cabo no frame do drone, usando poses dos segmentos próximos à conexão, em vez de depender de uma junta fixa que force orientação. Se um mecanismo explícito de 2 DOF for modelado no futuro, ele deve preservar essa propriedade de não transmissão de momento artificial.
