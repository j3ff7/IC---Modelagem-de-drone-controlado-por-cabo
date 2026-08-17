# Decisions

Este arquivo registra decisões técnicas relevantes já inferíveis pelo repositório ou pelo contexto disponível nesta sessão.

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

Decisão:
Configuração atual usa `50` segmentos de `0.00588 kg`, `50` dummies de `0.0001 kg`, raiz `0.0005 kg` e ponta `0.0005 kg`, totalizando `0.300 kg`.

Justificativa:
Solicitação experimental e consistência com `tether_parameters.json`.

Alternativas consideradas:
Massa anterior não registrada com segurança neste documento.

Consequências:
Mudanças em `num_links` ou `length` devem redistribuir massa por segmento se a massa total de `0.30 kg` for preservada.

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
