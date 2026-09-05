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

---

## Adendo — segunda revisão (Claude Code), 2026-09-05

Revisão independente sobre o mesmo checkout/commit (confirmado idêntico via `git log -1`). Nenhuma divergência encontrada em relação ao handoff acima; apenas confirmação com evidência adicional e uma lacuna preenchida (classificação de portabilidade). Detalhes completos, citações de linha e a seção "REVISÃO CRUZADA — CODEX × CLAUDE CODE" estão em `MARSUPIAL_TETHER_REFERENCE_ANALYSIS.md`. Resumo com fonte marcada:

- **ARQUITETURA CONFIRMADA** [code][runtime-ext]: Gazebo Classic/ODE, três `spawn_entity.py` independentes; reexecutei a leitura do `gazebo_ros_link_attacher.cpp` no SHA `2879cf838565a2603bf03ba4f1ea202965ad0304` já clonado pelo Codex — código idêntico ao citado.
- **TOPOLOGIA CONFIRMADA** [code]: dupla filiação de `box_central` confirmada não só no SDF mas também no `urdf/rs_robot.urdf:225-299`; comentário de autor em `rs_robot.sdf:771-774` mostra que é intencional ("ambos joints devem ter parâmetros idênticos... spring_stiffness=0 é crítico").
- **SOLUÇÃO CONFIRMADA PARA CLOSED KINEMATIC LOOP** [code][inference]: continua **não existindo** solução de árvore; o comentário de crash em `gazebo_ros_link_attacher.cpp` ("An entity without a parent model should not happen" / "Inertial pointer is NULL") mostra que `SetModel()` é um contorno de limitação específica do Classic/ODE, não uma técnica de reparenting portável — reforça que não deve ser copiada para DART/Gazebo Sim.
- **MECANISMO DE CONEXÃO AO VEÍCULO / GROUND STATION** [code]: inalterado; confirmado por leitura.
- **MECANISMO DE REEL/WINCH** [code]: inalterado; adição — `ros2_control` declara `state_interface effort` para `joint_izquierdo` (`rs_robot.sdf:907-914`) e as rodas, sem nenhum consumidor Python no fluxo ativo. Caminho candidato para futura instrumentação de tensão no X500 se o hardware plugin equivalente popular esse campo.
- **MECANISMO DE COMPRIMENTO VARIÁVEL** [code][runtime — reexecução Python isolada]: reproduzi numericamente a geometria do gerador (distância total 17.862585799065698 m, offset do endpoint idêntico) — os números do Codex foram **reproduzidos por execução**, não apenas lidos.
- **MECANISMO DE TENSÃO** [code][paper]: confirmado que não há medição em nenhum dos dois; busca textual no PDF extraído não encontrou "load cell"/"force sensor"/"wrench"/"torque", só menções qualitativas de "tension".
- **DEPENDÊNCIAS DO PHYSICS ENGINE** [code]: confirmado via `CMakeLists.txt`/`package.xml` (gazebo_dev, gazebo_ros, gazebo_msgs — nenhum é Gazebo Sim).
- **O QUE É TRANSFERÍVEL PARA GAZEBO SIM/DART** [inference] — nova classificação objetiva por mecanismo adicionada em `MARSUPIAL_TETHER_REFERENCE_ANALYSIS.md §16` (PORTÁVEL DIRETAMENTE / ADAPTAR / REIMPLEMENTAR / INCOMPATÍVEL), preenchendo uma lacuna que o handoff original não detalhava em forma de tabela. Resumo: controle Python (winch, catenária offline) é PORTÁVEL DIRETAMENTE como esqueleto; a cadeia multibody é ADAPTAR; os plugins C++ locais e o `LiftDragPlugin` PRECISAM SER REIMPLEMENTADOS na API ECS do Gazebo Sim; o attach Classic e o duplo rolamento são INCOMPATÍVEIS/NÃO RECOMENDADOS.
- **O QUE NÃO DEVE SER COPIADO DIRETAMENTE** [code][inference]: confirma a lista original; reforça especialmente o attach via `SetModel()` (motivo técnico agora documentado com a citação do crash original) e o duplo rolamento.
- **PRÓXIMO EXPERIMENTO RECOMENDADO NO X500/PX4** [inference]: sem alteração da ordem recomendada pelo Codex (protótipo mínimo de dois endpoints → instrumentar forças → calibrar → payout → logging). Adiciono apenas: ao investigar `state_interface effort` como base de instrumentação de tensão no X500, validar primeiro se o hardware plugin usado (`gz_ros2_control`/equivalente) realmente popula esse campo para o joint do carretel antes de depender dele — no `marsupial_simulator_ros2` a interface existe no SDF mas não há evidência de que seja lida por nenhum consumidor, então não se pode inferir do código de referência que o valor é fisicamente correto quando populado.

**Divergências:** nenhuma identificada nesta revisão. Nenhum fato do handoff original foi removido ou substituído.
