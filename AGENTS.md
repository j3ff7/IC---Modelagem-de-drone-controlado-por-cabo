# AGENTS.md

Este repositório desenvolve uma simulação ROS 2/Gazebo de um drone conectado a um cabo flexível, com instrumentação para medir azimuth/elevation do cabo no frame do drone e ambientes auxiliares para validar esses cálculos.

Antes de modificar código, leia:

- [README.md](README.md)
- [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)
- [docs/DECISIONS.md](docs/DECISIONS.md)
- [docs/CURRENT_STATUS.md](docs/CURRENT_STATUS.md)
- [docs/EXPERIMENTS.md](docs/EXPERIMENTS.md), quando a mudança envolver testes, simulação ou diagnóstico

## Organização

- `src/pacote_do_drone`: simulação principal do drone, cabo, launch, controlador, sensores e configs.
- `src/cabo_avaliacao`: mundos estáticos com postes para validar ângulos do cabo.
- `Gazebo`, `Chrono`, `Coppelia`: material legado/experimental. A fonte principal atual para ROS 2/Gazebo está em `src/`.
- `README.md`: uso prático e comandos principais.
- `docs/`: memória técnica persistente.

## Comandos

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select pacote_do_drone cabo_avaliacao
source install/setup.bash
```

Simulação principal:

```bash
ros2 launch pacote_do_drone start_sim.launch.py
```

Com controlador:

```bash
ros2 launch pacote_do_drone start_sim.launch.py \
  controlador_trajetoria:=true \
  waypoints_file:=config/trajetoria_hover_z2_unico.json \
  cmd_vel_frame:=body
```

Testes Python:

```bash
colcon test --packages-select pacote_do_drone cabo_avaliacao
colcon test-result --verbose
```

## Convenções E Restrições

- O frame local usa `x` para frente, `y` para esquerda e `z` para cima.
- Azimuth/elevation do cabo usam `azimuth = atan2(y, x)` e `elevation = atan2(-z, sqrt(x^2+y^2))`.
- Para o controlador validado até aqui, preserve `cmd_vel_frame:=body` salvo justificativa explícita.
- Não altere ganhos do controlador para mascarar problemas de condição inicial do cabo sem registrar o experimento.
- `src/pacote_do_drone/models/cabo.sdf` é gerado por `src/pacote_do_drone/models/gerar_cabo.py`; evite editar o SDF gerado manualmente.
- Há worktree com alterações não commitadas neste branch; não reverta mudanças sem pedido explícito.

## Validação E Documentação

Após mudanças relevantes:

- Atualize `docs/ARCHITECTURE.md` se mudar a arquitetura, tópicos, frames ou fluxo de dados.
- Atualize `docs/DECISIONS.md` se uma decisão técnica importante for tomada.
- Atualize `docs/CURRENT_STATUS.md` se mudar o estado, bloqueios ou próximo passo.
- Atualize `docs/EXPERIMENTS.md` se realizar ensaios, descartar hipóteses ou obter resultados quantitativos.

Evite alterações de documentação para mudanças triviais.
