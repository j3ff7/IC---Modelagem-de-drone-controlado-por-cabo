#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT_DIR="$ROOT_DIR/results/controller_evidence"
CSV="$OUT_DIR/evidencia_z060.csv"
NAME="evidencia_z060"

mkdir -p "$OUT_DIR"

echo "[1/5] Preparando ambiente ROS 2"
cd "$ROOT_DIR"
set +u
source /opt/ros/humble/setup.bash
set -u
colcon build --symlink-install --packages-select pacote_do_drone
set +u
source install/pacote_do_drone/share/pacote_do_drone/package.bash
set -u

echo "[2/5] Iniciando simulacao sem cabo"
ros2 launch pacote_do_drone controller_no_tether.launch.py > "$OUT_DIR/simulacao.log" 2>&1 &
SIM_PID=$!

cleanup() {
  if kill -0 "$SIM_PID" >/dev/null 2>&1; then
    kill "$SIM_PID" >/dev/null 2>&1 || true
    wait "$SIM_PID" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT

sleep 5

echo "[3/5] Rodando controlador para waypoint vertical"
python3 src/pacote_do_drone/scripts/test_moviment.py \
  --ros-args \
  -p x_alvo:=2.0 \
  -p y_alvo:=0.0 \
  -p altura_alvo:=0.60 \
  -p duracao_teste:=6.0 \
  -p csv_path:="$CSV" \
  > "$OUT_DIR/controlador.log" 2>&1

echo "[4/5] Gerando metricas e graficos"
python3 tools/plot_controller_test.py "$CSV" --name "$NAME" --out-dir "$OUT_DIR" \
  > "$OUT_DIR/metricas.json"

echo "[5/5] Avaliando evidencia numerica"
python3 - "$CSV" "$OUT_DIR/metricas.json" <<'PY'
import csv
import json
import math
import sys
from pathlib import Path

csv_path = Path(sys.argv[1])
metrics_path = Path(sys.argv[2])

with csv_path.open(newline="", encoding="utf-8") as f:
    rows = [
        {k: float(v) for k, v in row.items() if v not in ("", None)}
        for row in csv.DictReader(f)
    ]

if len(rows) < 10:
    raise SystemExit("FAIL: poucas amostras no CSV.")

with metrics_path.open(encoding="utf-8") as f:
    metrics = json.load(f)

z_ref = rows[-1]["z_ref"]
z0 = rows[0]["z"]
zf = rows[-1]["z"]
erro_z0 = abs(z_ref - z0)
erro_zf = abs(z_ref - zf)
melhor = min(rows, key=lambda row: abs(row["erro_z"]))
melhor_erro_z = abs(melhor["erro_z"])
reducao_z = 100.0 * (erro_z0 - erro_zf) / erro_z0 if erro_z0 > 1e-9 else 0.0
cmd_z_max = max(abs(row["cmd_z"]) for row in rows)
cmd_z_medio_inicio = sum(row["cmd_z"] for row in rows[: min(20, len(rows))]) / min(20, len(rows))
cmd_z_medio_fim = sum(row["cmd_z"] for row in rows[-min(20, len(rows)):]) / min(20, len(rows))
roll_max = metrics["roll_max_abs_deg"]
pitch_max = metrics["pitch_max_abs_deg"]

passou = (
    erro_zf < 0.08
    and melhor_erro_z < 0.03
    and reducao_z > 70.0
    and cmd_z_max > 0.2
    and abs(cmd_z_medio_fim) < abs(cmd_z_medio_inicio)
    and roll_max < 20.0
    and pitch_max < 20.0
)

print()
print("Evidencia do controlador - eixo Z sem cabo")
print("------------------------------------------")
print(f"z_ref                 : {z_ref:.3f} m")
print(f"z inicial             : {z0:.3f} m")
print(f"z final               : {zf:.3f} m")
print(f"erro z inicial        : {erro_z0:.3f} m")
print(f"erro z final          : {erro_zf:.3f} m")
print(f"menor erro z atingido : {melhor_erro_z:.4f} m em t={melhor['t_sim']:.2f} s")
print(f"reducao do erro z     : {reducao_z:.1f} %")
print(f"RMSE z                : {metrics['rmse_z']:.3f} m")
print(f"overshoot z           : {metrics['overshoot_z']:.3f} m")
print(f"convergencia |ez|<0.10: {metrics['tempo_convergencia_z_0p10']} s")
print(f"cmd_z max             : {cmd_z_max:.3f} m/s")
print(f"cmd_z medio inicial   : {cmd_z_medio_inicio:.3f} m/s")
print(f"cmd_z medio final     : {cmd_z_medio_fim:.3f} m/s")
print(f"roll max abs          : {roll_max:.2f} deg")
print(f"pitch max abs         : {pitch_max:.2f} deg")
print()
print("Resultado:", "PASS" if passou else "FAIL")
print()
print(f"CSV     : {csv_path}")
print(f"Metricas: {metrics_path}")
print(f"Graficos: {metrics_path.parent}")

if not passou:
    raise SystemExit(1)
PY
