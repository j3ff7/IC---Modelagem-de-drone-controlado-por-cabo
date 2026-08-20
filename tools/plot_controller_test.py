#!/usr/bin/env python3
import argparse
import csv
import json
import math
from pathlib import Path

from PIL import Image, ImageDraw


CAMPOS = [
    't_sim', 'dt_sim', 't_wall', 'rtf',
    'x_ref', 'y_ref', 'z_ref',
    'x', 'y', 'z',
    'erro_x', 'erro_y', 'erro_z',
    'vx', 'vy', 'vz',
    'cmd_x', 'cmd_y', 'cmd_z',
    'roll', 'pitch', 'yaw',
]

CORES = [
    (31, 119, 180),
    (214, 39, 40),
    (44, 160, 44),
    (148, 103, 189),
    (255, 127, 14),
]


def carregar_csv(caminho):
    dados = {campo: [] for campo in CAMPOS}
    with Path(caminho).open(newline='', encoding='utf-8') as f:
        for row in csv.DictReader(f):
            for campo in CAMPOS:
                dados[campo].append(float(row.get(campo, 0.0) or 0.0))
    if not dados['t_sim']:
        raise ValueError(f'CSV sem dados: {caminho}')
    return dados


def rmse(valores):
    return math.sqrt(sum(v * v for v in valores) / len(valores)) if valores else 0.0


def primeiro_tempo_convergencia(dados, eixo, tolerancia):
    erro = [abs(v) for v in dados[f'erro_{eixo}']]
    tempos = dados['t_sim']
    for i, valor in enumerate(erro):
        if valor > tolerancia:
            continue
        if all(v <= tolerancia for v in erro[i:]):
            return tempos[i]
    return None


def metricas(dados):
    erro_final_3d = math.sqrt(
        (dados['x_ref'][-1] - dados['x'][-1]) ** 2 +
        (dados['y_ref'][-1] - dados['y'][-1]) ** 2 +
        (dados['z_ref'][-1] - dados['z'][-1]) ** 2
    )
    z_ref = dados['z_ref'][-1]
    z0 = dados['z'][0]
    sentido_z = 1.0 if z_ref >= z0 else -1.0
    overshoot_z = max(0.0, max(sentido_z * (z - z_ref) for z in dados['z']))

    return {
        'rmse_x': rmse(dados['erro_x']),
        'rmse_y': rmse(dados['erro_y']),
        'rmse_z': rmse(dados['erro_z']),
        'erro_final_3d': erro_final_3d,
        'overshoot_z': overshoot_z,
        'tempo_convergencia_z_0p10': primeiro_tempo_convergencia(dados, 'z', 0.10),
        'rtf_medio': sum(dados['rtf']) / len(dados['rtf']),
        't_sim_final': dados['t_sim'][-1],
        't_wall_final': dados['t_wall'][-1],
        'roll_max_abs_deg': max(abs(v) for v in dados['roll']),
        'pitch_max_abs_deg': max(abs(v) for v in dados['pitch']),
    }


def limites(series):
    valores = [v for _, ys, _ in series for v in ys]
    vmin = min(valores)
    vmax = max(valores)
    if abs(vmax - vmin) < 1e-9:
        vmin -= 1.0
        vmax += 1.0
    margem = 0.08 * (vmax - vmin)
    return vmin - margem, vmax + margem


def desenhar_plot(caminho, titulo, xlabel, ylabel, x, series):
    w, h = 1200, 700
    left, right, top, bottom = 90, 40, 70, 90
    img = Image.new('RGB', (w, h), 'white')
    d = ImageDraw.Draw(img)

    xmin, xmax = min(x), max(x)
    if abs(xmax - xmin) < 1e-9:
        xmax = xmin + 1.0
    ymin, ymax = limites(series)

    def px(xv):
        return left + (xv - xmin) * (w - left - right) / (xmax - xmin)

    def py(yv):
        return h - bottom - (yv - ymin) * (h - top - bottom) / (ymax - ymin)

    d.rectangle([left, top, w - right, h - bottom], outline=(0, 0, 0))
    d.text((left, 20), titulo, fill=(0, 0, 0))
    d.text((w // 2 - 80, h - 35), xlabel, fill=(0, 0, 0))
    d.text((10, top + 10), ylabel, fill=(0, 0, 0))

    for i in range(6):
        gx = left + i * (w - left - right) / 5
        gy = top + i * (h - top - bottom) / 5
        d.line([(gx, top), (gx, h - bottom)], fill=(230, 230, 230))
        d.line([(left, gy), (w - right, gy)], fill=(230, 230, 230))
        xv = xmin + i * (xmax - xmin) / 5
        yv = ymax - i * (ymax - ymin) / 5
        d.text((gx - 20, h - bottom + 8), f'{xv:.1f}', fill=(0, 0, 0))
        d.text((8, gy - 7), f'{yv:.2f}', fill=(0, 0, 0))

    for idx, (label, ys, cor) in enumerate(series):
        pontos = [(px(xv), py(yv)) for xv, yv in zip(x, ys)]
        if len(pontos) >= 2:
            d.line(pontos, fill=cor, width=3)
        lx = left + 20 + (idx % 3) * 250
        ly = top + 15 + (idx // 3) * 22
        d.line([(lx, ly + 7), (lx + 35, ly + 7)], fill=cor, width=4)
        d.text((lx + 42, ly), label, fill=(0, 0, 0))

    img.save(caminho)


def plotar_teste(dados, nome, out_dir):
    t = dados['t_sim']

    for eixo in ['x', 'y', 'z']:
        desenhar_plot(
            out_dir / f'{nome}_position_{eixo}.png',
            f'{nome}: posicao {eixo.upper()}',
            'tempo simulado [s]',
            f'{eixo} [m]',
            t,
            [
                (f'{eixo}_ref', dados[f'{eixo}_ref'], CORES[0]),
                (eixo, dados[eixo], CORES[1]),
            ],
        )

    desenhar_plot(
        out_dir / f'{nome}_errors.png',
        f'{nome}: erros de posicao',
        'tempo simulado [s]',
        'erro [m]',
        t,
        [
            ('erro_x', dados['erro_x'], CORES[0]),
            ('erro_y', dados['erro_y'], CORES[1]),
            ('erro_z', dados['erro_z'], CORES[2]),
        ],
    )

    desenhar_plot(
        out_dir / f'{nome}_commands.png',
        f'{nome}: comandos do controlador',
        'tempo simulado [s]',
        'velocidade comandada [m/s]',
        t,
        [
            ('cmd_x', dados['cmd_x'], CORES[0]),
            ('cmd_y', dados['cmd_y'], CORES[1]),
            ('cmd_z', dados['cmd_z'], CORES[2]),
        ],
    )

    desenhar_plot(
        out_dir / f'{nome}_attitude.png',
        f'{nome}: atitude',
        'tempo simulado [s]',
        'angulo [deg]',
        t,
        [
            ('roll', dados['roll'], CORES[0]),
            ('pitch', dados['pitch'], CORES[1]),
            ('yaw', dados['yaw'], CORES[2]),
        ],
    )

    desenhar_plot(
        out_dir / f'{nome}_trajectory_xy.png',
        f'{nome}: trajetoria XY',
        'x [m]',
        'y [m]',
        dados['x'],
        [
            ('trajetoria real', dados['y'], CORES[0]),
            ('referencia linear', [
                dados['y'][0] + i * (dados['y_ref'][-1] - dados['y'][0]) / max(1, len(dados['y']) - 1)
                for i in range(len(dados['y']))
            ], CORES[1]),
        ],
    )


def interp_linear(xs, ys, xq):
    if xq <= xs[0]:
        return ys[0]
    if xq >= xs[-1]:
        return ys[-1]
    lo = 0
    hi = len(xs) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if xs[mid] <= xq:
            lo = mid
        else:
            hi = mid
    a = (xq - xs[lo]) / (xs[hi] - xs[lo])
    return ys[lo] + a * (ys[hi] - ys[lo])


def comparar_rtf(csv_alto, csv_baixo, nome, out_dir, eixo):
    alto = carregar_csv(csv_alto)
    baixo = carregar_csv(csv_baixo)
    t_max = min(alto['t_sim'][-1], baixo['t_sim'][-1])
    amostras = 400
    grade = [i * t_max / (amostras - 1) for i in range(amostras)]
    alto_i = [interp_linear(alto['t_sim'], alto[eixo], t) for t in grade]
    baixo_i = [interp_linear(baixo['t_sim'], baixo[eixo], t) for t in grade]
    erro = [a - b for a, b in zip(alto_i, baixo_i)]
    resultado = {'rmse_comparacao': rmse(erro), 'eixo': eixo, 't_sim_comparado': t_max}
    desenhar_plot(
        out_dir / f'{nome}_rtf_compare_{eixo}.png',
        f'{nome}: comparacao RTF alto x baixo',
        'tempo simulado [s]',
        f'{eixo} [m]',
        grade,
        [
            (f'{eixo} RTF alto', alto_i, CORES[0]),
            (f'{eixo} RTF baixo', baixo_i, CORES[1]),
        ],
    )
    return resultado


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('csv', nargs='?')
    parser.add_argument('--name', default='')
    parser.add_argument('--out-dir', default='results/controller_tests')
    parser.add_argument('--compare-low')
    parser.add_argument('--compare-name', default='rtf_compare')
    parser.add_argument('--axis', default='z', choices=['x', 'y', 'z'])
    args = parser.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    if args.compare_low:
        if not args.csv:
            raise SystemExit('Informe o CSV de RTF alto antes de --compare-low.')
        resultado = comparar_rtf(args.csv, args.compare_low, args.compare_name, out_dir, args.axis)
        with (out_dir / f'{args.compare_name}_metrics.json').open('w', encoding='utf-8') as f:
            json.dump(resultado, f, indent=2)
        print(json.dumps(resultado, indent=2))
        return

    if not args.csv:
        raise SystemExit('Informe um CSV para plotar.')

    nome = args.name or Path(args.csv).stem
    dados = carregar_csv(args.csv)
    plotar_teste(dados, nome, out_dir)
    resultado = metricas(dados)
    with (out_dir / f'{nome}_metrics.json').open('w', encoding='utf-8') as f:
        json.dump(resultado, f, indent=2)
    print(json.dumps(resultado, indent=2))


if __name__ == '__main__':
    main()
