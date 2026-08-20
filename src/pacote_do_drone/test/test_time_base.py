from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CONTROLADORES = [
    ROOT / 'scripts' / 'test_moviment.py',
    ROOT / 'scripts' / 'test_tensão.py',
]


def test_controladores_nao_usam_tempo_de_parede():
    proibidos = [
        'time.time',
        'time.monotonic()',
        'time.perf_counter',
        'datetime',
    ]

    for caminho in CONTROLADORES:
        texto = caminho.read_text(encoding='utf-8')
        for termo in proibidos:
            assert termo not in texto, f'{caminho.name} usa {termo}'


def test_controladores_usam_clock_simulado_para_dt():
    for caminho in CONTROLADORES:
        texto = caminho.read_text(encoding='utf-8')
        assert "Clock" in texto
        assert "'/clock'" in texto
        assert 'calcular_dt_controle' in texto
        assert 'self.sim_time_ns' in texto


def test_controladores_nao_usam_dt_fixo_no_pid():
    proibidos = [
        '* self.dt',
        '/ self.dt',
    ]

    for caminho in CONTROLADORES:
        texto = caminho.read_text(encoding='utf-8')
        for termo in proibidos:
            assert termo not in texto, f'{caminho.name} ainda usa {termo}'
