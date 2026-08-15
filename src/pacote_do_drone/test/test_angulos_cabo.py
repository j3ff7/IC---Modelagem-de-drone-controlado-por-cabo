import math
import unittest
from types import SimpleNamespace

from pacote_do_drone.cabo_angulos import (
    calcular_angulos_ancora_drone_graus,
    calcular_angulos_tangente_cabo_graus,
    calcular_angulos_vetor_graus,
    elevation_saturado,
    extrair_angulos_graus,
)


class TestAngulosCabo(unittest.TestCase):
    def test_extrai_angulos_em_graus(self):
        msg = SimpleNamespace(
            name=['cabo_azimuth_joint', 'cabo_elevation_joint'],
            position=[0.7853981633974483, -0.5235987755982988],
        )

        azimuth_deg, elevation_deg = extrair_angulos_graus(msg)

        self.assertAlmostEqual(azimuth_deg, 45.0)
        self.assertAlmostEqual(elevation_deg, -30.0)

    def test_detecta_elevation_perto_do_limite(self):
        self.assertFalse(elevation_saturado(80.0))
        self.assertTrue(elevation_saturado(88.5))
        self.assertTrue(elevation_saturado(-89.0))

    def test_elevation_geometrico_horizontal_vertical(self):
        _, el_horizontal = calcular_angulos_vetor_graus((-2.0, 0.0, 0.0))
        _, el_vertical = calcular_angulos_vetor_graus((0.0, 0.0, -2.0))

        self.assertAlmostEqual(el_horizontal, 0.0)
        self.assertAlmostEqual(el_vertical, 90.0)

    def test_elevation_geometrico_diagonal(self):
        azimuth_deg, elevation_deg = calcular_angulos_vetor_graus((-1.0, 0.0, -1.0))

        self.assertAlmostEqual(abs(azimuth_deg), 180.0)
        self.assertAlmostEqual(elevation_deg, 45.0)

    def test_angulo_ancora_drone_com_offset_do_sensor(self):
        azimuth_deg, elevation_deg = calcular_angulos_ancora_drone_graus(
            posicao_drone=(2.0, 0.18, 0.38),
            orientacao_drone=(0.0, 0.0, 0.0, 1.0),
            posicao_ancora=(0.0, 0.18, 0.33),
            offset_sensor_corpo=(0.0, 0.0, -0.05),
        )

        self.assertAlmostEqual(abs(azimuth_deg), 180.0)
        self.assertAlmostEqual(elevation_deg, 0.0)

    def test_tangente_vertical_com_drone_nivelado(self):
        azimuth_deg, elevation_deg = calcular_angulos_tangente_cabo_graus(
            orientacao_drone=(0.0, 0.0, 0.0, 1.0),
            orientacao_segmento_final=(0.0, -0.7071067811865475, 0.0, 0.7071067811865476),
        )

        self.assertAlmostEqual(azimuth_deg, 0.0)
        self.assertAlmostEqual(elevation_deg, 90.0)

    def test_tangente_vertical_com_drone_inclinado_dez_graus(self):
        meio_pitch = 0.5 * 0.17453292519943295
        azimuth_deg, elevation_deg = calcular_angulos_tangente_cabo_graus(
            orientacao_drone=(0.0, math.sin(meio_pitch), 0.0, math.cos(meio_pitch)),
            orientacao_segmento_final=(0.0, -0.7071067811865475, 0.0, 0.7071067811865476),
        )

        self.assertAlmostEqual(azimuth_deg, 0.0)
        self.assertAlmostEqual(elevation_deg, 80.0, places=6)


if __name__ == '__main__':
    unittest.main()
