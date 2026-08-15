import unittest
import json
import tempfile

from cabo_avaliacao.cenarios import CASOS, nomes_casos, parametros_caso


class TestCenarios(unittest.TestCase):
    def test_oito_casos(self):
        self.assertEqual(set(CASOS), {'n', 's', 'e', 'w', 'ne', 'nw', 'se', 'sw'})

    def test_elevation_igual_para_todos(self):
        elevacoes = {round(parametros_caso(caso)['elevation_esperado_graus'], 6) for caso in CASOS}
        self.assertEqual(len(elevacoes), 1)

    def test_azimuth_cardeal_leste(self):
        self.assertAlmostEqual(parametros_caso('e')['azimuth_esperado_graus'], 90.0)

    def test_azimuth_cardeal_norte(self):
        self.assertAlmostEqual(parametros_caso('n')['azimuth_esperado_graus'], 180.0)

    def test_azimuth_cardeal_sul(self):
        self.assertAlmostEqual(parametros_caso('s')['azimuth_esperado_graus'], 0.0)

    def test_azimuth_cardeal_oeste(self):
        self.assertAlmostEqual(parametros_caso('w')['azimuth_esperado_graus'], -90.0)

    def test_config_customizado(self):
        config = {
            'cabo_comprimento': 3.0,
            'poste_altura': 2.0,
            'ancora': [0.0, 0.0, 0.1],
            'sensor_yaw_graus': 90.0,
            'casos': {
                'custom': {'x': 0.0, 'y': -2.0},
            },
        }
        with tempfile.NamedTemporaryFile('w', suffix='.json') as f:
            json.dump(config, f)
            f.flush()

            self.assertEqual(nomes_casos(f.name), ('custom',))
            params = parametros_caso('custom', f.name)

        self.assertEqual(params['poste_topo'], (0.0, -2.0, 2.1))
        self.assertAlmostEqual(params['azimuth_esperado_graus'], 0.0)
        self.assertAlmostEqual(params['elevation_esperado_graus'], 45.0)

    def test_catenaria_com_folga_altera_tangente_no_sensor(self):
        config = {
            'cabo_comprimento': 2.0,
            'poste_altura': 1.2,
            'ancora': [0.0, 0.0, 0.05],
            'sensor_yaw_graus': 90.0,
            'casos': {
                'c1': {'x': 0.0, 'y': -0.8},
            },
        }
        with tempfile.NamedTemporaryFile('w', suffix='.json') as f:
            json.dump(config, f)
            f.flush()

            reta = parametros_caso('c1', f.name, geometria='reta')
            catenaria = parametros_caso('c1', f.name, geometria='catenaria')

        self.assertAlmostEqual(catenaria['azimuth_esperado_graus'], 0.0)
        self.assertGreater(catenaria['elevation_esperado_graus'], reta['elevation_esperado_graus'])


if __name__ == '__main__':
    unittest.main()
