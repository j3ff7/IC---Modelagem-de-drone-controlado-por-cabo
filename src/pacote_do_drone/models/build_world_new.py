import math
import json
import os

# ============================================================
# CAMINHOS E PASTAS
# ============================================================

caminho_json   = '/home/joseubu/IC/src/pacote_do_drone/tether_parameters.json'
pasta_models   = '/home/joseubu/IC/src/pacote_do_drone/models/'
pasta_worlds   = '/home/joseubu/IC/src/pacote_do_drone/worlds/'
caminho_sdf    = os.path.join(pasta_models, 'cabo.sdf')
caminho_world  = os.path.join(pasta_worlds, 'my_world.sdf')

os.makedirs(pasta_models, exist_ok=True)
os.makedirs(pasta_worlds, exist_ok=True)

def clamp_min(valor, minimo):
    return max(float(valor), minimo)

with open(caminho_json, 'r') as f:
    params = json.load(f)

length = clamp_min(params.get("length", 0.05), 0.01)

radius = clamp_min(params.get("radius", 0.003), 0.001) + 0.20

theta = math.radians(5)

num_links = math.pi / math.atan(
    (length * math.cos(theta)) / (2 * radius)
)

phi = ((num_links - 2) * 180) / num_links

p = (length*math.sin(theta))*num_links

print(f"O angulo phi é {phi:.2f}º")
print(f"A quantidades de elos por volta é {num_links:.2f} e o passo é {p:.4f}")
