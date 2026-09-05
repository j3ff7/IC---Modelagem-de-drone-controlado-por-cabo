#!/usr/bin/env python3
import argparse
import math
import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PX4_X500_SDF = ROOT / 'px4' / 'PX4-Autopilot' / 'Tools' / 'simulation' / 'gz' / 'models' / 'x500' / 'model.sdf'
PX4_X500_CONFIG = ROOT / 'px4' / 'PX4-Autopilot' / 'Tools' / 'simulation' / 'gz' / 'models' / 'x500' / 'model.config'
OUT_DIR = ROOT / 'src' / 'pacote_do_drone' / 'models' / 'x500_tethered'
OUT_SDF = OUT_DIR / 'model.sdf'
OUT_CONFIG = OUT_DIR / 'model.config'


def cylinder_inertia(mass, radius, length):
    i_transverse = mass * (3.0 * radius * radius + length * length) / 12.0
    i_axial = 0.5 * mass * radius * radius
    return i_transverse, i_axial


def inertia_xml(ixx, iyy, izz, indent):
    return f'''{indent}<inertia>
{indent}  <ixx>{ixx:.9g}</ixx>
{indent}  <ixy>0</ixy>
{indent}  <ixz>0</ixz>
{indent}  <iyy>{iyy:.9g}</iyy>
{indent}  <iyz>0</iyz>
{indent}  <izz>{izz:.9g}</izz>
{indent}</inertia>'''


def link_xml(index, n_links, link_length, mass, radius, axis, link_collisions):
    name = f'tether_link_{index}'
    if axis == 'x':
        pose = '0 0 0 0 1.57079632679 0'
        child_offset = f'{link_length:.9g} 0 0 0 0 0'
        com_pose = f'{0.5 * link_length:.9g} 0 0 0 1.57079632679 0'
    else:
        pose = '0 0 0 0 0 0'
        child_offset = f'0 0 {-link_length:.9g} 0 0 0'
        com_pose = f'0 0 {-0.5 * link_length:.9g} 0 0 0'

    parent_frame = 'tether_attach_link' if index == 1 else f'tether_link_{index - 1}'
    relative_pose = '0 0 0 0 0 0' if index == 1 else child_offset
    i_transverse, i_axial = cylinder_inertia(mass, radius, link_length)
    if axis == 'x':
        ixx, iyy, izz = i_axial, i_transverse, i_transverse
    else:
        ixx, iyy, izz = i_transverse, i_transverse, i_axial

    color = '0.02 0.02 0.02 1.0' if index < n_links else '0.05 0.05 0.05 1.0'
    collision = ''
    if link_collisions:
        collision = f'''
      <collision name="{name}_collision">
        <pose>{com_pose}</pose>
        <geometry>
          <cylinder>
            <radius>{0.75 * radius:.9g}</radius>
            <length>{link_length:.9g}</length>
          </cylinder>
        </geometry>
      </collision>'''

    return f'''
    <link name="{name}">
      <pose relative_to="{parent_frame}">{relative_pose}</pose>
      <inertial>
        <pose>{com_pose}</pose>
        <mass>{mass:.9g}</mass>
{inertia_xml(ixx, iyy, izz, '        ')}
      </inertial>
      <visual name="{name}_visual">
        <pose>{com_pose}</pose>
        <geometry>
          <cylinder>
            <radius>{radius:.9g}</radius>
            <length>{link_length:.9g}</length>
          </cylinder>
        </geometry>
        <material>
          <diffuse>{color}</diffuse>
          <ambient>{color}</ambient>
        </material>
      </visual>
{collision}
    </link>'''


def joint_xml(index, link_length, axis):
    parent = 'tether_attach_link' if index == 1 else f'tether_link_{index - 1}'
    child = f'tether_link_{index}'
    if index == 1:
        joint_pose = '0 0 0 0 0 0'
        relative_to = 'tether_attach_link'
    else:
        joint_pose = f'{link_length:.9g} 0 0 0 0 0' if axis == 'x' else f'0 0 {-link_length:.9g} 0 0 0'
        relative_to = parent

    sensor = ''
    if index == 1:
        sensor = '''
      <sensor name="tether_attach_force_torque" type="force_torque">
        <always_on>1</always_on>
        <update_rate>100</update_rate>
        <force_torque>
          <frame>child</frame>
          <measure_direction>child_to_parent</measure_direction>
        </force_torque>
      </sensor>'''

    return f'''
    <joint name="tether_joint_{index}" type="ball">
      <pose relative_to="{relative_to}">{joint_pose}</pose>
      <parent>{parent}</parent>
      <child>{child}</child>{sensor}
    </joint>'''


def tether_block(n_links, total_length, rho, radius, initial_axis, link_collisions, anchored, anchor_pose):
    link_length = total_length / n_links
    total_mass = rho * total_length
    link_mass = total_mass / n_links

    attach_mass = 0.005
    attach_inertia = 2.0e-7
    parts = [f'''
    <link name="tether_attach_link">
      <pose relative_to="base_link">0 0 -0.12 0 0 0</pose>
      <inertial>
        <mass>{attach_mass:.9g}</mass>
        <inertia>
          <ixx>{attach_inertia:.9g}</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>{attach_inertia:.9g}</iyy>
          <iyz>0</iyz>
          <izz>{attach_inertia:.9g}</izz>
        </inertia>
      </inertial>
      <visual name="tether_attach_visual">
        <geometry>
          <sphere>
            <radius>0.025</radius>
          </sphere>
        </geometry>
        <material>
          <diffuse>0.85 0.05 0.03 1.0</diffuse>
          <ambient>0.85 0.05 0.03 1.0</ambient>
        </material>
      </visual>
      <collision name="tether_attach_collision">
        <geometry>
          <sphere>
            <radius>0.015</radius>
          </sphere>
        </geometry>
      </collision>
    </link>
    <joint name="tether_attach_fixed" type="fixed">
      <parent>base_link</parent>
      <child>tether_attach_link</child>
    </joint>''']

    for i in range(1, n_links + 1):
        parts.append(link_xml(i, n_links, link_length, link_mass, radius, initial_axis, link_collisions))
        parts.append(joint_xml(i, link_length, initial_axis))

    if anchored:
        x, y, z = anchor_pose
        parts.append(f'''
    <link name="tether_anchor_link">
      <pose>{x:.9g} {y:.9g} {z:.9g} 0 0 0</pose>
      <inertial>
        <mass>0.001</mass>
        <inertia>
          <ixx>1e-7</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>1e-7</iyy>
          <iyz>0</iyz>
          <izz>1e-7</izz>
        </inertia>
      </inertial>
      <visual name="tether_anchor_visual">
        <geometry>
          <sphere>
            <radius>0.035</radius>
          </sphere>
        </geometry>
        <material>
          <diffuse>0.05 0.15 0.9 1.0</diffuse>
          <ambient>0.05 0.15 0.9 1.0</ambient>
        </material>
      </visual>
    </link>
    <joint name="tether_anchor_world_fixed" type="fixed">
      <parent>world</parent>
      <child>tether_anchor_link</child>
    </joint>
    <joint name="tether_anchor_joint" type="ball">
      <pose relative_to="tether_anchor_link">0 0 0 0 0 0</pose>
      <parent>tether_anchor_link</parent>
      <child>tether_link_{n_links}</child>
    </joint>''')

    return ''.join(parts), link_length, link_mass, total_mass


def generate(n_links, total_length, rho, radius, initial_axis, link_collisions, anchored, anchor_pose):
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    sdf = PX4_X500_SDF.read_text()
    sdf = sdf.replace("<model name='x500'>", "<model name='x500_tethered'>", 1)
    block, link_length, link_mass, total_mass = tether_block(
        n_links, total_length, rho, radius, initial_axis, link_collisions, anchored, anchor_pose
    )
    sdf = sdf.replace('    <link name="rotor_0">', block + '\n    <link name="rotor_0">', 1)
    OUT_SDF.write_text(sdf)

    config = PX4_X500_CONFIG.read_text()
    config = config.replace('<name>x500</name>', '<name>x500_tethered</name>', 1)
    config = re.sub(
        r'<description>.*?</description>',
        (
            '<description>'
            f'Project-local X500 variant for tether integration. N={n_links}, '
            f'L={total_length:.3f} m, rho={rho:.3f} kg/m, anchored={anchored}.'
            '</description>'
        ),
        config,
        count=1,
        flags=re.DOTALL,
    )
    OUT_CONFIG.write_text(config)

    i_transverse, i_axial = cylinder_inertia(link_mass, radius, link_length)
    print(f'N_links={n_links}')
    print(f'comprimento_total={total_length:.6f} m')
    print(f'comprimento_por_link={link_length:.6f} m')
    print(f'rho_linear={rho:.6f} kg/m')
    print(f'massa_total={total_mass:.6f} kg')
    print(f'massa_por_link={link_mass:.6f} kg')
    print(f'raio={radius:.6f} m')
    print(f'inercia_link_transversal={i_transverse:.9g} kg.m^2')
    print(f'inercia_link_axial={i_axial:.9g} kg.m^2')
    print(f'inicializacao={initial_axis}')
    print(f'colisoes_links={link_collisions}')
    print(f'ancorado={anchored}')
    if anchored:
        print(
            'ancora='
            f'({anchor_pose[0]:.6f}, {anchor_pose[1]:.6f}, {anchor_pose[2]:.6f}) m'
        )
    print(f'sdf={OUT_SDF}')


def main():
    parser = argparse.ArgumentParser(description='Generate project-local x500_tethered model.')
    parser.add_argument('--links', type=int, required=True)
    parser.add_argument('--length', type=float, default=0.10)
    parser.add_argument('--rho', type=float, default=0.06)
    parser.add_argument('--radius', type=float, default=0.003)
    parser.add_argument('--initial-axis', choices=('x', 'z'), default='x')
    parser.add_argument('--link-collisions', action='store_true')
    parser.add_argument('--anchored', action='store_true')
    parser.add_argument('--anchor-x', type=float, default=0.0)
    parser.add_argument('--anchor-y', type=float, default=0.0)
    parser.add_argument('--anchor-z', type=float, default=0.02)
    args = parser.parse_args()
    if args.length <= 0.0:
        raise SystemExit('length must be positive')
    if args.rho <= 0.0:
        raise SystemExit('rho must be positive')
    if args.radius <= 0.0:
        raise SystemExit('radius must be positive')
    if args.links <= 0:
        raise SystemExit('links must be positive')
    anchor_pose = (args.anchor_x, args.anchor_y, args.anchor_z)
    generate(
        args.links,
        args.length,
        args.rho,
        args.radius,
        args.initial_axis,
        args.link_collisions,
        args.anchored,
        anchor_pose,
    )


if __name__ == '__main__':
    main()
