num_links = 35
length = 0.02
radius = 0.004
mass = 0.001

urdf = f"""<?xml version="1.0"?>
<robot name="cabo_flexivel">
  <link name="raiz_cabo">
    <inertial>
      <mass value="0.01"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>
"""

parent_link = "raiz_cabo"

for i in range(1, num_links + 1):
    urdf += f"""
  <link name="dummy_{i}">
    <inertial>
      <mass value="0.01"/> <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="joint_{i}_y" type="continuous">
    <parent link="{parent_link}"/>
    <child link="dummy_{i}"/>
    <origin xyz="0 0 {length}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <dynamics damping="0.2" friction="0.005"/>
  </joint>

  <link name="segment_{i}">
    <visual>
      <geometry><cylinder radius="{radius}" length="{length}"/></geometry>
      <origin xyz="0 0 {length/2}" rpy="0 0 0"/>
      <material name="black"><color rgba="0 0 0 1"/></material>
    </visual>
    <collision>
      <geometry><cylinder radius="{radius}" length="{length}"/></geometry>
      <origin xyz="0 0 {length/2}" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="{mass}"/>
      <inertia ixx="0.0000003958" ixy="0" ixz="0" iyy="0.0000003958" iyz="0" izz="0.000000125"/>
    </inertial>
  </link>

  <joint name="joint_{i}_x" type="continuous">
    <parent link="dummy_{i}"/>
    <child link="segment_{i}"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <dynamics damping="0.02" friction="0.005"/>
  </joint>
"""
    # Adiciona os sensores apenas na primeira junta (i == 1)
    if i == 1:
        urdf += """
  <gazebo>
    <plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher">
      <topic>/angulos_cabo</topic>
      <joint_name>joint_1_x</joint_name>
      <joint_name>joint_1_y</joint_name>
    </plugin>
  </gazebo>
"""
    parent_link = f"segment_{i}"

urdf += "</robot>\n"

with open("cabo.urdf", "w") as f:
    f.write(urdf)
    
print("Arquivo cabo.urdf gerado com sucesso pelo Python!")