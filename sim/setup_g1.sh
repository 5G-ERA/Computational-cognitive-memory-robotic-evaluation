#!/usr/bin/env bash
# ============================================================
# setup_g1.sh   (ejecutar en el Mac, dentro de  g1_ws/src )
# Convierte el G1 (unitree_ros) en un paquete usable por Gazebo
# para NAVEGACION:
#   - copia g1_description como paquete ament (con meshes)
#   - genera urdf/g1_nav.urdf:  quita 'world', rigidiza juntas,
#     arregla rutas de meshes -> package://, anade base_footprint,
#     LiDAR y el plugin planar_move (cmd_vel -> movimiento + odom)
#   - saca unitree_ros de src (monorepo gigante)
# ============================================================
set -euo pipefail
UR=unitree_ros/robots/g1_description
[ -d "$UR" ] || { echo "ERROR: ejecuta esto dentro de g1_ws/src (no encuentro $UR)"; exit 1; }
[ -f "$UR/g1_23dof.urdf" ] || { echo "ERROR: no existe $UR/g1_23dof.urdf"; exit 1; }

echo "== 1) Copiando g1_description como paquete =="
rm -rf g1_description
cp -r "$UR" g1_description
mkdir -p g1_description/urdf

echo "== 2) package.xml + CMakeLists (ament) =="
cat > g1_description/package.xml <<'EOF'
<?xml version="1.0"?>
<package format="3">
  <name>g1_description</name>
  <version>0.0.1</version>
  <description>Unitree G1 description (URDF + meshes) for Gazebo navigation</description>
  <maintainer email="adrian@example.com">adrian</maintainer>
  <license>BSD-3-Clause</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <export><build_type>ament_cmake</build_type></export>
</package>
EOF
cat > g1_description/CMakeLists.txt <<'EOF'
cmake_minimum_required(VERSION 3.8)
project(g1_description)
find_package(ament_cmake REQUIRED)
install(DIRECTORY meshes urdf DESTINATION share/${PROJECT_NAME})
ament_package()
EOF

echo "== 3) Generando urdf/g1_nav.urdf (rigido + base_footprint + lidar + planar_move) =="
python3 - "$PWD/g1_description/g1_23dof.urdf" "$PWD/g1_description/urdf/g1_nav.urdf" <<'PY'
import sys, xml.etree.ElementTree as ET
src, dst = sys.argv[1], sys.argv[2]
t = ET.parse(src); r = t.getroot()
# quitar bloques existentes que estorban
for tag in ('gazebo','ros2_control','transmission'):
    for e in list(r.findall(tag)): r.remove(e)
# quitar link 'world' y las juntas con parent=world (si no, queda clavado)
for l in list(r.findall('link')):
    if l.get('name') == 'world': r.remove(l)
for j in list(r.findall('joint')):
    p = j.find('parent')
    if p is not None and p.get('link') == 'world': r.remove(j)
# rigidizar: toda junta movil -> fixed (sin ragdoll, sin controladores)
for j in r.findall('joint'):
    if j.get('type') in ('revolute','continuous','prismatic','floating','planar'):
        j.set('type','fixed')
        for tg in ('axis','limit','dynamics','mimic','safety_controller'):
            e = j.find(tg)
            if e is not None: j.remove(e)
# rutas de meshes relativas -> package://
for m in r.iter('mesh'):
    fn = m.get('filename') or ''
    if fn.startswith('meshes/'): m.set('filename', 'package://g1_description/'+fn)
def inertial(parent, mass, i):
    ine = ET.SubElement(parent,'inertial')
    ET.SubElement(ine,'origin',{'xyz':'0 0 0','rpy':'0 0 0'})
    ET.SubElement(ine,'mass',{'value':str(mass)})
    ET.SubElement(ine,'inertia',{'ixx':i,'iyy':i,'izz':i,'ixy':'0','ixz':'0','iyz':'0'})
# base_footprint (raiz) + junta a pelvis
bf = ET.SubElement(r,'link',{'name':'base_footprint'}); inertial(bf,0.1,'0.001')
bj = ET.SubElement(r,'joint',{'name':'base_joint','type':'fixed'})
ET.SubElement(bj,'parent',{'link':'base_footprint'}); ET.SubElement(bj,'child',{'link':'pelvis'})
ET.SubElement(bj,'origin',{'xyz':'0 0 0.0','rpy':'0 0 0'})
# lidar a 0.8 m sobre base_footprint (ve las paredes de 1 m)
ll = ET.SubElement(r,'link',{'name':'lidar_link'}); inertial(ll,0.05,'0.0001')
lj = ET.SubElement(r,'joint',{'name':'lidar_joint','type':'fixed'})
ET.SubElement(lj,'parent',{'link':'torso_link'}); ET.SubElement(lj,'child',{'link':'lidar_link'})
ET.SubElement(lj,'origin',{'xyz':'0.0002835 0.00003 0.40618','rpy':'0 0 0'})   # = posicion real del Mid-360, nivelado para scan 2D
t.write(dst, xml_declaration=False, encoding='utf-8')
gz = '''
  <gazebo>
    <plugin name="planar_move" filename="libgazebo_ros_planar_move.so">
      <ros><namespace>/</namespace></ros>
      <command_topic>cmd_vel</command_topic>
      <odometry_topic>odom</odometry_topic>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_footprint</robot_base_frame>
      <odometry_rate>30.0</odometry_rate>
      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
    </plugin>
  </gazebo>
  <gazebo reference="lidar_link">
    <sensor name="lidar" type="ray">
      <always_on>true</always_on><update_rate>10</update_rate>
      <ray><scan><horizontal>
        <samples>360</samples><resolution>1</resolution>
        <min_angle>-3.14159</min_angle><max_angle>3.14159</max_angle>
      </horizontal></scan>
      <range><min>0.15</min><max>12.0</max><resolution>0.01</resolution></range></ray>
      <plugin name="lidar_ros" filename="libgazebo_ros_ray_sensor.so">
        <ros><namespace>/</namespace><remapping>~/out:=scan</remapping></ros>
        <output_type>sensor_msgs/LaserScan</output_type>
        <frame_name>lidar_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>
'''
data = open(dst).read()
open(dst,'w').write(data.replace('</robot>', gz+'</robot>'))
print("OK ->", dst)
PY

echo "== 4) Sacando unitree_ros de src (es enorme y rompe colcon) =="
mv unitree_ros "$HOME/Downloads/unitree_ros_backup" 2>/dev/null || rm -rf unitree_ros

echo
echo "LISTO. Ahora en el escritorio del contenedor (vnc_lite):"
echo "  cd ~/g1_ws && colcon build && source install/setup.bash && ros2 launch g1_sim sim.launch.py"
