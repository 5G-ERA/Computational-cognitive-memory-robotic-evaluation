#!/usr/bin/env bash
# Regenera g1_description/urdf/g1_nav.urdf correctamente (sin el bug de truncado).
# Ejecutar en g1_ws/src (en el Mac o en el contenedor).
set -euo pipefail
SRC=g1_description/g1_23dof.urdf
DST=g1_description/urdf/g1_nav.urdf
[ -f "$SRC" ] || { echo "ERROR: no existe $SRC (ejecuta en g1_ws/src)"; exit 1; }
mkdir -p g1_description/urdf
python3 - "$SRC" "$DST" <<'PY'
import sys, xml.etree.ElementTree as ET
src, dst = sys.argv[1], sys.argv[2]
t = ET.parse(src); r = t.getroot()
for tag in ('gazebo','ros2_control','transmission'):
    for e in list(r.findall(tag)): r.remove(e)
for l in list(r.findall('link')):
    if l.get('name') == 'world': r.remove(l)
for j in list(r.findall('joint')):
    p = j.find('parent')
    if p is not None and p.get('link') == 'world': r.remove(j)
for j in r.findall('joint'):
    if j.get('type') in ('revolute','continuous','prismatic','floating','planar'):
        j.set('type','fixed')
        for tg in ('axis','limit','dynamics','mimic','safety_controller'):
            e = j.find(tg)
            if e is not None: j.remove(e)
# Remove collision and inertial from all original G1 links
# (collision: no underground physics; inertial: no tipping torque when planar_move applies velocity)
for l in r.findall('link'):
    for col in list(l.findall('collision')): l.remove(col)
    for ine in list(l.findall('inertial')): l.remove(ine)
for m in r.iter('mesh'):
    fn = m.get('filename') or ''
    if fn.startswith('meshes/'): m.set('filename', 'package://g1_description/'+fn)
def inertial(parent, mass, i):
    ine = ET.SubElement(parent,'inertial')
    ET.SubElement(ine,'origin',{'xyz':'0 0 0','rpy':'0 0 0'})
    ET.SubElement(ine,'mass',{'value':str(mass)})
    ET.SubElement(ine,'inertia',{'ixx':i,'iyy':i,'izz':i,'ixy':'0','ixz':'0','iyz':'0'})
# base_footprint: gravity disabled via gazebo tag below, no collision needed
bf = ET.SubElement(r,'link',{'name':'base_footprint'})
ine = ET.SubElement(bf,'inertial')
ET.SubElement(ine,'origin',{'xyz':'0 0 0','rpy':'0 0 0'})
ET.SubElement(ine,'mass',{'value':'35.0'})
ET.SubElement(ine,'inertia',{'ixx':'1','iyy':'1','izz':'0.5','ixy':'0','ixz':'0','iyz':'0'})
bj = ET.SubElement(r,'joint',{'name':'base_joint','type':'fixed'})
ET.SubElement(bj,'parent',{'link':'base_footprint'}); ET.SubElement(bj,'child',{'link':'pelvis'})
ET.SubElement(bj,'origin',{'xyz':'0 0 0.793','rpy':'0 0 0'})  # pelvis ~0.793m above ground when standing
ll = ET.SubElement(r,'link',{'name':'lidar_link'}); inertial(ll,0.05,'0.0001')
lj = ET.SubElement(r,'joint',{'name':'lidar_joint','type':'fixed'})
ET.SubElement(lj,'parent',{'link':'torso_link'}); ET.SubElement(lj,'child',{'link':'lidar_link'})
ET.SubElement(lj,'origin',{'xyz':'0.0002835 0.00003 0.40618','rpy':'0 0 0'})   # = posicion real del Mid-360, nivelado para scan 2D
t.write(dst, xml_declaration=False, encoding='utf-8')
gz = '''
  <gazebo reference="base_footprint">
    <gravity>false</gravity>
  </gazebo>
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
data = open(dst).read()                       # <-- leer ANTES
open(dst,'w').write(data.replace('</robot>', gz+'</robot>'))   # <-- luego escribir
n = len(open(dst).read())
print("OK ->", dst, "(", n, "bytes )")
PY
echo "Hecho. Reconstruye:  cd ~/g1_ws && colcon build && source install/setup.bash"
