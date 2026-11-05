#!/usr/bin/env python3
"""Explicit CAD fingertip-midpoint TCP variant; not a physical calibration."""
from pathlib import Path
import json, xml.etree.ElementTree as ET
root=Path(__file__).resolve().parents[1]
tree=ET.parse(root/'models/r12/r12_arm.urdf'); robot=tree.getroot()
ET.SubElement(robot,'link',name='grasp_tcp')
j=ET.SubElement(robot,'joint',name='grasp_tcp_fixed',type='fixed')
ET.SubElement(j,'parent',link='arm_link5');ET.SubElement(j,'child',link='grasp_tcp')
ET.SubElement(j,'origin',xyz='0.12475 0 0',rpy='0 0 0')
ET.indent(tree);tree.write(root/'models/r12/r12_tcp.urdf',encoding='utf-8',xml_declaration=True)
(root/'models/r12/r12_tcp.usda').write_text('''#usda 1.0
(defaultPrim = "Robot"\n metersPerUnit = 1\n upAxis = "Z")
def Xform "Robot" (prepend references = @r12_arm.usda@) {
 over "arm_link5" {
  def Xform "grasp_tcp" {
   double3 xformOp:translate = (0.12475, 0, 0)
   uniform token[] xformOpOrder = ["xformOp:translate"]
  }
 }
}
''')
(root/'configs/r12_tcp_calibration.json').write_text(json.dumps({'parent':'arm_link5','frame':'grasp_tcp','translation_m':[.12475,0,0],'rpy_rad':[0,0,0],'method':'CAD opposing fingertip midpoint; +X approach, Y opening, Z up at home','physically_calibrated':False,'gripper_actuated':False,'policy_compatible':False},indent=2))
