#!/usr/bin/env python3
"""Generate mock-control URDF and joint limits from the prepared R12 arm."""
from pathlib import Path
import json
import xml.etree.ElementTree as ET

root = Path(__file__).resolve().parents[1]
config_dir = root/'ros2/r12_arm_moveit_config/config'
tree = ET.parse(root/'models/r12/r12_arm.urdf')
robot = tree.getroot()
for mesh in robot.iter('mesh'):
    mesh.set('filename', 'package://r12_description/meshes/'+Path(mesh.get('filename')).name)
control = ET.SubElement(robot, 'ros2_control', name='R12MockSystem', type='system')
hardware = ET.SubElement(control, 'hardware')
ET.SubElement(hardware, 'plugin').text = 'mock_components/GenericSystem'
cfg = json.loads((root/'configs/r12_robot.json').read_text())
limits = ['default_velocity_scaling_factor: 0.2', 'default_acceleration_scaling_factor: 0.2', 'joint_limits:']
for i, name in enumerate(cfg['joint_names']):
    joint = ET.SubElement(control, 'joint', name=name)
    command = ET.SubElement(joint, 'command_interface', name='position')
    ET.SubElement(command, 'param', name='min').text = str(cfg['lower'][i])
    ET.SubElement(command, 'param', name='max').text = str(cfg['upper'][i])
    state = ET.SubElement(joint, 'state_interface', name='position')
    ET.SubElement(state, 'param', name='initial_value').text = '0.0'
    ET.SubElement(joint, 'state_interface', name='velocity')
    limits += [f'  {name}:', '    has_velocity_limits: true', f'    max_velocity: {cfg["velocity"][i]}',
               '    has_acceleration_limits: true', '    max_acceleration: 0.5']
ET.indent(tree)
tree.write(config_dir/'r12.urdf', encoding='utf-8', xml_declaration=True)
(config_dir/'joint_limits.yaml').write_text('\n'.join(limits)+'\n')
print(config_dir)
