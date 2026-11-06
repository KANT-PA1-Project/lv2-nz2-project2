"""URDF forward kinematics for sampling geometrically reachable position targets."""
import xml.etree.ElementTree as ET
import numpy as np


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    norm = np.linalg.norm(axis)
    if norm == 0:
        raise ValueError('Joint axis must be nonzero')
    x, y, z = axis / norm
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + np.sin(angle)*skew + (1-np.cos(angle))*(skew @ skew)


def forward_transform(urdf_path, tcp_link, positions):
    root = ET.parse(urdf_path).getroot()
    links = {x.get('name') for x in root.findall('link')}
    if tcp_link not in links:
        raise ValueError(f'Unknown TCP {tcp_link}')
    by_child = {j.find('child').get('link'): j for j in root.findall('joint')}
    chain, current = [], tcp_link
    while current in by_child:
        if current in {j.find('child').get('link') for j in chain}:
            raise ValueError('Cyclic chain')
        joint = by_child[current]
        chain.append(joint)
        current = joint.find('parent').get('link')
    transform = np.eye(4)
    for joint in reversed(chain):
        origin = joint.find('origin')
        xyz = np.fromstring(origin.get('xyz', '0 0 0') if origin is not None else '0 0 0', sep=' ')
        rpy = np.fromstring(origin.get('rpy', '0 0 0') if origin is not None else '0 0 0', sep=' ')
        local = np.eye(4)
        local[:3, 3] = xyz
        local[:3, :3] = rotation([0,0,1], rpy[2]) @ rotation([0,1,0], rpy[1]) @ rotation([1,0,0], rpy[0])
        motion = np.eye(4)
        if joint.get('type') != 'fixed':
            axis_node = joint.find('axis')
            axis = np.fromstring(axis_node.get('xyz') if axis_node is not None else '1 0 0', sep=' ')
            value = positions[joint.get('name')]
            if joint.get('type') == 'prismatic':
                motion[:3, 3] = axis / np.linalg.norm(axis) * value
            else:
                motion[:3, :3] = rotation(axis, value)
        transform = transform @ local @ motion
    return transform


def forward_position(urdf_path, tcp_link, positions):
    return forward_transform(urdf_path, tcp_link, positions)[:3, 3]
