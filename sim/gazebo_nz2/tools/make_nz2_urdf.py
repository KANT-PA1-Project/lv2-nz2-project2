"""assembly.urdf -> assembly_nz2.urdf  /  v99_nz2fix.urdf -> assembly_nz2_v99.urdf (11/14)
0) 입력에 base_footprint 가 이미 있으면(v99_nz2fix) 1) 건너뜀
1) base_footprint 추가 (정면 = 팔 쪽 = base_link -X)
2) 질량 총 4.296 kg 로 비례 축소 (관성도 같은 비율)
3) 조향 한계 ±1.92 rad (110°) [가정, 실물 케이블 확인 필요]
4) Gazebo: 조향 위치 제어, 바퀴 속도 제어, 팔·집게 고정, joint_states 발행, 바퀴 마찰
5) (11/13) 바퀴 joint velocity 10 → 32 rad/s (XM430-W210 77 rpm × 증속 4 = 1.06 m/s, 하드웨어 한계)
   조향 위치 제어 P 10 → 50, cmd_max 6 → 7.9 rad/s (펌웨어 Profile Velocity 330 = 75.6 rpm)
   이유: r12 조향 슬루 4 rad/s 를 P 10 으로 따라가면 추종 지연 4/10 = 0.4 rad(23°) → 50 이면 0.08 rad(4.6°)
"""
import sys, xml.etree.ElementTree as ET
src, dst = sys.argv[1], sys.argv[2]
TOTAL_MASS = 4.296
STEER_LIMIT = 1.92
WHEEL_VEL_LIMIT = 32.0   # [rad/s] 바퀴 축
STEER_P = 50.0
STEER_CMD_MAX = 7.9      # [rad/s]
tree = ET.parse(src); r = tree.getroot()

# 2) 질량
links = r.findall('link')
m0 = sum(float(l.find('inertial/mass').get('value')) for l in links if l.find('inertial') is not None)
k = TOTAL_MASS / m0
for l in links:
    i = l.find('inertial')
    if i is None: continue
    m = i.find('mass'); m.set('value', repr(float(m.get('value')) * k))
    I = i.find('inertia')
    for a in ('ixx','ixy','ixz','iyy','iyz','izz'):
        I.set(a, repr(float(I.get(a)) * k))

# 3) 조향 한계
steer = [j.get('name') for j in r.findall('joint') if j.get('name').endswith('_steer_joint')]
wheel = [j.get('name') for j in r.findall('joint') if j.get('name').endswith('_wheel_joint')]
for j in r.findall('joint'):
    if j.get('name') in steer:
        lim = j.find('limit'); lim.set('lower', str(-STEER_LIMIT)); lim.set('upper', str(STEER_LIMIT))
    if j.get('name') in wheel:
        j.find('limit').set('velocity', str(WHEEL_VEL_LIMIT))

# 1) base_footprint (루트) — 이미 있으면(v99_nz2fix) 값만 확인
if any(l.get('name') == 'base_footprint' for l in r.findall('link')):
    j = [j for j in r.findall('joint') if j.find('child').get('link') == 'base_link'][0]
    o = j.find('origin')
    print(f"base_footprint 이미 있음: {j.get('name')} xyz={o.get('xyz')} rpy={o.get('rpy')}")
    assert j.find('parent').get('link') == 'base_footprint', 'base_link 의 부모가 base_footprint 가 아님'
else:
    fp = ET.Element('link', name='base_footprint')
    r.insert(1, fp)
    fj = ET.Element('joint', name='base_footprint_joint', type='fixed')
    ET.SubElement(fj, 'origin', xyz='0 0 0.181', rpy='0 0 3.14159265359')
    ET.SubElement(fj, 'parent', link='base_footprint'); ET.SubElement(fj, 'child', link='base_link')
    r.insert(2, fj)

# 4) Gazebo 플러그인
def plugin(fname, name, **kv):
    g = ET.SubElement(r, 'gazebo'); p = ET.SubElement(g, 'plugin', filename=fname, name=name)
    for key, val in kv.items(): ET.SubElement(p, key).text = str(val)
plugin('gz-sim-joint-state-publisher-system', 'gz::sim::systems::JointStatePublisher', topic='/joint_states')
for j in steer:
    plugin('gz-sim-joint-position-controller-system', 'gz::sim::systems::JointPositionController',
           joint_name=j, topic=f'/swerve/sim/{j}/cmd_pos', p_gain=STEER_P, i_gain=0.0, d_gain=0.0,
           cmd_max=STEER_CMD_MAX, cmd_min=-STEER_CMD_MAX, use_velocity_commands='true')
for j in wheel:
    plugin('gz-sim-joint-controller-system', 'gz::sim::systems::JointController',
           joint_name=j, topic=f'/swerve/sim/{j}/cmd_vel', initial_velocity=0.0)
for j in ['arm_yaw_joint','shoulder_joint','elbow_joint','wrist_pitch_joint','wrist_roll_joint','jaw_left_joint','jaw_right_joint']:
    plugin('gz-sim-joint-position-controller-system', 'gz::sim::systems::JointPositionController',
           joint_name=j, topic=f'/arm/sim/{j}/cmd_pos', p_gain=10.0, i_gain=0.0, d_gain=0.0,
           cmd_max=2.0, cmd_min=-2.0, use_velocity_commands='true')
for j in wheel:
    g = ET.SubElement(r, 'gazebo', reference=j.replace('_joint',''))
    ET.SubElement(g, 'mu1').text = '1.0'; ET.SubElement(g, 'mu2').text = '1.0'

ET.indent(tree, space='  ')
tree.write(dst, encoding='utf-8', xml_declaration=True)
print(f'mass {m0:.4f} -> {TOTAL_MASS} (x{k:.6f}); steer {steer}; wheel {wheel}')
