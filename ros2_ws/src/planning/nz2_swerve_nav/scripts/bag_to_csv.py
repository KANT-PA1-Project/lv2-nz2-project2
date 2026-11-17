#!/usr/bin/env python3
# nz2_swerve_nav/scripts/bag_to_csv.py
# 실행 위치: 개발 PC / Jetson
# 역할: Nav2 주행 bag (mcap/sqlite) → 20 Hz CSV 한 장 (Claude 분석용, /joint_states 1 kHz 등으로 큰 bag 을 줄임)
#
#   ros2 run nz2_swerve_nav bag_to_csv.py bags/nav_facing_2              (→ bags/nav_facing_2.csv)
#   ros2 run nz2_swerve_nav bag_to_csv.py bags/nav_facing_2 --rate 50 --out /tmp/x.csv
#
# 열: t(s, 첫 메시지 기준)
#     cmd_vx cmd_vy cmd_wz                 /cmd_vel (Nav2, base_footprint)
#     r12_vx r12_vy r12_wz                 /r12/cmd_vel (어댑터 출력, base_link)
#     gate_err gate_scale                  /cmd_frame_adapter/gate
#     cs_FL cs_FR cs_RL cs_RR              r12 조향 명령 [rad]   (/r12/swerve_joint_commands)
#     cw_FL cw_FR cw_RL cw_RR              r12 바퀴 명령 [rad/s]
#     ms_FL ms_FR ms_RL ms_RR              측정 조향 [rad]       (/joint_states)
#     mw_FL mw_FR mw_RL mw_RR              측정 바퀴 [rad/s]
#     odom_x odom_y odom_yaw odom_vx odom_vy odom_wz
#     gt_x gt_y gt_yaw gt_vx gt_vy gt_wz   /model/robot/ground_truth (시뮬)
# 각 행 = 그 시각까지 받은 각 토픽의 마지막 값 (없으면 빈칸)
import argparse
import csv
import math
import os
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

MOD = ['front_left', 'front_right', 'rear_left', 'rear_right']
SH = ['FL', 'FR', 'RL', 'RR']


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--rate', type=float, default=20.0)
    ap.add_argument('--out', default='')
    a = ap.parse_args()
    bag = os.path.abspath(os.path.expanduser(a.bag.rstrip('/')))
    out = a.out or bag + '.csv'

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id=''),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    msg_cls = {n: get_message(t) for n, t in types.items()}
    print(f'[bag_to_csv] {bag}\n  토픽: {sorted(types)}')

    cols = (['t', 'cmd_vx', 'cmd_vy', 'cmd_wz', 'r12_vx', 'r12_vy', 'r12_wz', 'gate_err', 'gate_scale']
            + [f'cs_{s}' for s in SH] + [f'cw_{s}' for s in SH]
            + [f'ms_{s}' for s in SH] + [f'mw_{s}' for s in SH]
            + ['odom_x', 'odom_y', 'odom_yaw', 'odom_vx', 'odom_vy', 'odom_wz',
               'gt_x', 'gt_y', 'gt_yaw', 'gt_vx', 'gt_vy', 'gt_wz'])
    cur = {c: '' for c in cols}
    period = 1.0 / a.rate
    t0 = None
    next_t = None
    n_rows = 0

    def odo(prefix, m):
        p, tw = m.pose.pose, m.twist.twist
        cur[prefix + 'x'], cur[prefix + 'y'], cur[prefix + 'yaw'] = p.position.x, p.position.y, yaw_of(p.orientation)
        cur[prefix + 'vx'], cur[prefix + 'vy'], cur[prefix + 'wz'] = tw.linear.x, tw.linear.y, tw.angular.z

    def joints(m, pos_prefix, vel_prefix):
        idx = {n: i for i, n in enumerate(m.name)}
        for mod, s in zip(MOD, SH):
            sj, wj = mod + '_steer_joint', mod + '_wheel_joint'
            if sj in idx and len(m.position) > idx[sj]:
                cur[pos_prefix + s] = m.position[idx[sj]]
            if wj in idx and len(m.velocity) > idx[wj]:
                cur[vel_prefix + s] = m.velocity[idx[wj]]

    with open(out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(cols)
        while reader.has_next():
            topic, data, ts = reader.read_next()
            t = ts * 1e-9
            if t0 is None:
                t0, next_t = t, t
            while t >= next_t:                      # 이 메시지 전 상태로 행 출력
                cur['t'] = round(next_t - t0, 3)
                w.writerow([cur[c] if not isinstance(cur[c], float) else round(cur[c], 5) for c in cols])
                n_rows += 1
                next_t += period
            if topic not in msg_cls:
                continue
            m = deserialize_message(data, msg_cls[topic])
            if topic == '/cmd_vel':
                cur['cmd_vx'], cur['cmd_vy'], cur['cmd_wz'] = m.twist.linear.x, m.twist.linear.y, m.twist.angular.z
            elif topic == '/r12/cmd_vel':
                cur['r12_vx'], cur['r12_vy'], cur['r12_wz'] = m.twist.linear.x, m.twist.linear.y, m.twist.angular.z
            elif topic == '/cmd_frame_adapter/gate' and len(m.data) >= 2:
                cur['gate_err'], cur['gate_scale'] = m.data[0], m.data[1]
            elif topic == '/r12/swerve_joint_commands':
                joints(m, 'cs_', 'cw_')
            elif topic == '/joint_states':
                joints(m, 'ms_', 'mw_')
            elif topic == '/odom':
                odo('odom_', m)
            elif topic == '/model/robot/ground_truth':
                odo('gt_', m)
    print(f'[bag_to_csv] {n_rows} 행 ({a.rate:.0f} Hz) → {out} ({os.path.getsize(out) / 1e6:.1f} MB)')


if __name__ == '__main__':
    sys.exit(main())
