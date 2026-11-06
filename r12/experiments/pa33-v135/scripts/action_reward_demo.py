#!/usr/bin/env python3
from arm_practice.core import joint_targets, reach_score

print("target rad:", joint_targets([0.2, -0.5, 2.0], [0, 0, 0], [.5, .5, .5], [-1]*3, [1]*3))
for x in (0.1, 0.25, 0.3):
    print(reach_score([x, 0, .2], [.3, 0, .2], [0, 0, 0], [0, 0, 0]))
