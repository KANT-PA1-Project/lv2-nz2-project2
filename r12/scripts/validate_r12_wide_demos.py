#!/usr/bin/env python3
"""Home-connected large-angle examples for the simulation exploration profile."""
import math
import validate_r12_paths as v
v.scene();home=[0.]*5
angles=[(0,-150),(0,150),(1,-60),(1,20),(2,-40),(2,60),(3,-120),(3,100),(4,-180),(4,180)]
for i,(joint,deg) in enumerate(angles):
    q=home.copy();q[joint]=math.radians(deg)
    v.check('wide_demo',i,home,q)
v.node.destroy_node();v.rclpy.shutdown()
