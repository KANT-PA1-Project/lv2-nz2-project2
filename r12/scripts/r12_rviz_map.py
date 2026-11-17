"""RViz occupancy-cell markers for systems with indexed-map shader errors."""
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy._rclpy_pybind11 import RCLError
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import math
rclpy.init();node=Node('r12_rviz_map');qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
pubs={kind:node.create_publisher(Marker,'/r12/rviz/'+kind,qos) for kind in ['map','costmap']}
def display(m,kind):
 marker=Marker();marker.header=m.header;marker.ns=kind;marker.id=0;marker.type=Marker.CUBE_LIST;marker.action=Marker.ADD;marker.pose=m.info.origin;marker.scale.x=m.info.resolution;marker.scale.y=m.info.resolution;marker.scale.z=.008;marker.color.a=1.
 for i,v in enumerate(m.data):
  if v<50:continue
  marker.points.append(Point(x=(i%m.info.width+.5)*m.info.resolution,y=(i//m.info.width+.5)*m.info.resolution,z=.01 if kind=='map' else .025))
  marker.colors.append(ColorRGBA(r=.65 if kind=='map' else 1.,g=.72 if kind=='map' else .2,b=.80 if kind=='map' else .1,a=.9 if kind=='map' else .35))
 pubs[kind].publish(marker)
node.create_subscription(OccupancyGrid,'/map',lambda m:display(m,'map'),qos)
node.create_subscription(OccupancyGrid,'/global_costmap/costmap',lambda m:display(m,'costmap'),10)
try:
 rclpy.spin(node)
except ExternalShutdownException:
 # ROS signal handling has already shut down the context. Other failures,
 # including an unexpected exception while the context is live, propagate.
 if rclpy.ok():raise
except RCLError as error:
 # A signal may invalidate the context between the spin check and WaitSet
 # construction. Only that specific shutdown race is expected here.
 if rclpy.ok() or 'failed to initialize wait set: the given context is not valid' not in str(error):raise
finally:
 node.destroy_node()
 rclpy.try_shutdown()
