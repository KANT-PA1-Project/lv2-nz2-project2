# =============================================================
# 패키지 : r12_swerve_control
# 파일   : gz_command_relay_node.py
# 역할   : OpenCR 브리지의 가제보(R25 rev1) 버전.
#          /r12/swerve_joint_commands(JointState) → 관절별 Float64 (ros_gz_bridge 로 gz 로 전달).
#
# [입력]  /r12/swerve_joint_commands (sensor_msgs/JointState): 조향 position(rad), 휠 velocity(rad/s).
# [출력]  /swerve/sim/<joint>_steer_joint/cmd_pos  (std_msgs/Float64, rad)   조향 4개
#         /swerve/sim/<joint>_wheel_joint/cmd_vel  (std_msgs/Float64, rad/s) 휠 4개
#         토픽명은 r25_rev1.urdf 의 gz JointPositionController/JointController 토픽과 같다.
#         ros_gz_bridge 가 이 ROS Float64 를 gz.msgs.Double 로 넘긴다.
# [주의]  시뮬(R25 rev1) 전용. 실물은 opencr_bridge_node. r25_rev1.urdf 가 제어 플러그인을 내장하므로
#         ros2_control·controllers.yaml 은 쓰지 않는다.
# =============================================================
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

ORDER = ['front_left', 'front_right', 'rear_left', 'rear_right']   # FL FR RL RR
STEER = {p + '_steer_joint': p for p in ORDER}
WHEEL = {p + '_wheel_joint': p for p in ORDER}


class GzCommandRelay(Node):
    def __init__(self):
        super().__init__('gz_command_relay')
        self.hz = self.declare_parameter('relay_hz', 50.0).value
        self.steer = {}
        self.wheel = {}
        self.create_subscription(JointState, '/r12/swerve_joint_commands', self._on_cmd, 10)
        # rev1 의 관절별 gz 토픽과 동일 이름(ros_gz_bridge 가 같은 이름으로 gz 에 전달)
        self.pos_pub = {p: self.create_publisher(Float64, f'/swerve/sim/{p}_steer_joint/cmd_pos', 10)
                        for p in ORDER}
        self.vel_pub = {p: self.create_publisher(Float64, f'/swerve/sim/{p}_wheel_joint/cmd_vel', 10)
                        for p in ORDER}
        self.create_timer(1.0 / self.hz, self._tick)

    def _on_cmd(self, msg):
        for i, name in enumerate(msg.name):
            if name in STEER and i < len(msg.position):
                self.steer[STEER[name]] = msg.position[i]
            if name in WHEEL and i < len(msg.velocity):
                self.wheel[WHEEL[name]] = msg.velocity[i]

    def _tick(self):
        for p in ORDER:
            if p in self.steer:
                self.pos_pub[p].publish(Float64(data=float(self.steer[p])))
            if p in self.wheel:
                self.vel_pub[p].publish(Float64(data=float(self.wheel[p])))


def main(args=None):
    rclpy.init(args=args)
    node = GzCommandRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
