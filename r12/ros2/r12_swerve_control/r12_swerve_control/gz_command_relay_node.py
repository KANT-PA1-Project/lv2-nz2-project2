# =============================================================
# 패키지 : r12_swerve_control
# 파일   : gz_command_relay_node.py
# 역할   : OpenCR 브리지의 가제보 버전. /r12/swerve_joint_commands(JointState) →
#          gz_ros2_control 컨트롤러 명령(Float64MultiArray).
#
# [입력]  /r12/swerve_joint_commands (sensor_msgs/JointState): 조향 position, 휠 velocity.
# [출력]  /steer_position_controller/commands, /wheel_velocity_controller/commands
#         (std_msgs/Float64MultiArray) — 컨트롤러 yaml 의 조인트 순서(FL FR RL RR)대로.
# [주의]  실물 경로가 아니라 시뮬 전용. 실물은 opencr_bridge_node 를 쓴다.
# =============================================================
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

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
        self.sp = self.create_publisher(Float64MultiArray, '/steer_position_controller/commands', 10)
        self.wp = self.create_publisher(Float64MultiArray, '/wheel_velocity_controller/commands', 10)
        self.create_timer(1.0 / self.hz, self._tick)

    def _on_cmd(self, msg):
        for i, name in enumerate(msg.name):
            if name in STEER and i < len(msg.position):
                self.steer[STEER[name]] = msg.position[i]
            if name in WHEEL and i < len(msg.velocity):
                self.wheel[WHEEL[name]] = msg.velocity[i]

    def _tick(self):
        if self.steer:
            self.sp.publish(Float64MultiArray(data=[self.steer.get(p, 0.0) for p in ORDER]))
        if self.wheel:
            self.wp.publish(Float64MultiArray(data=[self.wheel.get(p, 0.0) for p in ORDER]))


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
