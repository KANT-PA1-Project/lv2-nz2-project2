# 선반 pose (NZ2)
# 토픽: /perception/shelf_pose  (shelf_detector → perception_master)
std_msgs/Header header    # frame_id: map
int32 shelf_id            # 선반에 붙은 AprilTag ID (-1 = 미확인)
geometry_msgs/Pose pose   # 선반 앞 모서리 중앙, x축이 선반 바깥(로봇 쪽)을 향함
float32 top_height        # 선반 윗면 높이 (m), 기준 0.080
float32 confidence        # 0.0 ~ 1.0
