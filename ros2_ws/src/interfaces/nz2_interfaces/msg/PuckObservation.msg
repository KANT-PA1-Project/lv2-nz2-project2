# 인지 노드 → 미션 FSM : 퍽 1개 관측 (상태 변경은 FSM만 한다)
builtin_interfaces/Time stamp
int32 puck_id
string color          # red / green / "" (미확정)
string shape          # cyl / cube / ""
float32 conf_color
float32 conf_shape
int32 box_id          # 관측한 시작 박스 번호, 모르면 -1
float64 x_mm          # 지도(경기장) 좌표
float64 y_mm
