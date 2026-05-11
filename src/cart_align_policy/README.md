# cart_align_policy

IsaacLab에서 export한 ONNX 정책을 ROS2 노드로 실행하여,
메시지(`geometry_msgs/msg/PoseStamped`, `cartrider_rmd_sdk/msg/MotorStateArray`)를 입력으로 받아
바퀴 속도 명령을 출력합니다.

## 패키지 구성

- `cart_align_policy`
  - `policy_node` (ONNX 추론 노드)
  - `launch/policy.launch.py`

## 토픽 및 메시지

### 1) Target 입력 (Nav -> Policy)
- Topic: `/align/target_local` (front/rear 공통)
- Type: `geometry_msgs/msg/PoseStamped`
- 매핑:
  - `pose.position.x` = `target_x_local`
  - `pose.position.y` = `target_y_local`
  - `pose.orientation`(quaternion)에서 yaw 추출 = `heading_error` (wrap_to_pi)

### 2) 현재 모터 속도 입력 -> Policy
- Topic (robot_type 기준):
  - front(default): `/front/rmd_state`
  - rear: `/rmd_state`
- Type: `cartrider_rmd_sdk/msg/MotorStateArray` (기본값)
- 매핑:
  - `states` 배열에서 `id=1`의 `speed` = `left_wheel_joint_vel` (rad/s)
  - `states` 배열에서 `id=2`의 `speed` = `right_wheel_joint_vel` (rad/s)

### 3) 바퀴 속도 출력 (Policy -> Nav)
- Topic (robot_type 기준):
  - front(default): `/front/rmd_command`
  - rear: `/rmd_command`
- Type: `cartrider_rmd_sdk/msg/MotorCommandArray`
- 매핑:
  - `commands` 배열에 2개 명령 publish
  - `id=1` -> left motor, `target=cmd_vel_l` (rad/s)
  - `id=2` -> right motor, `target=cmd_vel_r` (rad/s)

## policy_node 동작

- 제어 주기(`control_rate_hz`, default 40Hz)마다 최신 데이터로 obs(1x5) 구성
- obs 순서(고정):
  1. `target.pose.position.x`
  2. `target.pose.position.y`
  3. `target.pose.orientation`에서 추출한 yaw
  4. `motor_state(id=1).speed` (left motor vel)
  5. `motor_state(id=2).speed` (right motor vel)
- ONNX 출력 action(2D) -> `[-1, 1]` clamp -> `action_scale` 곱하여 rad/s 명령 생성
- yaw는 deadzone 없이 원본값(rad)을 그대로 policy 입력으로 사용
- `target_yaw_stop_tolerance_deg`는 정렬 완료(정지) 판정 조건에서만 사용
- 타겟 거리 `sqrt(x^2+y^2)`가 `near_target_distance_m` 이하면
  `near_target_speed_limit_rad_s`로 명령 제한
- `|target_x_local| <= 0.05`m, `|target_y_local| <= 0.05`m, `|heading_error| <= 5deg`이면 양쪽 모터 0 명령 publish
- `target` 또는 `motor` 메시지가 stale(timeout)면 안전하게 0 명령 publish
- ONNX Runtime은 CPU provider만 사용

## 파라미터

- `robot_type` (default: `front`, choices: `rear`, `front`)
- `model_path` (default: `__auto__`)
- `action_scale` (default: `__auto__`)
- `near_target_speed_limit_rad_s` (default: `__auto__`)
- `near_target_distance_m` (default: `__auto__`)
- `target_topic` (default: `/align/target_local`)
- `motor_state_topic` (default: `__auto__`)
- `motor_state_type` (default: `cartrider_rmd_sdk/msg/MotorStateArray`)
- `wheel_cmd_topic` (default: `__auto__`)
- `wheel_cmd_type` (default: `cartrider_rmd_sdk/msg/MotorCommandArray`)
- `wheel_cmd_item_type` (default: `cartrider_rmd_sdk/msg/MotorCommand`)
- `left_motor_id` (default: `1`)
- `right_motor_id` (default: `2`)
- `control_rate_hz` (default: `40.0`)
- `target_timeout_sec` (default: `1000.0`)
- `motor_timeout_sec` (default: `1000.0`)
- `target_xy_stop_tolerance_m` (default: `0.05`)
- `target_yaw_stop_tolerance_deg` (default: `5.0`, 정렬 완료 판정 tolerance)
- `invert_left` (default: `__auto__`)
- `invert_right` (default: `__auto__`)

`__auto__`일 때 robot_type별 기본값:
- front: `model=front_policy.onnx`, `action_scale=3.5`, `near_target_speed_limit_rad_s=0.9`,
  `near_target_distance_m=0.5`, `motor_state_topic=/front/rmd_state`,
  `wheel_cmd_topic=/front/rmd_command`, `invert_left=false`, `invert_right=true`
- rear: `model=policy.onnx`, `action_scale=2.0`, `near_target_speed_limit_rad_s=0.5`,
  `near_target_distance_m=0.5`, `motor_state_topic=/rmd_state`,
  `wheel_cmd_topic=/rmd_command`, `invert_left=true`, `invert_right=false`

## 빌드

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select cart_align_policy
source install/setup.bash
```

`motor_state_type`, `wheel_cmd_type` 기본값을 사용할 경우
`cartrider_rmd_sdk` 패키지가 같은 ROS 환경에 설치되어 있어야 합니다.

`onnxruntime`가 설치되지 않은 환경이라면 아래 중 하나로 설치하세요.

```bash
# 권장(ROS 환경)
sudo apt-get update && sudo apt-get install -y python3-onnxruntime

# 또는
sudo apt-get install -y python3-pip
python3 -m pip install --user onnxruntime
```

## 실행

### policy 노드 실행

```bash
ros2 launch cart_align_policy policy.launch.py
```

리어봇으로 실행:

```bash
ros2 launch cart_align_policy policy.launch.py robot_type:=rear
```

예시: front 프로필에서 action_scale만 수동 override

```bash
ros2 launch cart_align_policy policy.launch.py \
  robot_type:=front \
  action_scale:=4.0
```

외부 모터 노드의 타입이 기본값과 다를 때만 변경하세요.

```bash
ros2 launch cart_align_policy policy.launch.py \
  motor_state_type:=my_motor_msgs/msg/MotorStateArray
```

### 출력 확인

```bash
ros2 topic echo /front/rmd_command
```
