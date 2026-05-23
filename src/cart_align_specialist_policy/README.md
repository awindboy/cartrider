# cart_align_specialist_policy

IsaacLab에서 export한 specialist ONNX 정책을 ROS2 노드로 실행하여,
비전 기반 타겟 정보(`geometry_msgs/msg/Pose2D`)와 현재 로봇 속도 정보를 입력으로 받아
로봇 중심 기준 `cmd_vel` 명령을 출력합니다.

## 패키지 구성

- `cart_align_specialist_policy`
  - `specialist_policy_node` (ONNX 추론 노드)
  - `launch/specialist_policy.launch.py`

## 토픽 및 메시지

### 1) Target 입력 (Vision -> Policy)
- Topic: `/rs/cart_pose` (front/rear 공통, 기본값)
- Type: `geometry_msgs/msg/Pose2D`
- 매핑:
  - `x` = `target_x_local`
  - `y` = `target_y_local`
  - `theta` = 비전 yaw error
  - policy 입력에는 `heading_error = wrap_to_pi(-theta)` 를 사용

### 2) 현재 로봇 속도 입력 -> Policy
- Topic (robot_type 기준):
  - front(default): `/front/rmd_state`
  - rear: `/rmd_state`
- Type: `cartrider_rmd_sdk/msg/MotorStateArray` (기본값)
- 매핑:
  - `states` 배열에서 `left_motor_id`, `right_motor_id`에 해당하는 `speed`를 사용
  - 필요 시 `state_invert_left`, `state_invert_right`로 부호 보정
  - 모터 각속도(rad/s)를 `external_reduction`으로 나누어 바퀴 각속도로 변환
  - 각 바퀴 각속도(rad/s)를 `wheel_radius_m`, `wheel_separation_m`으로 변환
  - `linear_velocity_m_s = (v_left + v_right) / 2`
  - `angular_velocity_rad_s = (v_right - v_left) / wheel_separation_m`

### 3) 로봇 속도 출력 (Policy -> Nav)
- Topic (robot_type 기준):
  - front(default): `/front/cmd_vel`
  - rear: `/cmd_vel`
- Type: `geometry_msgs/msg/Twist`
- 매핑:
  - `linear.x` = `cmd_linear_velocity_m_s`
  - `angular.z` = `cmd_angular_velocity_rad_s`

## specialist_policy_node 동작

- 제어 주기(`control_rate_hz`, default 40Hz)마다 최신 데이터로 obs(1x5) 구성
- obs 순서(고정):
  1. `target.x`
  2. `target.y`
  3. `wrap_to_pi(-target.theta)`
  4. 현재 로봇 선속도 `linear_velocity_m_s`
  5. 현재 로봇 각속도 `angular_velocity_rad_s`
- ONNX 출력 action(2D) -> `[-1, 1]` clamp -> 아래 scale로 실제 `cmd_vel` 생성
  - `linear_velocity_scale_m_s`
  - `angular_velocity_scale_rad_s`
- `target_yaw_stop_tolerance_deg`는 정렬 완료(정지) 판정 조건에서만 사용
- 타겟 거리 `sqrt(x^2+y^2)`가 `near_target_distance_m` 이하면
  `near_target_linear_speed_limit_m_s`, `near_target_angular_speed_limit_rad_s`로 명령 제한
- 출력 twist가 좌우 바퀴를 서로 반대 방향으로 돌리는 궤적일 때
  `spin_in_place_angular_limit_rad_s`로 `angular.z` 제한
- `|target_x_local| <= 0.05`m, `|target_y_local| <= 0.05`m, `|heading_error| <= 5deg`이면 0 `cmd_vel` publish
- `target` 또는 `motor` 메시지가 stale(timeout)면 안전하게 0 `cmd_vel` publish
- ONNX Runtime은 CPU provider만 사용

## 파라미터

- `robot_type` (default: `front`, choices: `rear`, `front`)
- `model_path` (default: `__auto__`)
- `linear_velocity_scale_m_s` (default: `__auto__`)
- `angular_velocity_scale_rad_s` (default: `__auto__`)
- `spin_in_place_angular_limit_rad_s` (default: `__auto__`)
- `near_target_linear_speed_limit_m_s` (default: `__auto__`)
- `near_target_angular_speed_limit_rad_s` (default: `__auto__`)
- `near_target_distance_m` (default: `__auto__`)
- `target_topic` (default: `/rs/cart_pose`)
- `motor_state_topic` (default: `__auto__`)
- `motor_state_type` (default: `cartrider_rmd_sdk/msg/MotorStateArray`)
- `cmd_vel_topic` (default: `__auto__`)
- `wheel_radius_m` (default: `__auto__`, 현재는 직접 지정 필요)
- `wheel_separation_m` (default: `__auto__`)
- `external_reduction` (default: `__auto__`)
- `state_invert_left` (default: `__auto__`)
- `state_invert_right` (default: `__auto__`)
- `left_motor_id` (default: `1`)
- `right_motor_id` (default: `2`)
- `control_rate_hz` (default: `40.0`)
- `target_timeout_sec` (default: `1000.0`)
- `motor_timeout_sec` (default: `1000.0`)
- `target_xy_stop_tolerance_m` (default: `0.05`)
- `target_yaw_stop_tolerance_deg` (default: `5.0`)

`__auto__`일 때 robot_type별 기본값:
- front: `model=front_specialist_policy.onnx`, `motor_state_topic=/front/rmd_state`,
  `cmd_vel_topic=/front/cmd_vel`, `state_invert_left=false`, `state_invert_right=true`,
  `near_target_distance_m=0.5`, `wheel_radius_m=0.0635`, `wheel_separation_m=0.2460`,
  `external_reduction=3.0`
- rear: `model=specialist_policy.onnx`, `motor_state_topic=/rmd_state`,
  `cmd_vel_topic=/cmd_vel`, `state_invert_left=true`, `state_invert_right=false`,
  `near_target_distance_m=0.5`, `wheel_radius_m=0.1100`, `wheel_separation_m=0.3000`,
  `external_reduction=1.0`

아래 항목은 기존 wheel-space 기본값을 wheel geometry로 twist-space에 자동 환산합니다.
- `linear_velocity_scale_m_s`
- `angular_velocity_scale_rad_s`
- `spin_in_place_angular_limit_rad_s`
- `near_target_linear_speed_limit_m_s`
- `near_target_angular_speed_limit_rad_s`

여기서 감속비(`external_reduction`)는 출력 쪽 자동 환산에는 사용하지 않습니다.
모터 노드가 `cmd_vel`을 만족하도록 감속비를 내부에서 처리한다고 가정하고,
감속비는 `/rmd_state`의 순수 모터 속도를 실제 바퀴 속도로 바꿀 때만 사용합니다.

자동 환산 기준이 되는 legacy 값:
- front: `action_scale=3.5`, `spin_in_place_limit_rad_s=0.0`, `near_target_speed_limit_rad_s=0.9`
- rear: `action_scale=2.0`, `spin_in_place_limit_rad_s=0.4`, `near_target_speed_limit_rad_s=0.5`

## 빌드

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select cart_align_specialist_policy
source install/setup.bash
```

`motor_state_type` 기본값을 사용할 경우
`cartrider_rmd_sdk` 패키지가 같은 ROS 환경에 설치되어 있어야 합니다.

`onnxruntime`가 설치되지 않은 환경이라면 아래 중 하나로 설치하세요.

```bash
# 권장(ROS 환경)
sudo apt-get update && sudo apt-get install -y python3-onnxruntime

# 또는
sudo apt-get install -y python3-pip
python3 -m pip install --user onnxruntime
```

## 모델 파일

기본 모델 경로는 아래처럼 가정합니다.
- front: `models/front_specialist_policy.onnx`
- rear: `models/specialist_policy.onnx`

파일명이 다르면 `model_path:=...` 로 override 하세요.

## 실행

```bash
ros2 launch cart_align_specialist_policy specialist_policy.launch.py \
  robot_type:=front
```

리어봇으로 실행:

```bash
ros2 launch cart_align_specialist_policy specialist_policy.launch.py \
  robot_type:=rear
```

예시: 모델 경로와 output topic만 override

```bash
ros2 launch cart_align_specialist_policy specialist_policy.launch.py \
  robot_type:=front \
  model_path:=/path/to/front_specialist_policy.onnx \
  cmd_vel_topic:=/front/cmd_vel
```
