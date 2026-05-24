# RL_special

IsaacLab에서 export한 specialist ONNX 정책을 ROS2 노드로 실행하여,
비전 기반 타겟 정보(`geometry_msgs/msg/Pose2D`)와 현재 로봇 속도 정보를 입력으로 받아
로봇 중심 기준 `cmd_vel` 명령을 출력합니다.

## 패키지 구성

- `RL_special`
  - `specialist_policy_node` (ONNX 추론 노드)
  - `launch/specialist_policy.launch.py`

## 토픽 및 메시지

### 1) Target 입력 (Vision -> Policy)
- Topic (robot_type 기준):
  - front(default): `/front/rs/cart_pose`
  - rear: `/rear/rs/cart_pose`
- Type: `geometry_msgs/msg/Pose2D`
- 매핑:
  - `x` = `raw_target_x_local` (`base_link` 중심 기준)
  - `y` = `raw_target_y_local` (`base_link` 중심 기준)
  - `theta` = 비전 yaw error
  - 먼저 `base_link_to_axle_center_x_m`와 robot별 부호를 적용해 구동축 중심 기준 `axle_target_x_local`, `axle_target_y_local` 를 계산
  - 그 다음 `target_x_offset_m`를 비전 `theta` 방향으로 투영해 최종 `target_x_local`, `target_y_local` 를 계산
  - 내부 상태와 오도메트리 fallback은 이 최종 `target_x_local`, `target_y_local` 기준으로 유지
  - policy 입력에는 `heading_error = wrap_to_pi(-theta)` 를 사용
  - front는 policy 입력에 넣기 직전에 `target_x_local`, `target_y_local`에 `-1`을 곱하고, rear는 그대로 사용
  - 비전 업데이트가 끊기면 policy를 계속 돌리지 않고 calibration mode로 전환

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

### 4) 그리퍼 토글 출력
- Topic (robot_type 기준):
  - front(default): `/front/cart_docking`
  - rear: `/gripper_toggle`
- Type: `std_msgs/msg/Bool`
- 매핑:
  - 정렬 완료 후 `final_forward_distance_m` 이동까지 끝나면 `data=true` 1회 publish
  - publish 후 노드 종료

## specialist_policy_node 동작

- 제어 주기(`control_rate_hz`, default 30Hz)마다 최신 데이터로 obs(1x5) 구성
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
- `base_link_to_axle_center_x_m`로 비전 좌표의 기준점을 `base_link` 중심에서 로봇 구동축 중심으로 이동
- 이때 rear는 `x_axle = x_base - base_link_to_axle_center_x_m`, front는 `x_axle = x_base + base_link_to_axle_center_x_m`
- `target_x_offset_m`로 비전 기준점과 실제 정렬 목표점 사이의 longitudinal 차이를 보정
- 비전이 잠시 끊기면 정책을 계속 돌리지 않고 calibration mode로 진입
- calibration mode는 마지막으로 기억된 최종 target state의 `y` 부호를 보고
  rear는 `y < 0`이면 좌회전 `calibration_escape_turn_deg` -> 30cm 후진 -> 우회전 복귀,
  `y > 0`이면 우회전 `calibration_escape_turn_deg` -> 30cm 후진 -> 좌회전 복귀,
  `y ~= 0`이면 회전 없이 바로 후진
- front는 같은 `y` 부호 기준으로 좌우 전방 대각선 방향으로 빠지도록,
  `y < 0`이면 우회전 `calibration_escape_turn_deg` -> 30cm 전진 -> 좌회전 복귀,
  `y > 0`이면 좌회전 `calibration_escape_turn_deg` -> 30cm 전진 -> 우회전 복귀,
  `y ~= 0`이면 회전 없이 바로 전진
- calibration 회전 각도는 `/rmd_state`에서 계산한 현재 각속도를 적분해 측정
- calibration 이동 거리는 `/rmd_state`에서 계산한 현재 선속도를 적분해 측정
- calibration mode 중에는 비전이 다시 들어와도 바로 정책을 재개하지 않고, 새 측정은 보류했다가 calibration 종료 후에만 다시 정책 입력으로 사용
- 정렬 완료되면 이후에는 타겟 정보와 무관하게 `final_forward_distance_m`만큼 직진
- rear는 이 직진을 전진으로, front는 후진으로 수행
- 이 직진/후진 속도는 `near_target_linear_speed_limit_m_s`, 회전 속도는 `0.0`을 사용
- 직진 이동거리는 `/rmd_state`에서 변환한 현재 선속도를 적분해 계산
- 직진 완료 후 0 `cmd_vel` publish, `/gripper_toggle=true` publish 후 노드 종료
- 타겟 거리 `sqrt(x^2+y^2)`가 `near_target_distance_m` 이하면
  `near_target_linear_speed_limit_m_s`, `near_target_angular_speed_limit_rad_s`로 명령 제한
- `|target_x_local| <= 0.05`m, `|target_y_local| <= 0.05`m, `|heading_error| <= 5deg`이면 0 `cmd_vel` publish
- `target` 또는 `motor` 메시지가 stale(timeout)면 안전하게 0 `cmd_vel` publish
- ONNX Runtime은 CPU provider만 사용

## 파라미터

- `robot_type` (default: `front`, choices: `rear`, `front`)
- `model_path` (default: `__auto__`)
- `linear_velocity_scale_m_s` (default: `__auto__`)
- `angular_velocity_scale_rad_s` (default: `__auto__`)
- `near_target_linear_speed_limit_m_s` (default: `__auto__`)
- `near_target_angular_speed_limit_rad_s` (default: `__auto__`)
- `near_target_distance_m` (default: `__auto__`)
- `target_topic` (default: `__auto__`)
- `motor_state_topic` (default: `__auto__`)
- `motor_state_type` (default: `cartrider_rmd_sdk/msg/MotorStateArray`)
- `cmd_vel_topic` (default: `__auto__`)
- `gripper_toggle_topic` (default: `__auto__`)
- `wheel_radius_m` (default: `__auto__`, 현재는 직접 지정 필요)
- `wheel_separation_m` (default: `__auto__`)
- `external_reduction` (default: `__auto__`)
- `state_invert_left` (default: `__auto__`)
- `state_invert_right` (default: `__auto__`)
- `left_motor_id` (default: `1`)
- `right_motor_id` (default: `2`)
- `control_rate_hz` (default: `30.0`)
- `target_timeout_sec` (default: `0.3`)
- `motor_timeout_sec` (default: `1000.0`)
- `target_xy_stop_tolerance_m` (default: `0.05`)
- `target_yaw_stop_tolerance_deg` (default: `5.0`)
- `base_link_to_axle_center_x_m` (default: `__auto__`)
- `base_link_to_axle_center_x_sign` (default: `__auto__`)
- `target_x_offset_m` (default: `__auto__`)
- `invert_target_xy_for_policy` (default: `__auto__`)
- `final_forward_distance_m` (default: `0.35`)
- `final_forward_motion_sign` (default: `__auto__`)
- `calibration_escape_distance_m` (default: `0.30`)
- `calibration_escape_turn_deg` (default: `30.0`)
- `calibration_escape_motion_sign` (default: `__auto__`)

`__auto__`일 때 robot_type별 기본값:
- front: `model=front_specialist_policy.onnx`, `target_topic=/front/rs/cart_pose`, `motor_state_topic=/front/rmd_state`,
  `gripper_toggle_topic=/front/cart_docking`,
  `cmd_vel_topic=/front/cmd_vel`, `state_invert_left=false`, `state_invert_right=true`,
  `near_target_distance_m=0.5`, `wheel_radius_m=0.0635`, `wheel_separation_m=0.2460`,
  `external_reduction=1.0`, `base_link_to_axle_center_x_m=0.095`, `base_link_to_axle_center_x_sign=1.0`, `target_x_offset_m=0.0`,
  `invert_target_xy_for_policy=true`, `final_forward_motion_sign=-1.0`, `calibration_escape_motion_sign=1.0`,
  `linear_velocity_scale_m_s=0.22`,
  `angular_velocity_scale_rad_s=1.81`,
  `near_target_linear_speed_limit_m_s=0.06`,
  `near_target_angular_speed_limit_rad_s=0.46`
- rear: `model=specialist_policy.onnx`, `target_topic=/rear/rs/cart_pose`, `motor_state_topic=/rmd_state`,
  `gripper_toggle_topic=/gripper_toggle`,
  `cmd_vel_topic=/cmd_vel`, `state_invert_left=true`, `state_invert_right=false`,
  `near_target_distance_m=0.5`, `wheel_radius_m=0.1100`, `wheel_separation_m=0.3000`,
  `external_reduction=1.0`, `base_link_to_axle_center_x_m=0.120`, `base_link_to_axle_center_x_sign=-1.0`, `target_x_offset_m=0.20`,
  `invert_target_xy_for_policy=false`, `final_forward_motion_sign=1.0`, `calibration_escape_motion_sign=-1.0`,
  `linear_velocity_scale_m_s=0.22`,
  `angular_velocity_scale_rad_s=1.47`,
  `near_target_linear_speed_limit_m_s=0.06`,
  `near_target_angular_speed_limit_rad_s=0.37`

출력 스케일과 제한값은 위 숫자를 그대로 기본값으로 사용합니다.
즉 아래 항목들은 더 이상 모터 각속도 제한이나 wheel geometry로 런타임 환산하지 않습니다.
- `linear_velocity_scale_m_s`
- `angular_velocity_scale_rad_s`
- `near_target_linear_speed_limit_m_s`
- `near_target_angular_speed_limit_rad_s`

`wheel_radius_m`, `wheel_separation_m`, `external_reduction`는 출력 제한 계산용이 아니라
`/rmd_state`의 순수 모터 속도를 현재 로봇의 선속도/각속도로 변환할 때만 사용합니다.
`base_link_to_axle_center_x_m`는 비전이 만든 `base_link` 중심 기준 좌표를 로봇 구동축 중심 기준 좌표로 바꾸는 x축 이동량입니다.
코드에서는 rear에 대해 `x_axle = x_base - base_link_to_axle_center_x_m`, front에 대해 `x_axle = x_base + base_link_to_axle_center_x_m`, 그리고 공통으로 `y_axle = y_base`를 적용합니다.

`target_x_offset_m`는 비전이 보는 카트 기준점과 실제 정렬 목표점 사이의 longitudinal 오프셋입니다.
코드에서는 그 다음 `x_target = x_axle - target_x_offset_m * cos(theta_vision)`,
`y_target = y_axle - target_x_offset_m * sin(theta_vision)`를 적용합니다.
비전의 yaw error 부호가 정책 기대값과 반대이므로, policy 입력에는 `heading_error = wrap_to_pi(-theta_vision)`를 사용합니다.
front는 rear와 같은 방식으로 최종 `x_target`, `y_target`를 계산한 뒤, policy 입력에 넣을 때만 `-x_target`, `-y_target`를 사용합니다.

비전 업데이트가 멈추면 노드는 마지막으로 받은 최종 `y_target`의 부호를 기준으로
캘리 방향을 한 번 결정합니다.
- rear
  - `y_target < 0`: 좌회전 `calibration_escape_turn_deg` -> 후진 `calibration_escape_distance_m` -> 우회전 복귀
  - `y_target > 0`: 우회전 `calibration_escape_turn_deg` -> 후진 `calibration_escape_distance_m` -> 좌회전 복귀
  - `y_target ~= 0`: 회전 없이 후진 `calibration_escape_distance_m`
- front
  - `y_target < 0`: 우회전 `calibration_escape_turn_deg` -> 전진 `calibration_escape_distance_m` -> 좌회전 복귀
  - `y_target > 0`: 좌회전 `calibration_escape_turn_deg` -> 전진 `calibration_escape_distance_m` -> 우회전 복귀
  - `y_target ~= 0`: 회전 없이 전진 `calibration_escape_distance_m`

회전 각도와 이동 거리는 모두 `/rmd_state`에서 계산한 현재 `angular_velocity_rad_s`,
`linear_velocity_m_s`를 적분해 측정합니다.

## 빌드

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select RL_special
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
ros2 launch RL_special specialist_policy.launch.py \
  robot_type:=front
```

리어봇으로 실행:

```bash
ros2 launch RL_special specialist_policy.launch.py \
  robot_type:=rear
```

예시: 모델 경로와 output topic만 override

```bash
ros2 launch RL_special specialist_policy.launch.py \
  robot_type:=front \
  model_path:=/path/to/front_specialist_policy.onnx \
  cmd_vel_topic:=/front/cmd_vel
```
