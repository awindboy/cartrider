# RL_special

`RL_special`은 specialist ONNX 정책을 ROS2에서 계속 실행하는 폐루프 도킹 노드입니다.  
노드는 종료되지 않고 살아 있으면서 `/docking_target` 명령에 따라 `대기 -> 정렬 -> 캘리 -> 최종 도킹 이동 -> 대기`를 반복합니다.

## 도킹 타겟 입력

- Topic: `/docking_target`
- Type: `std_msgs/msg/Int32`
- 의미
  - `0`: 대기
  - `1`: front↔rear 로봇 결합
  - `2`: 카트↔로봇 결합

기본 상태는 `waiting_docking_target`입니다.  
`docking_target != 0`을 받으면 정렬을 시작하고, 완료되면 내부 active target을 다시 `0`으로 리셋한 뒤 대기 모드로 복귀합니다.

중간에 값이 바뀌면 현재 align/calibration/final 단계는 즉시 취소되고, 새 target 기준으로 다시 시작합니다.

rear 프로파일은 `docking_target=1`을 받으면 동작하지 않고 `rear_robot_docking_idle` 상태로 대기합니다.

## 토픽 구성

### Vision target 입력

- Type: `geometry_msgs/msg/Pose2D`
- front 기본값: `/front/rs/cart_pose`
- rear 기본값: `/rear/rs/cart_pose`

비전이 주는 `x, y, theta`는 `base_link` 중심 기준입니다. 내부에서는 다음 순서로 가공합니다.

1. front만 카메라 기준 `x, y`를 로봇 진행방향 기준으로 변환
   - `x_robot = -x_camera`
   - `y_robot = -y_camera`
2. `base_link -> 구동축 중심` 변환
   - rear: `x_axle = x_base - base_link_to_axle_center_x_m`
   - front: `x_axle = x_base + base_link_to_axle_center_x_m`
3. 카트 표면 offset 적용
   - `x_target = x_axle - target_x_offset_m * cos(theta_vision)`
   - `y_target = y_axle - target_x_offset_m * sin(theta_vision)`
4. policy yaw 입력
   - `heading_error = wrap_to_pi(-theta_vision)`

즉 front는 target 입력 좌표를 먼저 로봇 진행방향 기준으로 바꾸고, 그 뒤의 가공 체인은 rear와 동일합니다.

idle 중에도 최신 비전 target은 계속 캐시됩니다.  
명령이 들어오면 이 캐시를 바로 사용할 수 있고, 캐시가 stale이면 새 비전 입력을 기다립니다.

### 모터 상태 입력

- Type 기본값: `cartrider_rmd_sdk/msg/MotorStateArray`
- front 기본값: `/front/rmd_state`
- rear 기본값: `/rmd_state`

`states`에서 `left_motor_id`, `right_motor_id`의 `speed`를 읽고, front/rear 프로파일별로

- `state_invert_left/right`
- `external_reduction`
- `wheel_radius_m`
- `wheel_separation_m`

를 적용해 현재 로봇의 `linear_velocity_m_s`, `angular_velocity_rad_s`를 계산합니다.

front는 현재 `/front/rmd_state`가 이미 감속 후 속도라고 가정해서 기본 `external_reduction=1.0`을 사용합니다.

### cmd_vel 출력

- Type: `geometry_msgs/msg/Twist`
- front 기본값: `/front/cmd_vel`
- rear 기본값: `/cmd_vel`

사용 필드:

- `linear.x`
- `angular.z`

### 완료 신호 출력

- front + `docking_target=1`: `/front/robot_docking`에 `Bool(true)` 1회
- front + `docking_target=2`: `/front/cart_docking`에 `Bool(true)` 1회
- rear + `docking_target=2`: `/gripper_toggle`에 `Bool(true)` 1회
- rear + `docking_target=1`: publish 없음

## 상태 머신

### 1. `waiting_docking_target`

- 기본 대기 상태
- 항상 `cmd_vel=0`
- 최신 비전 target은 계속 캐시

### 2. `align`

정책 입력 obs는 고정 5차원입니다.

1. `target_x_local`
2. `target_y_local`
3. `heading_error = wrap_to_pi(-theta_vision)`
4. 현재 선속도
5. 현재 각속도

정렬 완료 조건:

- `abs(target_x_local) <= target_xy_stop_tolerance_m`
- `abs(target_y_local) <= target_xy_stop_tolerance_m`
- `abs(heading_error) <= target_yaw_stop_tolerance_rad`

완료되면 `final_docking_motion`으로 넘어갑니다.

### 3. `calibration`

비전 target이 `target_timeout_sec` 이상 끊기면 calibration으로 들어갑니다.

현재 calibration은 가변 회전 escape 시퀀스입니다.

- `rotate_out`
- `reverse_escape`
- `rotate_back`

첫 회전은 마지막으로 기억한 **오프셋 적용 완료 후의 실제 target state**에서 계산합니다.

- 기준은 robot frame이 아니라 **target-centered frame**입니다.
- 목표는 “첫 회전 후 `calibration_escape_distance_m`만큼 직선 이동했을 때, 로봇 구동축 중심이 target frame의 `x-`축 위에 놓이게 하는 것”입니다.
- 이를 위해 현재 target state를 target frame으로 변환한 뒤, 30cm 이동 후 도달해야 할 점을 `y_T = 0`이면서 `x_T < 0`인 유일한 점으로 바로 계산합니다.
- 즉 1차 회전은 “후보 두 개를 만든 뒤 고르는 방식”이 아니라, **고정 이동거리 후 로봇이 target frame의 `x-`축 위에 오도록 하는 단일 해**를 직접 계산합니다.
- rear는 이후 후진 30cm, front는 이후 전진 30cm이므로 같은 target state에서도 다른 첫 회전각이 나올 수 있습니다.

이후 `calibration_escape_distance_m`만큼 직선 이동하고, 마지막 회전은 이동 중 오도메트리로 계속 적분된 현재 `target_theta_vision_rad`를 기준으로 계산합니다.

- `rotate_back_target = -target_theta_vision_rad`
- 의미: 마지막 회전이 끝났을 때 yaw error가 0이 되도록 맞춥니다.

rear는 기존처럼 후진 `calibration_escape_distance_m`, front는 기존처럼 전진 `calibration_escape_distance_m`를 유지합니다.

회전 각도와 이동 거리는 모두 `/rmd_state`에서 계산한 현재 속도를 적분해 측정하고, calibration 중에도 내부 target state를 계속 오도메트리로 업데이트합니다.

calibration 중 비전이 다시 들어와도 즉시 정책을 재개하지 않습니다. 새 측정은 보류해두고, calibration이 끝난 다음에만 다시 align에 반영합니다.

### 4. `final_docking_motion`

정렬 완료 후에는 타겟과 무관하게 지정 거리만큼 직선 이동합니다.

- `docking_target=2`: `cart_docking_final_distance_m`
- `docking_target=1`: `robot_docking_final_distance_m`

이동 방향은 `final_docking_motion_sign`으로 결정됩니다.

- front 기본값: `-1.0` (후진)
- rear 기본값: `+1.0` (전진)

거리 적분은 현재 선속도를 사용합니다.

완료되면:

1. `cmd_vel=0`
2. target 종류에 맞는 completion topic에 `Bool(true)` 1회 publish
3. 내부 active target을 `0`으로 리셋
4. `waiting_docking_target` 복귀

## front / rear 차이

### front

- target topic: `/front/rs/cart_pose`
- motor state topic: `/front/rmd_state`
- cmd_vel topic: `/front/cmd_vel`
- completion topic
  - robot docking: `/front/robot_docking`
  - cart docking: `/front/cart_docking`
- `base_link_to_axle_center_x_sign = +1.0`
- `invert_target_xy_for_policy = false`
- `final_docking_motion_sign = -1.0`
- `calibration_escape_motion_sign = +1.0`
- `external_reduction = 1.0`

### rear

- target topic: `/rear/rs/cart_pose`
- motor state topic: `/rmd_state`
- cmd_vel topic: `/cmd_vel`
- completion topic
  - robot docking: `/gripper_toggle` but runtime no-op
  - cart docking: `/gripper_toggle`
- `base_link_to_axle_center_x_sign = -1.0`
- `invert_target_xy_for_policy = false`
- `final_docking_motion_sign = +1.0`
- `calibration_escape_motion_sign = -1.0`
- `external_reduction = 1.0`

## 주요 파라미터

- `robot_type` (`front` or `rear`)
- `docking_target_topic`
- `target_topic`
- `motor_state_topic`
- `cmd_vel_topic`
- `robot_docking_completion_topic`
- `cart_docking_completion_topic`
- `linear_velocity_scale_m_s`
- `angular_velocity_scale_rad_s`
- `near_target_distance_m`
- `near_target_linear_speed_limit_m_s`
- `near_target_angular_speed_limit_rad_s`
- `target_timeout_sec`
- `motor_timeout_sec`
- `target_xy_stop_tolerance_m`
- `target_yaw_stop_tolerance_deg`
- `base_link_to_axle_center_x_m`
- `base_link_to_axle_center_x_sign`
- `target_x_offset_m`
- `invert_target_xy_for_policy`
- `cart_docking_final_distance_m`
- `robot_docking_final_distance_m`
- `final_docking_motion_sign`
- `calibration_escape_distance_m`
- `calibration_escape_motion_sign`
- `wheel_radius_m`
- `wheel_separation_m`
- `external_reduction`
- `state_invert_left`
- `state_invert_right`

`launch`의 `__auto__`는 robot profile 기본값을 사용합니다.

## 빌드

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select RL_special
source install/setup.bash
```

## 실행

front:

```bash
ros2 launch RL_special specialist_policy.launch.py robot_type:=front
```

rear:

```bash
ros2 launch RL_special specialist_policy.launch.py robot_type:=rear
```

런치 후에는 `/docking_target`에 아래처럼 명령을 주면 됩니다.

```bash
ros2 topic pub /docking_target std_msgs/msg/Int32 "{data: 2}" -1
```

- `0`: 대기
- `1`: robot docking
- `2`: cart docking
