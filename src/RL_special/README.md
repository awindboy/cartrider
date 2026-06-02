# RL_special

`RL_special`은 specialist ONNX 정책을 ROS2에서 계속 실행하는 폐루프 도킹 노드입니다.  
노드는 종료되지 않고 살아 있으면서 `/docking_target` 명령에 따라 `대기 -> 정렬 -> 캘리 -> 최종 도킹 이동 -> 대기`를 반복합니다.

## 도킹 타겟 입력

- rear Topic: `/docking_target`
- front Topic: `/front/docking_target`
- Type: `std_msgs/msg/Int32`
- 의미
  - `0`: 대기
  - `1`: front↔rear 로봇 결합
  - `2`: 카트↔로봇 결합

기본 상태는 `waiting_docking_target`입니다.  
`docking_target != 0`을 받으면 정렬을 시작하고, 완료되면 내부 active target을 다시 `0`으로 리셋한 뒤 대기 모드로 복귀합니다.

중간에 값이 바뀌면 현재 align/calibration/final 단계는 즉시 취소되고, 새 target 기준으로 다시 시작합니다.

rear 프로파일은 `docking_target=1`을 받으면 주행은 하지 않고 `/gripper_toggle`을 1회 publish한 뒤 `rear_robot_docking_idle` 상태로 복귀합니다.

front와 rear 모두 target이 아직 보이더라도, canonical `target_x_local`이 각 로봇의 캘리 진입 거리 조건을 넘었고 `y` 또는 `yaw`가 아직 tolerance 밖이면 로봇이 offset target에 너무 가까워진 것으로 보고 그 순간의 target을 고정한 뒤 calibration으로 진입합니다.

- front: `target_x_local > robot_docking_calibration_target_x_threshold_m`
- rear: `target_x_local < rear_calibration_target_x_threshold_m`

## 토픽 구성

### Vision target 입력

- Type: `geometry_msgs/msg/PointStamped`
- front 기본값: `/front/target_pose`
- rear 기본값: `/rear/target_pose`

`point.x`, `point.y`, `point.z`를 각각 기존의 `x`, `y`, `theta(rad)`로 사용합니다.
아루코 ID는 `header.frame_id`의 정수값으로 읽습니다.

- front는 ID `0`일 때만 유효 타겟으로 사용
- rear는 ID `1`일 때만 유효 타겟으로 사용
- 다른 ID가 들어오면 해당 측정은 무시하고, 현재 active docking 중이면 마지막으로 기억한 canonical target 기준으로 calibration으로 진입합니다.

비전이 주는 `x, y, theta`는 `base_link` 중심 기준입니다. 내부에서는 수신 즉시 “구동축 중앙 기준 로봇 로컬 프레임”의 canonical target으로 변환하고, 이후 policy/align/calibration/odometry는 모두 이 값만 사용합니다.

1. front만 카메라 기준 `x, y`를 로봇 진행방향 기준으로 변환
   - `x_robot = -x_camera`
   - `y_robot = -y_camera`
2. `base_link -> 구동축 중심` 변환
   - front/rear 공통: `x_axle = x_base - base_link_to_axle_center_x_m`
3. policy/target yaw 기준 생성
   - `target_yaw_error = wrap_to_pi(-theta)`
4. 카트 표면 offset 적용
   - rear: `x_target = x_axle - target_x_offset_m * cos(target_yaw_error)`, `y_target = y_axle - target_x_offset_m * sin(target_yaw_error)`
   - front: `x_target = x_axle + target_x_offset_m * cos(target_yaw_error)`, `y_target = y_axle + target_x_offset_m * sin(target_yaw_error)`
5. rear 전용 y offset 적용
   - rear만 `y_target += rear_target_y_offset_m`

즉 front는 target 입력 좌표를 먼저 로봇 진행방향 기준으로 바꾸고, 그 뒤의 가공 체인은 rear와 동일합니다. 정책 입력도 canonical target인 `[x_target, y_target, target_yaw_error, current_v, current_omega]`를 그대로 사용합니다.

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
- `linear_odometry_scale`
- `angular_odometry_scale`

를 적용해 현재 로봇의 `linear_velocity_m_s`, `angular_velocity_rad_s`를 계산합니다.

front는 현재 `/front/rmd_state`가 이미 감속 후 속도라고 가정해서 기본 `external_reduction=1.0`을 사용합니다.

실제 로봇이 캘리/최종 이동에서 살짝 덜 가거나 덜 돌면 odometry scale을 낮춰 보정합니다. 예를 들어 3% 덜 이동하면 `linear_odometry_scale:=0.97`, 회전이 3% 부족하면 `angular_odometry_scale:=0.97`처럼 시작합니다.

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
- rear + `docking_target=1`: `/gripper_toggle`에 `Bool(true)` 1회
- rear 도킹 성공시: `/rl_docking_done`에 `Bool(true)`를 반복 publish
- front 도킹 성공시: `/front/rl_docking_done`에 `Bool(true)`를 반복 publish

외부 `Bool` 토픽 `/docking_state`를 구독합니다.
`/docking_state == true`가 들어오면 내부 `rl_docking_done` 상태를 초기화해서 반복 publish를 멈추고, 다음 성공 이벤트를 다시 publish할 수 있게 합니다.

## 상태 머신

### 1. `waiting_docking_target`

- 기본 대기 상태
- 상태 진입 시 1회 `cmd_vel=0`
- 대기 중에는 `cmd_vel`을 계속 publish하지 않음
- 최신 비전 target은 계속 캐시

### 2. `align`

정책 입력 obs는 고정 5차원입니다.

1. `target_x_local`
2. `target_y_local`
3. `target_yaw_error = wrap_to_pi(-theta)`
4. 현재 선속도
5. 현재 각속도

정렬 완료 조건:

- `abs(target_x_local) <= target_xy_stop_tolerance_m`
- `abs(target_y_local) <= target_xy_stop_tolerance_m`
- `abs(target_yaw_error) <= target_yaw_stop_tolerance_rad`

`docking_target=1` 로봇 도킹에서는 위 조건 대신 robot docking 전용 tolerance를 사용합니다.

- `abs(target_x_local) <= robot_docking_target_xy_stop_tolerance_m`
- `abs(target_y_local) <= robot_docking_target_xy_stop_tolerance_m`
- `abs(target_yaw_error) <= robot_docking_target_yaw_stop_tolerance_rad`

완료되면 `final_docking_motion`으로 넘어갑니다.

### 3. `calibration`

비전 target이 `target_timeout_sec` 이상 끊기면 calibration으로 들어갑니다.

target이 끊기지 않았더라도, align 중 canonical `target_x_local`이 로봇별 캘리 진입 거리 조건을 넘었고 `y` 또는 `yaw`가 아직 tolerance 밖이면 calibration으로 들어갑니다.

현재 calibration은 가변 회전 escape 시퀀스입니다.

- `rotate_out`
- `move_to_axis`
- `rotate_back`

첫 회전은 마지막으로 기억한 **오프셋 적용 완료 후의 실제 target state**에서 계산합니다.

- 기준은 canonical target state와 노드 내부 odometry 모델입니다.
- 목표는 로봇 구동축 중심을 target frame의 허용 반축 위 최근접점으로 보내는 것입니다.
- rear는 target frame의 `x-`축 위 최근접점으로, front는 `x+`축 위 최근접점으로 이동합니다.
- front는 target에 너무 붙은 상태에서 캘리가 시작되면, 안전을 위해 최소 `front_calibration_safe_axis_x_m`만큼 앞쪽의 `x+` 위치를 목표로 사용합니다.
- rear도 target에 너무 붙은 상태에서 캘리가 시작되면, 안전을 위해 최소 `rear_calibration_safe_axis_x_m`만큼 뒤쪽의 `x-` 위치를 목표로 사용합니다.
- 따라서 고정 30cm 규칙은 없고, 필요한 만큼만 직선 이동합니다.
- 1차 회전각은 현재 로봇 위치에서 그 최근접 축점으로 직선 이동할 수 있도록 계산합니다.
- 이때 전진/후진을 미리 고정하지 않고, 더 적은 회전으로 갈 수 있는 방향을 선택합니다.

이후 계산된 최근접 축점까지 직선 이동하고, 마지막 회전은 이동 중 오도메트리로 계속 적분된 현재 `target_yaw_error_rad`를 기준으로 계산합니다.

- `rotate_back_target = target_yaw_error_rad`
- 의미: 마지막 회전이 끝났을 때 yaw error가 0이 되도록 맞춥니다.

회전 각도와 이동 거리는 모두 `/rmd_state`에서 계산한 현재 속도를 적분해 측정하고, calibration 중에도 내부 target state를 계속 오도메트리로 업데이트합니다.

calibration 중 비전이 다시 들어와도 즉시 정책을 재개하지 않습니다. 새 측정은 보류해두고, calibration이 끝난 다음에만 다시 align에 반영합니다.

calibration이 끝나면 front/rear 공통으로 `0.5초` 동안 `cmd_vel=0`으로 정지한 뒤 다시 `align`을 시작합니다.

### 4. `final_docking_motion`

정렬 완료 후에는 타겟과 무관하게 지정 거리만큼 직선 이동합니다.

- `docking_target=2`: `cart_docking_final_distance_m`
- `docking_target=1`: `robot_docking_final_distance_m`

이동 방향은 `final_docking_motion_sign`으로 결정됩니다.

- front 기본값: `-1.0` (후진)
- rear 기본값: `+1.0` (전진)

최종 이동 속도는 기본적으로 `near_target_linear_speed_limit_m_s`를 사용하지만, `docking_target=1` 로봇 도킹만 `robot_docking_final_linear_speed_m_s`를 따로 사용합니다.

거리 적분은 현재 선속도를 사용합니다.

완료되면:

1. `cmd_vel=0`
2. target 종류에 맞는 completion topic에 `Bool(true)` 1회 publish
3. 내부 active target을 `0`으로 리셋
4. `waiting_docking_target` 복귀

## front / rear 차이

### front

- docking_target topic: `/front/docking_target`
- target topic: `/front/target_pose`
- motor state topic: `/front/rmd_state`
- cmd_vel topic: `/front/cmd_vel`
- completion topic
  - robot docking: `/front/robot_docking`
  - cart docking: `/front/cart_docking`
- rl docking done topic: `/front/rl_docking_done`
- `final_docking_motion_sign = -1.0`
- `robot_docking_final_linear_speed_m_s = 0.15`
- `robot_docking_target_xy_stop_tolerance_m = 0.03`
- `robot_docking_target_yaw_stop_tolerance_deg = 2.0`
- `robot_docking_calibration_target_x_threshold_m = -0.10`
- `external_reduction = 1.0`

### rear

- docking_target topic: `/docking_target`
- target topic: `/rear/target_pose`
- motor state topic: `/rmd_state`
- cmd_vel topic: `/cmd_vel`
- cart docking completion topic: `/gripper_toggle`
- rl docking done topic: `/rl_docking_done`
- `final_docking_motion_sign = +1.0`
- `external_reduction = 1.0`

rear는 `docking_target=1`에서 주행 없이 `/gripper_toggle`만 1회 publish한 뒤 idle로 빠지므로, `robot_docking_*` 파라미터와 robot-docking completion publisher는 front 전용으로만 실제 사용됩니다.

## 주요 파라미터

- `robot_type` (`front` or `rear`)
- `docking_target_topic`
- `target_topic`
- `docking_state_topic`
- `motor_state_topic`
- `cmd_vel_topic`
- `robot_docking_completion_topic` : front only
- `cart_docking_completion_topic`
- `rl_docking_done_topic`
- `linear_velocity_scale_m_s`
- `angular_velocity_scale_rad_s`
- `near_target_distance_m`
- `near_target_linear_speed_limit_m_s`
- `near_target_angular_speed_limit_rad_s`
- `target_timeout_sec`
- `motor_timeout_sec`
- `target_xy_stop_tolerance_m`
- `target_yaw_stop_tolerance_deg`
- `robot_docking_target_xy_stop_tolerance_m` : front only
- `robot_docking_target_yaw_stop_tolerance_deg` : front only
- `base_link_to_axle_center_x_m`
- `target_x_offset_m`
- `rear_target_y_offset_m` : rear only
- `front_calibration_safe_axis_x_m` : front only
- `rear_calibration_safe_axis_x_m` : rear only
- `cart_docking_final_distance_m`
- `robot_docking_final_distance_m` : front only
- `final_docking_motion_sign`
- `robot_docking_final_linear_speed_m_s` : front only
- `robot_docking_calibration_target_x_threshold_m` : front only
- `rear_calibration_target_x_threshold_m` : rear only
- `wheel_radius_m`
- `wheel_separation_m`
- `external_reduction`
- `linear_odometry_scale`
- `angular_odometry_scale`
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

canonical target과 calibration 기하 조건은 아래 스크립트로 재검증할 수 있습니다.

```bash
python3 src/RL_special/docs/verify_target_calibration_geometry.py
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

## 오도메트리 키보드 디버그

정책과 비전 없이 `/rmd_state` 오도메트리 적분만으로 정해진 거리/각도를 움직이는 디버그 런치입니다.

```bash
ros2 launch RL_special debug_odometry.launch.py robot_type:=front
ros2 launch RL_special debug_odometry.launch.py robot_type:=rear
```

키 입력:

- `↑` 또는 `w`: 로봇 로컬 `+x` 방향 30cm
- `↓` 또는 `s`: 로봇 로컬 `-x` 방향 30cm
- `←` 또는 `a`: `+yaw` 방향 90도 좌회전
- `→` 또는 `d`: `-yaw` 방향 90도 우회전
- `space`: 즉시 정지
- `q`: 정지 후 종료

기본값은 profile별 `cmd_vel`, `rmd_state`, 바퀴 반지름, 트랙 폭, 모터 부호 보정을 사용합니다.
진행률은 명령 시간이 아니라 `/rmd_state`에서 계산한 실제 `current_v/current_omega`를 적분해 판단합니다.
