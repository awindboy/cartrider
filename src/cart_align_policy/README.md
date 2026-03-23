# cart_align_policy

IsaacLab에서 export한 `policy_ensemble_hardswitch.onnx`를 ROS2 노드로 실행하여,
메시지(`geometry_msgs/msg/PoseStamped`, `cartrider_rmd_sdk/msg/MotorStateArray`)를 입력으로 받아
바퀴 속도 명령을 출력합니다.

## 패키지 구성

- `cart_align_policy`
  - `policy_node` (ONNX 추론 노드)
  - `dummy_target_echo` (더미 target/motor publish + wheel_cmd echo)
  - `fixed_input_test` (고정값 target/motor publish + wheel_cmd 로그)
  - `launch/policy.launch.py`

## 토픽 및 메시지

### 1) Target 입력 (Nav -> Policy)
- Topic: `/align/target_local`
- Type: `geometry_msgs/msg/PoseStamped`
- 매핑:
  - `pose.position.x` = `target_x_local`
  - `pose.position.y` = `target_y_local`
  - `pose.orientation`(quaternion)에서 yaw 추출 = `heading_error` (wrap_to_pi)

### 2) 현재 모터 속도 입력 -> Policy
- Topic: `/rmd_state`
- Type: `cartrider_rmd_sdk/msg/MotorStateArray` (기본값)
- 매핑:
  - `states` 배열에서 `id=1`의 `speed` = `left_wheel_joint_vel` (rad/s)
  - `states` 배열에서 `id=2`의 `speed` = `right_wheel_joint_vel` (rad/s)

### 3) 바퀴 속도 출력 (Policy -> Nav)
- Topic: `/rmd_command`
- Type: `cartrider_rmd_sdk/msg/MotorCommandArray`
- 매핑:
  - `commands` 배열에 2개 명령 publish
  - `id=1` -> left motor, `target=cmd_vel_l` (rad/s)
  - `id=2` -> right motor, `target=cmd_vel_r` (rad/s)

## policy_node 동작

- 제어 주기(`control_rate_hz`, default 10Hz)마다 최신 데이터로 obs(1x5) 구성
- obs 순서(고정):
  1. `target.pose.position.x`
  2. `target.pose.position.y`
  3. `target.pose.orientation`에서 추출한 yaw
  4. `motor_state(id=1).speed` (left motor vel)
  5. `motor_state(id=2).speed` (right motor vel)
- ONNX 출력 action(2D) -> `[-1, 1]` clamp -> `action_scale` 곱하여 rad/s 명령 생성
- `|target_x_local| <= 0.01`m, `|target_y_local| <= 0.01`m, `|heading_error| <= 5deg`이면 양쪽 모터 0 명령 publish
- `target` 또는 `motor` 메시지가 stale(timeout)면 안전하게 0 명령 publish
- ONNX Runtime은 CPU provider만 사용

## 파라미터

- `model_path` (default: 패키지 설치 경로의 `models/policy_ensemble_hardswitch.onnx`)
- `target_topic` (default: `/align/target_local`)
- `motor_state_topic` (default: `/rmd_state`)
- `motor_state_type` (default: `cartrider_rmd_sdk/msg/MotorStateArray`)
- `wheel_cmd_topic` (default: `/rmd_command`)
- `wheel_cmd_type` (default: `cartrider_rmd_sdk/msg/MotorCommandArray`)
- `wheel_cmd_item_type` (default: `cartrider_rmd_sdk/msg/MotorCommand`)
- `left_motor_id` (default: `1`)
- `right_motor_id` (default: `2`)
- `action_scale` (default: `3.0`)
- `control_rate_hz` (default: `10.0`)
- `target_timeout_sec` (default: `0.3`)
- `motor_timeout_sec` (default: `0.3`)
- `target_xy_stop_tolerance_m` (default: `0.01`)
- `target_yaw_stop_tolerance_deg` (default: `5.0`)
- `invert_left` (default: `false`)
- `invert_right` (default: `false`)

## 빌드

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select cart_align_msgs cart_align_policy
source install/setup.bash
```

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

외부 모터 노드의 타입이 기본값과 다를 때만 변경하세요.

```bash
ros2 launch cart_align_policy policy.launch.py \
  motor_state_type:=my_motor_msgs/msg/MotorStateArray
```

### 더미 테스트 노드 실행

```bash
ros2 run cart_align_policy dummy_target_echo
```

### 고정 입력 스모크 테스트

터미널 1:

```bash
ros2 launch cart_align_policy policy.launch.py
```

터미널 2:

```bash
ros2 run cart_align_policy fixed_input_test
```

파라미터 예시:

```bash
ros2 run cart_align_policy fixed_input_test --ros-args \
  -p target_x_local:=1.5 \
  -p target_y_local:=0.2 \
  -p heading_error:=0.15 \
  -p left_motor_vel:=0.2 \
  -p right_motor_vel:=-0.1
```

### 출력 확인

```bash
ros2 topic echo /rmd_command
```
