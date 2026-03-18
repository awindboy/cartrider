# cart_align_policy

IsaacLab에서 export한 `policy.onnx`를 ROS2 노드로 실행하여,
네비게이션 파트에서 전달한 로컬 타겟 정보를 바탕으로 좌/우 바퀴 속도 명령을 publish합니다.

## 패키지 구성

- `cart_align_msgs`
  - `AlignTargetLocal.msg`
  - `WheelCmd.msg`
- `cart_align_policy`
  - `policy_node` (ONNX 추론 노드)
  - `dummy_target_echo` (더미 타겟 publish + wheel_cmd echo)
  - `fixed_input_test` (고정 target/joint_states publish + wheel_cmd 로그)
  - `launch/policy.launch.py`

## 토픽 및 메시지

### 1) Nav -> Policy
- Topic: `/align/target_local`
- Type: `cart_align_msgs/msg/AlignTargetLocal`

```text
std_msgs/Header header
float32 target_x_local
float32 target_y_local
float32 heading_error
```

### 2) Policy -> Nav
- Topic: `/align/wheel_cmd`
- Type: `cart_align_msgs/msg/WheelCmd`

```text
std_msgs/Header header
float32 cmd_vel_r
float32 cmd_vel_l
float32 left_action_raw
float32 right_action_raw
```

### 3) Policy 입력 보조
- Topic: `/joint_states`
- Type: `sensor_msgs/msg/JointState`
- 설정한 `left_joint_name`, `right_joint_name`의 실제 joint velocity를 사용

## policy_node 동작

- 제어 주기(`control_rate_hz`, default 10Hz)마다 최신 데이터로 obs(1x5) 구성
- obs 순서(고정):
  1. `target_x_local`
  2. `target_y_local`
  3. `heading_error`
  4. `left_wheel_joint_vel`
  5. `right_wheel_joint_vel`
- ONNX 출력 action(2D) -> `[-1, 1]` clamp -> `action_scale` 곱하여 rad/s 명령 생성
- `target_local`이 timeout(`target_timeout_sec`)보다 오래되었거나,
  필요한 `joint_states`가 아직 없으면 안전하게 0 명령 publish

## 파라미터

- `model_path` (default: 패키지 설치 경로의 `models/policy.onnx`)
- `left_joint_name` (default: `left_wheel_joint`)
- `right_joint_name` (default: `right_wheel_joint`)
- `action_scale` (default: `4.0055306333`)
- `control_rate_hz` (default: `10.0`)
- `target_timeout_sec` (default: `0.3`)
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

예시 (joint 이름 지정):

```bash
ros2 launch cart_align_policy policy.launch.py \
  left_joint_name:=Link_f1_wheel_joint \
  right_joint_name:=Link_f2_wheel_joint
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

기본값으로 `/align/target_local`과 `/joint_states`를 10Hz로 publish하고,
`/align/wheel_cmd`를 주기적으로 로그합니다.

파라미터 예시:

```bash
ros2 run cart_align_policy fixed_input_test --ros-args \
  -p target_x_local:=1.5 \
  -p target_y_local:=0.2 \
  -p heading_error:=0.15 \
  -p left_wheel_joint_vel:=0.2 \
  -p right_wheel_joint_vel:=-0.1
```

### 확인용 echo

```bash
ros2 topic echo /align/wheel_cmd
```

## 참고

- `policy.onnx`는 패키지의 `models/`에 포함되어 설치됩니다.
- `WheelCmd`의 좌/우 cmd는 `cmd_vel_l`, `cmd_vel_r` 필드로 publish됩니다.
