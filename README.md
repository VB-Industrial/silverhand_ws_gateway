# silverhand_ws_gateway

Robot-side websocket gateway for SilverHand robot domains.

Важно:
- репозиторий уже общий: `silverhand_ws_gateway`
- ROS package name пока сохранён как `silverhand_arm_ws_gateway` для совместимости во время миграции
- внутри пакета уже есть два домена: `arm` и `rover`

Назначение:
- принимает команды от GUI по websocket
- переводит их в robot-side backend
- возвращает доменные state-сообщения обратно в GUI

Поддерживаемые backend-режимы:

- `arm/mock`
- `arm/ros`
- `arm/moveit`
- `rover/mock`
- `rover/ros`

## Зависимости

```bash
sudo apt-get update
sudo apt-get install -y python3-websockets
```

Для `ros`/`moveit`-режимов также нужен рабочий ROS 2 Jazzy workspace.

## Сборка

```bash
cd /home/r/silver_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select silverhand_arm_ws_gateway
source install/setup.bash
```

## Режимы запуска

### 1. Arm mock

```bash
ros2 run silverhand_arm_ws_gateway gateway --domain arm --mode mock --host 0.0.0.0 --port 8765
```

или:

```bash
ros2 launch silverhand_arm_ws_gateway mock_gateway.launch.py
```

Helper:

```bash
cd /home/r/silver_ws/src/silverhand_arm_ws_gateway
./scripts/start_gateway_mock.sh
./scripts/run_mock_gateway.sh
```

Smoke test:

```bash
cd /home/r/silver_ws/src/silverhand_arm_ws_gateway
python3 scripts/mock_smoke_test.py --domain arm --url ws://127.0.0.1:8765
```

### 2. Arm ros2_control

Этот режим шлёт arm-команды напрямую в:

- `/arm_controller/follow_joint_trajectory`

Запуск:

```bash
ros2 run silverhand_arm_ws_gateway gateway --domain arm --mode ros --host 0.0.0.0 --port 8765
```

или:

```bash
ros2 launch silverhand_arm_ws_gateway ros_gateway.launch.py
```

Helper:

```bash
cd /home/r/silver_ws/src/silverhand_arm_ws_gateway
./scripts/start_gateway_ros.sh
```

### 3. Arm MoveIt

Этот режим шлёт arm/gripper команды в:

- `MoveGroup` action, по умолчанию `/move_action`

Запуск:

```bash
ros2 run silverhand_arm_ws_gateway gateway --domain arm --mode moveit --host 0.0.0.0 --port 8765 --move-group-action /move_action
```

или:

```bash
ros2 launch silverhand_arm_ws_gateway moveit_gateway.launch.py
```

Helper:

```bash
cd /home/r/silver_ws/src/silverhand_arm_ws_gateway
./scripts/start_gateway_moveit.sh
```

### 4. Rover

ROS bringup:

```bash
ros2 run silverhand_arm_ws_gateway gateway \
  --domain rover \
  --mode ros \
  --host 0.0.0.0 \
  --port 8766 \
  --rover-cmd-vel-topic /rover_base_controller/cmd_vel_unstamped \
  --rover-odom-topic /rover_base_controller/odom \
  --rover-battery-topic /battery_state \
  --rover-headlights-service /power_board/set_headlights
```

Mock bringup:

```bash
ros2 run silverhand_arm_ws_gateway gateway --domain rover --mode mock --host 0.0.0.0 --port 8766
```

Launch:

```bash
ros2 launch silverhand_arm_ws_gateway rover_gateway.launch.py
```

Helper:

```bash
cd /home/r/silver_ws/src/silverhand_arm_ws_gateway
SILVERHAND_WS_MODE=ros ./scripts/start_gateway_rover.sh
SILVERHAND_WS_MODE=mock ./scripts/start_gateway_rover.sh
```

## Что поддерживается сейчас

### Общие сообщения

- `hello`
- `hello_ack`
- `ping`
- `pong`
- `fault_state`

### Arm / gripper goal path

- `set_joint_goal`
- `plan`
- `execute`
- `stop`
- `estop`
- `reset_estop`

### Состояние

- `joint_state`
- `planning_state`
- `execution_state`

## Важные замечания по режимам

### Mock

- хорош для сети и GUI smoke test
- не проверяет реальную коллизионную валидность `MoveIt`

### Ros

- arm идёт напрямую через `joint_trajectory_controller`
- `set_pose_goal` пока не маппится, используйте `set_joint_goal`

### MoveIt

- arm и gripper идут через `MoveGroup`
- основная рабочая команда для GUI сейчас тоже `set_joint_goal`
- `set_pose_goal` пока не маппится в `moveit_adapter`
- в логах gateway теперь видно:
  - отправку goal
  - accepted/rejected
  - result status
  - `error_code`
  - `message`

## Типовой сценарий arm + hand + MoveIt

На robot machine:

1. Поднять `silverhand_system_bringup`:

```bash
cd ~/silver_ws
source /opt/ros/jazzy/setup.bash
source ~/silver_ws/install/setup.bash
ros2 launch silverhand_system_bringup silverhand_system_arm_hand_moveit.launch.py use_mock_hardware:=true use_rviz:=false
```

2. Поднять gateway:

```bash
cd ~/silver_ws
source /opt/ros/jazzy/setup.bash
source ~/silver_ws/install/setup.bash
ros2 launch silverhand_arm_ws_gateway moveit_gateway.launch.py
```

3. В GUI подключиться к:

```text
ws://<robot-ip>:8765
```

## Полезные проверки

Порт gateway:

```bash
ss -ltnp | grep 8765
```

Контроллеры:

```bash
ros2 control list_controllers
```

MoveIt action:

```bash
ros2 action list | grep move_action
```

## Логи

Если запускаешь вручную через `nohup`, удобно писать в:

- `~/silver_ws/run_logs/ws_gateway_moveit.log`
- `~/silver_ws/run_logs/ws_gateway_ros.log`

Именно по `ws_gateway` логам сейчас лучше всего видно:

- дошёл ли goal
- accepted/rejected
- чем закончился `MoveIt` request

## systemd

В пакете есть user-service template:

- `systemd/user/silverhand-ws-gateway@.service`

Экземпляры:

- `mock`
- `ros`
- `moveit`

Установка:

```bash
mkdir -p ~/.config/systemd/user
cp /home/r/silver_ws/src/silverhand_arm_ws_gateway/systemd/user/silverhand-ws-gateway@.service ~/.config/systemd/user/
systemctl --user daemon-reload
```

Запуск:

```bash
systemctl --user enable --now silverhand-ws-gateway@mock.service
systemctl --user enable --now silverhand-ws-gateway@ros.service
systemctl --user enable --now silverhand-ws-gateway@moveit.service
```

Автозапуск без логина:

```bash
loginctl enable-linger "$USER"
```

Полезные команды:

```bash
systemctl --user status silverhand-ws-gateway@moveit.service
journalctl --user -u silverhand-ws-gateway@moveit.service -f
systemctl --user restart silverhand-ws-gateway@moveit.service
```
