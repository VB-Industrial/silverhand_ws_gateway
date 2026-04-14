# silverhand_ws_gateway

Robot-side websocket gateway for SilverHand domains.

Поддерживаемые режимы:

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

Для `ros` и `moveit` режимов нужен рабочий ROS 2 Jazzy workspace.

## Сборка

```bash
cd /home/r/silver_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select silverhand_ws_gateway
source install/setup.bash
```

## Основные скрипты запуска

```bash
cd /home/r/silver_ws/src/silverhand_ws_gateway
./scripts/start_arm_mock.sh
./scripts/start_arm_ros.sh
./scripts/start_arm_moveit.sh
./scripts/start_rover_mock.sh
./scripts/start_rover_ros.sh
```

Порты по умолчанию:

- arm: `8765`
- rover: `8766`

Полезные переменные окружения:

- `SILVERHAND_WS_HOST`
- `SILVERHAND_WS_PORT`
- `SILVERHAND_WS_LOG_LEVEL`
- `SILVERHAND_MOVE_GROUP_ACTION`
- `SILVERHAND_ROVER_CMD_VEL_TOPIC`
- `SILVERHAND_ROVER_ODOM_TOPIC`
- `SILVERHAND_ROVER_BATTERY_TOPIC`
- `SILVERHAND_ROVER_HEADLIGHTS_SERVICE`

## Launch-файлы

- `/home/r/silver_ws/src/silverhand_ws_gateway/launch/arm_mock.launch.py`
- `/home/r/silver_ws/src/silverhand_ws_gateway/launch/arm_ros.launch.py`
- `/home/r/silver_ws/src/silverhand_ws_gateway/launch/arm_moveit.launch.py`
- `/home/r/silver_ws/src/silverhand_ws_gateway/launch/rover_mock.launch.py`
- `/home/r/silver_ws/src/silverhand_ws_gateway/launch/rover_ros.launch.py`

Если нужен запуск именно через `ros2 launch`, сначала добавьте локальный prefix gateway в окружение:

```bash
source /opt/ros/jazzy/setup.bash
source /home/r/silver_ws/install/setup.bash
export AMENT_PREFIX_PATH=/home/r/silver_ws/install/silverhand_ws_gateway:$AMENT_PREFIX_PATH
```

## Smoke test

Arm mock:

```bash
cd /home/r/silver_ws/src/silverhand_ws_gateway
python3 scripts/mock_smoke_test.py --domain arm --url ws://127.0.0.1:8765
```

Rover mock:

```bash
cd /home/r/silver_ws/src/silverhand_ws_gateway
python3 scripts/mock_smoke_test.py --domain rover --url ws://127.0.0.1:8766
```

## Типовой запуск arm + MoveIt

На robot machine:

```bash
cd /home/r/silver_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch silverhand_system_bringup silverhand_system_arm_hand_moveit.launch.py use_mock_hardware:=true use_rviz:=false
```

Во втором терминале:

```bash
cd /home/r/silver_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export AMENT_PREFIX_PATH=/home/r/silver_ws/install/silverhand_ws_gateway:$AMENT_PREFIX_PATH
ros2 launch /home/r/silver_ws/src/silverhand_ws_gateway/launch/arm_moveit.launch.py
```

GUI:

```text
ws://<robot-ip>:8765
```

## Полезные проверки

Порты:

```bash
ss -ltnp | grep -E '8765|8766'
```

Контроллеры:

```bash
ros2 control list_controllers
```

MoveIt action:

```bash
ros2 action list | grep move_action
```

## systemd

System service template:

- `systemd/system/silverhand-ws-gateway@.service`

Установка:

```bash
sudo install -Dm644 /home/r/silver_ws/src/silverhand_ws_gateway/systemd/system/silverhand-ws-gateway@.service /etc/systemd/system/silverhand-ws-gateway@.service
sudo systemctl daemon-reload
```

Инстансы:

```bash
sudo systemctl enable --now silverhand-ws-gateway@arm_mock.service
sudo systemctl enable --now silverhand-ws-gateway@arm_ros.service
sudo systemctl enable --now silverhand-ws-gateway@arm_moveit.service
sudo systemctl enable --now silverhand-ws-gateway@rover_mock.service
sudo systemctl enable --now silverhand-ws-gateway@rover_ros.service
```

Логи и статус:

```bash
systemctl status silverhand-ws-gateway@arm_moveit.service
journalctl -u silverhand-ws-gateway@arm_moveit.service -f
sudo systemctl restart silverhand-ws-gateway@arm_moveit.service
```
