# silverhand_arm_ws_gateway

Лёгкий robot-side websocket gateway для SilverHand.

Что уже есть:
- websocket server под согласованный протокол GUI
- `mock`-adapter для теста сети без MoveIt
- каркас `ros`-adapter для дальнейшей привязки к топикам, action и service

## Зависимости

Для runtime websocket server нужен пакет `websockets`:

```bash
sudo apt install python3-websockets
```

или через `rosdep`, если он у вас заведён в окружении.

## Сборка

```bash
cd /home/r/silver_ws
colcon build --packages-select silverhand_arm_ws_gateway
source install/setup.bash
```

## Запуск mock gateway

```bash
ros2 run silverhand_arm_ws_gateway gateway --mode mock --host 0.0.0.0 --port 8765
```

или через launch:

```bash
ros2 launch silverhand_arm_ws_gateway mock_gateway.launch.py
```

Если `ros2 run` в overlay ведёт себя капризно, есть проверенный helper-скрипт:

```bash
cd /home/r/silver_ws/src/silverhand_arm_ws_gateway
./scripts/run_mock_gateway.sh
```

## Smoke test mock gateway

```bash
cd /home/r/silver_ws/src/silverhand_arm_ws_gateway
python3 scripts/mock_smoke_test.py --url ws://127.0.0.1:8765
```

## Запуск ros gateway

Пока это scaffold под будущую ROS-интеграцию:

```bash
ros2 run silverhand_arm_ws_gateway gateway --mode ros --host 0.0.0.0 --port 8765
```

## Что умеет mock

- `hello` / `hello_ack`
- `ping` / `pong`
- `set_joint_goal`
- `set_pose_goal` с грубым mock-преобразованием pose -> joints
- `plan`
- `execute`
- `stop`
- `estop`
- `reset_estop`
- `joint_state`
- `planning_state`
- `execution_state`
- `fault_state`
