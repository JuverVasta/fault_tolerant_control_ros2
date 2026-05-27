# Fault-Tolerant Control — нативный порт на ROS 2 Humble

Портирование проекта [uzh-rpg/fault_tolerant_control](https://github.com/uzh-rpg/fault_tolerant_control)
с ROS 1 на ROS 2 Humble. Полностью нативный: без Docker, без ros1_bridge.
Симулятор RotorS заменён собственным Gazebo Classic плагином.

## Что делает проект

Квадрокоптер летает в симуляторе Gazebo. По команде один из моторов
«отключается». Благодаря методу Sun & Scaramuzza дрон не падает, а переходит
в управляемое вращение вокруг вертикальной оси и продолжает удерживать и
менять позицию.

## Структура

| Пакет | Назначение | Происхождение |
|-------|-----------|---------------|
| `quad_msgs` | Определения ROS 2 сообщений | портирован с ROS 1 |
| `ftc_ctrl` | Контроллер + узел-интерфейс симуляции | портирован с ROS 1 |
| `ftc_gazebo_plugin` | Gazebo-плагин динамики моторов | новый (замена RotorS) |
| `ftc_description` | URDF-модель квадрокоптера | новый |
| `ftc_bringup` | Launch-файлы, конфиги, мир | новый |

## Архитектура потока данных

```
   Gazebo (физика)
        │  ground_truth/odometry  (nav_msgs/Odometry)
        ▼
   state_interface  ──── state_est ────►  ftc_ctrl  (контроллер)
        ▲                                    │
        │  command/motor_speed               │ control_command
        │  (quad_msgs/Actuators)             │ (quad_msgs/ControlCommand)
        └──────────── state_interface ◄──────┘
        │
        ▼
   ftc_gazebo_plugin (применяет тягу к моделям моторов)
```

## Быстрый старт

```bash
# 1. Собрать
cd ftc_ros2_ws
colcon build --symlink-install
source install/setup.bash

# 2. Запустить симуляцию (откроется Gazebo)
ros2 launch ftc_bringup simulation.launch.py

# 3. В новом терминале — взлёт
source install/setup.bash
ros2 run ftc_ctrl takeoff.sh

# 4. Полёт по точкам
ros2 run ftc_ctrl goto.sh 1.0 0.0 1.5

# 5. Отказ мотора — дрон закрутится и не упадёт
ros2 run ftc_ctrl fail_motor.sh
```

Подробная пошаговая инструкция со всеми проверками — в файле
`ИНСТРУКЦИЯ.docx`.

## Важно про реальный дрон и ArduPilot

Этот проект — симуляция (2-й семестр). Для реального дрона потребуется
отдельная работа: алгоритм требует прямого управления тягой каждого мотора,
что несовместимо со стандартным режимом ArduPilot. Способы интеграции
(ArduPilot Lua scripting, SITL, прямое управление ESC) описаны в Word-документе
в разделе «Реальный дрон».

## Лицензия

Контроллер и сообщения наследуют GPL-3.0 от оригинального проекта
uzh-rpg/fault_tolerant_control. Новые пакеты (`ftc_gazebo_plugin`,
`ftc_description`, `ftc_bringup`) — GPL-3.0.
