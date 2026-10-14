# Практическая работа 2. Выполнил Кузнецов Ярослав

Вариант 7. Пакет `webots_digits`: площадка 6 × 6 м с двумя статическими препятствиями в форме цифр `0` и `7` и робот TurtleBot3 Burger.

## Состав пакета

- `webots_digits/worlds/my_world.wbt` — сцена: площадка, цифры, робот
- `webots_digits/protos/Digit_0.proto`, `Digit_7.proto` — цифры из брусков (без `Physics`, с `boundingObject`)
- `webots_digits/launch/spawn_robot.launch.py` — запуск Webots, робота, драйвера и контроллеров

## Сборка

В контейнере курса (папка `practices/` подключена как `~/practices_ws/src`):

```bash
cd ~/practices_ws
colcon build --base-paths src/practice-02 --symlink-install
source install/setup.zsh
```

В bash вместо `setup.zsh` используйте `setup.bash`.

## Запуск симуляции

```bash
ros2 launch webots_digits spawn_robot.launch.py
```

## Движение по окружности

В другом терминале (в том же контейнере, после `source install/setup.zsh`):

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/TwistStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'}, twist: {linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}}"
```
