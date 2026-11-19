# Практическая работа 4. Локализация по известной карте

Вариант 7. Пакет `localization_practice`: TurtleBot3 Burger, площадка 6 × 6 м и препятствия в форме цифр `0` и `7` из практической работы 2.

## Текущее состояние

Пакет запускает сцену с цифрами, карту, AMCL, супервизор истинной позы, движение по кругу и сбор данных эксперимента.

## Состав пакета

- `localization_practice/worlds/digits.wbt` - сцена с цифрами.
- `localization_practice/protos/Digit_0.proto`, `Digit_7.proto` - статические препятствия из второй практики.
- `localization_practice/protos/TurtleBot3Burger.proto` - локальное описание робота из примера курса.
- `localization_practice/maps/digits.yaml` и `digits.png` - карта сцены с цифрами.
- `localization_practice/localization_practice/ground_truth.py` - публикация `/clock` и динамического TF `map → base_link_gt`.
- `localization_practice/localization_practice/circle_motion.py` - публикация команд движения по окружности.
- `localization_practice/localization_practice/collect_localization.py` - запись поз из TF2, расчёт ошибок и построение графиков.
- `localization_practice/urdf/ground_truth_supervisor.urdf` - подключение супервизора.
- `localization_practice/config/amcl.yaml` - исходные настройки AMCL из примера.
- `localization_practice/launch/spawn_robot.launch.py` - запуск сцены и локализации.
- `localization_practice/rviz/config.rviz` - карта, сканы, частицы и системы координат в RViz.

Сцены `one_box`, `three_boxes` и соответствующие карты сохранены из учебного примера для проверки запуска. Они не заменяют сцену и карту варианта 7.

## Сборка

В контейнере курса папка `practices/` подключена как `~/practices_ws/src`.

```bash
cd ~/practices_ws
colcon build --base-paths src/practice-04 --symlink-install
source install/setup.zsh
```

В bash используйте `source install/setup.bash`. После первой сборки подключите окружение в каждом уже открытом терминале.

## Просмотр сцены с цифрами

Сцену можно запустить без локализации и RViz:

```bash
ros2 launch localization_practice spawn_robot.launch.py \
  use_localization:=false use_rviz:=false
```

Стартовая поза робота в `digits.wbt`: `x = 2.0 м`, `y = 0.0 м`, `yaw = pi/2 рад`. Высота цифр - `0.3 м`. Арена и препятствия находятся в глобальной системе координат сцены с осью Z вверх (`ENU`).

## Подключение карты

Карта варианта загружается из `localization_practice/maps/digits.yaml`. Один пиксель соответствует 1 см, нижний левый угол карты находится в точке `(-3.03, -3.03)` системы координат Webots. Карта включает стены толщиной 3 см и сечения цифр на высоте сканирования. Чёрные клетки заняты, белые свободны.

После сборки пакет запускается с этой картой по умолчанию:

```bash
ros2 launch localization_practice spawn_robot.launch.py
```

## Движение по окружности

В сцене `digits` узел `circle_motion` запускается после активации колёсного контроллера. Если включена локализация, робот остаётся на месте до первой оценки AMCL. Затем узел публикует `TwistStamped` в `/cmd_vel` с частотой 10 Гц: линейная скорость 0,20 м/с, угловая 0,10 рад/с. Расчётный радиус равен 2 м. Робот стартует в точке `(2, 0)` вдоль оси Y и поворачивает против часовой стрелки, объезжая обе цифры.

В проверочном запуске по истинной позе робот прошёл три полных круга. Минимальное расстояние от центра робота до стены составило 0,97 м, до цифр - 0,48 м.

## Системы координат

Супервизор публикует `map → base_link_gt`, колёсный контроллер - `odom → base_link`, AMCL - `map → odom`. Статический `map → odom` не запускается.

## Сбор данных

После сборки и подключения окружения запустите из папки практики:

```bash
cd ~/practices_ws/src/practice-04
ros2 launch localization_practice spawn_robot.launch.py output_prefix:=results/experiment
```

`t_0` - метка времени первого сообщения `/amcl_pose` после инициализации. Сборщик использует это сообщение только для отсчёта времени: обе позы берёт из TF2 на одной метке времени. Первое измерение может быть позже `t₀`, когда доступны обе ветки TF2. Не меняйте `use_sim_time` при проведении экспериментов.

По окончании сбора в `results` сохраняются CSV с позами и ошибками, PNG с тремя графиками и TXT с временем сходимости либо сообщением, что она не подтверждена. Время сходимости - начало первого интервала, в котором обе ошибки выдержали пороги в течение заданного времени. После сообщения о сохранении остановите запуск через Ctrl+C.

## Эксперименты

Во всех запусках использовались сцена `digits`, одна и та же карта и стартовая поза `(2 м, 0 м, π/2 рад)`. Команды движения одинаковы. Длительность сбора - 200 с симуляции, пороги - 0,15 м и 0,12 рад, интервал подтверждения - 10 с. Минимум частиц - 500 в обоих режимах. В пробном запуске с точной начальной позой ошибки после начала движения не превышали примерно 0,04 м и 0,04 рад; выбранные пороги оставляют запас для колебаний оценки.

Малая ошибка начальной догадки: `(+0.15 м, +0.15 м, +0.2 рад)`; большая: `(+0.5 м, +0.5 м, +1.0 рад)`. В глобальном режиме начальная догадка не задаётся.

Из папки практики выполните команду, добавив параметры одной строки из таблицы результатов. Для каждого эксперимента запускайте симуляцию заново:

```bash
ros2 launch localization_practice spawn_robot.launch.py \
  scene:=digits use_localization:=true use_sim_time:=true mode:=fast use_rviz:=false \
  duration:=200.0 position_threshold:=0.15 yaw_threshold:=0.12 \
  confirmation_duration:=10.0 min_particles:=500
```

| График | Параметры запуска | Время сходимости, с |
| --- | --- | ---: |
| [global_1000](results/global_1000.png) | `localization_mode:=global max_particles:=1000 output_prefix:=results/global_1000` | не подтверждена |
| [global_15000](results/global_15000.png) | `localization_mode:=global max_particles:=15000 output_prefix:=results/global_15000` | 27,776 |
| [initial_small_1000](results/initial_small_1000.png) | `localization_mode:=initial max_particles:=1000 error_x:=0.15 error_y:=0.15 error_yaw:=0.2 output_prefix:=results/initial_small_1000` | 8,832 |
| [initial_small_15000](results/initial_small_15000.png) | `localization_mode:=initial max_particles:=15000 error_x:=0.15 error_y:=0.15 error_yaw:=0.2 output_prefix:=results/initial_small_15000` | 7,872 |
| [initial_large_1000](results/initial_large_1000.png) | `localization_mode:=initial max_particles:=1000 error_x:=0.5 error_y:=0.5 error_yaw:=1.0 output_prefix:=results/initial_large_1000` | 18,496 |
| [initial_large_15000](results/initial_large_15000.png) | `localization_mode:=initial max_particles:=15000 error_x:=0.5 error_y:=0.5 error_yaw:=1.0 output_prefix:=results/initial_large_15000` | 15,616 |

При малой начальной ошибке AMCL сошёлся быстрее, чем при большой, для обоих лимитов частиц. При глобальной инициализации с лимитом 1 000 сходимость не подтвердилась за 200 с: в последние 10 с медианные ошибки составили 2,80 м и 1,57 рад. С лимитом 15 000 глобальная оценка сошлась за 27,776 с. В этой серии более высокий лимит дал сходимость во всех трёх сценариях, но по одному запуску на комбинацию нельзя судить о повторяемости результата: AMCL использует случайную выборку, а лимит не равен фактическому числу частиц.

## Источники

Основа пакета - `examples/example_webots_localization` из репозитория курса. Сцена с цифрами и их PROTO-файлы перенесены из `webots_digits` в ветке `practice-02`.
