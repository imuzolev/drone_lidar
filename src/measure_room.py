"""
Скрипт для измерения размеров комнаты.
Дрон взлетает, сканирует лидаром 360 градусов и показывает
расстояния до стен во всех направлениях.

Использование:
    python src/measure_room.py          # Реальный AirSim
    python src/measure_room.py --mock   # Режим имитации
"""

import numpy as np
import math
import sys
import argparse
import time

try:
    import airsim
except ImportError:
    airsim = None

import mock_airsim


def measure_room(use_mock=False):
    # Подключение
    if not use_mock and airsim is not None:
        try:
            print("[ИНФО] Подключение к AirSim...")
            client = airsim.MultirotorClient()
            client.confirmConnection()
            print("[ИНФО] Подключено к AirSim.")
            is_mock = False
        except Exception as e:
            print(f"[ОШИБКА] Не удалось подключиться к AirSim: {e}")
            print("[ИНФО] Переключаюсь на режим имитации.")
            client = mock_airsim.MultirotorClient()
            client.confirmConnection()
            is_mock = True
    else:
        client = mock_airsim.MultirotorClient()
        client.confirmConnection()
        is_mock = True

    # Взлёт
    client.enableApiControl(True)
    client.armDisarm(True)
    print("[ИНФО] Взлёт...")
    client.takeoffAsync().join()

    # Подняться на 3 метра
    state = client.getMultirotorState()
    pos = state.kinematics_estimated.position
    start_x, start_y = pos.x_val, pos.y_val
    print(f"[ИНФО] Стартовая позиция: ({start_x:.2f}, {start_y:.2f})")

    client.moveToPositionAsync(start_x, start_y, -3.0, 2.0).join()
    print("[ИНФО] На высоте 3м. Сканирование лидаром...")

    # Получить данные лидара
    try:
        lidar_data = client.getLidarData()
    except Exception as e:
        print(f"[ОШИБКА] Лидар не доступен: {e}")
        print("[ПОДСКАЗКА] Проверьте настройки лидара в settings.json AirSim.")
        client.landAsync().join()
        client.armDisarm(False)
        client.enableApiControl(False)
        return

    if len(lidar_data.point_cloud) < 3:
        print("[ОШИБКА] Лидар не вернул данных. Проверьте настройки.")
        client.landAsync().join()
        client.armDisarm(False)
        client.enableApiControl(False)
        return

    points = np.array(lidar_data.point_cloud, dtype=np.float32).reshape(-1, 3)
    print(f"[ИНФО] Получено {len(points)} точек от лидара.")

    # Получить текущий yaw для преобразования координат
    orient = client.getMultirotorState().kinematics_estimated.orientation
    siny = 2.0 * (orient.w_val * orient.z_val + orient.x_val * orient.y_val)
    cosy = 1.0 - 2.0 * (orient.y_val ** 2 + orient.z_val ** 2)
    yaw = math.atan2(siny, cosy)

    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)

    # Преобразовать из системы координат дрона в мировую
    world_x = start_x + points[:, 0] * cos_y - points[:, 1] * sin_y
    world_y = start_y + points[:, 0] * sin_y + points[:, 1] * cos_y

    # Найти крайние точки (расстояния до стен)
    x_min_world = np.min(world_x)
    x_max_world = np.max(world_x)
    y_min_world = np.min(world_y)
    y_max_world = np.max(world_y)

    # Расстояния от дрона до стен
    dist_forward  = x_max_world - start_x   # +X
    dist_backward = start_x - x_min_world   # -X
    dist_right    = y_max_world - start_y    # +Y
    dist_left     = start_y - y_min_world    # -Y

    room_width_x  = x_max_world - x_min_world
    room_width_y  = y_max_world - y_min_world

    # Вывод результатов
    print("")
    print("=" * 55)
    print("  РЕЗУЛЬТАТЫ ИЗМЕРЕНИЯ КОМНАТЫ")
    print("=" * 55)
    print(f"  Позиция дрона:   ({start_x:.2f}, {start_y:.2f})")
    print("")
    print(f"  Размер комнаты:  {room_width_x:.1f} м  x  {room_width_y:.1f} м")
    print(f"  Площадь:         {room_width_x * room_width_y:.0f} кв.м")
    print("")
    print(f"  Расстояния от дрона до стен:")
    print(f"    Вперёд  (+X): {dist_forward:.1f} м")
    print(f"    Назад   (-X): {dist_backward:.1f} м")
    print(f"    Вправо  (+Y): {dist_right:.1f} м")
    print(f"    Влево   (-Y): {dist_left:.1f} м")
    print("")
    print(f"  Координаты стен (мировые):")
    print(f"    X: [{x_min_world:.1f} ... {x_max_world:.1f}]")
    print(f"    Y: [{y_min_world:.1f} ... {y_max_world:.1f}]")
    print("=" * 55)
    print("")
    print(f"  Для запуска исследования используйте:")
    print(f"    python src/autonomous_exploration.py "
          f"--room-size {max(room_width_x, room_width_y):.0f}")
    print("")

    # Посадка
    print("[ИНФО] Посадка...")
    client.landAsync().join()
    client.armDisarm(False)
    client.enableApiControl(False)
    print("[ИНФО] Готово.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Измерение размеров комнаты")
    parser.add_argument("--mock", action="store_true",
                        help="Использовать имитацию AirSim")
    args = parser.parse_args()
    measure_room(use_mock=args.mock)
