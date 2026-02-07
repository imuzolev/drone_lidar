"""
Тест полёта по квадрату - простой скрипт без GUI.
Запуск: python src/test_square_flight.py
"""

import airsim
import time

def test_square_flight():
    """Тестовый полёт по квадрату 5x5м на высоте 10м."""
    
    # Параметры
    SIDE = 5  # метров
    ALTITUDE = 10  # метров  
    SPEED = 3  # м/с
    
    print("=" * 60)
    print("ТЕСТ ПОЛЁТА ПО КВАДРАТУ")
    print(f"Сторона: {SIDE}м, Высота: {ALTITUDE}м, Скорость: {SPEED}м/с")
    print("=" * 60)
    
    # Подключение
    print("\n[1] Подключение к симулятору...")
    client = airsim.MultirotorClient()
    client.confirmConnection()
    print("[OK] Подключено!")
    
    # Включаем управление
    print("\n[2] Включение API управления...")
    client.enableApiControl(True)
    client.armDisarm(True)
    print("[OK] API управление включено")
    
    # Взлёт
    print("\n[3] Взлёт...")
    client.takeoffAsync().join()
    print("[OK] Дрон в воздухе")
    
    # Получаем начальную позицию
    state = client.getMultirotorState()
    start_x = state.kinematics_estimated.position.x_val
    start_y = state.kinematics_estimated.position.y_val
    print(f"[INFO] Начальная позиция: X={start_x:.1f}, Y={start_y:.1f}")
    
    # Высота в NED (отрицательная = вверх)
    target_z = -ALTITUDE
    
    # Waypoints квадрата
    waypoints = [
        (start_x, start_y, target_z, "Подъём на высоту"),
        (start_x + SIDE, start_y, target_z, "Вперёд (сторона 1)"),
        (start_x + SIDE, start_y + SIDE, target_z, "Вправо (сторона 2)"),
        (start_x, start_y + SIDE, target_z, "Назад (сторона 3)"),
        (start_x, start_y, target_z, "Влево (сторона 4) - возврат"),
    ]
    
    print("\n[4] Полёт по квадрату...")
    print("-" * 60)
    
    for i, (x, y, z, desc) in enumerate(waypoints, 1):
        print(f"\n>>> Шаг {i}/5: {desc}")
        print(f"    Цель: X={x:.1f}, Y={y:.1f}, Z={-z:.1f}м")
        
        # Движение к точке
        client.moveToPositionAsync(x, y, z, SPEED).join()
        
        # Проверяем реальную позицию
        state = client.getMultirotorState()
        pos = state.kinematics_estimated.position
        print(f"    Факт: X={pos.x_val:.1f}, Y={pos.y_val:.1f}, Z={-pos.z_val:.1f}м")
        
        time.sleep(1)  # Пауза на углу
    
    print("\n" + "-" * 60)
    print("[5] Квадрат завершён! Посадка...")
    
    # Посадка
    client.landAsync().join()
    client.armDisarm(False)
    client.enableApiControl(False)
    
    print("[OK] Дрон приземлился")
    print("\n" + "=" * 60)
    print("ТЕСТ ЗАВЕРШЁН УСПЕШНО!")
    print("=" * 60)


if __name__ == "__main__":
    try:
        test_square_flight()
    except Exception as e:
        print(f"\n[ERROR] {e}")
        import traceback
        traceback.print_exc()
