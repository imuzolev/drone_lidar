"""
Тест подключения к Colosseum/AirSim для управления дроном.

Перед запуском убедитесь, что:
1. Unreal Engine проект Blocks запущен
2. Симуляция началась (нажат Play в редакторе)

Запуск:
    python src/test_colosseum.py
"""

import airsim
import time
import sys

def test_connection():
    """Тест подключения к симулятору."""
    print("=" * 60)
    print("Тест подключения к Colosseum/AirSim")
    print("=" * 60)
    
    try:
        # Создаём клиента для мультиротора (дрона)
        client = airsim.MultirotorClient()
        print("[INFO] Клиент создан")
        
        # Пытаемся подключиться
        print("[INFO] Подключение к симулятору...")
        client.confirmConnection()
        print("[SUCCESS] Подключение установлено!")
        
        # Получаем информацию о состоянии
        state = client.getMultirotorState()
        print(f"\n[INFO] Состояние дрона:")
        print(f"  - Позиция: x={state.kinematics_estimated.position.x_val:.2f}, "
              f"y={state.kinematics_estimated.position.y_val:.2f}, "
              f"z={state.kinematics_estimated.position.z_val:.2f}")
        print(f"  - Landed: {state.landed_state}")
        
        return True
        
    except Exception as e:
        print(f"[ERROR] Ошибка подключения: {e}")
        print("\nУбедитесь, что:")
        print("  1. Unreal проект Blocks запущен")
        print("  2. Нажата кнопка Play в редакторе")
        print("  3. Плагин AirSim включен (не ProjectAirSim)")
        return False


def test_basic_flight():
    """Тест базового полёта."""
    print("\n" + "=" * 60)
    print("Тест базового полёта")
    print("=" * 60)
    
    try:
        client = airsim.MultirotorClient()
        client.confirmConnection()
        
        # Включаем API управление
        print("[INFO] Включение API управления...")
        client.enableApiControl(True)
        
        # Армируем дрон
        print("[INFO] Армирование дрона...")
        client.armDisarm(True)
        
        # Взлёт
        print("[INFO] Взлёт на высоту 5 метров...")
        client.takeoffAsync().join()
        
        print("[INFO] Полёт вперёд на 5 метров...")
        client.moveToPositionAsync(5, 0, -5, 5).join()
        
        print("[INFO] Зависание на 3 секунды...")
        client.hoverAsync().join()
        time.sleep(3)
        
        # Возврат и посадка
        print("[INFO] Возврат на исходную позицию...")
        client.moveToPositionAsync(0, 0, -5, 5).join()
        
        print("[INFO] Посадка...")
        client.landAsync().join()
        
        # Отключаем API управление
        client.armDisarm(False)
        client.enableApiControl(False)
        
        print("[SUCCESS] Тестовый полёт завершён успешно!")
        return True
        
    except Exception as e:
        print(f"[ERROR] Ошибка полёта: {e}")
        return False


def test_camera():
    """Тест получения изображения с камеры."""
    print("\n" + "=" * 60)
    print("Тест камеры дрона")
    print("=" * 60)
    
    try:
        client = airsim.MultirotorClient()
        client.confirmConnection()
        
        # Получаем изображение
        print("[INFO] Получение изображения с фронтальной камеры...")
        responses = client.simGetImages([
            airsim.ImageRequest("front_center", airsim.ImageType.Scene, False, False)
        ])
        
        if responses and len(responses) > 0:
            response = responses[0]
            print(f"[SUCCESS] Изображение получено!")
            print(f"  - Размер: {response.width}x{response.height}")
            print(f"  - Данные: {len(response.image_data_uint8)} байт")
            return True
        else:
            print("[WARNING] Изображение не получено")
            return False
            
    except Exception as e:
        print(f"[ERROR] Ошибка камеры: {e}")
        return False


def main():
    """Главная функция."""
    print("\n" + "=" * 60)
    print("Colosseum/AirSim Python API Test")
    print("=" * 60 + "\n")
    
    # Тест подключения
    if not test_connection():
        sys.exit(1)
    
    # Спрашиваем пользователя о продолжении
    print("\n" + "-" * 60)
    answer = input("Запустить тест полёта? (y/n): ").strip().lower()
    
    if answer == 'y':
        test_basic_flight()
        test_camera()
    
    print("\n" + "=" * 60)
    print("Тестирование завершено")
    print("=" * 60)


if __name__ == "__main__":
    main()
