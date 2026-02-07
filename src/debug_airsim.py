"""
Диагностический скрипт для отладки проблемы с движением дрона в AirSim.
"""

import airsim
import time

def debug_move():
    """Тест движения с подробной диагностикой."""
    print("=" * 60)
    print("Диагностика движения AirSim")
    print("=" * 60)
    
    try:
        # Подключение
        client = airsim.MultirotorClient()
        client.confirmConnection()
        print("[OK] Подключение установлено")
        
        # Получаем состояние
        print("\n[INFO] Получение состояния дрона...")
        state = client.getMultirotorState()
        print(f"  Type of state: {type(state)}")
        print(f"  State: {state}")
        
        # Проверяем kinematics_estimated
        print("\n[INFO] Проверка kinematics_estimated...")
        kin = state.kinematics_estimated
        print(f"  Type of kinematics: {type(kin)}")
        
        # Проверяем position
        print("\n[INFO] Проверка position...")
        pos = kin.position
        print(f"  Type of position: {type(pos)}")
        print(f"  Position object: {pos}")
        
        # Получаем координаты
        print("\n[INFO] Получение координат...")
        try:
            x = pos.x_val
            y = pos.y_val
            z = pos.z_val
            print(f"  x={x} (type: {type(x)})")
            print(f"  y={y} (type: {type(y)})")
            print(f"  z={z} (type: {type(z)})")
        except AttributeError as e:
            print(f"  [ERROR] AttributeError: {e}")
            print(f"  Trying alternative access methods...")
            
            # Попробуем как dict
            if hasattr(pos, 'items'):
                print(f"  pos.items(): {dict(pos.items())}")
            elif isinstance(pos, (list, tuple)):
                print(f"  pos as list: {list(pos)}")
                x, y, z = pos[0], pos[1], pos[2]
            elif hasattr(pos, '__dict__'):
                print(f"  pos.__dict__: {pos.__dict__}")
            else:
                print(f"  dir(pos): {dir(pos)}")
        
        # Взлёт
        print("\n[INFO] Взлёт...")
        client.enableApiControl(True)
        client.armDisarm(True)
        client.takeoffAsync().join()
        print("[OK] Дрон взлетел")
        
        # Получаем позицию после взлёта
        print("\n[INFO] Позиция после взлёта...")
        state = client.getMultirotorState()
        pos = state.kinematics_estimated.position
        x, y, z = float(pos.x_val), float(pos.y_val), float(pos.z_val)
        print(f"  Позиция: ({x:.2f}, {y:.2f}, {z:.2f})")
        
        # Тест движения
        print("\n[INFO] Тест moveToPositionAsync...")
        new_x = float(x + 2.0)
        new_y = float(y)
        new_z = float(z)
        velocity = 3.0
        
        print(f"  Параметры: x={new_x}, y={new_y}, z={new_z}, velocity={velocity}")
        print(f"  Типы: x={type(new_x)}, y={type(new_y)}, z={type(new_z)}, v={type(velocity)}")
        
        # Проверим сигнатуру функции
        print(f"\n[INFO] Сигнатура moveToPositionAsync:")
        import inspect
        try:
            sig = inspect.signature(client.moveToPositionAsync)
            print(f"  {sig}")
        except:
            print("  Не удалось получить сигнатуру")
        
        # Вызов с явными float
        print("\n[INFO] Вызов moveToPositionAsync...")
        try:
            result = client.moveToPositionAsync(new_x, new_y, new_z, velocity)
            print(f"  Result type: {type(result)}")
            result.join()
            print("[OK] Движение успешно!")
        except Exception as e:
            print(f"[ERROR] Ошибка движения: {e}")
            print(f"  Exception type: {type(e)}")
            import traceback
            traceback.print_exc()
        
        # Посадка
        print("\n[INFO] Посадка...")
        client.landAsync().join()
        client.armDisarm(False)
        client.enableApiControl(False)
        print("[OK] Дрон приземлился")
        
    except Exception as e:
        print(f"[ERROR] {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    debug_move()
