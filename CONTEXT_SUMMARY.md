# Проект AADNS - Дрон в Colosseum/AirSim

## Статус: РАБОТАЕТ ✓ (06.02.2026)

### Что работает
1. **Unreal Engine 5.2 симуляция** - дрон летает
2. **Python Colosseum клиент** - API работает
3. **Полёт по квадрату** - 5x5м на высоте 10м - УСПЕШНО
4. **GUI main.py** - панель управления

### Быстрый старт

```powershell
# 1. Открыть Unreal проект и нажать Play
Start-Process "F:\test_dron_1\unreal\Blocks\Blocks.uproject"

# 2. Активировать venv и запустить
cd f:\test_dron_1
.\venv\Scripts\Activate.ps1
python src\test_square_flight.py   # простой тест
# или
python src\main.py                  # GUI
```

### ВАЖНО: Python пакет Colosseum

**НЕ использовать `pip install airsim`!** Нужен клиент от Colosseum:

```powershell
pip uninstall airsim -y
pip install git+https://github.com/CodexLabsLLC/Colosseum.git#subdirectory=PythonClient
pip install "numpy<2.0"
```

### Структура проекта

```
f:\test_dron_1\
├── src/
│   ├── main.py                  # GUI с полётом по квадрату
│   ├── test_square_flight.py    # Простой тест квадрата (без GUI)
│   ├── test_colosseum.py        # Тест подключения
│   ├── decision_maker.py        # Нейросеть принятия решений
│   ├── sensor.py, obstacle.py   # Датчики и препятствия
│   └── navigation.py            # Навигация (Kalman)
├── unreal/Blocks/               # UE5.2 проект с AirSim плагином
├── venv/                        # Python окружение
└── CONTEXT_SUMMARY.md           # Этот файл
```

### Конфигурация

**AirSim settings.json**: `C:\Users\muzol\Documents\AirSim\settings.json`
```json
{
  "SettingsVersion": 1.2,
  "SimMode": "Multirotor",
  "Vehicles": {
    "Drone1": {
      "VehicleType": "SimpleFlight",
      "X": 0, "Y": 0, "Z": 0
    }
  }
}
```

### Параметры полёта (main.py)

В методе `start_operation()`:
```python
SQUARE_SIDE = 5      # метров - сторона квадрата
FLIGHT_ALTITUDE = 10 # метров - высота полёта
FLIGHT_SPEED = 3     # м/с - скорость
```

### API примеры

```python
import airsim

client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)
client.takeoffAsync().join()

# Движение (NED: z отрицательный = вверх)
client.moveToPositionAsync(x, y, -altitude, speed).join()

# Позиция
state = client.getMultirotorState()
pos = state.kinematics_estimated.position
print(f"X={pos.x_val}, Y={pos.y_val}, Alt={-pos.z_val}m")

client.landAsync().join()
```

### Ссылки

- Colosseum: https://github.com/CodexLabsLLC/Colosseum
- Docs: https://codexlabsllc.github.io/Colosseum/
