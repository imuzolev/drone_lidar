# AADNS - Advanced Autonomous Drone Navigation System

Автономная система навигации дрона с симуляцией в Unreal Engine 5.2 через Colosseum (форк AirSim).

## Возможности

- Полёт по заданным маршрутам (квадрат, змейка)
- Исследование карты с обходом препятствий
- Визуализация траектории красной линией в UE5 в реальном времени
- Обнаружение препятствий на расстоянии 2м
- Автоматическое избегание: остановка → отход → поворот влево → продолжение
- GUI панель управления (Tkinter)
- Нейросеть принятия решений (PyTorch)
- Навигация с фильтром Калмана
- Шифрование связи, управление энергией, рой дронов

## Быстрая установка

### Вариант 1: Автоматическая (рекомендуется)

```powershell
# PowerShell
.\setup.ps1
```

или

```batch
# CMD
setup.bat
```

### Вариант 2: Ручная

```powershell
# 1. Создать виртуальное окружение
python -m venv venv
.\venv\Scripts\Activate.ps1

# 2. Установить зависимости
pip install -r requirements.txt

# 3. ВАЖНО: Установить Colosseum клиент (НЕ pip install airsim!)
pip uninstall airsim -y
pip install git+https://github.com/CodexLabsLLC/Colosseum.git#subdirectory=PythonClient

# 4. Убедиться что numpy < 2.0
pip install "numpy<2.0"
```

## Настройка Unreal Engine

### 1. Установить Unreal Engine 5.2
- Скачать Epic Games Launcher: https://www.unrealengine.com/
- Установить UE 5.2.x

### 2. Собрать Colosseum плагин
```powershell
git clone https://github.com/CodexLabsLLC/Colosseum.git
cd Colosseum
# Следовать инструкциям: https://codexlabsllc.github.io/Colosseum/build_windows/
```

### 3. Создать проект
- Создать UE5 проект типа "Blocks" с подключённым AirSim плагином
- Или использовать готовый проект из `unreal/Blocks/` (не включён в репозиторий из-за размера)

### 4. Настроить AirSim

Создать файл `C:\Users\<ВАШ_ПОЛЬЗОВАТЕЛЬ>\Documents\AirSim\settings.json`:

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

## Запуск

```powershell
# 1. Открыть Unreal Engine проект и нажать Play

# 2. Активировать venv
.\venv\Scripts\Activate.ps1

# 3. Запустить GUI
python src\main.py

# Или простой тест полёта по квадрату
python src\test_square_flight.py
```

В GUI нажмите **Start** для начала полёта.

## Параметры полёта (src/main.py)

| Параметр | Значение | Описание |
|----------|----------|----------|
| `FLIGHT_ALTITUDE` | 3 м | Высота полёта |
| `FLIGHT_SPEED` | 1.5 м/с | Скорость |
| `MAP_SIZE_X` | 30 м | Область исследования X |
| `MAP_SIZE_Y` | 30 м | Область исследования Y |
| `LINE_SPACING` | 5 м | Шаг между линиями сканирования |
| Obstacle threshold | 2 м | Дистанция обнаружения препятствий |

## Структура проекта

```
├── src/
│   ├── main.py                  # GUI + контроллер дрона + исследование карты
│   ├── test_square_flight.py    # Простой тест полёта по квадрату
│   ├── test_colosseum.py        # Тест подключения к симулятору
│   ├── decision_maker.py        # Нейросеть принятия решений (PyTorch)
│   ├── decision_net.py          # Архитектура нейросети
│   ├── navigation.py            # Навигация (фильтр Калмана)
│   ├── sensor.py                # Модуль датчиков
│   ├── obstacle.py              # Обнаружение препятствий
│   ├── flight_plan.py           # Планирование маршрута (A*)
│   ├── frequency_hopper.py      # Частотный хоппинг связи
│   ├── drone_encryption.py      # Шифрование
│   ├── energy_management.py     # Управление энергией
│   ├── emergency.py             # Аварийные процедуры
│   ├── drone_swarm.py           # Управление роем
│   ├── weather_interaction.py   # Взаимодействие с погодой
│   ├── user_interface.py        # Tkinter GUI панель
│   └── exceptions.py            # Пользовательские исключения
├── unreal/                      # UE5 проект (не в репозитории)
├── requirements.txt             # Python зависимости
├── setup.ps1                    # Автоустановка (PowerShell)
├── setup.bat                    # Автоустановка (CMD)
├── CONTEXT_SUMMARY.md           # Контекст проекта
├── .cursorrules                 # Правила для Cursor AI
└── .gitignore
```

## API Reference (AirSim/Colosseum)

```python
import airsim

client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)
client.takeoffAsync().join()

# Движение (NED: z отрицательный = вверх)
client.moveToPositionAsync(x, y, -altitude, speed).join()

# Получить позицию
state = client.getMultirotorState()
pos = state.kinematics_estimated.position

# Обнаружение столкновений
collision = client.simGetCollisionInfo()
if collision.has_collided: ...

# Визуализация траектории
client.simPlotLineStrip([p1, p2], color_rgba=[1,0,0,1], thickness=8, is_persistent=True)

client.landAsync().join()
```

## Требования

- **Python** 3.11+
- **Unreal Engine** 5.2
- **Colosseum** (форк AirSim от CodexLabsLLC)
- **Windows** 10/11
- **GPU** с поддержкой DirectX 11 (для UE5)

## Ссылки

- [Colosseum GitHub](https://github.com/CodexLabsLLC/Colosseum)
- [Colosseum Docs](https://codexlabsllc.github.io/Colosseum/)
- [AirSim API Reference](https://codexlabsllc.github.io/Colosseum/api_docs/html/)
