# Настройка AirSim для работы с Lidar

Для корректной работы автономной навигации необходимо добавить сенсор Lidar в конфигурацию AirSim.

## Шаг 1: Найдите файл настроек
Обычно файл находится по пути:
`Documents\AirSim\settings.json`

## Шаг 2: Добавьте конфигурацию Lidar
Добавьте секцию `"Sensors"` в ваш `settings.json`. Пример полного файла:

```json
{
  "SeeDocsAt": "https://github.com/Microsoft/AirSim/blob/master/docs/settings.md",
  "SettingsVersion": 1.2,
  "SimMode": "Multirotor",
  "Vehicles": {
    "Drone1": {
      "VehicleType": "SimpleFlight",
      "X": 0, "Y": 0, "Z": 0,
      "Sensors": {
        "LidarSensor1": {
          "SensorType": 6,
          "Enabled": true,
          "NumberOfChannels": 16,
          "RotationsPerSecond": 10,
          "PointsPerSecond": 100000,
          "X": 0, "Y": 0, "Z": -1,
          "Roll": 0, "Pitch": 0, "Yaw": 0,
          "VerticalFOVUpper": -15,
          "VerticalFOVLower": -25,
          "HorizontalFOVStart": -180,
          "HorizontalFOVEnd": 180,
          "DrawDebugPoints": true,
          "DataFrame": "SensorLocalFrame"
        }
      }
    }
  }
}
```

## Шаг 3: Настройки Lidar для МАКСИМАЛЬНОЙ ТОЧНОСТИ карты

Если облако точек получается рваным, с шумами и неточными поверхностями — используйте настройки ниже для максимальной детализации:

| Параметр | Низкая нагрузка (стабильность) | Обычное | Макс. точность |
|----------|-------------------------------|---------|----------------|
| `NumberOfChannels` | **16** | 16 | **32** или **64** |
| `PointsPerSecond` | **50000** | 100000 | **300000–500000** |
| `RotationsPerSecond` | 10 | **2–5** (медленнее = больше точек на угол) |
| `VerticalFOVUpper` / `VerticalFOVLower` | -15 / -25 | Расширьте по необходимости |

Пример конфигурации для максимальной точности:

```json
"LidarSensor1": {
  "SensorType": 6,
  "Enabled": true,
  "NumberOfChannels": 32,
  "RotationsPerSecond": 3,
  "PointsPerSecond": 400000,
  "X": 0, "Y": 0, "Z": -1,
  "Roll": 0, "Pitch": 0, "Yaw": 0,
  "VerticalFOVUpper": -15,
  "VerticalFOVLower": -25,
  "HorizontalFOVStart": -180,
  "HorizontalFOVEnd": 180,
  "DrawDebugPoints": true,
  "DataFrame": "SensorLocalFrame"
}
```

**Запуск с флагом высокой точности:**
```bash
python polet_work.py --high-accuracy
```

**Дополнительная постобработка** для сглаживания (после сохранения PLY):
```bash
python smooth_ply.py point_cloud.ply -o point_cloud_smooth.ply --k 15 --alpha 0.4
```

## Шаг 4: Перезапустите симуляцию
После сохранения файла перезапустите проект Unreal Engine / AirSim.

---

## Ошибка «StaticFindObjectFast() while garbage collecting» (Lidar crash)

При использовании Lidar в UE5 возможен фатальный краш:

```
Fatal error: Illegal call to StaticFindObjectFast() while serializing object data or garbage collecting!
UnrealEditor_AirSim!UAirBlueprintLib::GetObstacle() [...] UnrealLidarSensor::shootLaser()
```

**Причина:** `LineTraceSingleByChannel` внутри вызывает `StaticFindObjectFast`, что запрещено во время GC или сериализации (`UE::IsSavingPackage` / `IsGarbageCollectingAndLockingUObjectHashTables`).

**Решение:** В `UnrealLidarSensor.cpp` добавлена проверка — растрэйс пропускается во время GC и сериализации пакетов. Временно может теряться несколько точек в облаке, но краш устраняется.

**Действия:**
1. **Пересобрать проект Unreal** (Build → Rebuild в Editor), чтобы применить исправление.
2. Значения по умолчанию в коде снижены: `PointsPerSecond` 50000, лимит 50 000 точек/кадр. При проблемах в `settings.json` явно задайте `PointsPerSecond`: 50000, `NumberOfChannels`: 16.
3. Убедитесь, что используется **последовательная** версия Lidar (без ParallelFor) — текущий код уже исправлен.

---

## Ошибка «BP_CameraDirector contains a newer version»

Если при запуске Unreal появляется **Load Errors: Package '/AirSim/Blueprints/BP_CameraDirector' contains a newer version**, решение уже внесено:

1. В `Config/DefaultEngine.ini` добавлен `[CoreRedirects]` — пакет помечен как удалённый.
2. Запустите `unreal\fix_bp_camera_director_error.bat` — скрипт сбросит кэш и при необходимости отключит проблемный ассет.
3. Перезапустите Unreal Editor.
