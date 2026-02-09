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

## Шаг 3: Перезапустите симуляцию
После сохранения файла перезапустите проект Unreal Engine / AirSim.
