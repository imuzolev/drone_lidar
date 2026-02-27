# Контекст для нового агента

## Текущий Unreal-проект
**Путь:** `C:/Users/Иван/Documents/Unreal Projects/yarik 5.2/yarik.uproject`

Это проект UE 5.2. При работе с Unreal Engine, плагинами AirSim/Colosseum или ошибками загрузки — учитывать этот путь.

---

## Python-проект (дрон, сканирование)
- **Рабочая папка:** `c:\CORTEXIS\test_dron_1`
- **Основной скрипт:** `polet_work.py` — автономное сканирование, Lidar, навигация
- **Устаревший скрипт:** `autonomous_exploration.py` — не использовать, изменения не вносить
- **Точность сканирования:** флаг `--high-accuracy` для плотного облака точек
- **Настройки Lidar:** `C:\Users\Иван\Documents\AirSim\settings.json`
- **Документация:** `docs/AIRSIM_SETUP.md`

---

## Ошибка BP_CameraDirector (Load Error)
Если появляется: *Package '/AirSim/Blueprints/BP_CameraDirector' contains a newer version* — ассет сохранён в UE 5.4, проект в UE 5.2.

**Решение:**
1. Отключить ассет: `BP_CameraDirector.uasset` → `BP_CameraDirector.uasset.disabled` во всех плагинах AirSim.
2. Скрипт: `unreal\fix_bp_camera_director_error.bat` — ищет и отключает по `C:\CORTEXIS`, но проект yarik в `Documents\Unreal Projects`, нужен отдельный поиск по этой папке.
3. CoreRedirects в `Config/DefaultEngine.ini`:
   ```ini
   [CoreRedirects]
   +PackageRedirects=(OldName="/AirSim/Blueprints/BP_CameraDirector",Removed=true)
   ```

**Для yarik 5.2:** проверить `C:\Users\Иван\Documents\Unreal Projects\yarik 5.2\Plugins\` на наличие AirSim и BP_CameraDirector; при наличии — применить те же шаги.

---

## Структура Unreal-проектов
- `test_dron_1\unreal\Blocks\` — Blocks с AirSim
- `test_dron_1\unreal\Blocks 5.2\` — Blocks с ProjectAirSim
- `C:\CORTEXIS\airsim\Unreal\Environments\` — ещё копии Blocks
- `C:\Users\Иван\Documents\Unreal Projects\yarik 5.2\` — активный проект yarik

---

## Дополнительные материалы
- `docs/CONTEXT_BP_CAMERA_DIRECTOR_FIX.md` — детали про BP_CameraDirector
- `docs/AIRSIM_SETUP.md` — настройка Lidar и режим `--high-accuracy`
