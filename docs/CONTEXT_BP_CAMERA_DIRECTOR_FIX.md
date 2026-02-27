# Контекст: исправление ошибки BP_CameraDirector

## Проблема
При запуске Unreal Engine (проекты Blocks или Blocks 5.2) появляется **Load Error** в Message Log:
> Package '/AirSim/Blueprints/BP_CameraDirector' contains a newer version than the current process supports. PackageVersion 1 012, MaxExpected...

**Причина:** BP_CameraDirector сохранён в **UE 5.4**, проекты используют **UE 5.2**.

Дрон зависает при сканировании (scan_rotate timeout) — это отдельный баг. Активный код — **polet_work.py**.

## Структура проекта
- **`test_dron_1/unreal/Blocks/`** — UE5.2, плагин AirSim
- **`test_dron_1/unreal/Blocks 5.2/`** — UE5.2, плагин ProjectAirSim
- **`C:\CORTEXIS\airsim\Unreal\Environments\`** — ещё копии Blocks (Blocks, Blocks 5.2, Blocks 5.2 - 2)
- **`C:\CORTEXIS\Colosseum\Unreal\`** — Colosseum, AirSim
- **`polet_work.py`** — основной скрипт сканирования

## Реализованное решение
1. **Отключение ассета** — `BP_CameraDirector.uasset` → `BP_CameraDirector.uasset.disabled` во **всех** папках C:\CORTEXIS (включая airsim, Colosseum, Saved/EditorCooked, StagedBuilds).

2. **CoreRedirects** в DefaultEngine.ini всех проектов — fallback, если файл снова появится.

3. **Скрипт** `unreal/fix_bp_camera_director_error.bat` — рекурсивный поиск и отключение, очистка кэша.

## Гипотезы для отладки (если ошибка вернётся)
1. **Референс в .umap** — BlocksMap или AirSimAssets.umap может содержать размещённый в уровне BP_CameraDirector.
2. **C++ код плагина** — AirSim может явно загружать BP_CameraDirector в коде.
3. **Asset Registry** — путь в Saved/ может кэшировать пакет; нужна полная очистка Saved + Intermediate.
4. **Другая версия файла** — возможно, BP_CameraDirector есть в Blocks 5.2 или в другом плагине (ProjectAirSim наследует/включает AirSim?).
5. **Формат пути** — UE может использовать `/Game/`, `/Plugins/AirSim/` или другой mount point.

## Что НЕ ломать
- Логику polet_work.py (сканирование, Lidar, навигация).
- Плагины AirSim / ProjectAirSim — только проблемный ассет.
- Конфигурацию симуляции (settings.json, DefaultEngine.ini кроме CoreRedirects).

## Файлы для проверки
- `unreal/Blocks/Plugins/AirSim/Content/Blueprints/` — BP_CameraDirector.uasset.bak
- `unreal/Blocks/Config/DefaultEngine.ini` — CoreRedirects
- `unreal/Blocks 5.2/Config/DefaultEngine.ini` — CoreRedirects
- `unreal/Blocks/Plugins/AirSim/Content/AirSimAssets.umap` — уровень, может содержать референсы
- `unreal/Blocks/Content/` — BlocksMap.umap

## Цель
Надёжно устранить ошибку Load Error BP_CameraDirector без поломки симуляции.

## Выполнено (27.02.2026)
- BP_CameraDirector.uasset отключён (.uasset.disabled) в 9 местах: airsim Blocks/Blocks 5.2/Blocks 5.2-2, Colosseum, test_dron_1, Saved/EditorCooked, StagedBuilds
- CoreRedirects добавлены в DefaultEngine.ini: test_dron_1 Blocks + Blocks 5.2, airsim (3 проекта), Colosseum BlocksV2
- Скрипт fix_bp_camera_director_error.bat обновлён: поиск по всем C:\CORTEXIS, очистка кэша 6 проектов
