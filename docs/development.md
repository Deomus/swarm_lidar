# Разработка

## Python и VS Code в WSL

Открывайте репозиторий через VS Code Remote — WSL и используйте интерпретатор
`/usr/bin/python3` (Python 3.12 для Ubuntu 24.04). Настройки проекта в
`.vscode/settings.json` добавляют ROS 2 Jazzy и исходники пакетов в поиск
Pylance. Сгенерированные сообщения доступны после сборки `swarm_interfaces`.

Если ранее выбран другой интерпретатор, выполните `Python: Select Interpreter`
и укажите `/usr/bin/python3`: `defaultInterpreterPath` не заменяет уже
сохранённый выбор. Затем выполните `Python: Restart Language Server` или
`Developer: Reload Window`.

Настройки Pylance не подключают окружение терминала. Перед запуском команд:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

## Чекпоинты

Каждый проверенный этап сохраняется отдельным коммитом Conventional Commits.
Тип — на английском, описание — на русском, например:

```text
fix: исправить поиск ROS-пакетов в Pylance
```

В теле коммита указываются выполненные изменения, проверки и существенные
ограничения. История уже опубликованных коммитов не переписывается.
