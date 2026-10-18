# Запуск симуляции на Ubuntu с видеокартой NVIDIA

Проверено только на Mac (OrbStack, без окна Gazebo). Шаги ниже для Ubuntu написаны по документации и
**на реальной Ubuntu не прогонялись**: если что-то не заведётся, смотрите раздел «Если не работает».

## 0. Что нужно
- Ubuntu 22.04 или новее, ~15 ГБ свободного места под образ и сборку (плюс датасеты и серии: `data/` ~400 МБ на серию карт).
- Драйвер NVIDIA: команда `nvidia-smi` должна показать видеокарту.
- Интернет на время сборки образа (~6 ГБ, 20–30 минут).
- Сессия **X11** (не Wayland): на экране входа выберите «Ubuntu on Xorg». Проверка: `echo $XDG_SESSION_TYPE` должно вывести `x11`.

## 1. Docker и поддержка GPU (один раз)
```bash
# Docker Engine: https://docs.docker.com/engine/install/ubuntu/
sudo apt-get update && sudo apt-get install -y ca-certificates curl
curl -fsSL https://get.docker.com | sudo sh
sudo usermod -aG docker $USER      # потом выйти и зайти заново

# NVIDIA Container Toolkit: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

# проверка: должна показать вашу видеокарту
docker run --rm --gpus all ubuntu nvidia-smi
```

## 2. Перенос проекта
На Mac архив уже собран: `~/Desktop/ugv-vslam/ugv_ws_for_ubuntu.tar.gz` (без папки `data/` с тяжёлыми
сериями и картами). Скопируйте его любым способом (`scp`, флешка, облако) и распакуйте:
```bash
mkdir -p ~/ugv && tar -xzf ugv_ws_for_ubuntu.tar.gz -C ~/ugv && cd ~/ugv/ugv_ws
```

## 3. Сборка образа (один раз, 20–30 минут)
```bash
cd docker && docker build -t ugv-ros2:humble . && cd ..
```
Dockerfile сам берёт нужную архитектуру (на обычной Ubuntu это x86_64). Образ называется `ugv-ros2:humble`,
под этим именем его ждут все скрипты.

## 4. Запуск с окном Gazebo
```bash
tools/run_sim_gui.sh
```
Через 1–2 минуты откроется окно Gazebo (арена, робот), а интерфейс оператора будет на
http://localhost:8080 : левой кнопкой перетащите по карте (направление перетаскивания = курс робота),
нажмите «Go to goal, then return to start». Логи: `docker logs -f ugv-sim`. Остановка: `docker rm -f ugv-sim`.

Первая сборка пакетов (colcon) происходит внутри контейнера при старте и занимает около минуты.

## 5. Серии прогонов и метрики (без окна, быстрее)
```bash
tools/run_series.sh my_series test_world true 2        # 16 прогонов; результаты в data/my_series/
tools/run_injection_series.sh my_inj true 2            # с искусственными скачками карты
tools/run_map_then_localize.sh my_map                  # карта -> сохранение -> режим localization
```
Графики и видео по результатам (внутри образа):
```bash
docker run --rm -v "$PWD:/w" -w /w ugv-ros2:humble python3 tools/plot_series.py data/my_series docs/figures/my_series
```
Файлы, созданные контейнером, принадлежат root. Если мешает: `sudo chown -R $USER:$USER data docs`.

## Если не работает
| Симптом | Что проверить |
|---|---|
| `permission denied` у docker | пользователь не в группе docker: `sudo usermod -aG docker $USER` и перелогиниться |
| `could not select device driver "nvidia"` | не поставлен/не настроен `nvidia-container-toolkit` (шаг 1), перезапуск docker |
| окно Gazebo не появилось или чёрное | `echo $XDG_SESSION_TYPE` (нужен `x11`); `xhost +local:docker`; в контейнере `docker exec ugv-sim nvidia-smi` |
| `cannot connect to X server` | не задан `DISPLAY`: запускайте из терминала в графической сессии, не по ssh |
| интерфейс на :8080 не открывается | `docker logs ugv-sim | grep rosbridge`; контейнер использует `--network host`, порты не пробрасываются отдельно |
| симуляция идёт медленнее реального времени | это нормально без мощного GPU; метрики считаются по симуляционному времени, так что результаты не страдают |

Задержки и скорость на Ubuntu могут отличаться от Mac, поэтому цифры для диплома лучше перепрогнать на
целевой машине.
