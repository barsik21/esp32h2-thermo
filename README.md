| Supported Targets | ESP32-C6 | ESP32-H2 |
| ----------------- | -------- | -------- |

# Пример термометра с дополнительными атрибутами, управлением реле.
В качестве датчика температуры и влажности используется SHT30

## Endpoint 1:
- Кластер температуры с тремя атрибутами: верхний порог температуры, нижний порог температуры, период репортов температуры
- Кластер времени
## Endpoint 2:
- Кластер on/off для управления реле/светом (client)

Репорт по умолчанию настроен на 5 минут.

# To-Do:
- [ ] Реализовать сохранение значений атрибутов максимальной/минимальной температуры (если есть пример - welcome to issue)
- [ ] Доделать изменение периода репортов, сохранение
- [ ] Чтение температуры с внешнего датчика по команде
- [ ] Почистить исходники от лишнего кода, так как данный репо - сборная солянка.

## Отдельное спасибо
- [Efekta](https://github.com/smartboxchannel) за примеры конвертеров.
- [xyzroe](https://github.com/xyzroe/Q_sensor) за примеры кода в его репозитории.
- А также чату [Zigdev](https://t.me/zigdev).