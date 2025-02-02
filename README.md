# esphome-pulsar-m
Подключение EspHome к счетчикам-регистраторам импульсов и счетчикам воды и газа с цифровым выходом по протоколу Пульсар-М через RS-485 
(Пульсар-2М, Пульсар-4М, Пульсар-6М, Пульсар-10М, Пульсар-16М, Пульсар-24М и другие).

Инструкции по подключению esp32/esp8266 к регистратору можно увидеть в соседнем компоненте https://github.com/latonita/esphome-energomera-iec

# Функции
## Реализовано
- Чтение выбранных каналов
- Автоопределение размерности данных (16/32/64 бит)
- Указание типа данных (integer / float - целые / вещественные числа)
- Чтение даты и времени
- Работа со счетчиком по указанному адресу либо поиск 
- Работа с несколькими счетчиками на одной или разных шинах (необходимо *обязательно* указать адрес каждого прибора)
- Синхронизация времени

## На будущее можно реализовать
- Запись текущих значений
- Использование/Чтение/запись весов импульсов - под вопросом, недостаточно документации, разные модели ведут себя по-разному
- .. предлагайте ваши варианты


# Параметры

Все параметры опциональные. НО крайне рекомендуется всегда указывать адрес.
| параметр | тип | описание | значение по-умолчанию | 
|---|---|---|---|
| `address` | число, без ведущих нулей при их наличии | серийный номер прибора. если не указать - будет произведена попытка поиска, но рекомендуется всегда указывать номер, т.к. поиск не надёжен  | `0`  |
| `value_type` | `float` или `integer` | тип значений из инструкции к прибору. Например, счетчики импульсов выдают вещественные значения, а счетчик воды - целочисленные |`float` | 
| `receive_timeout` | длительность | максимальное время ожидания ответа |`500ms`| 
| `flow_control_ping` | `pin`| ножка включения режима передачи | - |
| `update_interval`| длительность | период опроса прибора | `60s` |

## Пример использования

```yaml
uart:
  - id: uart_pulsar
    tx_pin:  GPIO25
    rx_pin:  GPIO23
    rx_buffer_size: 256
    baud_rate: 9600
    stop_bits: 1
    data_bits: 8
    parity: NONE
    
pulsar_m:
  - id: pulsar_1
    uart_id: uart_pulsar   # можно не указывать, если порт только один
    address: 3520285       # серийный номер
    value_type: integer    # см.инструкцию к прибору. например, счетчик воды выдает целые числа
    update_interval: 60s
    receive_timeout: 500ms
    flow_control_pin: GPIO21 # ножка включения передачи на модуле rx485

text_sensor:
  - platform: pulsar_m
    pulsar_m_id: pulsar_1
    datetime: "Date"
    address: "Address"

sensor:
  - platform: pulsar_m
    pulsar_m_id: pulsar_1
    channel: 1
    name: "Cold Water" 
    unit_of_measurement: "m³"
    filters:
      multiply: 0.001
    accuracy_decimals: 1
    device_class: water
    state_class: total_increasing

```
## Установка времени
Из любой лямбда функции можно установить время в приборе. Например, использовав компонент `sntp`:
```cpp
  if (id(sntp_time).now().is_valid()) {
    auto ts = id(sntp_time).now();
    id(pulsar_1).set_device_time(ts.timestamp);
  }
```

# Примеры готовых yaml конфигов

<details><summary>Пример файла конфигурации, протестированого на Пульсар-2М</summary>

```yaml
esphome:
  name: pulsar-2m
  friendly_name: pulsar-2m

esp32:
  board: esp32dev
  framework:
    type: arduino

#...


external_components:
  - source: github://latonita/esphome-pulsar-m
    components: [pulsar_m]
    refresh: 1s

uart:
  - id: uart_1
    tx_pin:  GPIO17
    rx_pin:  GPIO16
    rx_buffer_size: 256
    baud_rate: 9600
    stop_bits: 1
    data_bits: 8
    parity: NONE

pulsar_m:
  - id: pulsar_1
    uart_id: uart_1
    receive_timeout: 500ms
    address: 12345678  # опционально. лучше указать, метод поиска может отличаться от версии к версии
  # flow_control_pin: GPIO21 # если требуется для модуля rs485

text_sensor:
  - platform: pulsar_m
    datetime: "Date/Time"
    address: "Address"

sensor:
  - platform: pulsar_m
    channel: 1
    name: "Channel #1 Gas" 
    unit_of_measurement: "m³"
    accuracy_decimals: 1
    device_class: gas
    state_class: total_increasing

  - platform: pulsar_m
    channel: 2
    name: "Channel #2 Water" 
    unit_of_measurement: "m³"
    accuracy_decimals: 1
    device_class: water
    state_class: total_increasing

# Синхронизация времени

time:
  - platform: sntp
    id: sntp_time
    timezone: "MSK-3"

interval:
  - interval: 24h
    then:
      - lambda: |-
          if (id(sntp_time).now().is_valid()) {
            auto ts = id(sntp_time).now();
            id(pulsar_1).set_device_time(ts.timestamp);
          } else {
            ESP_LOGW("pulsar_m", "Time not valid yet!");
          }


```
</details>

<details><summary>Пример файла конфигурации с тремя счетчиками на двух шинах</summary>

```yaml
esphome:
  name: pulsar-multi
  friendly_name: pulsar-multi

esp32:
  board: esp32dev
  framework:
    type: arduino

#...


external_components:
  - source: github://latonita/esphome-pulsar-m
    components: [pulsar_m]
    refresh: 1s

uart:
  - id: uart_1
    tx_pin:  GPIO17
    rx_pin:  GPIO16
    rx_buffer_size: 256
    baud_rate: 9600
    stop_bits: 1
    data_bits: 8
    parity: NONE

  - id: uart_2
    rx_pin: GPIO33
    tx_pin: GPIO32
    rx_buffer_size: 256
    baud_rate: 9600
    stop_bits: 1
    data_bits: 8
    parity: NONE

pulsar_m:
  - id: pulsar_1
    uart_id: uart_1
    address: 111111
    #update_interval: 30s
    #receive_timeout: 500ms
    #flow_control_pin: GPIO21
  - id: pulsar_2
    uart_id: uart_1
    address: 222222
    #update_interval: 30s
    #receive_timeout: 500ms
    #flow_control_pin: GPIO21
  - id: pulsar_3
    uart_id: uart_2
    address: 333333
    #update_interval: 30s
    #receive_timeout: 500ms
    #flow_control_pin: GPIO21
  

text_sensor:
  - platform: pulsar_m
    pulsar_m_id: pulsar_1
    datetime: "P1 Date/Time"
    address: "P1 Address"

  - platform: pulsar_m
    pulsar_m_id: pulsar_2
    datetime: "P2 Date/Time"
    address: "P2 Address"
  
  - platform: pulsar_m
    pulsar_m_id: pulsar_3
    datetime: "P3 Date/Time"
    address: "P3 Address"

sensor:
  - platform: pulsar_m
    pulsar_m_id: pulsar_1
    channel: 1
    name: "P1 ch #1" 
    unit_of_measurement: "m³"
    accuracy_decimals: 1
    device_class: gas
    state_class: total_increasing

  - platform: pulsar_m
    pulsar_m_id: pulsar_1
    channel: 2
    name: "P1 ch #2 " 
    unit_of_measurement: "m³"
    accuracy_decimals: 1
    device_class: water
    state_class: total_increasing

  - platform: pulsar_m
    pulsar_m_id: pulsar_2
    channel: 1
    name: "P2 ch #1" 
    unit_of_measurement: "m³"
    accuracy_decimals: 1
    device_class: gas
    state_class: total_increasing

  - platform: pulsar_m
    pulsar_m_id: pulsar_3
    channel: 1
    name: "P3 ch #1" 
    unit_of_measurement: "m³"
    accuracy_decimals: 1
    device_class: gas
    state_class: total_increasing



# Синхронизация времени

time:
  - platform: sntp
    id: sntp_time
    timezone: "MSK-3"

interval:
  - interval: 24h
    then:
      - lambda: |-
          if (id(sntp_time).now().is_valid()) {
            auto ts = id(sntp_time).now();
            id(pulsar_1).set_device_time(ts.timestamp);
            id(pulsar_2).set_device_time(ts.timestamp);
            id(pulsar_3).set_device_time(ts.timestamp);
          } else {
            ESP_LOGW("pulsar_m", "Time not valid yet!");
          }



```
</details>
