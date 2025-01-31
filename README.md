# esphome-pulsar-m
Подключение EspHome к счетчикам-регистраторам импульсов и счетчикам воды с цифровым выходом по протоколу Пульсар-М через RS-485 
(Пульсар-2М, Пульсар-4М, Пульсар-6М, Пульсар-10М, Пульсар-16М и другие).

Инструкции по подключению esp32/esp8266 к регистратору можно увидеть в соседнем компоненте https://github.com/latonita/esphome-energomera-iec

# Реализовано:
- Чтение выбранных каналов
- Чтение даты и времени
- Работа со счетчиком по указанному адресу либо поиск 
- Работа с несколькими счетчиками на одной или разных шинах (необходимо обязательно указать адрес каждого прибора)

# На будущее можно реализовать:
- Запись текущих значений
- Чтение/запись весов импульсов
- Синхронизация времени
- .. предлагайте ваши варианты



Пример файла конфигурации, протестированого на Пульсар-2М

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

```

Пример файла конфигурации с тремя счетчиками на двух шинах

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


```
