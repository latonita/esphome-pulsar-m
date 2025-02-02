from esphome import pins
import esphome.codegen as cg
from esphome.components import uart
import esphome.config_validation as cv
from esphome.const import (
    CONF_ADDRESS,
    CONF_FLOW_CONTROL_PIN,
    CONF_ID,
    CONF_RECEIVE_TIMEOUT,
    CONF_UPDATE_INTERVAL,
)

CODEOWNERS = ["@latonita"]

MULTI_CONF = True

DEPENDENCIES = ["uart"]
AUTO_LOAD = ["sensor", "text_sensor"]

CONF_PULSAR_M_ID = "pulsar_m_id"
MAX_CHANNELS = 20

CONF_VALUE_TYPE = "value_type"

VALUE_TYPES = {
     "INTEGER":1, 
     "FLOAT":2
}


pulsar_m_ns = cg.esphome_ns.namespace("esphome::pulsar_m")
PulsarMComponent = pulsar_m_ns.class_(
    "PulsarMComponent", cg.PollingComponent, uart.UARTDevice
)

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(PulsarMComponent),
            cv.Optional(CONF_FLOW_CONTROL_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_ADDRESS, default=0): cv.int_range(min=0x0, max=0xFFFFFFFF),
            cv.Optional(
                CONF_RECEIVE_TIMEOUT, default="500ms"
            ): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_UPDATE_INTERVAL, default="30s"): cv.update_interval,
            cv.Optional(CONF_VALUE_TYPE, default="float"): cv.enum(VALUE_TYPES, upper=True),
        }
    ).extend(uart.UART_DEVICE_SCHEMA),
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    cg.add(var.set_receive_timeout(config[CONF_RECEIVE_TIMEOUT].total_milliseconds))
    cg.add(var.set_meter_address(config[CONF_ADDRESS]))

    # todo: make proper type selection if needed
    cg.add(var.set_is_integer(config[CONF_VALUE_TYPE] == "INTEGER"))

    if CONF_FLOW_CONTROL_PIN in config:
        pin = await cg.gpio_pin_expression(config[CONF_FLOW_CONTROL_PIN])
        cg.add(var.set_flow_control_pin(pin))
