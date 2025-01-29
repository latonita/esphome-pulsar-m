import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import CONF_CHANNEL

from . import CONF_PULSAR_M_ID, PulsarMComponent, pulsar_m_ns

CODEOWNERS = ["@latonita"]

DEPENDENCIES = ["pulsar_m"]

PulsarMSensor = pulsar_m_ns.class_("PulsarMSensor", sensor.Sensor)


# user shall define each sensor in the configuration
# each sensor have unique channel id
# everything else is optional, user can configure measurement units, device class, state class as he wants
# schema shall be extended from standard sensor schema
CONFIG_SCHEMA = cv.All(
    sensor.sensor_schema(
        PulsarMSensor,
    ).extend(
        {
            cv.GenerateID(CONF_PULSAR_M_ID): cv.use_id(PulsarMComponent),
            cv.Required(CONF_CHANNEL): cv.int_range(min=1, max=20),
        }
    ),
    cv.has_exactly_one_key(CONF_CHANNEL),
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_PULSAR_M_ID])
    var = await sensor.new_sensor(config)
    cg.add(var.set_channel(config[CONF_CHANNEL]))
    cg.add(hub.register_sensor(var))
