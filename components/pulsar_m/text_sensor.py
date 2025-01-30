import esphome.codegen as cg
from esphome.components import text_sensor
import esphome.config_validation as cv
from esphome.const import CONF_ADDRESS, CONF_DATETIME, CONF_ID

from . import CONF_PULSAR_M_ID, PulsarMComponent

AUTO_LOAD = ["pulsar_m"]
CODEOWNERS = ["@latonita"]

CONF_SERIAL_NR = "serial_nr"
CONF_STATE = "state"


TEXT_SENSORS = [
    CONF_DATETIME,
    CONF_ADDRESS,
    CONF_SERIAL_NR,
    CONF_STATE,
]

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_PULSAR_M_ID): cv.use_id(PulsarMComponent),
        cv.Optional(CONF_DATETIME): text_sensor.text_sensor_schema(),
        cv.Optional(CONF_ADDRESS): text_sensor.text_sensor_schema(),
        # cv.Optional(CONF_SERIAL_NR): text_sensor.text_sensor_schema(),
        # cv.Optional(CONF_STATE): text_sensor.text_sensor_schema(),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_PULSAR_M_ID])
    for key in TEXT_SENSORS:
        if key in config:
            conf = config[key]
            sens = cg.new_Pvariable(conf[CONF_ID])
            await text_sensor.register_text_sensor(sens, conf)
            cg.add(getattr(hub, f"set_{key}_text_sensor")(sens))
