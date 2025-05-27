import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import CONF_FORMAT, CONF_ID, CONF_TIMEOUT

from .. import CONF_P1_MINI_ID, CONF_OBIS_CODE, P1Mini, obis_code, p1_mini_ns

AUTO_LOAD = ["p1_mini"]

P1MiniSensor = p1_mini_ns.class_(
    "P1MiniSensor", sensor.Sensor, cg.Component)

CONFIG_SCHEMA = sensor.sensor_schema(P1MiniSensor).extend(
    {
        cv.GenerateID(): cv.declare_id(P1MiniSensor),
        cv.GenerateID(CONF_P1_MINI_ID): cv.use_id(P1Mini),
        cv.Required(CONF_OBIS_CODE): cv.string
    }
)

async def to_code(config):
    var = cg.new_Pvariable(
        config[CONF_ID],
        config[CONF_OBIS_CODE],
    )
    await cg.register_component(var, config)
    await sensor.register_sensor(var, config)
    p1_mini = await cg.get_variable(config[CONF_P1_MINI_ID])
    cg.add(p1_mini.register_sensor(var))
