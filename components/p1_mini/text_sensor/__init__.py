import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor
from esphome.const import CONF_FORMAT, CONF_ID, CONF_TIMEOUT

from .. import CONF_P1_MINI_ID, CONF_IDENTIFIER, P1Mini, identifier, p1_mini_ns

AUTO_LOAD = ["p1_mini"]

P1MiniTextSensor = p1_mini_ns.class_(
    "P1MiniTextSensor", text_sensor.TextSensor, cg.Component)

CONFIG_SCHEMA = text_sensor.text_sensor_schema(P1MiniTextSensor).extend(
    {
        cv.GenerateID(): cv.declare_id(P1MiniTextSensor),
        cv.GenerateID(CONF_P1_MINI_ID): cv.use_id(P1Mini),
        cv.Required(CONF_IDENTIFIER): cv.string
    }
)

async def to_code(config):
    var = cg.new_Pvariable(
        config[CONF_ID],
        config[CONF_IDENTIFIER],
    )
    await cg.register_component(var, config)
    await text_sensor.register_text_sensor(var, config)
    p1_mini = await cg.get_variable(config[CONF_P1_MINI_ID])
    cg.add(p1_mini.register_text_sensor(var))
