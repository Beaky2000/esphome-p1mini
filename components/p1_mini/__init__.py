import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID, CONF_TRIGGER_ID
from esphome import automation

DEPENDENCIES = ['uart' ]
p1_mini_ns = cg.esphome_ns.namespace('p1_mini')
P1Mini = p1_mini_ns.class_('P1Mini', cg.Component, uart.UARTDevice)
data_formats = p1_mini_ns.enum("data_formats", is_class=True)
DATA_FORMATS = {
    "ASCII": data_formats.ASCII,
    "BINARY": data_formats.BINARY,
}
MULTI_CONF = True

CONF_P1_MINI_ID = "p1_mini_id"
CONF_OBIS_CODE = "obis_code"
CONF_MULTIPLIER = "multiplier"
CONF_IDENTIFIER = "identifier"
CONF_MINIMUM_PERIOD = "minimum_period"
CONF_BUFFER_SIZE = "buffer_size"
CONF_DATA_FORMATS = "data_formats"
CONF_SECONDARY_P1 = "secondary_p1"
CONF_ON_READY_TO_RECEIVE = "on_ready_to_receive"
CONF_ON_UPDATE_RECEIVED = "on_update_received"
CONF_ON_COMMUNICATION_ERROR = "on_communication_error"



# Triggers
ReadyToReceiveTrigger = p1_mini_ns.class_("ReadyToReceiveTrigger", automation.Trigger.template())
UpdateReceivedTrigger = p1_mini_ns.class_("UpdateReceivedTrigger", automation.Trigger.template())
CommunicationErrorTrigger = p1_mini_ns.class_("CommunicationErrorTrigger", automation.Trigger.template())

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(P1Mini),
    cv.Optional(CONF_SECONDARY_P1, False): cv.boolean,
    cv.Optional(CONF_SECONDARY_P1, False): cv.boolean,
    cv.Optional(CONF_MINIMUM_PERIOD, default="0s"): cv.time_period,
    cv.Optional(CONF_BUFFER_SIZE, default=3072): cv.int_range(min=512, max=32768),
    cv.Optional(CONF_DATA_FORMATS, default="ASCII"): cv.enum(DATA_FORMATS, upper=True),
    cv.Optional(CONF_ON_READY_TO_RECEIVE): automation.validate_automation(
        {
            cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(ReadyToReceiveTrigger),
        }
    ),
    cv.Optional(CONF_ON_UPDATE_RECEIVED): automation.validate_automation(
        {
            cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(UpdateReceivedTrigger),
        }
    ),
    cv.Optional(CONF_ON_COMMUNICATION_ERROR): automation.validate_automation(
        {
            cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(CommunicationErrorTrigger),
        }
    )
}).extend(cv.COMPONENT_SCHEMA).extend(uart.UART_DEVICE_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(
        config[CONF_ID],
        config[CONF_MINIMUM_PERIOD].total_milliseconds,
        config[CONF_BUFFER_SIZE],
        config[CONF_SECONDARY_P1],
        config[CONF_DATA_FORMATS],
        )
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    for conf in config.get(CONF_ON_READY_TO_RECEIVE, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID])
        cg.add(var.register_ready_to_receive_trigger(trigger))
        await automation.build_automation(trigger, [], conf)

    for conf in config.get(CONF_ON_UPDATE_RECEIVED, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID])
        cg.add(var.register_update_received_trigger(trigger))
        await automation.build_automation(trigger, [], conf)

    for conf in config.get(CONF_ON_COMMUNICATION_ERROR, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID])
        cg.add(var.register_communication_error_trigger(trigger))
        await automation.build_automation(trigger, [], conf)

def obis_code(value):
    value = cv.string(value)
    #match = re.match(r"^\d{1,3}-\d{1,3}:\d{1,3}\.\d{1,3}\.\d{1,3}$", value)
    # if match is None:
    #    raise cv.Invalid(f"{value} is not a valid OBIS code")
    return value

def identifier(value):
    value = cv.string(value)
    return value
