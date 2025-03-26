from dataclasses import dataclass
import logging

from esphome import pins
import esphome.codegen as cg
from esphome.components import uart, esp32_rmt
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID, 
    CONF_TX_PIN, 
    CONF_RX_PIN,
    CONF_BAUD_RATE,
    CONF_RMT_CHANNEL,
    CONF_RMT_SYMBOLS,
    CONF_RX_BUFFER_SIZE,
    PLATFORM_ESP32
)

from esphome.core import CORE

CONF_STOP_BITS = "stop_bits"
CONF_DATA_BITS = "data_bits"
CONF_PARITY = "parity"


CONF_RMT_TX_CHANNEL = "rmt_tx_channel"
CONF_RMT_RX_CHANNEL = "rmt_rx_channel"

CONF_RMT_TX_SYMBOLS = "rmt_tx_symbols"
CONF_RMT_RX_SYMBOLS = "rmt_rx_symbols"

_LOGGER = logging.getLogger(__name__)

DEPENDENCIES = ["esp32"]
AUTO_LOAD = ["uart", "esp32_rmt"]

CODEOWNERS = ["@DJJo14"]

rmt_uart_ns = cg.esphome_ns.namespace("esp32_rmt_uart")
RMTUARTComponent = rmt_uart_ns.class_("RMTUARTComponent", cg.Component,uart.UARTComponent)
MULTI_CONF = True

UARTParityOptions = rmt_uart_ns.enum("UARTParityOptions")
UART_PARITY_OPTIONS = {
    "NONE": UARTParityOptions.UART_CONFIG_PARITY_NONE,
    "EVEN": UARTParityOptions.UART_CONFIG_PARITY_EVEN,
    "ODD": UARTParityOptions.UART_CONFIG_PARITY_ODD,
}


def not_with_new_rmt_driver(obj):
    if esp32_rmt.use_new_rmt_driver():
        raise cv.Invalid(
            "This feature is not available for the IDF framework version 5."
        )
    return obj

def only_with_new_rmt_driver(obj):
    if not esp32_rmt.use_new_rmt_driver():
        raise cv.Invalid(
            "This feature is only available for the IDF framework version 5."
        )
    return obj

class OptionalForIDF5(cv.SplitDefault):
    @property
    def default(self):
        if not esp32_rmt.use_new_rmt_driver():
            return cv.UNDEFINED
        return super().default

    @default.setter
    def default(self, value):
        # Ignore default set from vol.Optional
        pass

CONFIG_SCHEMA = cv.All( cv.Schema({
        cv.GenerateID(CONF_ID): cv.declare_id(RMTUARTComponent),
        cv.Required(CONF_TX_PIN): pins.internal_gpio_output_pin_number,
        cv.Required(CONF_RX_PIN): pins.internal_gpio_output_pin_number,
        cv.Optional(CONF_BAUD_RATE, default=9600): cv.positive_int,  # Default to 9600 baud
        cv.Optional(CONF_RMT_TX_CHANNEL): cv.All(
            not_with_new_rmt_driver, esp32_rmt.validate_rmt_channel(tx=True)
        ),
        cv.Optional(CONF_RMT_RX_CHANNEL): cv.All(
            not_with_new_rmt_driver, esp32_rmt.validate_rmt_channel(tx=False)
        ),
        cv.Optional(CONF_RX_BUFFER_SIZE, default=1000): cv.validate_bytes,
        cv.Optional(CONF_STOP_BITS, default=1): cv.one_of(1, 2, int=True),
        cv.Optional(CONF_PARITY, default="NONE"): cv.enum(
            uart.UART_PARITY_OPTIONS, upper=True
        ),
        cv.Optional(CONF_DATA_BITS, default=8): cv.int_range(min=7, max=8),
        OptionalForIDF5(
                    CONF_RMT_TX_SYMBOLS,
                    esp32_idf=192,
                    esp32_s2_idf=192,
                    esp32_s3_idf=192,
                    esp32_c3_idf=96,
                    esp32_c6_idf=96,
                    esp32_h2_idf=96,
                ): cv.All(only_with_new_rmt_driver, cv.int_range(min=2)),
        OptionalForIDF5(
                    CONF_RMT_RX_SYMBOLS,
                    esp32_idf=192,
                    esp32_s2_idf=192,
                    esp32_s3_idf=192,
                    esp32_c3_idf=96,
                    esp32_c6_idf=96,
                    esp32_h2_idf=96,
                ): cv.All(only_with_new_rmt_driver, cv.int_range(min=2)),
    }).extend(cv.COMPONENT_SCHEMA),
    cv.only_on([PLATFORM_ESP32]),
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    #, config[CONF_TX_PIN], config[CONF_RX_PIN], config[CONF_BAUD_RATE]
    cg.add(var.set_tx_pin(config[CONF_TX_PIN]))
    cg.add(var.set_rx_pin(config[CONF_RX_PIN]))
    cg.add(var.set_baud_rate(config[CONF_BAUD_RATE]))  # Set baud rate
    cg.add(var.set_rx_buffer_size(config[CONF_RX_BUFFER_SIZE]))
    cg.add(var.set_stop_bits(config[CONF_STOP_BITS]))
    cg.add(var.set_data_bits(config[CONF_DATA_BITS]))
    cg.add(var.set_parity(config[CONF_PARITY]))

    if esp32_rmt.use_new_rmt_driver():
        cg.add(var.set_tx_rmt_symbols(config[CONF_RMT_TX_SYMBOLS]))
        cg.add(var.set_rx_rmt_symbols(config[CONF_RMT_RX_SYMBOLS]))
    else:
        rmt_channel_t = cg.global_ns.enum("rmt_channel_t")
        cg.add(
            var.set_rmt_channel(
                getattr(rmt_channel_t, f"RMT_CHANNEL_TX_{config[CONF_RMT_TX_CHANNEL]}")
            )
        )
        cg.add(
            var.set_rmt_channel(
                getattr(rmt_channel_t, f"RMT_CHANNEL_RX_{config[CONF_RMT_RX_CHANNEL]}")
            )
        )




    # yield cg.register_component(var, config)
